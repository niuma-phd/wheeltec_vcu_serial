<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 Noetic 适配层

本 catkin 包是已安装、与中间件无关的 C++14 库 `wheeltec_vcu_serial` 的薄
ROS 1 边界，目标平台为 Ubuntu 20.04 与 ROS 1 Noetic。节点直接组合核心的
`PosixSerialTransport`、`FeedbackParser` 和 `SafetySession`；协议与安全行为
继续由 ROS-free 核心负责。

`adapter_support` target 和 header 仅为包内实现及测试接缝，静态链接进节点，
不安装也不导出。长期维护的公共 ROS 合同仅包括下文所述的生成消息、服务、
节点可执行文件、参数、topic 与 service。

> **安全限制：** host write 完成只是一项操作系统侧结果，不是 VCU ACK、命令
> 回显、控制器接受命令的证明，也不能证明车辆已经停止。物理急停链必须始终
> 独立存在。

## 构建

先把核心安装到一个前缀，再在构建 catkin workspace 时暴露该前缀：

```bash
/usr/bin/cmake -S /path/to/wheeltec_vcu_serial \
  -B /tmp/wheeltec-core-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DWHEELTEC_BUILD_APPS=OFF
/usr/bin/cmake --build /tmp/wheeltec-core-build --parallel 2
/usr/bin/cmake --install /tmp/wheeltec-core-build \
  --prefix /tmp/wheeltec-core-install

source /opt/ros/noetic/setup.bash
mkdir -p /tmp/wheeltec-ros1-ws/src
ln -s /path/to/wheeltec_vcu_serial/ros1/wheeltec_vcu_serial_ros1 \
  /tmp/wheeltec-ros1-ws/src/wheeltec_vcu_serial_ros1
(cd /tmp/wheeltec-ros1-ws && catkin_make \
  -DCMAKE_PREFIX_PATH="/tmp/wheeltec-core-install;/opt/ros/noetic")
```

[`../ci/run_ros1_ci.sh`](../ci/run_ros1_ci.sh) 自动执行本机已验证的精确流程。
测试覆盖转换、门禁、消息合同，以及使用空设备 offline 配置运行的真实节点；
任何测试都不会进入 actuation 路径，也不会打开串口设备。

## AutoRoverField 现场部署

以下流程适用于 Ubuntu 20.04、ROS 1 Noetic，以及由 udev 固定为
`/dev/wheeltec_controller` 的单台 VCU。不要在部署脚本中枚举或猜测
`ttyACM*` / `ttyUSB*`。

### 1. 建立身份固定的 udev 别名

先对实际设备读取 USB 父设备属性：

```bash
udevadm info --attribute-walk --name=/dev/ttyACM0
```

用现场确认的 VID、PID 和序列号创建 `/etc/udev/rules.d/99-wheeltec-controller.rules`：

```udev
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTRS{serial}=="<SERIAL>", GROUP:="dialout", MODE:="0660", SYMLINK+="wheeltec_controller"
```

不要把占位符原样安装，也不要使用 `MODE="0777"`。安装或修改规则后执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm settle
readlink -f /dev/wheeltec_controller
```

运行节点的账号必须属于 `dialout` 组；新增组成员后需要重新登录：

```bash
sudo usermod -aG dialout "$USER"
id -nG
```

### 2. 安装配置

从随包的 offline 配置复制一份车辆专用文件，保留 watchdog 参数并显式设置设备与限值：

```bash
sudo install -d -m 0755 /etc/auto-rover
sudo install -m 0640 \
  ros1/wheeltec_vcu_serial_ros1/config/offline.ini \
  /etc/auto-rover/wheeltec-vcu.ini
sudoedit /etc/auto-rover/wheeltec-vcu.ini
```

至少核对以下字段：

```ini
[serial]
device = /dev/wheeltec_controller

[limits]
max_linear_speed_mps = 6.00
max_abs_yaw_rate_radps = 1.00
```

`6.00 m/s` 是软件允许的上限，不是建议启动速度；实际应用仍应以自己的低速预设下发
命令。串口线协议固定为 115200 bit/s、8N1、无流控。

### 3. 先做 offline 验收

不带任何 actuation 参数启动，确认节点不会打开串口：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
roslaunch wheeltec_vcu_serial_ros1 adapter.launch
```

另一个终端检查：

```bash
rostopic echo -n1 /wheeltec_vcu_serial_adapter/adapter_state
rosservice call /wheeltec_vcu_serial_adapter/disarm '{}'
```

预期 `actuation_enabled=false`、`session_state=offline`，且 disarm 返回本地已处于未授权状态。

### 4. 启动实车适配器

确认串口没有被其它进程占用、车辆处于 DISARM、物理急停可操作后，再显式打开三项门禁：

```bash
roslaunch wheeltec_vcu_serial_ros1 adapter.launch \
  config_file:=/etc/auto-rover/wheeltec-vcu.ini \
  acknowledge_unverified_protocol:=true \
  enable_actuation:=true \
  operator_confirmation:=I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

启动成功后应先处于 `connected_inhibited`，不会因串口连接本身自动授权。受信 supervisor
需要用严格递增且非零的 token 调用：

```bash
rosservice call /wheeltec_vcu_serial_adapter/authorize 'token: 1'
```

普通上锁使用独立 disarm 服务，不需要急停 reset token：

```bash
rosservice call /wheeltec_vcu_serial_adapter/disarm '{}'
```

调用结果只代表主机本地状态机接受了请求；该 VCU 线协议没有 ACK。停进程前先 disarm，
再用 `Ctrl-C` 正常退出。物理急停链始终独立存在。

### 5. 更新、回滚和验收

更新只部署经过审查的 `ros1/noetic` 提交或标签，重新构建核心和 catkin 包，然后重复
offline 验收。回滚时切回上一个已验证提交并重新安装，不要混用新旧核心库。

实车上线前至少验证：udev 别名指向预期 USB 身份、节点初始未授权、反馈新鲜、授权后低速
前进/倒车符号正确、disarm 立即撤销授权并进入零帧 episode、命令超时归零、软件急停锁存，
以及重连后必须重新授权。

## 失败即关闭的启动

必填私有参数 `~config_file` 必须指向符合核心严格语法的 INI 文件。
`max_linear_speed_mps` 必须为有限值，且满足
`0 < max_linear_speed_mps <= 6.0`；核心还会独立检查每个有符号 16-bit 线协议
字段。

三项门禁参数只有以下两种组合合法：

- `false`、`false`、空 confirmation：进入持续 **offline** 模式。节点继续
  发布状态并提供 ROS API，但绝不调用串口 `open()`，也不重连；状态中报告
  `actuation_enabled=false`。
- `true`、`true`、下方精确短语：允许进入 actuation 模式。只有此模式额外
  要求配置非空的直接 `/dev/...` 路径，并可能打开 TTY。

布尔值只开启一部分、禁用时提供非空 confirmation，或 confirmation 不完全
匹配，都会在构造可运行的适配器及打开串口之前故障闭合。actuation 组合为：

```yaml
acknowledge_unverified_protocol: true
enable_actuation: true
operator_confirmation: I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

随包提供的 [`adapter.launch`](launch/adapter.launch) 有意选择 device 为空的
INI，将两项布尔门禁设为 false，并将 confirmation 留空。不加修改地运行时，
节点会持续处于 offline 模式，无法打开硬件。

完成具体车辆配置与标定后，launch 调用处仍必须清楚展示全部门禁，例如：

```bash
roslaunch wheeltec_vcu_serial_ros1 adapter.launch \
  config_file:=/absolute/path/to/commissioned.ini \
  acknowledge_unverified_protocol:=true \
  enable_actuation:=true \
  operator_confirmation:=I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

## ROS 接口契约

以下名称默认均为节点私有名称。

### `~drive_command`（`wheeltec_vcu_serial_ros1/DriveCommand`）

消息字段接口契约为：

```text
uint64 sequence_id
float64 linear_speed_mps
float64 yaw_rate_radps
duration valid_for
```

- `sequence_id` 必须非零，并在整个进程生命周期内严格递增；串口重连也不会
  重置该要求。
- `linear_speed_mps` 单位为 m/s：正值前进，负值后退。
- `yaw_rate_radps` 单位为 rad/s：正值左转，负值右转。
- `valid_for` 必须为正，且不得大于配置的 `command_timeout_ms`。
- 有效期从 callback 收到消息的时刻开始。适配层采样 `CLOCK_MONOTONIC` 并
  构造核心所需的两个时间戳；watchdog 决策不信任发送方时间或 ROS time。
- 任意格式错误或非有限的适配层命令都会在本地撤销授权；核心的范围、授权、
  防重放、反馈新鲜度和恢复检查仍然全部生效。

本包有意不订阅 `/cmd_vel`。通用 twist 无法承载 sequence、相对接收时刻的
有效期、授权和本车辆边界要求的失败语义。应用必须把已经检查过的车辆命令
显式转换成 `DriveCommand`。

该命令接口契约没有 `frame_id`。线速度以 m/s 表示，正值前进、负值后退；角速度
以 rad/s 表示，正 yaw 向左、负 yaw 向右。启用 actuation 前，必须针对已安装
车辆完成这些符号约定的标定。

### `~feedback`（`wheeltec_vcu_serial_ros1/Feedback`）

消息字段接口契约为：

```text
time receipt_time
uint8 composite_stop_flag_raw
bool control_allowed
bool control_inhibited
float64 linear_speed_mps
float64 lateral_speed_mps
float64 yaw_rate_radps
float64[3] linear_acceleration_mps2
float64[3] angular_velocity_radps
float64 supply_voltage_v
bool vcu_ack_available
bool source_time_available
```

读取并验证帧后，`receipt_time` 从 ROS clock 采样；核心 watchdog 独立使用
单调时钟的接收时刻。纵向/横向速度单位为 m/s，yaw rate 和角速度单位为
rad/s，线加速度单位为 m/s²，供电电压单位为 V。`control_allowed` 和
`control_inhibited` 只表示已解码的二值综合 inhibit 字段，不是故障诊断或
acknowledgement。

这些数值对应控制器原生通道位置，并带 host 接收时刻；消息没有 ROS
`frame_id`，也不声称符合 REP-103 或与车辆上 IMU 安装轴对齐。消费者必须先
标定安装方向与符号约定，并在用于定位或控制前完成显式坐标系转换。

`vcu_ack_available` 和 `source_time_available` 恒为 false，因为该线协议既不
携带 ACK，也不携带源时间。反馈帧不会与任何命令 sequence 建立关联。

### `~adapter_state`（`wheeltec_vcu_serial_ros1/AdapterState`）

消息字段接口契约为：

```text
time receipt_time
string session_state
bool actuation_enabled
bool connected
uint64 connection_generation
bool authorized
bool software_estop_latched
bool vcu_ack_available
bool source_time_available
```

这个 latched 状态 topic 包含 `receipt_time`、`session_state`、
`actuation_enabled`、连接状态/代次、本地授权状态、
`software_estop_latched`，以及两项元数据可用性标志。offline 模式报告
`session_state="offline"`、`actuation_enabled=false`、未连接、generation 为
零且未授权，并明确 VCU ACK 与 source time 均不可用。actuation 模式下每次
重连都会增加 generation 并撤销授权。

### 服务

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `~authorize` | `Authorize` | 仅在 initial zero 完成且收到新鲜的允许控制反馈后，接受新的、非零、严格递增 token。 |
| `~disarm` | `std_srvs/Trigger` | 撤销本地运动授权、清除运动意图，并开始有界零帧 episode。 |
| `~stop` | `std_srvs/Trigger` | 清除本地运动意图，并在条件允许时开始有界零帧 episode。 |
| `~emergency_stop` | `std_srvs/Trigger` | 锁存独立的软件急停路径；绝不自动恢复。 |
| `~reset_emergency_stop` | `ResetEstop` | 要求新的 token、当前 connection generation、已完成的零帧以及新鲜的允许控制反馈；随后仍需重新授权才能运动。 |

`Authorize` 和 `ResetEstop` 的 request 均为 `uint64 token`，response 均为
`bool accepted` 与 `string reason`。`std_srvs/Trigger` 使用空 request，并返回
`bool success` 与 `string message`。

服务响应只表示本地状态机是否接受请求，从不表示 VCU 接受请求。offline 模式
拒绝授权和运动，把 disarm/stop 视为已经处于本地无 I/O 状态，且绝不尝试串口输出。

### Token 与 ROS graph 安全边界

> 授权 token 和软件急停 reset token 各自维护独立高水位，分别要求严格递增，
> 二者不共享序列。它们只是防重放值，**不是** password、credential、
> capability 或调用者身份证明。

本适配层不会对 ROS 1 service 调用者做身份认证。任何能够访问同一 ROS graph
及 ROS master 的进程或网络参与者，都可能调用 `~authorize`、
`~reset_emergency_stop` 和其他服务。部署时必须限制主机进程、网络与 ROS
master 的访问，并由受信 supervisor 或等效的部署侧 ACL/隔离机制决定谁有权
授权或 reset。三道启动门禁只约束启动时是否进入 actuation 模式，**不保护**
运行时 ROS graph 上的服务调用。

## 运行时安全行为

节点使用单线程和 `ros::spinOnce()`，因此 callback、parser 输入与核心 session
调用均在同一线程串行执行。offline 模式绝不进入 open、read、write 或 reconnect
路径。actuation 模式每次成功打开串口后都会尝试 initial zero。链路丢失会清除
运动意图；重连会重置 parser、创建新的 connection generation、尝试 initial
zero，并要求新鲜的允许控制反馈及更大的新授权 token。配置的 command 与
feedback watchdog 会在输入过期时请求零帧。

运动帧写入失败会撤销授权，并触发核心零帧路径。关闭时会清除本地运动意图，
并单独执行次数有界的精确零帧 host-write 尝试；最终零帧 host write 未完成时，
节点返回 9。无论尝试失败还是 host write 完成，都无法由此确认 VCU 已收到或
执行零命令。断电、`SIGKILL`、内核故障、线缆断开，或控制器继续保持旧目标等
情况，都无法由这个用户态节点保证安全。
