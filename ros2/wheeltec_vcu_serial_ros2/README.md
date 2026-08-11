<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 2 Humble 适配器

`wheeltec_vcu_serial_ros2` 是已安装的、ROS-free 的
`wheeltec_vcu_serial` CMake 包的薄 ROS 2 边界。目标平台是 Ubuntu 22.04
上的 ROS 2 Humble，语言标准为 C++14。串口成帧、deadline、重连代次、
watchdog、授权、零帧 episode 和软件急停锁存均保留在核心库中。

本包不订阅 `/cmd_vel`，不对其重新解释，也不把它作为内部接口契约。输入契约是
面向本车型、以回调接收时刻为基准的 `DriveCommand` 消息。

> 主机完成串口写入不等于 VCU acknowledgement、控制器接受、命令回显，
> 也不能证明车辆实际运动或停止。物理急停和牵引能源隔离必须始终作为独立的
> 安全机制保留。

## ROS 图接口契约与信任边界

下列名称均为节点私有名称。使用默认节点名时，它们展开在
`/wheeltec_vcu_serial_adapter` 下；launch 或应用组合可以显式 remap
任一端点。

| 名称 | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `~/drive_command` | `wheeltec_vcu_serial_ros2/msg/DriveCommand` | 订阅 | 经过防护的纵向速度与 yaw-rate 请求 |
| `~/feedback` | `wheeltec_vcu_serial_ros2/msg/Feedback` | 发布 | 已验证的 24-byte VCU feedback，带主机接收时间 |
| `~/adapter_state` | `wheeltec_vcu_serial_ros2/msg/AdapterState` | 发布 | transient-local 的连接与安全状态 |
| `~/authorize` | `wheeltec_vcu_serial_ros2/srv/Authorize` | 服务 | 使用严格递增 token 授权一个连接代次 |
| `~/stop` | `std_srvs/srv/Trigger` | 服务 | 接受幂等的本地停止意图；active 时启动有界零帧 episode |
| `~/emergency_stop` | `std_srvs/srv/Trigger` | 服务 | 锁存独立的本地软件急停路径，offline 时同样有效 |
| `~/reset_emergency_stop` | `wheeltec_vcu_serial_ros2/srv/ResetEstop` | 服务 | 条件清除后使用新 token 复位锁存 |

授权 token 与急停复位 token 各自使用独立的进程内高水位，分别要求严格
单调递增，二者不共享序列。它们只是防重放值，不是凭据
（**not a credential**），不是身份证明、秘密或调用者认证机制。承载命令与
服务的 ROS 图必须受信且隔离（trusted and isolated）。`~/authorize` 和
`~/reset_emergency_stop` 的调用者身份与权限必须由部署安全机制
（deployment security）保证，例如主机/进程隔离以及适当的 ROS 2 security
配置。三个启动 actuation gate 只控制启动时是否允许打开执行路径；节点启动后，
它们不能保护运行时 ROS 图。

offline 时，`~/stop` 返回成功，因为系统在执行已被抑制的状态下接受了幂等的
本地 fail-safe 停止意图；这不声称零帧已送达 VCU。`~/emergency_stop` 在
offline 时也返回成功，设置节点本地锁存，并通过
`AdapterState.software_estop_latched` 对外可见。`~/authorize` 与
`~/reset_emergency_stop` 会拒绝 offline 请求；active 核心 session 处于
disconnected 时，复位同样失败，而且绝不会清除 offline 锁存。

`DriveCommand.valid_for` 从适配器回调的本地 `CLOCK_MONOTONIC` 接收时刻起算。
消息刻意不包含源时间戳；回调接收之前的传输或调度延迟不属于本接口契约。QoS 只
保留最新命令。`valid_for.sec` 必须大于等于 0，`valid_for.nanosec` 必须小于
`1000000000`；二者表示的总时长必须大于 0，且不得超过
`watchdog.command_timeout_ms`。在节点整个生命周期内，`sequence_id` 必须严格
递增。

命令的线速度单位为 m/s，正值表示前进、负值表示后退；yaw rate 单位为 rad/s，
正值表示向左、负值表示向右。消息没有 `frame_id`；执行前必须针对实际安装车辆
完成符号约定的 commissioning。

`Feedback.receipt_time` 是主机解析帧时采样的 ROS clock 时间，不是控制器源
时间。`composite_stop_flag_raw` 保留经验证的原始协议字节，同时提供其
`control_allowed`/`control_inhibited` 解释。此 wire profile 中，
`vcu_ack_available` 与 `source_time_available` 始终为 `false`。二值
allow/inhibit 字段不得提升解释为 ACK、故障码、回显或序列匹配。

feedback 的纵向/横向速度单位为 m/s，yaw rate 与角速度单位为 rad/s，线加速度
单位为 m/s²，电源电压单位为 V。这些字段按控制器原生通道布局发布，并带主机接收
时间；消息不提供 ROS `frame_id`，也不承诺符合 REP-103 或实际安装后的 IMU
轴对齐。消费者必须先确认安装方向和符号约定，并在用于定位或控制前显式完成
坐标系转换。

`AdapterState` 按顺序发布主机接收时间、session 状态、actuation gate 结果、
连接状态与 `connection_generation`、授权与软件急停状态，以及显式的
`vcu_ack_available=false` 和 `source_time_available=false` 字段。

## 失败即关闭的生命周期

可执行程序显式使用 `rclcpp::executors::SingleThreadedExecutor`，因此所有核心
状态机调用均串行执行。active 连接遵循以下流程：

1. 打开新的 transport generation 后，立即启动有界的精确零帧 episode。
2. `control_allowed=true` 的有效 feedback 必须保持新鲜。
3. `authorize` 必须收到新的非零 token。
4. 只有收到配置数量的、新鲜且序列递增的命令后，motion 才能进入 active。
5. 命令超时、feedback 超时、inhibit feedback、畸形命令、串口故障或软件急停
   都会清除运动意图并请求零帧。
6. 每次重连都会创建新的 generation，执行 startup zero，并要求新的 feedback
   和新的 authorization token。

active-mode 串口 open 之前会先创建全部 publisher、subscription、service 和
timer。如果 open 成功后有任何操作抛出异常，constructor 会捕获异常，执行同一
套有界 final-zero 清理、关闭文件描述符，然后重新抛出。正常关停最多执行
`zero_retry_attempts` 次 final-zero 写入；主机侧一旦完成便立即停止重试。正常
关停时最终零帧失败或结果不确定，程序以状态码 9 退出；捕获标准异常时返回 2，
捕获未知异常时返回 3。进程被强制终止、断电、USB 故障、内核故障或控制器保留
旧目标，都可能阻止该清理完成。

## 必需配置与启动门禁

`config_file` 是必需的节点参数，使用与核心 CLI 相同的严格 INI schema。其中
`limits.max_linear_speed_mps` 必须存在、为有限数、严格大于 0 且严格小于
6.0 m/s；每个 wire 字段还会分别校验是否落在 signed 16-bit 范围内。

launch 默认使用 [config/offline.ini](config/offline.ini)，其中 `device` 为空，
三个 actuation gate 也全部处于禁用值，因此不会打开任何串口路径：

```bash
ros2 launch wheeltec_vcu_serial_ros2 wheeltec_vcu_serial.launch.py
```

任意只启用部分 gate 的组合都会在启动时被拒绝。完成 commissioning 的 launch
必须提供包含直接 `/dev/...` 路径的配置，并精确给出全部三个 gate：

```bash
ros2 launch wheeltec_vcu_serial_ros2 wheeltec_vcu_serial.launch.py \
  config_file:=/absolute/path/to/commissioned.ini \
  acknowledge_unverified_protocol:=true \
  enable_actuation:=true \
  operator_confirmation:=I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

在协议 profile、符号约定、limits、watchdog、物理停止链和测试区域分别完成
commissioning 之前，不得使用上述命令。

## 构建与测试

先安装仓库根目录的核心包，再基于其安装前缀构建 ROS 2 wrapper：

```bash
cmake -S . -B /tmp/wheeltec-core-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DWHEELTEC_BUILD_APPS=OFF \
  -DCMAKE_INSTALL_PREFIX=/tmp/wheeltec-core-install
cmake --build /tmp/wheeltec-core-build --parallel 2
cmake --install /tmp/wheeltec-core-build

source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="/tmp/wheeltec-core-install:${CMAKE_PREFIX_PATH}"
colcon build --base-paths ros2/wheeltec_vcu_serial_ros2
colcon test --base-paths ros2/wheeltec_vcu_serial_ros2
colcon test-result --verbose
```

分支 CI 会运行上述流程，并断言 GTest、contract、`lint_cmake` 与 `xmllint`
测试确实已注册；随后执行 interface 检查和 offline launch smoke test。测试环境
使用 [`ros2/DEPENDENCIES.md`](../DEPENDENCIES.md) 中记录的不可变镜像。CI
绝不会把主机设备映射进容器。测试仅使用纯逻辑输入与静态合同检查，不访问真实
VCU。

## 长期维护边界

公开 ROS 消息与服务属于 middleware 合同。任何破坏字段布局或语义的改动都
必须新增带版本的 interface type，并提供迁移说明。节点可以 remap 到应用 ROS
图，但通用导航命令必须在应用边界进行显式转换。不得把 ROS header、参数读取、
clock、logging、publisher 或 subscription 移入根目录核心库。
