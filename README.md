<!-- SPDX-License-Identifier: Apache-2.0 -->
# Wheeltec VCU 串口工具库

这是一个独立编写、无需 ROS 的 C++14 库与命令行工具集，用于适配一种
Wheeltec 整车控制器（VCU）串口协议规格。项目将二进制编解码、有界流解析、
POSIX 传输和受保护的运动会话相互分离，使中间件适配层能够保持轻量。

> **实验性硬件接口。** 主机写入成功不代表 VCU 已应答（ACK），也不代表
> 命令被回显、控制器已接受命令或车辆已经停止。现场必须配备经过验证、可实际
> 操作的独立牵引能源切断装置。

默认配置不选择任何串口设备，因此只有操作员明确修改配置并通过三个相互独立的
CLI 安全门控后，软件才可能驱动硬件。

## 长期维护的分支

| 分支 | 平台 | 内容 |
| --- | --- | --- |
| `main` | Ubuntu 20.04，C++14 | 无 ROS 的核心库、CLI、监视器和键盘前端 |
| `ros1/noetic` | Ubuntu 20.04，ROS 1 Noetic | `main` 加[轻量 ROS 1 边界适配层](ros1/wheeltec_vcu_serial_ros1) |
| `ros2/humble` | Ubuntu 22.04，ROS 2 Humble | `main` 加轻量 ROS 2 边界适配层 |

ROS 分支使用与中间件无关的 API。`/cmd_vel` 不是核心接口契约，也不会被视为通用
车辆命令。

## 已实现功能

- 11 字节命令帧编码和精确零运动帧构造；
- 24 字节反馈帧校验和有界滚动解析；
- 带符号的前进/后退速度与横摆角速度字段；
- 对必填参数 `max_linear_speed_mps` 进行显式校验；
- 带截止时间的 POSIX 串口 I/O，处理短写、`EINTR`、`EAGAIN`、部分写入结果、
  断连和重连；
- 启动/重连零帧尝试、发送频率限制、命令与反馈看门狗、重放高水位、以新鲜命令
  恢复、重新授权、每次连接独立的零帧故障锁存，以及本地锁存的软件急停；
- 严格解析 INI 配置的键盘前端，以及带安全防护、采用逐行输入协议的串口 CLI；
- 明确只接收的监视器，以 `O_RDONLY` 打开指定 TTY；
- 合成数据单元测试、注入式系统调用测试和 PTY 进程集成测试。

可互操作的字节布局见 [docs/protocol.md](docs/protocol.md)。选择物理设备前，
请先阅读 [docs/safety.md](docs/safety.md)。

## 在 Ubuntu 20.04 上构建和测试

核心库运行时只依赖 C++ 标准库和 POSIX。目标工具链为 CMake 3.16、GCC 9 和
Python 3.8。

```bash
/usr/bin/cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON
/usr/bin/cmake --build build --parallel 2
(cd build && /usr/bin/ctest --output-on-failure)
```

所有测试都使用合成帧、伪 I/O 或 PTY；测试不需要也不会打开物理控制器。安装到
临时前缀的方法如下：

```bash
/usr/bin/cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
/usr/bin/cmake --build build-release --parallel 2
/usr/bin/cmake --install build-release --prefix "$PWD/install"
```

`ci/run_ci.sh` 会在 [DEPENDENCIES.md](DEPENDENCIES.md) 所述的固定
Ubuntu 20.04 环境中重复执行 Debug、Release、sanitizer、安装、CLI 冒烟、
Python、仓库策略和依赖版本检查。

## 配置遵循失败即关闭原则

复制 [config/wheeltec_vcu_serial.ini](config/wheeltec_vcu_serial.ini)，并且只在
完成车辆调试确认后填写明确、直接的 `/dev/...` 路径。示例速度限制是可编辑的
保守值；库代码中不存在固定的 0.50 m/s 上限。

只有当 `max_linear_speed_mps` 存在、为有限数，并且严格大于 0 且小于
6.0 m/s 时，配置才会被接受。缺失值、零、负数、NaN、无穷大以及大于或等于
6.0 的值，都会在打开驱动设备前被拒绝。编码时还会独立检查每个字段是否超出
线协议（wire）格式的有符号 16 位整数范围。

无需打开串口即可校验配置文件：

```bash
./build/wheeltec_vcu_cli \
  --config config/wheeltec_vcu_serial.ini \
  --validate-config
```

## 键盘控制

前端将机器协议写到标准输出，将所有提示写到标准错误；它自身不会打开串口设备。

| 按键 | 操作 |
| --- | --- |
| `r` | 签发新的本地授权 token；随后需要重新按下方向键 |
| `w` / `s` | 以当前所选速度绝对值前进/后退 |
| `a` / `d` | 按配置曲率向左/向右前进转弯 |
| `q` / `e` | 在配置的上下界内增加/减小当前所选速度绝对值 |
| Space（空格键） | 停止并清除当前按住的方向 |
| `x` | 锁存软件急停；复位有意留给外部授权流程 |
| Esc 或 Ctrl-C | 停止并退出 |

下面完整列出一个驱动流水线示例，以免隐藏任何安全门控：

```bash
python3 python/wheeltec_keyboard_teleop.py \
  --config config/my_vehicle.ini |
./build/wheeltec_vcu_cli \
  --config config/my_vehicle.ini \
  --run \
  --acknowledge-unverified-protocol \
  --enable-actuation \
  --operator-confirmation I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

ANSI 终端无法报告物理按键松开事件。其后端会在 `release_timeout_ms` 时间内
未收到按键重复时，将其视为松键并发送 `STOP`。如需真实的按下/松开事件，请明确
指定一个 Linux 输入事件设备：

```bash
python3 python/wheeltec_keyboard_teleop.py \
  --config config/my_vehicle.ini \
  --event-device /dev/input/eventN
```

按键松开、输入超时、已处理的异常、正常退出和捕获的终止信号都会请求发送零帧。
任何用户态进程都无法保证在 `SIGKILL`、断电、内核故障、USB 断开，或控制器继续
保持先前目标值时完成退出清理。

## 只接收监视

监视器没有传输写入调用，并以 `O_RDONLY` 请求打开设备。即便如此，打开和配置
USB TTY 仍可能改变线路状态或使某些设备复位，因此操作员必须确认理解这一影响：

```bash
./build/wheeltec_vcu_monitor \
  --read-only \
  --device /dev/ttyACM0 \
  --frames 10 \
  --timeout-ms 3000 \
  --operator-confirmation I_UNDERSTAND_OPENING_A_TTY_CAN_AFFECT_THE_DEVICE
```

它只输出解码后的汇总反馈。不得将其中的 inhibit 字段解释为 ACK、故障码、源端
时间戳或命令序列号。

## 库 API

公共头文件位于 `include/wheeltec_vcu_serial`。应用通常按以下顺序组合组件：

1. 使用 `loadRuntimeConfig` 加载精确、严格的配置；
2. 使用 `FeedbackParser` 有界解析接收帧；
3. 使用带显式访问模式的 `PosixSerialTransport`；
4. 使用 `SafetySession` 管理单线程授权、看门狗、零帧和重连状态；
5. 仅在协议边界调用 `encodeCommand`。

`SafetySession` 有意设计为单线程组件；所有调用必须在同一个 I/O 循环中串行执行。
其 `host_write_complete` 结果只描述本地操作系统的状态。API 绝不会将该结果表述为
控制器已收到命令或已给出 ACK。

## 项目边界与来源

仓库不包含厂家固件、厂家源码、中间件库、硬件抓取数据或第三方运行时代码；测试帧
均为合成数据。发布边界详见 [PROVENANCE.md](PROVENANCE.md)、
[DEPENDENCIES.md](DEPENDENCIES.md) 和 [NOTICE](NOTICE)。这是独立、非官方的
互操作实现，与硬件厂家没有从属、赞助或背书关系。

贡献代码必须保持 [CONTRIBUTING.md](CONTRIBUTING.md) 中记录的安全要求和依赖
方向。项目采用 Apache-2.0 许可证，详见 [LICENSE](LICENSE)。
