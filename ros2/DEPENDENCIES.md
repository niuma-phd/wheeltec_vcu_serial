<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 2 Humble 依赖锁定

`ros2/humble` 分支使用以下不可变 OCI 镜像作为构建与测试环境：

```text
ros:humble-ros-base-jammy@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4
```

该镜像提供 Ubuntu 22.04、ROS 2 Humble、CMake、GCC、colcon、`rclcpp`、
`builtin_interfaces`、`std_srvs`、ROSIDL 生成器与运行时、launch 支持以及
ament 测试工具。CI 不执行任何包管理器安装，也不检出未经审查的外部源码。

为便于审计，所选镜像当前解析出的主要软件包版本如下。真正强制生效的锁定项
是上面的镜像摘要；下表仅用于说明镜像内容。

| 软件包 | 锁定镜像中的版本 |
| --- | --- |
| `cmake` | `3.22.1-1ubuntu1.22.04.2` |
| `gcc` / `g++` | `4:11.2.0-1ubuntu1` |
| `python3-colcon-core` | `0.21.0+upstream-1` |
| `ros-humble-rclcpp` | `16.0.19-1jammy.20260605.135013` |
| `ros-humble-builtin-interfaces` | `1.2.3-1jammy.20260605.111300` |
| `ros-humble-std-srvs` | `4.9.1-1jammy.20260605.120647` |
| `ros-humble-rosidl-default-generators` | `1.2.1-1jammy.20260605.110632` |
| `ros-humble-rosidl-default-runtime` | `1.2.1-1jammy.20260605.110631` |
| `ros-humble-launch` | `1.0.14-1jammy.20260304.195519` |
| `ros-humble-launch-ros` | `0.19.13-1jammy.20260605.140440` |
| `ros-humble-ament-cmake-lint-cmake` | `0.12.15-1jammy.20260304.195414` |
| `ros-humble-ament-cmake-xmllint` | `0.12.15-1jammy.20260304.200103` |

核心依赖从同一仓库修订版本构建，并在 colcon 配置适配器之前安装到临时 CMake
前缀。由此保持单向依赖：ROS 2 包依赖公开的核心 CMake 包，而核心永远不会发现
或链接 ROS。

GitHub Actions 的 checkout 固定到提交
`11bd71901bbe5b1630ceea73d27597364c9af683`（上游 v4.2.2），并禁用凭据持久化。
更改镜像摘要或 action 修订版本之前，必须分别进行显式审查。
