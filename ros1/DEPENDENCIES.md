<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 依赖登记

本分支 CI 使用官方多平台镜像
`ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`。
该 digest 固定镜像索引；GitHub 托管的 amd64 runner 会选择其中的 amd64
manifest。CI 中的 Ubuntu 24.04 托管 runner 只作为 Docker 宿主机，并非目标
构建环境。

所有 apt 输入均在 [`ci/apt-packages.txt`](ci/apt-packages.txt) 中固定到精确
版本。ROS 边界所需依赖如下：

| 软件包 | 固定的 Focal/Noetic 版本 | 用途 |
| --- | --- | --- |
| `ros-noetic-catkin` | `0.8.12-1focal.20250426.001935` | 构建系统与 gtest 集成 |
| `ros-noetic-message-generation` | `0.4.1-1focal.20250426.010337` | 构建期消息/服务代码生成 |
| `ros-noetic-message-runtime` | `0.4.13-1focal.20250426.011132` | 生成消息/服务的运行时 |
| `ros-noetic-roscpp` | `1.17.4-1focal.20250519.225343` | ROS 1 C++ graph 边界 |
| `ros-noetic-rospy` | `1.17.4-1focal.20250519.231646` | offline 节点合同测试客户端 |
| `ros-noetic-rostest` | `1.17.4-1focal.20250519.233130` | 隔离的 offline 节点 launch 测试 |
| `ros-noetic-std-srvs` | `1.11.4-1focal.20250426.011617` | 停车与急停触发服务 |

ROS-free 核心从同一分支 revision 构建，安装到临时前缀，并通过其导出的 CMake
config 被发现；核心源码不会复制进 catkin 包。`actions/checkout` 固定为
`11bd71901bbe5b1630ceea73d27597364c9af683`，并关闭凭据持久化。本仓库不
分发任何依赖的源码或二进制；各依赖继续适用其自身许可证。
