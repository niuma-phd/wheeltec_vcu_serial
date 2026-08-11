<!-- SPDX-License-Identifier: Apache-2.0 -->
# 依赖登记

运行时库仅依赖 C++14 标准库和 Linux/POSIX API。键盘程序仅使用 Python 3.8
标准库。ROS 不是 `main` 分支的依赖。

目标构建环境：

| 组件 | 固定的 Ubuntu 20.04 软件包/版本 | 用途 |
| --- | --- | --- |
| `build-essential` | `12.8ubuntu1.1` | 编译器和基础构建工具 |
| `gcc`, `g++` | `4:9.3.0-1ubuntu2` | GCC 9 工具链选择器 |
| `gcc-9`, `g++-9` | `9.4.0-1ubuntu1~20.04.2` | C++14 编译器 |
| `cmake` | `3.16.3-1ubuntu1.20.04.1` | 构建系统 |
| `make` | `4.2.1-1.2` | 构建执行工具 |
| `git` | `1:2.25.1-1ubuntu3.14` | CI 中的仓库校验 |
| `libc6-dev` | `2.31-0ubuntu9.18` | POSIX/Linux 开发头文件 |
| `python3` | `3.8.2-0ubuntu2` | Python 解释器选择器 |
| `python3.8` | `3.8.10-0ubuntu1~20.04.18` | 键盘程序和测试 |

CI 构建容器为
`ubuntu:20.04@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214`。
该摘要标识多平台镜像清单；CI 使用其中的 amd64 变体。编译前会通过
`dpkg-query` 检查软件包版本。

GitHub Actions：

| Action | 不可变修订版本 | 用途 |
| --- | --- | --- |
| `actions/checkout` | `11bd71901bbe5b1630ceea73d27597364c9af683` | 检出源码；不持久保存凭据 |

仓库不分发此表中的任何 Action 或软件包；它们各自沿用对应项目和 Ubuntu 提供的
许可证。
