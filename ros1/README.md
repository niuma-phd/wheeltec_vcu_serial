<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 Noetic 分支

本分支在保留仓库根目录 ROS-free 核心的基础上，增加一个可长期维护的薄适配层：

- [`wheeltec_vcu_serial_ros1`](wheeltec_vcu_serial_ros1)——提供基于接收时刻的
  自定义命令、显式授权与急停服务、解码后的反馈，以及对受保护核心
  transport/session 的直接组合。默认 launch 会在未选择设备的 offline 模式下
  持续运行；启用执行必须同时满足三道显式门禁，并使用已完成车辆标定的配置。

目标平台为 Ubuntu 20.04、ROS 1 Noetic、C++14。本包不以 `/cmd_vel` 作为
命令接口契约，也不包含任何厂家源码或真实硬件采集数据。启动门禁、单位、时钟、
失败语义和本机构建方法见包内 README；精确构建输入见
[`DEPENDENCIES.md`](DEPENDENCIES.md)。
