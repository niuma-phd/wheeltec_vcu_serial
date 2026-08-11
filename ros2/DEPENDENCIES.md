<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 2 Humble dependency lock

The `ros2/humble` build and test environment is the immutable OCI image:

```text
ros:humble-ros-base-jammy@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4
```

The image supplies Ubuntu 22.04, ROS 2 Humble, CMake, GCC, colcon, `rclcpp`,
`builtin_interfaces`, `std_srvs`, ROSIDL generators/runtime, launch support,
and ament test tools. CI performs no package-manager install and uses no
unreviewed source checkout.

For auditability, the selected image currently resolves the principal packages
as follows. The image digest, rather than these descriptive rows, is the
enforced lock.

| Package | Version in locked image |
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

The core dependency is built from the same repository revision and installed
to a temporary CMake prefix before colcon configures the adapter. This preserves
the dependency direction: ROS 2 depends on the public core package; the core
never discovers or links ROS.

GitHub Actions checkout is pinned to commit
`11bd71901bbe5b1630ceea73d27597364c9af683` (upstream release v4.2.2), with
credential persistence disabled. Review image and action revisions explicitly
before changing either pin.
