<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 dependency register

The branch CI runs in the official multi-platform image
`ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`.
The digest pins the image index; GitHub-hosted amd64 runners select its amd64
manifest. CI uses an Ubuntu 24.04 hosted runner only as the Docker host, not as
the target build environment.

Every apt input is exact-version pinned in
[`ci/apt-packages.txt`](ci/apt-packages.txt). The ROS-facing dependencies are:

| Package | Pinned Focal/Noetic version | Role |
| --- | --- | --- |
| `ros-noetic-catkin` | `0.8.12-1focal.20250426.001935` | Build system and gtest integration |
| `ros-noetic-message-generation` | `0.4.1-1focal.20250426.010337` | Build-time message/service generation |
| `ros-noetic-message-runtime` | `0.4.13-1focal.20250426.011132` | Generated message/service runtime |
| `ros-noetic-roscpp` | `1.17.4-1focal.20250519.225343` | ROS 1 C++ graph boundary |
| `ros-noetic-rospy` | `1.17.4-1focal.20250519.231646` | Offline-node contract test client |
| `ros-noetic-rostest` | `1.17.4-1focal.20250519.233130` | Isolated offline-node launch test |
| `ros-noetic-std-srvs` | `1.11.4-1focal.20250426.011617` | Stop and E-stop trigger services |

The ROS-free core is built from the same branch revision, installed to a
temporary prefix, and discovered with its exported CMake config. It is not
copied into the catkin package. `actions/checkout` is pinned at
`11bd71901bbe5b1630ceea73d27597364c9af683` with credential persistence
disabled. No dependency source or binary is redistributed by this repository.
