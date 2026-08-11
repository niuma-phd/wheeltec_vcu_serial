<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 Noetic branch

This branch adds one maintained thin adapter while preserving the ROS-free
core at the repository root:

- [`wheeltec_vcu_serial_ros1`](wheeltec_vcu_serial_ros1) — custom receipt-time
  commands, explicit authorization and E-stop services, decoded feedback, and
  direct composition of the guarded core transport/session. Its default launch
  stays alive in no-device offline mode; actuation requires all three explicit
  gates and a commissioned configuration.

Target: Ubuntu 20.04, ROS 1 Noetic, C++14. The package does not use `/cmd_vel`
as a command contract and does not include any vendor source or hardware
capture. See the package README for startup gates, units, timing, failure
semantics, and local build instructions. Exact build inputs are recorded in
[`DEPENDENCIES.md`](DEPENDENCIES.md).
