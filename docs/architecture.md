<!-- SPDX-License-Identifier: Apache-2.0 -->
# Architecture and branch policy

The default branch is middleware-independent. Domain commands, protocol
encoding, parsing, transport, configuration, watchdogs, authorization, and the
line-oriented CLI live in `main` and contain no ROS API.

Two maintained integration branches add only graph-edge adapters:

- `ros1/noetic`: ROS 1 Noetic on Ubuntu 20.04;
- `ros2/humble`: ROS 2 Humble on Ubuntu 22.04.

Each integration branch periodically merges from `main`, never the reverse. A
wrapper converts a middleware message into the core `TimedMotionCommand`,
delegates validation and serial behavior to the core library, and publishes
normalized feedback. The core public contract is signed linear speed plus yaw
rate and explicit timing; `/cmd_vel` is not an internal or universal command
contract.

```text
operator or middleware edge
          |
          v
  TimedMotionCommand + authorization
          |
          v
 guarded ROS-free session ----> POSIX serial transport ----> VCU
          ^                              |
          |                              v
 normalized Feedback <----------- bounded stream parser
```

The session owns connection generation, command sequence high-water marks,
authorization epochs, feedback freshness, command freshness, transmit rate,
software emergency-stop state, and bounded zero attempts. The transport owns
only file-descriptor lifecycle and deadline-aware byte I/O. The codec has no
file descriptor, clock, ROS type, logging system, or configuration lookup.
