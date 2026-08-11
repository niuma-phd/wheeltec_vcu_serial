<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 2 Humble adapter

`wheeltec_vcu_serial_ros2` is the thin ROS 2 boundary for the installed,
ROS-free `wheeltec_vcu_serial` CMake package. It targets ROS 2 Humble on Ubuntu
22.04 and uses C++14. Serial framing, deadlines, reconnect generations,
watchdogs, authorization, zero episodes, and the software emergency-stop latch
remain in the core library.

This package does not subscribe to `/cmd_vel`, does not reinterpret it, and
does not make it an internal contract. Its input is the vehicle-specific,
receipt-relative `DriveCommand` message.

> A completed host serial write is not a VCU acknowledgement, controller
> acceptance, command echo, or proof of physical motion or stop. The physical
> emergency-stop and traction-energy isolation remain independent safety
> mechanisms.

## Graph contract and trust boundary

All names are node-private. With the default node name they expand under
`/wheeltec_vcu_serial_adapter`; launch or application composition may remap
each endpoint explicitly.

| Name | Type | Direction | Meaning |
| --- | --- | --- | --- |
| `~/drive_command` | `wheeltec_vcu_serial_ros2/msg/DriveCommand` | subscribe | guarded longitudinal speed and yaw-rate request |
| `~/feedback` | `wheeltec_vcu_serial_ros2/msg/Feedback` | publish | validated 24-byte VCU feedback with host receipt time |
| `~/adapter_state` | `wheeltec_vcu_serial_ros2/msg/AdapterState` | publish | transient-local connection and safety state |
| `~/authorize` | `wheeltec_vcu_serial_ros2/srv/Authorize` | service | authorize one connection generation with a strictly increasing token |
| `~/stop` | `std_srvs/srv/Trigger` | service | accept an idempotent local stop intent; begin a bounded zero episode when active |
| `~/emergency_stop` | `std_srvs/srv/Trigger` | service | latch the independent local software emergency-stop path, including offline |
| `~/reset_emergency_stop` | `wheeltec_vcu_serial_ros2/srv/ResetEstop` | service | reset the latch with a fresh token after conditions clear |

The authorization/reset token is only a monotonically increasing anti-replay
value inside this process; it is **not a credential**, identity proof, secret,
or caller authentication mechanism. The ROS graph carrying commands and
services must be trusted and isolated. Caller identity and permission for
`~/authorize` and `~/reset_emergency_stop` must be enforced by deployment
security (for example host/process isolation and an appropriate ROS 2 security
configuration). The three startup actuation gates do not protect the runtime
graph after the node starts.

When offline, `~/stop` returns success because the idempotent local fail-safe
intent is accepted while actuation is already inhibited; this does not claim a
zero was delivered to the VCU. `~/emergency_stop` also returns success offline,
sets a node-local latch, and exposes it through
`AdapterState.software_estop_latched`. `~/authorize` and
`~/reset_emergency_stop` reject offline requests. Reset also fails while the
active core session is disconnected; it never clears the offline latch.

`DriveCommand.valid_for` is measured from the adapter callback's local
`CLOCK_MONOTONIC` receipt. The message intentionally has no source timestamp:
transport or scheduling age before callback receipt is outside this contract.
The QoS keeps only the newest command. `valid_for` must be positive, canonical,
and no greater than `watchdog.command_timeout_ms`. `sequence_id` must strictly
increase for the lifetime of the node.

The command uses metres per second with positive forward and negative reverse,
and radians per second with positive yaw to the left and negative yaw to the
right. It has no `frame_id`; the signs must be commissioned against the
installed vehicle before actuation.

`Feedback.receipt_time` is the ROS clock time at host parsing, not a controller
source time. `composite_stop_flag_raw` preserves the validated raw protocol
byte alongside its `control_allowed`/`control_inhibited` interpretation.
`vcu_ack_available` and `source_time_available` are always `false` for this
wire profile. The binary allow/inhibit field is not promoted into an ACK,
fault code, echo, or sequence match.

Feedback longitudinal/lateral velocity is in m/s, yaw rate and angular
velocity are in rad/s, linear acceleration is in m/s², and supply voltage is in
volts. These are controller-native channel positions with host receipt time;
the message supplies no ROS `frame_id` and does not assert REP-103 or installed
IMU-axis alignment. A consumer must commission the mounting/sign convention
and perform an explicit frame conversion before localization or control use.

`AdapterState` publishes, in order, the host receipt time, session state,
actuation-gate result, connection state and `connection_generation`,
authorization and software E-stop state, plus explicit
`vcu_ack_available=false` and `source_time_available=false` fields.

## Fail-closed lifecycle

The executable explicitly uses `rclcpp::executors::SingleThreadedExecutor`, so
every core state-machine call is serialized. An active connection proceeds as
follows:

1. Opening a new transport generation starts a bounded exact-zero episode.
2. Valid feedback with `control_allowed=true` must be fresh.
3. `authorize` must receive a new nonzero token.
4. The configured number of fresh, increasing commands must be received before
   motion becomes active.
5. Command timeout, feedback timeout, inhibit feedback, malformed command,
   serial failure, or software emergency stop clears motion intent and requests
   zero.
6. Every reconnect creates a new generation, performs startup zero, and
   requires fresh feedback and a new authorization token.

All publishers, subscriptions, services, and timers are created before an
active-mode serial open. If anything throws after a successful open, the
constructor catches it, attempts the same bounded final-zero cleanup, closes
the descriptor, and rethrows. Orderly shutdown makes at most
`zero_retry_attempts` final-zero writes, stopping immediately after host-side
completion. Failure or uncertainty makes the executable exit with status 9;
an exception also exits nonzero. Process kill, power loss, USB failure, kernel
failure, or controller retention can prevent that cleanup.

## Required configuration and gates

`config_file` is a required node parameter. It uses the same strict INI schema
as the core CLI. In particular, `limits.max_linear_speed_mps` must be present,
finite, greater than 0, and less than 6.0 m/s; every wire field is checked
independently against its signed 16-bit range.

The launch default uses [config/offline.ini](config/offline.ini), whose device
is empty. It sets all three actuation gates to their disabled values, so no
serial path is opened:

```bash
ros2 launch wheeltec_vcu_serial_ros2 wheeltec_vcu_serial.launch.py
```

Any partial gate combination is rejected at startup. A commissioned launch
must provide a configuration containing a direct `/dev/...` path and all three
exact gates:

```bash
ros2 launch wheeltec_vcu_serial_ros2 wheeltec_vcu_serial.launch.py \
  config_file:=/absolute/path/to/commissioned.ini \
  acknowledge_unverified_protocol:=true \
  enable_actuation:=true \
  operator_confirmation:=I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

Do not use that command until the protocol profile, sign conventions, limits,
watchdogs, physical stop chain, and test area have been independently
commissioned.

## Build and test

Install the root core first, then build the wrapper against its prefix:

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

The branch CI runs this sequence, asserts that its GTest, contract,
`lint_cmake`, and `xmllint` tests were actually registered, performs interface
inspection, and runs an offline launch smoke test in the immutable image recorded in
[`ros2/DEPENDENCIES.md`](../DEPENDENCIES.md). It never maps a host device into
the container. Tests use logic-only inputs and static contract checks; they do
not access a physical VCU.

## Maintenance boundary

Public ROS messages and services are middleware contracts. Breaking field or
semantic changes require new versioned interface types and migration notes.
The node may be remapped into an application graph, but generic navigation
commands must be converted explicitly at that application's edge. Do not move
ROS headers, parameter lookup, clocks, logging, publishers, or subscriptions
into the root core.
