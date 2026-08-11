<!-- SPDX-License-Identifier: Apache-2.0 -->
# ROS 1 Noetic adapter

This catkin package is a thin ROS 1 boundary around the installed,
middleware-independent `wheeltec_vcu_serial` C++14 library. It targets ROS 1
Noetic on Ubuntu 20.04. The node directly composes the core
`PosixSerialTransport`, `FeedbackParser`, and `SafetySession`; protocol and
safety behavior remain in the ROS-free library.

The `adapter_support` target and header are package-internal implementation and
test seams. They are linked statically into the node and are not installed or
exported. The maintained public ROS contracts are the generated messages,
services, node executable, parameters, topics, and services documented below.

> Host write completion is an operating-system result. It is not a VCU ACK,
> command echo, proof of controller acceptance, or proof that the vehicle
> stopped. The physical emergency-stop chain remains independent.

## Build

Install the core to a prefix, then expose that prefix while building a catkin
workspace:

```bash
/usr/bin/cmake -S /path/to/wheeltec_vcu_serial \
  -B /tmp/wheeltec-core-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DWHEELTEC_BUILD_APPS=OFF
/usr/bin/cmake --build /tmp/wheeltec-core-build --parallel 2
/usr/bin/cmake --install /tmp/wheeltec-core-build \
  --prefix /tmp/wheeltec-core-install

source /opt/ros/noetic/setup.bash
mkdir -p /tmp/wheeltec-ros1-ws/src
ln -s /path/to/wheeltec_vcu_serial/ros1/wheeltec_vcu_serial_ros1 \
  /tmp/wheeltec-ros1-ws/src/wheeltec_vcu_serial_ros1
(cd /tmp/wheeltec-ros1-ws && catkin_make \
  -DCMAKE_PREFIX_PATH="/tmp/wheeltec-core-install;/opt/ros/noetic")
```

The exact locally verified command is automated by
[`../ci/run_ros1_ci.sh`](../ci/run_ros1_ci.sh). Tests exercise conversions,
gates, message contracts, and a live node using the empty-device offline
configuration. No test enters the actuation path or opens a serial device.

## Fail-closed startup

The required private parameter `~config_file` must name a strict core INI
file. The configured `max_linear_speed_mps` must be finite and satisfy
`0 < max_linear_speed_mps < 6.0`; the core independently validates each
signed 16-bit wire field.

The three gate parameters have exactly two accepted combinations:

- `false`, `false`, and an empty confirmation starts persistent **offline**
  mode. The node publishes state and serves its ROS API, but never calls serial
  `open()` or reconnect and reports `actuation_enabled=false`.
- `true`, `true`, and the exact phrase below permits actuation. Only this mode
  additionally requires a nonempty direct `/dev/...` path and can open a TTY.

Any partial Boolean combination, nonempty confirmation while disabled, or
incorrect confirmation fails closed before construction of a running adapter.
The actuation combination is:

```yaml
acknowledge_unverified_protocol: true
enable_actuation: true
operator_confirmation: I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

The supplied [`adapter.launch`](launch/adapter.launch) deliberately selects an
INI file with an empty device, sets both Boolean gates to false, and leaves the
confirmation empty. Running it unchanged keeps the node alive offline and
cannot open hardware.

After commissioning a vehicle-specific INI, every gate must be visible at the
launch site, for example:

```bash
roslaunch wheeltec_vcu_serial_ros1 adapter.launch \
  config_file:=/absolute/path/to/commissioned.ini \
  acknowledge_unverified_protocol:=true \
  enable_actuation:=true \
  operator_confirmation:=I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

## ROS contracts

All names are private to the node by default.

### `~drive_command` (`wheeltec_vcu_serial_ros1/DriveCommand`)

- `sequence_id` is nonzero and must strictly increase for the lifetime of the
  process, including across reconnects.
- `linear_speed_mps` uses metres per second: positive forward, negative
  reverse.
- `yaw_rate_radps` uses radians per second: positive left, negative right.
- `valid_for` must be positive and no greater than the configured
  `command_timeout_ms`.
- Validity begins at callback receipt. The wrapper samples `CLOCK_MONOTONIC`
  and constructs both core timestamps; sender time and ROS time are not
  trusted for watchdog decisions.
- Any malformed/nonfinite wrapper-level command disarms locally. Core range,
  authorization, replay, feedback-freshness, and recovery checks still apply.

The package intentionally does not subscribe to `/cmd_vel`. A generic twist
cannot carry sequence, receipt-relative validity, authorization, or the
vehicle-specific failure semantics required by this boundary. Applications
must explicitly convert their checked vehicle command into `DriveCommand`.

This command contract uses metres per second with positive forward and
negative reverse, and radians per second with positive yaw to the left and
negative yaw to the right. It has no `frame_id`; the signs must be commissioned
against the installed vehicle before actuation.

### `~feedback` (`wheeltec_vcu_serial_ros1/Feedback`)

`receipt_time` is sampled from the ROS clock after a validated frame is read;
the core watchdog independently uses monotonic receipt time. Velocity is in
m/s and rad/s, acceleration in m/s², angular velocity in rad/s, and voltage in
volts. `control_allowed` and `control_inhibited` expose only the decoded binary
composite inhibit field. They are not a fault diagnosis or acknowledgement.

Feedback longitudinal/lateral velocity is in m/s, yaw rate and angular
velocity are in rad/s, linear acceleration is in m/s², and supply voltage is in
volts. These are controller-native channel positions with host receipt time;
the message supplies no ROS `frame_id` and does not assert REP-103 or installed
IMU-axis alignment. A consumer must commission the mounting/sign convention
and perform an explicit frame conversion before localization or control use.

`vcu_ack_available` and `source_time_available` are always false because this
wire profile carries neither. A feedback frame is never correlated with a
command sequence.

### `~adapter_state` (`wheeltec_vcu_serial_ros1/AdapterState`)

The latched state topic contains `receipt_time`, `session_state`,
`actuation_enabled`, connection status/generation, local authorization,
`software_estop_latched`, and the two metadata-availability flags. Offline mode
reports `session_state="offline"`, `actuation_enabled=false`, disconnected,
generation zero, and unauthorized. It repeats that VCU ACK and source time are
unavailable. In actuation mode, reconnection increments the generation and
revokes authorization.

### Services

| Service | Type | Semantics |
| --- | --- | --- |
| `~authorize` | `Authorize` | Accepts only a new, nonzero, increasing token after initial zero and fresh allowed feedback. |
| `~stop` | `std_srvs/Trigger` | Clears local motion and starts a bounded zero episode when possible. |
| `~emergency_stop` | `std_srvs/Trigger` | Latches the independent software E-stop path; it never auto-recovers. |
| `~reset_emergency_stop` | `ResetEstop` | Requires a new token, current generation, completed zero, and fresh allowed feedback; motion still needs reauthorization. |

A service response reports only local state-machine acceptance, never VCU
acceptance. Offline mode rejects authorization and motion, treats stop as an
already-local no-I/O state, and never attempts serial output.

The increasing authorization and emergency-stop reset tokens are replay
guards, not passwords, credentials, capabilities, or proof of caller identity.
ROS 1 provides no identity check for these services here. Deploy the node only
on a trusted graph and restrict host, network, and ROS master access; a trusted
supervisor or equivalent deployment security must decide who may authorize or
reset. The three startup gates do not protect runtime graph calls.

## Runtime safety behavior

The node uses one thread and `ros::spinOnce()` so callbacks, parser input, and
the core session are serialized. Offline mode never enters any open, read,
write, or reconnect path. Actuation mode attempts initial zero after every
successful open. Link loss clears motion intent; reconnect resets the parser,
creates a new connection generation, attempts initial zero, and requires fresh
allowed feedback plus a new higher authorization token. The configured command
and feedback watchdogs request zero when stale.

Motion write failure revokes authorization and triggers the core zero path.
Shutdown clears local intent and makes a separately bounded set of exact-zero
host-write attempts. A failed or completed attempt still cannot establish VCU
delivery. Power loss, `SIGKILL`, kernel failure, cable loss, and controller
retention of an old target cannot be made safe by this userspace node.
