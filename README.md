<!-- SPDX-License-Identifier: Apache-2.0 -->
# Wheeltec VCU serial

An independently written, ROS-free C++14 library and command-line toolkit for
one Wheeltec vehicle-controller serial profile. The project separates binary
encoding, bounded stream parsing, POSIX transport, and a guarded motion session
so middleware adapters can stay thin.

> **Experimental hardware interface.** A successful host write is not a VCU
> acknowledgement, command echo, proof of controller acceptance, or evidence
> that a vehicle stopped. Keep a tested physical traction-energy stop available.

The default configuration selects no serial device and therefore cannot actuate
hardware without an explicit operator edit and three independent CLI gates.

## Maintained branches

| Branch | Platform | Contents |
| --- | --- | --- |
| `main` | Ubuntu 20.04, C++14 | ROS-free core, CLI, monitor, keyboard frontend |
| `ros1/noetic` | Ubuntu 20.04, ROS 1 Noetic | `main` plus a thin ROS 1 boundary |
| `ros2/humble` | Ubuntu 22.04, ROS 2 Humble | `main` plus a thin ROS 2 boundary |

The ROS branches consume the middleware-independent API. `/cmd_vel` is not a
core contract and is not treated as a universal vehicle command.

## What is implemented

- 11-byte command encoding and exact zero-frame construction;
- 24-byte feedback validation and bounded rolling parsing;
- signed forward/reverse speed and yaw-rate fields;
- explicit, mandatory `max_linear_speed_mps` validation;
- deadline-aware POSIX serial I/O with short-write, `EINTR`, `EAGAIN`, partial
  outcome, disconnect, and reconnect handling;
- startup/reconnect zero attempts, transmit rate limiting, command and feedback
  watchdogs, replay high-water marks, fresh-command recovery, reauthorization,
  a per-connection zero-fault latch, and a locally latched software emergency
  stop;
- a strict-INI keyboard frontend and guarded line-oriented serial CLI;
- an explicitly receive-only monitor that opens a selected TTY with `O_RDONLY`;
- synthetic unit tests, injected-syscall tests, and PTY process integration.

The interoperable byte layout is documented in [docs/protocol.md](docs/protocol.md).
Read [docs/safety.md](docs/safety.md) before selecting a physical device.

## Build and test on Ubuntu 20.04

The core has no runtime dependency beyond the C++ standard library and POSIX.
CMake 3.16, GCC 9, and Python 3.8 are the target toolchain.

```bash
/usr/bin/cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWHEELTEC_WARNINGS_AS_ERRORS=ON
/usr/bin/cmake --build build --parallel 2
(cd build && /usr/bin/ctest --output-on-failure)
```

All tests use synthetic frames, fake I/O, or PTYs. They do not require or open
a physical controller. To install into a staging prefix:

```bash
/usr/bin/cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
/usr/bin/cmake --build build-release --parallel 2
/usr/bin/cmake --install build-release --prefix "$PWD/install"
```

`ci/run_ci.sh` repeats Debug, Release, sanitizer, install, CLI-smoke, Python,
repository-policy, and dependency-version checks in the pinned Ubuntu 20.04
environment described by [DEPENDENCIES.md](DEPENDENCIES.md).

## Configuration is fail-closed

Copy [config/wheeltec_vcu_serial.ini](config/wheeltec_vcu_serial.ini) and set an
explicit direct `/dev/...` path only after commissioning. The example limit is
an editable conservative value; the library does not contain a fixed 0.50 m/s
ceiling.

`max_linear_speed_mps` is accepted only when it is present, finite, and
strictly greater than 0 and less than 6.0 m/s. Missing values, zero, negative
values, NaN, infinity, and values at least 6.0 are rejected before an actuation
device is opened. Every encoded field is also checked independently against its
signed 16-bit wire range.

Validate a file without opening a serial device:

```bash
./build/wheeltec_vcu_cli \
  --config config/wheeltec_vcu_serial.ini \
  --validate-config
```

## Keyboard control

The frontend writes a machine protocol to stdout and all prompts to stderr. It
does not open the serial device itself.

| Key | Action |
| --- | --- |
| `r` | issue a new local authorization token; re-press direction afterward |
| `w` / `s` | forward / reverse at the selected speed magnitude |
| `a` / `d` | forward left / right arc at configured curvature |
| `q` / `e` | increase / decrease selected speed magnitude within configured bounds |
| Space | stop and clear held direction |
| `x` | latch software emergency stop; reset is deliberately external |
| Esc or Ctrl-C | stop and exit |

Example actuation pipeline, shown deliberately in full so no gate is hidden:

```bash
python3 python/wheeltec_keyboard_teleop.py \
  --config config/my_vehicle.ini |
./build/wheeltec_vcu_cli \
  --config config/my_vehicle.ini \
  --run \
  --acknowledge-unverified-protocol \
  --enable-actuation \
  --operator-confirmation I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK
```

An ANSI terminal cannot report a physical key release. Its backend treats a
gap in key repeat as release after `release_timeout_ms` and sends `STOP`. For
real press/release events, supply an explicitly selected Linux event device:

```bash
python3 python/wheeltec_keyboard_teleop.py \
  --config config/my_vehicle.ini \
  --event-device /dev/input/eventN
```

Release, input timeout, handled exception, normal exit, and caught termination
signals request zero. No userspace process can guarantee cleanup after
`SIGKILL`, power loss, kernel failure, USB loss, or a controller that retains a
previous target.

## Receive-only observation

The monitor has no transport write call and requests `O_RDONLY`. Opening and
configuring a USB TTY may still change line state or reset some devices, so the
operator must acknowledge that effect:

```bash
./build/wheeltec_vcu_monitor \
  --read-only \
  --device /dev/ttyACM0 \
  --frames 10 \
  --timeout-ms 3000 \
  --operator-confirmation I_UNDERSTAND_OPENING_A_TTY_CAN_AFFECT_THE_DEVICE
```

It prints aggregate decoded feedback only. Do not interpret its inhibit field
as an ACK, fault code, source timestamp, or command sequence.

## Library API

Public headers are under `include/wheeltec_vcu_serial`. Applications normally
compose:

1. `loadRuntimeConfig` for exact, strict configuration;
2. `FeedbackParser` for bounded receive framing;
3. `PosixSerialTransport` with an explicit access mode;
4. `SafetySession` for single-threaded authorization, watchdog, zero, and
   reconnect state;
5. `encodeCommand` only at the protocol boundary.

`SafetySession` is intentionally single-threaded; serialize all calls in one
I/O loop. Its `host_write_complete` result describes only the local operating
system. The API never exposes that result as controller delivery or ACK.

## Project boundaries and provenance

No vendor firmware, vendor source, middleware library, hardware capture, or
third-party runtime code is bundled. Test frames are synthetic. See
[PROVENANCE.md](PROVENANCE.md), [DEPENDENCIES.md](DEPENDENCIES.md), and
[NOTICE](NOTICE) for the distribution boundary. This is an independent,
unofficial interoperability implementation and is not affiliated with or
endorsed by the hardware manufacturer.

Contributions must preserve the safety and dependency direction documented in
[CONTRIBUTING.md](CONTRIBUTING.md). The project is licensed under Apache-2.0;
see [LICENSE](LICENSE).
