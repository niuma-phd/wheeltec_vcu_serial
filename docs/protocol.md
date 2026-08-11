<!-- SPDX-License-Identifier: Apache-2.0 -->
# Serial protocol contract

Status: interoperability profile; physical behavior remains hardware- and
firmware-dependent.

The serial line uses 115200 bit/s, eight data bits, no parity, one stop bit, and
no flow control. Multi-byte integers are big-endian two's-complement values
unless stated otherwise.

## Command frame

A normal command occupies 11 bytes:

| Byte range | Value or interpretation |
| --- | --- |
| `0` | frame marker `0x7B` |
| `1` | mode `0` for normal motion |
| `2` | unused in this profile; sent as zero |
| `3..4` | signed longitudinal speed in `0.001 m/s` |
| `5..6` | signed lateral speed in `0.001 m/s` |
| `7..8` | signed yaw rate in `0.001 rad/s` |
| `9` | XOR reduction of bytes `0..8` |
| `10` | frame marker `0x7D` |

The codec checks every floating-point input for finiteness, multiplies by 1000,
truncates toward zero, and verifies the resulting integer against the full
`int16_t` range before conversion. Longitudinal speed is separately constrained
by the caller's explicit `max_linear_speed_mps`; the magnitude must not exceed
that value. The limit itself is valid only when finite and strictly between 0
and 6 m/s.

`makeZeroCommandFrame()` constructs the all-zero motion frame independently of
motion configuration so shutdown paths remain available when configuration is
invalid. A host write of that frame is not proof that the controller accepted
it or that a vehicle stopped.

## Primary feedback frame

The primary feedback record occupies 24 bytes:

| Byte range | Value or interpretation |
| --- | --- |
| `0` | frame marker `0x7B` |
| `1` | composite inhibit: `0` currently allows control, `1` currently inhibits it |
| `2..3` | signed longitudinal speed divided by 1000 |
| `4..5` | signed lateral speed divided by 1000 |
| `6..7` | signed yaw rate divided by 1000 |
| `8..13` | three signed acceleration channels, each divided by `1671.84` |
| `14..19` | three signed gyro channels, each multiplied by `0.00026644` |
| `20..21` | unsigned supply-voltage value divided by 1000 |
| `22` | XOR reduction of bytes `0..21` |
| `23` | frame marker `0x7D` |

Values other than 0 or 1 in byte 1 are rejected. This byte is not a command
acknowledgement, command echo, sequence number, latched emergency stop, or
specific fault code.

The record contains no controller clock or source sequence. Applications must
label local `CLOCK_MONOTONIC` receipt time as receipt time, not source time.
Only a complete frame with valid markers, XOR, and inhibit domain may refresh a
feedback watchdog. Several frames returned by one read may be buffered history
and must not be presented as distinct fresh receipt events.

Other controller frame types may appear on the byte stream. The rolling parser
therefore advances one candidate at a time and bounds retained partial data to
23 bytes.
