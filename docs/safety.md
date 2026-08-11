<!-- SPDX-License-Identifier: Apache-2.0 -->
# Safety limitations

This software is not a certified safety controller. The wire profile does not
provide a VCU acknowledgement, source timestamp, command echo, or positive
evidence that a zero command caused a physical stop. A completed host write
means only that the local operating system accepted all bytes before the stated
deadline.

Do not assume that serial loss, process exit, USB removal, or a host watchdog
stops the controller. Keep an independently verified, physically operable
traction-energy stop chain. Software zero frames, the composite inhibit bit,
and the software emergency-stop latch do not replace it.

Key fail-closed properties are:

- no physical device is selected in the example configuration;
- motion requires an explicit finite maximum linear-speed magnitude with
  `0 < max_linear_speed_mps < 6.0`;
- missing, zero, negative, NaN, infinite, or at-least-6 limits are rejected
  before a serial device can be opened for actuation;
- wire `int16_t` checks are independent of the configured speed ceiling;
- startup and every reconnect begin inhibited and attempt an exact-zero frame;
- a reconnect clears command and authorization state, preserves replay
  high-water marks, and requires a new operator authorization plus a new run of
  fresh commands;
- stale commands, stale feedback, clock regression, inhibit feedback,
  transport failure, or latched software emergency stop revoke motion;
- invalid feedback never refreshes freshness; if no valid allowed feedback
  follows, feedback-watchdog expiry revokes motion and starts zero;
- zero writes use bounded retries and report uncertain outcome after partial
  writes, disconnects, or deadlines;
- exhausting the configured zero attempts latches a fault for that connection
  generation; repeated STOP, feedback, or ESTOP input cannot refresh the retry
  budget, and only a new transport generation can begin a new initial-zero
  episode;
- STOP received while disconnected is retained as local fail-safe intent and
  does not cancel reconnect, but no zero delivery is claimed.

Reverse speed is representable by the wire codec because the keyboard contract
includes `s`. That does not establish that a particular vehicle is mechanically
or operationally safe in reverse. Direction, yaw sign, steering response,
braking, holding, deadband, reconnect behavior, and the installed firmware must
be commissioned for the exact vehicle before actuation.

The read-only monitor opens a TTY with `O_RDONLY` and contains no write call.
Opening or configuring a USB serial interface can still change line state or
reset some devices; receive-only is not electrically consequence-free.

Each project transport requests Linux `TIOCEXCL`, which prevents later opens
while its descriptor remains alive. That ioctl cannot detect or evict a
non-cooperating process that opened the TTY first. Deployment must therefore
enforce one external owner for the serial interface; never run the actuation
CLI, monitor, a ROS adapter, or another serial program concurrently.

The TTY keyboard backend cannot observe physical key-up events. It converts the
absence of operating-system key repeat into a release after the configured
dead-man timeout. Use the optional Linux input-event backend when true release
events are required. Every release, input timeout, normal exit, caught signal,
and handled exception requests zero, but `SIGKILL`, power loss, kernel failure,
and a broken/disconnected transport cannot be covered by process cleanup.
