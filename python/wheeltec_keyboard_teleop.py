#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed keyboard input frontend for the Wheeltec VCU CLI.

Standard output is a deliberately small machine protocol.  All human-facing
text is written to standard error so this program can be piped into the C++
CLI without mixing prompts with commands.
"""

import argparse
import configparser
import contextlib
import dataclasses
import errno
import math
import os
import re
import select
import signal
import struct
import sys
import termios
import time
import tty
from typing import Callable, Dict, Iterable, List, Optional, Sequence, TextIO, Tuple


_INT64_MAX = (1 << 63) - 1
_UINT64_MAX = (1 << 64) - 1
_WIRE_POSITIVE_MAX = 32767.0 / 1000.0
_MAX_RELEASE_TIMEOUT_MS = 1000.0
_MAX_COMMAND_RATE_HZ = 1000.0
_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
_DECIMAL_NUMBER = re.compile(
    r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?\Z"
)


class ConfigurationError(ValueError):
    """The INI configuration cannot safely authorize motion."""


class InputBackendError(RuntimeError):
    """The selected keyboard input backend failed or disconnected."""


@dataclasses.dataclass(frozen=True)
class TeleopConfig:
    """Validated immutable and operator-selectable teleoperation limits."""

    max_linear_speed_mps: float
    min_selected_speed_mps: float
    initial_selected_speed_mps: float
    max_selected_speed_mps: float
    speed_step_mps: float
    curvature_inv_m: float
    release_timeout_ms: float
    command_rate_hz: float

    @property
    def release_timeout_ns(self) -> int:
        return int(self.release_timeout_ms * 1_000_000.0)

    @property
    def command_period_ns(self) -> int:
        return int(math.ceil(1_000_000_000.0 / self.command_rate_hz))


def _finite_number(section: configparser.SectionProxy, name: str) -> float:
    raw_value = section.get(name, fallback=None)
    if raw_value is None or raw_value.strip() == "":
        raise ConfigurationError("missing [{}] {}".format(section.name, name))
    if _DECIMAL_NUMBER.fullmatch(raw_value.strip()) is None:
        raise ConfigurationError(
            "[{}] {} must use strict decimal syntax".format(section.name, name)
        )
    try:
        value = float(raw_value.strip())
    except ValueError as error:
        raise ConfigurationError(
            "[{}] {} must be a number".format(section.name, name)
        ) from error
    if not math.isfinite(value):
        raise ConfigurationError(
            "[{}] {} must be finite".format(section.name, name)
        )
    return value


def validate_config(config: TeleopConfig) -> None:
    """Raise ``ConfigurationError`` unless every motion value is safe."""

    values = dataclasses.asdict(config)
    for name, value in values.items():
        if not math.isfinite(value):
            raise ConfigurationError("{} must be finite".format(name))

    immutable_limit = config.max_linear_speed_mps
    if not 0.0 < immutable_limit <= 6.0:
        raise ConfigurationError(
            "max_linear_speed_mps must satisfy 0 < value <= 6.0"
        )

    if not 0.0 < config.min_selected_speed_mps:
        raise ConfigurationError("min_selected_speed_mps must be positive")
    if not (
        config.min_selected_speed_mps
        <= config.initial_selected_speed_mps
        <= config.max_selected_speed_mps
    ):
        raise ConfigurationError(
            "selected speeds must satisfy min <= initial <= max"
        )
    if not config.min_selected_speed_mps < config.max_selected_speed_mps:
        raise ConfigurationError("selected speed range must be non-empty")
    if config.max_selected_speed_mps > immutable_limit:
        raise ConfigurationError(
            "max_selected_speed_mps exceeds immutable max_linear_speed_mps"
        )
    if not 0.0 < config.speed_step_mps <= immutable_limit:
        raise ConfigurationError(
            "speed_step_mps must be positive and no larger than the immutable limit"
        )
    if not 0.0 < config.curvature_inv_m:
        raise ConfigurationError("curvature_inv_m must be positive")
    if config.max_selected_speed_mps * config.curvature_inv_m > _WIRE_POSITIVE_MAX:
        raise ConfigurationError(
            "selected speed times curvature exceeds the positive int16 wire range"
        )

    if not 0.0 < config.release_timeout_ms <= _MAX_RELEASE_TIMEOUT_MS:
        raise ConfigurationError(
            "release_timeout_ms must satisfy 0 < value <= {}".format(
                int(_MAX_RELEASE_TIMEOUT_MS)
            )
        )
    if not 0.0 < config.command_rate_hz <= _MAX_COMMAND_RATE_HZ:
        raise ConfigurationError(
            "command_rate_hz must satisfy 0 < value <= {}".format(
                int(_MAX_COMMAND_RATE_HZ)
            )
        )
    if config.release_timeout_ns <= 0 or config.release_timeout_ns > _INT64_MAX:
        raise ConfigurationError("release_timeout_ms cannot be represented in ns")
    if config.command_period_ns <= 0:
        raise ConfigurationError("command_rate_hz produces an invalid period")
    if config.command_period_ns >= config.release_timeout_ns:
        raise ConfigurationError(
            "command period must be shorter than release_timeout_ms"
        )


def load_config(path: str) -> TeleopConfig:
    """Load an exact two-section INI file without implicit defaults."""

    parser = configparser.ConfigParser(
        interpolation=None,
        delimiters=("=",),
        comment_prefixes=("#", ";"),
        inline_comment_prefixes=None,
        strict=True,
        empty_lines_in_values=False,
    )
    parser.optionxform = str
    try:
        with open(path, "r", encoding="utf-8") as config_file:
            parser.read_file(config_file)
    except (OSError, UnicodeError, configparser.Error) as error:
        raise ConfigurationError("cannot read strict INI config: {}".format(error)) from error

    required_sections = {"limits", "teleop"}
    allowed_sections = {"serial", "limits", "watchdog", "teleop"}
    actual_sections = set(parser.sections())
    if not required_sections.issubset(actual_sections) or not actual_sections.issubset(
        allowed_sections
    ):
        raise ConfigurationError(
            "config must contain [limits]/[teleop] and only known project sections"
        )
    if parser.defaults():
        raise ConfigurationError("DEFAULT values are not permitted")

    required_options = {
        "limits": {"max_linear_speed_mps"},
        "teleop": {
            "min_selected_speed_mps",
            "initial_selected_speed_mps",
            "max_selected_speed_mps",
            "speed_step_mps",
            "curvature_inv_m",
            "release_timeout_ms",
            "command_rate_hz",
        },
    }
    allowed_options = {
        "limits": {"max_linear_speed_mps", "max_abs_yaw_rate_radps"},
        "teleop": required_options["teleop"],
    }
    for section_name, option_names in required_options.items():
        actual_names = set(parser[section_name].keys())
        if not option_names.issubset(actual_names) or not actual_names.issubset(
            allowed_options[section_name]
        ):
            missing = sorted(option_names - actual_names)
            unknown = sorted(actual_names - allowed_options[section_name])
            raise ConfigurationError(
                "[{}] options mismatch; missing={} unknown={}".format(
                    section_name, missing, unknown
                )
            )

    limits = parser["limits"]
    teleop = parser["teleop"]
    config = TeleopConfig(
        max_linear_speed_mps=_finite_number(limits, "max_linear_speed_mps"),
        min_selected_speed_mps=_finite_number(
            teleop, "min_selected_speed_mps"
        ),
        initial_selected_speed_mps=_finite_number(
            teleop, "initial_selected_speed_mps"
        ),
        max_selected_speed_mps=_finite_number(
            teleop, "max_selected_speed_mps"
        ),
        speed_step_mps=_finite_number(teleop, "speed_step_mps"),
        curvature_inv_m=_finite_number(teleop, "curvature_inv_m"),
        release_timeout_ms=_finite_number(teleop, "release_timeout_ms"),
        command_rate_hz=_finite_number(teleop, "command_rate_hz"),
    )
    validate_config(config)
    optional_yaw_limit = limits.get("max_abs_yaw_rate_radps", fallback=None)
    if optional_yaw_limit is not None:
        max_abs_yaw_rate_radps = _finite_number(
            limits, "max_abs_yaw_rate_radps"
        )
        if not 0.0 < max_abs_yaw_rate_radps <= _WIRE_POSITIVE_MAX:
            raise ConfigurationError(
                "[limits] max_abs_yaw_rate_radps must fit the positive int16 wire range"
            )
        if config.max_selected_speed_mps * config.curvature_inv_m > max_abs_yaw_rate_radps:
            raise ConfigurationError(
                "teleop turn command exceeds max_abs_yaw_rate_radps"
            )
    return config


@dataclasses.dataclass(frozen=True)
class KeyEvent:
    key: str
    pressed: bool
    repeat: bool = False


class ProtocolWriter:
    """Flush every complete machine-protocol line to the downstream CLI."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream

    def emit(self, line: str) -> None:
        if "\n" in line or "\r" in line:
            raise ValueError("protocol lines cannot contain newline characters")
        self._stream.write(line + "\n")
        self._stream.flush()


class ShutdownState:
    """Signal handlers only update this state; they never perform I/O."""

    def __init__(self) -> None:
        self.requested = False
        self.signal_number: Optional[int] = None

    def handle(self, signal_number: int, _frame: object) -> None:
        self.signal_number = signal_number
        self.requested = True


@contextlib.contextmanager
def installed_shutdown_handlers(state: ShutdownState) -> Iterable[None]:
    previous: Dict[int, object] = {}
    for signal_number in _SIGNALS:
        previous[signal_number] = signal.getsignal(signal_number)
        signal.signal(signal_number, state.handle)
    try:
        yield
    finally:
        for signal_number, handler in previous.items():
            signal.signal(signal_number, handler)


_DIRECTION_KEYS = frozenset(("w", "s", "a", "d"))


class TeleopController:
    """Pure fail-closed teleoperation state machine."""

    def __init__(
        self,
        config: TeleopConfig,
        emit_protocol: Callable[[str], None],
        emit_ui: Callable[[str], None],
    ) -> None:
        validate_config(config)
        self._config = config
        self._emit_protocol = emit_protocol
        self._emit_ui = emit_ui
        self._selected_speed_mps = config.initial_selected_speed_mps
        self._authorized = False
        self._estop_latched = False
        self._active_direction: Optional[str] = None
        self._last_direction_activity_ns: Optional[int] = None
        self._next_command_ns: Optional[int] = None
        self._last_clock_ns: Optional[int] = None
        self._last_auth_token = 0
        self._authorization_event_ns = 0
        self._sequence = 0
        self._direction_conflict_until_ns = 0
        self._stopped_emitted = False
        self.quit_requested = False
        self.finished = False

    @property
    def selected_speed_mps(self) -> float:
        return self._selected_speed_mps

    @property
    def authorized(self) -> bool:
        return self._authorized

    @property
    def active_direction(self) -> Optional[str]:
        return self._active_direction

    def show_help(self, backend_name: str) -> None:
        self._emit_ui("Wheeltec VCU keyboard teleop (independent, ROS-free)")
        self._emit_ui(
            "Keys: r=authorize, w/s=forward/reverse, a/d=left/right arc, "
            "q/e=increase/decrease selected |speed|, space=stop, "
            "x=ESTOP, Esc=quit"
        )
        self._emit_ui(
            "Configured immutable limit {:.3f} m/s; selectable {:.3f}..{:.3f} "
            "m/s; current {:.3f} m/s".format(
                self._config.max_linear_speed_mps,
                self._config.min_selected_speed_mps,
                self._config.max_selected_speed_mps,
                self._selected_speed_mps,
            )
        )
        if backend_name == "tty":
            self._emit_ui(
                "TTY has no real key-up events: release is synthesized when key "
                "auto-repeat is silent for {:.0f} ms, then STOP is sent.".format(
                    self._config.release_timeout_ms
                )
            )
        else:
            self._emit_ui(
                "Linux input_event backend uses real press/release events and also "
                "retains the {:.0f} ms deadman timeout.".format(
                    self._config.release_timeout_ms
                )
            )
        self._emit_ui(
            "After a CLI/serial reconnect, press r again and then re-press a "
            "direction key; authorization is never restored automatically."
        )
        self._emit_ui("Machine protocol is on stdout; these prompts are on stderr.")

    def _valid_now(self, now_ns: int) -> bool:
        if now_ns <= 0 or (
            self._last_clock_ns is not None and now_ns < self._last_clock_ns
        ):
            self._authorized = False
            self._clear_direction()
            self._emit_stop(force=True)
            self._emit_ui("Invalid/regressed monotonic clock: STOP; press r again.")
            return False
        self._last_clock_ns = now_ns
        return True

    def _clear_direction(self) -> None:
        self._active_direction = None
        self._last_direction_activity_ns = None
        self._next_command_ns = None

    def _emit_stop(self, force: bool = False) -> None:
        if force or not self._stopped_emitted:
            self._emit_protocol("STOP")
            self._stopped_emitted = True

    def force_stop(self) -> None:
        self._clear_direction()
        self._emit_stop(force=True)

    def _stop_and_clear(self, message: Optional[str] = None) -> None:
        self._clear_direction()
        self._emit_stop()
        if message is not None:
            self._emit_ui(message)

    def _authorize(self, now_ns: int) -> None:
        if self._estop_latched:
            self._authorized = False
            self._stop_and_clear(
                "ESTOP remains latched; an external authorized reset and a new "
                "teleop process are required."
            )
            return
        token = max(now_ns, self._last_auth_token + 1)
        if token > _INT64_MAX:
            self._authorized = False
            self._stop_and_clear("Authorization token overflow: STOP.")
            return
        self._last_auth_token = token
        self._authorized = True
        self._authorization_event_ns = now_ns
        self._direction_conflict_until_ns = 0
        self._clear_direction()
        self._emit_protocol("AUTH {}".format(token))
        self._emit_stop(force=True)
        self._emit_ui(
            "Authorization token sent. Re-press a direction key to request motion."
        )

    def _change_speed(self, delta_mps: float) -> None:
        if self._active_direction is not None:
            self._stop_and_clear("Speed selection changed during motion: STOP.")
        selected = self._selected_speed_mps + delta_mps
        selected = max(self._config.min_selected_speed_mps, selected)
        selected = min(self._config.max_selected_speed_mps, selected)
        self._selected_speed_mps = round(selected, 12)
        self._emit_ui(
            "Selected speed magnitude: {:.3f} m/s (bounds {:.3f}..{:.3f}).".format(
                self._selected_speed_mps,
                self._config.min_selected_speed_mps,
                self._config.max_selected_speed_mps,
            )
        )

    def _motion(self) -> Tuple[float, float]:
        speed = self._selected_speed_mps
        if self._active_direction == "w":
            return speed, 0.0
        if self._active_direction == "s":
            return -speed, 0.0
        if self._active_direction == "a":
            return speed, speed * self._config.curvature_inv_m
        if self._active_direction == "d":
            return speed, -speed * self._config.curvature_inv_m
        raise RuntimeError("motion requested without an active direction")

    def _emit_command(self, now_ns: int) -> None:
        if self._sequence >= _UINT64_MAX:
            self._authorized = False
            self._stop_and_clear("Command sequence exhausted: STOP; restart required.")
            return
        if now_ns > _INT64_MAX - self._config.release_timeout_ns:
            self._authorized = False
            self._stop_and_clear("Command deadline overflow: STOP; restart required.")
            return
        # A monotonic-clock floor keeps sequence IDs increasing across a
        # teleop-process restart while the guarded C++ CLI remains alive.
        self._sequence = max(self._sequence + 1, now_ns)
        deadline_ns = now_ns + self._config.release_timeout_ns
        linear_mps, yaw_radps = self._motion()
        self._emit_protocol(
            "CMD {} {} {} {:.9f} {:.9f}".format(
                self._sequence,
                now_ns,
                deadline_ns,
                linear_mps,
                yaw_radps,
            )
        )
        self._stopped_emitted = False
        self._next_command_ns = now_ns + self._config.command_period_ns

    def _expire_direction_if_needed(self, now_ns: int) -> bool:
        if (
            self._active_direction is not None
            and self._last_direction_activity_ns is not None
            and now_ns - self._last_direction_activity_ns
            >= self._config.release_timeout_ns
        ):
            self._stop_and_clear("Direction input timed out/released: STOP.")
            return True
        return False

    def handle_event(self, event: KeyEvent, now_ns: int) -> None:
        if not self._valid_now(now_ns):
            return
        self._expire_direction_if_needed(now_ns)
        key = event.key.lower()

        if not event.pressed:
            if key in _DIRECTION_KEYS and self._active_direction == key:
                self._stop_and_clear("Direction key released: STOP.")
            return

        if key == "r":
            self._authorize(now_ns)
            return
        if key == "q":
            self._change_speed(self._config.speed_step_mps)
            return
        if key == "e":
            self._change_speed(-self._config.speed_step_mps)
            return
        if key == "space":
            self._direction_conflict_until_ns = 0
            self._stop_and_clear("Operator stop: STOP.")
            return
        if key == "x":
            self._authorized = False
            self._estop_latched = True
            self._clear_direction()
            self._emit_protocol("ESTOP")
            self._emit_stop(force=True)
            self._emit_ui("ESTOP requested. Reset is external and never automatic.")
            return
        if key == "esc":
            self.quit_requested = True
            self._stop_and_clear("Quit requested: STOP.")
            return

        if key not in _DIRECTION_KEYS:
            if self._active_direction is not None:
                self._direction_conflict_until_ns = min(
                    _INT64_MAX, now_ns + self._config.release_timeout_ns
                )
                self._stop_and_clear("Unknown/conflicting input during motion: STOP.")
            return
        if now_ns < self._direction_conflict_until_ns:
            self._direction_conflict_until_ns = min(
                _INT64_MAX, now_ns + self._config.release_timeout_ns
            )
            self._stop_and_clear(
                "Direction remains conflict-inhibited: release all keys, wait, and re-press."
            )
            return
        if not self._authorized:
            self._stop_and_clear("Motion rejected locally: press r to authorize first.")
            return
        if now_ns <= self._authorization_event_ns:
            self._stop_and_clear(
                "Direction arrived with AUTH; STOP and re-press the direction key."
            )
            return
        if self._active_direction is not None and self._active_direction != key:
            self._direction_conflict_until_ns = min(
                _INT64_MAX, now_ns + self._config.release_timeout_ns
            )
            self._stop_and_clear(
                "Conflicting direction keys: STOP; release and press one direction."
            )
            return

        self._active_direction = key
        self._last_direction_activity_ns = now_ns
        if self._next_command_ns is None:
            self._next_command_ns = now_ns
        if now_ns >= self._next_command_ns:
            self._emit_command(now_ns)

    def tick(self, now_ns: int) -> None:
        if not self._valid_now(now_ns):
            return
        if self._expire_direction_if_needed(now_ns):
            return
        if (
            self._authorized
            and self._active_direction is not None
            and self._next_command_ns is not None
            and now_ns >= self._next_command_ns
        ):
            self._emit_command(now_ns)

    def next_poll_timeout_seconds(self, now_ns: int) -> float:
        wake_ns = now_ns + 100_000_000
        if self._active_direction is not None:
            if self._last_direction_activity_ns is not None:
                wake_ns = min(
                    wake_ns,
                    self._last_direction_activity_ns
                    + self._config.release_timeout_ns,
                )
            if self._next_command_ns is not None:
                wake_ns = min(wake_ns, self._next_command_ns)
        return max(0.0, float(wake_ns - now_ns) / 1_000_000_000.0)

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self._authorized = False
        self._clear_direction()
        self._emit_stop(force=True)
        self._emit_protocol("QUIT")


class TtyEventSource:
    backend_name = "tty"

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream
        self._fd = stream.fileno()
        self._saved_attributes: Optional[List[object]] = None

    def __enter__(self) -> "TtyEventSource":
        if not os.isatty(self._fd):
            raise InputBackendError(
                "stdin is not a TTY; use --event-device for Linux input_event"
            )
        try:
            self._saved_attributes = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd, termios.TCSANOW)
        except (OSError, termios.error) as error:
            self._saved_attributes = None
            raise InputBackendError("cannot enter TTY cbreak mode: {}".format(error)) from error
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        if self._saved_attributes is not None:
            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._saved_attributes
                )
            finally:
                self._saved_attributes = None

    def poll(self, timeout_seconds: float) -> List[KeyEvent]:
        try:
            readable, _, _ = select.select([self._fd], [], [], timeout_seconds)
        except OSError as error:
            if error.errno == errno.EINTR:
                return []
            raise InputBackendError("TTY select failed: {}".format(error)) from error
        if not readable:
            return []
        try:
            data = os.read(self._fd, 64)
        except OSError as error:
            if error.errno in (errno.EINTR, errno.EAGAIN, errno.EWOULDBLOCK):
                return []
            raise InputBackendError("TTY read failed: {}".format(error)) from error
        if not data:
            raise InputBackendError("TTY input closed")
        events: List[KeyEvent] = []
        for byte in data:
            if byte == 0x1B:
                events.append(KeyEvent("esc", True))
            elif byte == 0x20:
                events.append(KeyEvent("space", True))
            elif byte < 0x80:
                character = chr(byte).lower()
                if character in "wsadqerx":
                    events.append(KeyEvent(character, True))
        return events


_INPUT_EVENT = struct.Struct("@llHHI")
_EV_KEY = 0x01
_LINUX_KEY_NAMES = {
    1: "esc",
    16: "q",
    17: "w",
    18: "e",
    19: "r",
    30: "a",
    31: "s",
    32: "d",
    45: "x",
    57: "space",
}


def decode_linux_input_events(data: bytes) -> Tuple[List[KeyEvent], bytes]:
    """Decode complete native Linux ``input_event`` records.

    The returned remainder lets the nonblocking reader retain a fragmented
    record.  No event-device file is opened by this function.
    """

    events: List[KeyEvent] = []
    offset = 0
    while len(data) - offset >= _INPUT_EVENT.size:
        _seconds, _microseconds, event_type, code, value = _INPUT_EVENT.unpack_from(
            data, offset
        )
        offset += _INPUT_EVENT.size
        if event_type != _EV_KEY or code not in _LINUX_KEY_NAMES:
            continue
        if value not in (0, 1, 2):
            raise InputBackendError(
                "invalid Linux key event value {} for code {}".format(value, code)
            )
        events.append(
            KeyEvent(
                _LINUX_KEY_NAMES[code],
                pressed=value != 0,
                repeat=value == 2,
            )
        )
    return events, data[offset:]


class LinuxInputEventSource:
    backend_name = "input_event"

    def __init__(self, path: str) -> None:
        if not path or not os.path.isabs(path):
            raise InputBackendError("--event-device must be an absolute path")
        self._path = path
        self._fd = -1
        self._buffer = b""

    def __enter__(self) -> "LinuxInputEventSource":
        flags = os.O_RDONLY | os.O_NONBLOCK
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        try:
            self._fd = os.open(self._path, flags)
        except OSError as error:
            raise InputBackendError(
                "cannot open input_event device {}: {}".format(self._path, error)
            ) from error
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        if self._fd >= 0:
            try:
                os.close(self._fd)
            finally:
                self._fd = -1
                self._buffer = b""

    def poll(self, timeout_seconds: float) -> List[KeyEvent]:
        if self._fd < 0:
            raise InputBackendError("input_event source is not open")
        try:
            readable, _, _ = select.select([self._fd], [], [], timeout_seconds)
        except OSError as error:
            if error.errno == errno.EINTR:
                return []
            raise InputBackendError("input_event select failed: {}".format(error)) from error
        if not readable:
            return []
        try:
            chunk = os.read(self._fd, 4096)
        except OSError as error:
            if error.errno in (errno.EINTR, errno.EAGAIN, errno.EWOULDBLOCK):
                return []
            raise InputBackendError("input_event read failed: {}".format(error)) from error
        if not chunk:
            raise InputBackendError("input_event device disconnected")
        events, self._buffer = decode_linux_input_events(self._buffer + chunk)
        return events


def run_session(
    controller: TeleopController,
    event_source: object,
    clock_ns: Callable[[], int],
    shutdown_state: ShutdownState,
) -> None:
    """Run until Esc, a shutdown signal, or an input exception.

    ``controller.finish`` is guaranteed for every exit path.  The supplied
    source only needs ``backend_name`` and ``poll(timeout_seconds)`` attributes,
    making unit tests independent from terminals and device files.
    """

    try:
        controller.show_help(str(getattr(event_source, "backend_name")))
        controller.force_stop()
        while not controller.quit_requested and not shutdown_state.requested:
            now_ns = clock_ns()
            controller.tick(now_ns)
            if controller.quit_requested or shutdown_state.requested:
                break
            timeout_seconds = controller.next_poll_timeout_seconds(now_ns)
            events = getattr(event_source, "poll")(timeout_seconds)
            now_ns = clock_ns()
            if shutdown_state.requested:
                break
            for event in events:
                controller.handle_event(event, now_ns)
                if controller.quit_requested or shutdown_state.requested:
                    break
            controller.tick(now_ns)
    finally:
        controller.finish()


class _StderrArgumentParser(argparse.ArgumentParser):
    def _print_message(self, message: str, file: Optional[TextIO] = None) -> None:
        if message:
            sys.stderr.write(message)


def _argument_parser() -> argparse.ArgumentParser:
    project_or_prefix = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    source_config = os.path.join(
        project_or_prefix, "config", "wheeltec_vcu_serial.ini"
    )
    installed_config = os.path.join(
        project_or_prefix,
        "share",
        "wheeltec_vcu_serial",
        "config",
        "wheeltec_vcu_serial.ini",
    )
    default_config = source_config if os.path.isfile(source_config) else installed_config
    parser = _StderrArgumentParser(
        description="Guarded keyboard frontend for the Wheeltec VCU serial CLI"
    )
    parser.add_argument(
        "--config",
        default=default_config,
        help="strict INI configuration (default: %(default)s)",
    )
    parser.add_argument(
        "--event-device",
        help="optional absolute Linux /dev/input/event* path with real key releases",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    writer = ProtocolWriter(sys.stdout)
    controller: Optional[TeleopController] = None
    try:
        arguments = _argument_parser().parse_args(argv)
        config = load_config(arguments.config)
        controller = TeleopController(
            config,
            writer.emit,
            lambda message: print(message, file=sys.stderr, flush=True),
        )
        shutdown_state = ShutdownState()
        event_source = (
            LinuxInputEventSource(arguments.event_device)
            if arguments.event_device
            else TtyEventSource(sys.stdin)
        )
        with installed_shutdown_handlers(shutdown_state):
            with event_source:
                run_session(controller, event_source, time.monotonic_ns, shutdown_state)
        return 0
    except ConfigurationError as error:
        print("Configuration rejected: {}".format(error), file=sys.stderr, flush=True)
        return 2
    except Exception as error:
        print("Teleop stopped after error: {}".format(error), file=sys.stderr, flush=True)
        return 1
    finally:
        if controller is not None:
            controller.finish()
        else:
            # Even configuration/argument failures fail closed for a downstream
            # process that may already be listening on stdout.
            try:
                writer.emit("STOP")
                writer.emit("QUIT")
            except (BrokenPipeError, OSError):
                pass


if __name__ == "__main__":
    sys.exit(main())
