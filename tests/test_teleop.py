#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

import io
import os
import signal
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Callable, List, Optional, Tuple
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "python"))

import wheeltec_keyboard_teleop as teleop  # noqa: E402


def make_config(**overrides: float) -> teleop.TeleopConfig:
    values = {
        "max_linear_speed_mps": 0.30,
        "min_selected_speed_mps": 0.05,
        "initial_selected_speed_mps": 0.10,
        "max_selected_speed_mps": 0.20,
        "speed_step_mps": 0.05,
        "curvature_inv_m": 0.75,
        "release_timeout_ms": 180.0,
        "command_rate_hz": 20.0,
    }
    values.update(overrides)
    return teleop.TeleopConfig(**values)


def config_text(
    immutable_limit: str = "0.30",
    teleop_overrides: Optional[dict] = None,
    extra_limits: str = "max_abs_yaw_rate_radps = 1.00\n",
) -> str:
    values = {
        "min_selected_speed_mps": "0.05",
        "initial_selected_speed_mps": "0.10",
        "max_selected_speed_mps": "0.20",
        "speed_step_mps": "0.05",
        "curvature_inv_m": "0.75",
        "release_timeout_ms": "180",
        "command_rate_hz": "20",
    }
    if teleop_overrides:
        values.update(teleop_overrides)
    teleop_lines = "\n".join("{} = {}".format(key, value) for key, value in values.items())
    return """# synthetic unit-test configuration
[serial]
device =
reconnect_interval_ms = 1000

[limits]
max_linear_speed_mps = {immutable_limit}
{extra_limits}
[watchdog]
command_timeout_ms = 250

[teleop]
{teleop_lines}
""".format(
        immutable_limit=immutable_limit,
        extra_limits=extra_limits,
        teleop_lines=teleop_lines,
    )


def load_text(text: str) -> teleop.TeleopConfig:
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "teleop.ini")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(text)
        return teleop.load_config(path)


def command_fields(line: str) -> Tuple[int, int, int, float, float]:
    fields = line.split()
    if len(fields) != 6 or fields[0] != "CMD":
        raise AssertionError("not a CMD line: {}".format(line))
    return (
        int(fields[1]),
        int(fields[2]),
        int(fields[3]),
        float(fields[4]),
        float(fields[5]),
    )


class FakeClock:
    def __init__(self, now_ns: int = 1_000_000_000) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns

    def advance(self, amount_ns: int) -> None:
        self.now_ns += amount_ns


class ScriptedEventSource:
    backend_name = "fake"

    def __init__(
        self,
        clock: FakeClock,
        actions: List[Tuple[int, object]],
    ) -> None:
        self._clock = clock
        self._actions = list(actions)
        self.poll_timeouts: List[float] = []

    def poll(self, timeout_seconds: float) -> List[teleop.KeyEvent]:
        self.poll_timeouts.append(timeout_seconds)
        if not self._actions:
            raise AssertionError("scripted source exhausted before session ended")
        advance_ns, action = self._actions.pop(0)
        self._clock.advance(advance_ns)
        if isinstance(action, BaseException):
            raise action
        if callable(action):
            action()
            return []
        return list(action)


class ConfigTests(unittest.TestCase):
    def test_checked_in_full_runtime_config_is_accepted(self) -> None:
        config = teleop.load_config(
            str(REPOSITORY_ROOT / "config" / "wheeltec_vcu_serial.ini")
        )
        self.assertEqual(config.max_linear_speed_mps, 0.30)
        self.assertEqual(config.initial_selected_speed_mps, 0.10)
        self.assertLess(config.command_period_ns, config.release_timeout_ns)

    def test_immutable_limit_accepts_six_and_rejects_values_above_it(self) -> None:
        self.assertEqual(load_text(config_text(immutable_limit="6")).max_linear_speed_mps, 6.0)
        for value in ("0", "-1", "nan", "inf", "-inf", "6.1", "0_3"):
            with self.subTest(value=value):
                with self.assertRaises(teleop.ConfigurationError):
                    load_text(config_text(immutable_limit=value))

        missing = config_text().replace("max_linear_speed_mps = 0.30\n", "")
        with self.assertRaises(teleop.ConfigurationError):
            load_text(missing)

    def test_selected_speed_order_and_immutable_bound_are_strict(self) -> None:
        invalid_overrides = (
            {"min_selected_speed_mps": "0"},
            {"initial_selected_speed_mps": "0.01"},
            {"max_selected_speed_mps": "0.31"},
            {"min_selected_speed_mps": "0.20", "max_selected_speed_mps": "0.20"},
            {"speed_step_mps": "0"},
            {"speed_step_mps": "nan"},
        )
        for overrides in invalid_overrides:
            with self.subTest(overrides=overrides):
                with self.assertRaises(teleop.ConfigurationError):
                    load_text(config_text(teleop_overrides=overrides))

    def test_curvature_timeout_rate_and_wire_yaw_are_validated(self) -> None:
        invalid_overrides = (
            {"curvature_inv_m": "0"},
            {"curvature_inv_m": "inf"},
            {"release_timeout_ms": "0"},
            {"release_timeout_ms": "1001"},
            {"command_rate_hz": "0"},
            {"command_rate_hz": "1001"},
            {"release_timeout_ms": "10", "command_rate_hz": "20"},
            {"curvature_inv_m": "200"},
        )
        for overrides in invalid_overrides:
            with self.subTest(overrides=overrides):
                with self.assertRaises(teleop.ConfigurationError):
                    load_text(config_text(teleop_overrides=overrides))

        with self.assertRaises(teleop.ConfigurationError):
            load_text(
                config_text(extra_limits="max_abs_yaw_rate_radps = 0.10\n")
            )

    def test_unknown_or_duplicate_relevant_keys_are_rejected(self) -> None:
        unknown = config_text().replace(
            "command_rate_hz = 20", "command_rate_hz = 20\ncommand_rates_hz = 20"
        )
        duplicate = config_text().replace(
            "command_rate_hz = 20", "command_rate_hz = 20\ncommand_rate_hz = 20"
        )
        for text in (unknown, duplicate):
            with self.assertRaises(teleop.ConfigurationError):
                load_text(text)

    def test_source_tree_default_config_path_exists(self) -> None:
        arguments = teleop._argument_parser().parse_args([])
        self.assertEqual(
            Path(arguments.config),
            REPOSITORY_ROOT / "config" / "wheeltec_vcu_serial.ini",
        )


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lines: List[str] = []
        self.ui: List[str] = []
        self.config = make_config()
        self.controller = teleop.TeleopController(
            self.config, self.lines.append, self.ui.append
        )
        self.now_ns = 1_000_000_000

    def authorize(self) -> None:
        self.controller.handle_event(teleop.KeyEvent("r", True), self.now_ns)
        self.assertTrue(self.controller.authorized)
        self.now_ns += 1

    def latest_command(self) -> Tuple[int, int, int, float, float]:
        command_lines = [line for line in self.lines if line.startswith("CMD ")]
        self.assertTrue(command_lines)
        return command_fields(command_lines[-1])

    def test_direction_keys_map_to_signed_linear_and_yaw(self) -> None:
        self.authorize()
        expected = {
            "w": (0.10, 0.0),
            "s": (-0.10, 0.0),
            "a": (0.10, 0.075),
            "d": (0.10, -0.075),
        }
        previous_sequence = 0
        for key, (expected_linear, expected_yaw) in expected.items():
            self.now_ns += 1_000_000
            self.controller.handle_event(teleop.KeyEvent(key, True), self.now_ns)
            sequence, created, deadline, linear, yaw = self.latest_command()
            self.assertGreater(sequence, previous_sequence)
            self.assertEqual(created, self.now_ns)
            self.assertEqual(deadline - created, self.config.release_timeout_ns)
            self.assertAlmostEqual(linear, expected_linear)
            self.assertAlmostEqual(yaw, expected_yaw)
            previous_sequence = sequence
            self.controller.handle_event(
                teleop.KeyEvent(key, False), self.now_ns + 1
            )
            self.assertEqual(self.lines[-1], "STOP")

    def test_direction_requires_explicit_authorization(self) -> None:
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        self.assertFalse(any(line.startswith("CMD ") for line in self.lines))
        self.assertEqual(self.lines[-1], "STOP")
        self.assertIn("press r", self.ui[-1])

    def test_direction_in_same_input_batch_as_auth_requires_repress(self) -> None:
        self.controller.handle_event(teleop.KeyEvent("r", True), self.now_ns)
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        self.assertFalse(any(line.startswith("CMD ") for line in self.lines))
        self.assertEqual(self.lines[-1], "STOP")
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns + 1)
        self.assertTrue(self.lines[-1].startswith("CMD "))

    def test_q_and_e_clamp_selected_speed_to_explicit_bounds(self) -> None:
        for _ in range(20):
            self.controller.handle_event(teleop.KeyEvent("q", True), self.now_ns)
        self.assertAlmostEqual(
            self.controller.selected_speed_mps, self.config.max_selected_speed_mps
        )
        for _ in range(20):
            self.controller.handle_event(teleop.KeyEvent("e", True), self.now_ns)
        self.assertAlmostEqual(
            self.controller.selected_speed_mps, self.config.min_selected_speed_mps
        )
        self.assertTrue(any("Selected speed magnitude" in line for line in self.ui))

    def test_speed_change_during_motion_sends_stop_and_requires_repress(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        command_count = len([line for line in self.lines if line.startswith("CMD ")])
        self.controller.handle_event(
            teleop.KeyEvent("q", True), self.now_ns + 1_000_000
        )
        self.assertEqual(self.lines[-1], "STOP")
        self.assertIsNone(self.controller.active_direction)
        self.controller.tick(self.now_ns + self.config.command_period_ns)
        self.assertEqual(
            len([line for line in self.lines if line.startswith("CMD ")]),
            command_count,
        )

    def test_tty_silence_synthesizes_release_at_deadman_timeout(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        before = list(self.lines)
        self.controller.tick(self.now_ns + self.config.release_timeout_ns - 1)
        self.assertNotEqual(self.lines[-1], "STOP")
        self.assertGreater(len(self.lines), len(before))
        self.controller.tick(self.now_ns + self.config.release_timeout_ns)
        self.assertEqual(self.lines[-1], "STOP")
        self.assertIsNone(self.controller.active_direction)

    def test_repeat_refreshes_deadman_and_true_release_stops_immediately(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("a", True), self.now_ns)
        repeat_ns = self.now_ns + self.config.release_timeout_ns - 1
        self.controller.handle_event(
            teleop.KeyEvent("a", True, repeat=True), repeat_ns
        )
        self.controller.tick(repeat_ns + self.config.release_timeout_ns - 1)
        self.assertNotEqual(self.lines[-1], "STOP")
        self.controller.handle_event(
            teleop.KeyEvent("a", False), repeat_ns + self.config.release_timeout_ns
        )
        self.assertEqual(self.lines[-1], "STOP")

    def test_conflicting_directions_fail_closed(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        command_count = len([line for line in self.lines if line.startswith("CMD ")])
        self.controller.handle_event(
            teleop.KeyEvent("a", True), self.now_ns + 1_000_000
        )
        self.assertEqual(self.lines[-1], "STOP")
        self.assertIsNone(self.controller.active_direction)
        self.assertEqual(
            len([line for line in self.lines if line.startswith("CMD ")]),
            command_count,
        )
        self.controller.handle_event(
            teleop.KeyEvent("w", True, repeat=True), self.now_ns + 2_000_000
        )
        self.assertEqual(
            len([line for line in self.lines if line.startswith("CMD ")]),
            command_count,
        )
        self.assertEqual(self.lines[-1], "STOP")

    def test_space_estop_and_escape_are_explicit_fail_closed_paths(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("space", True), self.now_ns)
        self.assertEqual(self.lines[-1], "STOP")

        self.controller.handle_event(teleop.KeyEvent("x", True), self.now_ns + 1)
        self.assertEqual(self.lines[-2:], ["ESTOP", "STOP"])
        self.assertFalse(self.controller.authorized)
        auth_count = len([line for line in self.lines if line.startswith("AUTH ")])
        self.controller.handle_event(teleop.KeyEvent("r", True), self.now_ns + 2)
        self.assertFalse(self.controller.authorized)
        self.assertEqual(
            len([line for line in self.lines if line.startswith("AUTH ")]),
            auth_count,
        )

        self.controller.handle_event(teleop.KeyEvent("esc", True), self.now_ns + 3)
        self.assertTrue(self.controller.quit_requested)
        self.assertEqual(self.lines[-1], "STOP")

    def test_reauthorization_clears_motion_and_token_is_monotonic(self) -> None:
        self.authorize()
        first_token = int(self.lines[0].split()[1])
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        self.controller.handle_event(teleop.KeyEvent("r", True), self.now_ns)
        auth_lines = [line for line in self.lines if line.startswith("AUTH ")]
        second_token = int(auth_lines[-1].split()[1])
        self.assertGreater(second_token, first_token)
        self.assertIsNone(self.controller.active_direction)
        command_count = len([line for line in self.lines if line.startswith("CMD ")])
        self.controller.tick(self.now_ns + self.config.command_period_ns)
        self.assertEqual(
            len([line for line in self.lines if line.startswith("CMD ")]),
            command_count,
        )
        self.controller.handle_event(
            teleop.KeyEvent("w", True),
            self.now_ns + self.config.command_period_ns + 1,
        )
        self.assertTrue(self.lines[-1].startswith("CMD "))

    def test_clock_regression_revokes_authorization_and_stops(self) -> None:
        self.authorize()
        self.controller.handle_event(teleop.KeyEvent("w", True), self.now_ns)
        self.controller.tick(self.now_ns - 1)
        self.assertFalse(self.controller.authorized)
        self.assertEqual(self.lines[-1], "STOP")

    def test_help_is_clear_for_both_input_backends(self) -> None:
        self.controller.show_help("tty")
        joined = "\n".join(self.ui)
        self.assertIn("w/s=forward/reverse", joined)
        self.assertIn("q/e=increase/decrease selected |speed|", joined)
        self.assertIn("x=ESTOP", joined)
        self.assertIn("no real key-up", joined)
        self.assertIn("press r again", joined)
        self.assertIn("stderr", joined)

        event_ui: List[str] = []
        controller = teleop.TeleopController(
            self.config, lambda _line: None, event_ui.append
        )
        controller.show_help("input_event")
        self.assertIn("real press/release", "\n".join(event_ui))


class InputBackendTests(unittest.TestCase):
    def test_linux_input_event_decoder_handles_press_repeat_release_and_fragment(self) -> None:
        event_struct = struct.Struct("@llHHI")
        records = b"".join(
            (
                event_struct.pack(1, 2, 1, 17, 1),
                event_struct.pack(1, 3, 1, 17, 2),
                event_struct.pack(1, 4, 1, 17, 0),
                event_struct.pack(1, 5, 0, 17, 1),
            )
        )
        partial = event_struct.pack(2, 0, 1, 30, 1)[:7]
        events, remainder = teleop.decode_linux_input_events(records + partial)
        self.assertEqual(
            events,
            [
                teleop.KeyEvent("w", True, False),
                teleop.KeyEvent("w", True, True),
                teleop.KeyEvent("w", False, False),
            ],
        )
        self.assertEqual(remainder, partial)

    def test_linux_input_event_decoder_rejects_unknown_key_value(self) -> None:
        record = struct.Struct("@llHHI").pack(1, 0, 1, 17, 3)
        with self.assertRaises(teleop.InputBackendError):
            teleop.decode_linux_input_events(record)

    def test_tty_context_restores_attributes_after_exception(self) -> None:
        stream = mock.Mock()
        stream.fileno.return_value = 42
        saved = [1, 2, 3, 4, 5, 6, [7]]
        with mock.patch.object(teleop.os, "isatty", return_value=True), mock.patch.object(
            teleop.termios, "tcgetattr", return_value=saved
        ), mock.patch.object(teleop.tty, "setcbreak") as setcbreak, mock.patch.object(
            teleop.termios, "tcsetattr"
        ) as tcsetattr:
            with self.assertRaises(RuntimeError):
                with teleop.TtyEventSource(stream):
                    raise RuntimeError("synthetic failure")
        setcbreak.assert_called_once_with(42, teleop.termios.TCSANOW)
        tcsetattr.assert_called_once_with(42, teleop.termios.TCSADRAIN, saved)


class SessionTests(unittest.TestCase):
    def new_controller(self) -> Tuple[teleop.TeleopController, List[str], List[str]]:
        lines: List[str] = []
        ui: List[str] = []
        controller = teleop.TeleopController(make_config(), lines.append, ui.append)
        return controller, lines, ui

    def test_normal_scripted_session_ends_with_stop_and_quit(self) -> None:
        clock = FakeClock()
        source = ScriptedEventSource(
            clock,
            [
                (0, [teleop.KeyEvent("r", True)]),
                (1_000_000, [teleop.KeyEvent("w", True)]),
                (1_000_000, [teleop.KeyEvent("w", False)]),
                (1_000_000, [teleop.KeyEvent("esc", True)]),
            ],
        )
        controller, lines, _ui = self.new_controller()
        teleop.run_session(controller, source, clock, teleop.ShutdownState())
        self.assertTrue(any(line.startswith("AUTH ") for line in lines))
        self.assertTrue(any(line.startswith("CMD ") for line in lines))
        self.assertEqual(lines[-2:], ["STOP", "QUIT"])

    def test_source_exception_still_finishes_with_stop_and_quit(self) -> None:
        clock = FakeClock()
        source = ScriptedEventSource(
            clock, [(0, RuntimeError("synthetic input failure"))]
        )
        controller, lines, _ui = self.new_controller()
        with self.assertRaisesRegex(RuntimeError, "synthetic input failure"):
            teleop.run_session(controller, source, clock, teleop.ShutdownState())
        self.assertEqual(lines[-2:], ["STOP", "QUIT"])

    def test_signal_handler_only_sets_state_and_main_loop_stops(self) -> None:
        clock = FakeClock()
        shutdown = teleop.ShutdownState()
        controller, lines, _ui = self.new_controller()

        def request_shutdown() -> None:
            before = list(lines)
            shutdown.handle(signal.SIGTERM, None)
            self.assertEqual(lines, before)

        source = ScriptedEventSource(clock, [(0, request_shutdown)])
        teleop.run_session(controller, source, clock, shutdown)
        self.assertTrue(shutdown.requested)
        self.assertEqual(shutdown.signal_number, signal.SIGTERM)
        self.assertEqual(lines[-2:], ["STOP", "QUIT"])

    def test_all_required_signal_handlers_only_set_flags(self) -> None:
        for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal_number=signal_number):
                state = teleop.ShutdownState()
                state.handle(signal_number, None)
                self.assertTrue(state.requested)
                self.assertEqual(state.signal_number, signal_number)

    def test_protocol_writer_emits_only_stable_complete_lines(self) -> None:
        stream = io.StringIO()
        writer = teleop.ProtocolWriter(stream)
        writer.emit("STOP")
        writer.emit("AUTH 123")
        self.assertEqual(stream.getvalue(), "STOP\nAUTH 123\n")
        with self.assertRaises(ValueError):
            writer.emit("STOP\nQUIT")

    def test_main_config_error_fails_closed_without_opening_input(self) -> None:
        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch.object(teleop.sys, "stdout", stdout), mock.patch.object(
            teleop.sys, "stderr", stderr
        ):
            result = teleop.main(["--config", "/definitely/missing/config.ini"])
        self.assertEqual(result, 2)
        self.assertEqual(stdout.getvalue(), "STOP\nQUIT\n")
        self.assertIn("Configuration rejected", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
