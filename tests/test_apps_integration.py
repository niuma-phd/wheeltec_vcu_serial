#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Process-level PTY checks for the read-only monitor and guarded CLI."""

import errno
import os
from pathlib import Path
import pty
import select
import subprocess
import tempfile
import termios
import time
import unittest


READ_ONLY_CONFIRMATION = "I_UNDERSTAND_OPENING_A_TTY_CAN_AFFECT_THE_DEVICE"
ACTUATION_CONFIRMATION = "I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK"


def synthetic_feedback(control_inhibited=False):
    frame = bytearray(24)
    frame[0] = 0x7B
    frame[1] = 1 if control_inhibited else 0

    def put_signed(offset, value):
        frame[offset : offset + 2] = int(value).to_bytes(2, "big", signed=True)

    put_signed(2, 4)
    put_signed(4, 0)
    put_signed(6, -3)
    put_signed(8, 120)
    put_signed(10, -240)
    put_signed(12, 360)
    put_signed(14, 11)
    put_signed(16, -22)
    put_signed(18, 33)
    frame[20:22] = (12345).to_bytes(2, "big", signed=False)
    checksum = 0
    for value in frame[:22]:
        checksum ^= value
    frame[22] = checksum
    frame[23] = 0x7D
    return bytes(frame)


def valid_command_frame(frame):
    if len(frame) != 11 or frame[0] != 0x7B or frame[10] != 0x7D:
        return False
    checksum = 0
    for value in frame[:9]:
        checksum ^= value
    return checksum == frame[9]


def write_to_open_pty(master, payload, timeout_seconds=2.0):
    offset = 0
    deadline = time.monotonic() + timeout_seconds
    while offset < len(payload) and time.monotonic() < deadline:
        try:
            written = os.write(master, payload[offset:])
            if written > 0:
                offset += written
                continue
        except BlockingIOError:
            pass
        except OSError as error:
            if error.errno != errno.EIO:
                raise
        time.sleep(0.01)
    if offset != len(payload):
        raise RuntimeError("PTY slave did not open before the write deadline")


def wait_for_raw_pty(master, timeout_seconds=2.0):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        try:
            attributes = termios.tcgetattr(master)
            if (attributes[3] & termios.ICANON) == 0:
                return
        except (OSError, termios.error) as error:
            if isinstance(error, OSError) and error.errno not in (
                errno.EIO,
                errno.ENOTTY,
            ):
                raise
        time.sleep(0.01)
    raise RuntimeError("PTY slave did not enter raw mode before the deadline")


class PtyFixture:
    def __init__(self):
        self.master, slave = pty.openpty()
        self.slave_path = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(self.master, False)

    def close(self):
        if self.master >= 0:
            os.close(self.master)
            self.master = -1

    def disconnect(self):
        self.close()

    def replace_at_same_path(self):
        held = []
        deadline = time.monotonic() + 2.0
        try:
            while time.monotonic() < deadline:
                master, slave = pty.openpty()
                path = os.ttyname(slave)
                os.close(slave)
                if path == self.slave_path:
                    self.master = master
                    os.set_blocking(self.master, False)
                    return
                held.append(master)
                time.sleep(0.01)
        finally:
            for descriptor in held:
                os.close(descriptor)
        raise RuntimeError("could not recreate the disconnected PTY path")


def runtime_config(device):
    return """\
[serial]
device = {device}
reconnect_interval_ms = 100

[limits]
max_linear_speed_mps = 0.30
max_abs_yaw_rate_radps = 1.00

[watchdog]
command_timeout_ms = 500
feedback_timeout_ms = 250
transmit_period_ms = 20
fresh_commands_required = 2
write_timeout_ms = 10
zero_retry_interval_ms = 20
zero_retry_attempts = 3

[teleop]
min_selected_speed_mps = 0.05
initial_selected_speed_mps = 0.10
max_selected_speed_mps = 0.20
speed_step_mps = 0.05
curvature_inv_m = 0.75
release_timeout_ms = 180
command_rate_hz = 20
""".format(device=device)


class ApplicationIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.monitor = os.environ.get("WHEELTEC_MONITOR_PATH")
        cls.cli = os.environ.get("WHEELTEC_CLI_PATH")
        if not cls.monitor or not Path(cls.monitor).is_file():
            raise RuntimeError("WHEELTEC_MONITOR_PATH is not an executable file")
        if not cls.cli or not Path(cls.cli).is_file():
            raise RuntimeError("WHEELTEC_CLI_PATH is not an executable file")

    def test_read_only_monitor_decodes_feedback_without_transmitting(self):
        fixture = PtyFixture()
        process = None
        try:
            process = subprocess.Popen(
                [
                    self.monitor,
                    "--read-only",
                    "--device",
                    fixture.slave_path,
                    "--frames",
                    "2",
                    "--timeout-ms",
                    "1500",
                    "--operator-confirmation",
                    READ_ONLY_CONFIRMATION,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            wait_for_raw_pty(fixture.master)
            write_to_open_pty(fixture.master, synthetic_feedback())
            time.sleep(0.06)
            write_to_open_pty(
                fixture.master, synthetic_feedback(control_inhibited=True)
            )
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn("receipt_frames=2", stdout)
            self.assertIn("control_allowed=1", stdout)
            self.assertIn("control_inhibited=1", stdout)
            self.assertIn("vcu_ack_available=false", stdout)

            readable, _, _ = select.select([fixture.master], [], [], 0)
            if readable:
                try:
                    outbound = os.read(fixture.master, 4096)
                except BlockingIOError:
                    outbound = b""
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    # Linux reports EIO on a PTY master after the last slave
                    # closes; no bytes were transmitted by the monitor.
                    outbound = b""
                self.assertEqual(outbound, b"")
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            fixture.close()

    def test_cli_requires_feedback_authorization_and_fresh_commands(self):
        fixture = PtyFixture()
        process = None
        config_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", delete=False
            ) as config_file:
                config_file.write(runtime_config(fixture.slave_path))
                config_path = config_file.name

            process = subprocess.Popen(
                [
                    self.cli,
                    "--config",
                    config_path,
                    "--run",
                    "--acknowledge-unverified-protocol",
                    "--enable-actuation",
                    "--operator-confirmation",
                    ACTUATION_CONFIRMATION,
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIsNotNone(process.stdin)
            captured = bytearray()
            frames = []

            def drain_frames():
                while True:
                    try:
                        chunk = os.read(fixture.master, 4096)
                    except BlockingIOError:
                        break
                    except OSError as error:
                        if error.errno == errno.EIO:
                            break
                        raise
                    if not chunk:
                        break
                    captured.extend(chunk)
                while len(captured) >= 11:
                    frame = bytes(captured[:11])
                    del captured[:11]
                    self.assertTrue(valid_command_frame(frame))
                    frames.append(frame)

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not frames:
                drain_frames()
                time.sleep(0.01)
            self.assertTrue(frames, "CLI did not emit its startup zero")
            self.assertEqual(int.from_bytes(frames[0][3:5], "big", signed=True), 0)

            write_to_open_pty(fixture.master, synthetic_feedback())
            time.sleep(0.06)
            write_to_open_pty(fixture.master, synthetic_feedback())
            time.sleep(0.03)

            auth_token = time.monotonic_ns()
            process.stdin.write("AUTH {}\n".format(auth_token))
            process.stdin.flush()
            time.sleep(0.03)

            for sequence in (1, 2):
                created = time.monotonic_ns()
                process.stdin.write(
                    "CMD {} {} {} 0.120 0.060\n".format(
                        sequence, created, created + 400_000_000
                    )
                )
                process.stdin.flush()
                write_to_open_pty(fixture.master, synthetic_feedback())
                time.sleep(0.03)

            motion_seen = False
            deadline = time.monotonic() + 2.0
            next_feedback = 0.0
            while time.monotonic() < deadline and not motion_seen:
                if time.monotonic() >= next_feedback:
                    write_to_open_pty(fixture.master, synthetic_feedback())
                    next_feedback = time.monotonic() + 0.05
                drain_frames()
                motion_seen = any(
                    int.from_bytes(frame[3:5], "big", signed=True) == 120
                    and int.from_bytes(frame[7:9], "big", signed=True) == 60
                    for frame in frames
                )
                time.sleep(0.01)
            self.assertTrue(motion_seen, "authorized fresh command was not emitted")

            process.stdin.write("STOP\n")
            process.stdin.flush()
            time.sleep(0.08)
            drain_frames()
            self.assertEqual(
                int.from_bytes(frames[-1][3:5], "big", signed=True),
                0,
                "STOP must return the wire command to exact zero",
            )

            process.stdin.write("QUIT\n")
            process.stdin.flush()
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertEqual(stdout, "")
            self.assertIn("final_zero_host_write_complete=true", stderr)
            self.assertIn("controller_ack_available=false", stderr)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            if config_path is not None:
                os.unlink(config_path)
            fixture.close()

    def test_stop_during_disconnect_does_not_cancel_reconnect(self):
        fixture = PtyFixture()
        process = None
        config_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", delete=False
            ) as config_file:
                config_file.write(runtime_config(fixture.slave_path))
                config_path = config_file.name

            process = subprocess.Popen(
                [
                    self.cli,
                    "--config",
                    config_path,
                    "--run",
                    "--acknowledge-unverified-protocol",
                    "--enable-actuation",
                    "--operator-confirmation",
                    ACTUATION_CONFIRMATION,
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIsNotNone(process.stdin)

            startup = bytearray()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and len(startup) < 11:
                try:
                    startup.extend(os.read(fixture.master, 4096))
                except BlockingIOError:
                    pass
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                time.sleep(0.01)
            self.assertGreaterEqual(len(startup), 11)
            self.assertTrue(valid_command_frame(bytes(startup[:11])))

            fixture.disconnect()
            time.sleep(0.15)
            process.stdin.write("STOP\n")
            process.stdin.flush()
            time.sleep(0.20)
            self.assertIsNone(
                process.poll(), "a fail-safe STOP must not terminate reconnect"
            )

            fixture.replace_at_same_path()
            reconnected = bytearray()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and len(reconnected) < 11:
                try:
                    reconnected.extend(os.read(fixture.master, 4096))
                except BlockingIOError:
                    pass
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                time.sleep(0.01)
            self.assertGreaterEqual(
                len(reconnected), 11, "reconnect did not emit an initial zero"
            )
            self.assertTrue(valid_command_frame(bytes(reconnected[:11])))
            self.assertEqual(
                int.from_bytes(reconnected[3:5], "big", signed=True), 0
            )

            process.stdin.write("QUIT\n")
            process.stdin.flush()
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertEqual(stdout, "")
            self.assertIn("serial_reopen=ok generation=2", stderr)
            self.assertIn("reauthorization_required=true", stderr)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            if process is not None:
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream is not None:
                        stream.close()
            if config_path is not None:
                os.unlink(config_path)
            fixture.close()

    def test_cli_rejects_signed_text_for_unsigned_fields(self):
        for invalid_line in (
            "AUTH -1\n",
            "CMD -1 1 2 0.0 0.0\n",
            "RESET_ESTOP -1\n",
        ):
            with self.subTest(line=invalid_line.strip()):
                fixture = PtyFixture()
                process = None
                config_path = None
                try:
                    with tempfile.NamedTemporaryFile(
                        mode="w", encoding="utf-8", delete=False
                    ) as config_file:
                        config_file.write(runtime_config(fixture.slave_path))
                        config_path = config_file.name
                    process = subprocess.Popen(
                        [
                            self.cli,
                            "--config",
                            config_path,
                            "--run",
                            "--acknowledge-unverified-protocol",
                            "--enable-actuation",
                            "--operator-confirmation",
                            ACTUATION_CONFIRMATION,
                        ],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                    )
                    self.assertIsNotNone(process.stdin)
                    startup = bytearray()
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and len(startup) < 11:
                        try:
                            startup.extend(os.read(fixture.master, 4096))
                        except BlockingIOError:
                            pass
                        except OSError as error:
                            if error.errno != errno.EIO:
                                raise
                        time.sleep(0.01)
                    self.assertGreaterEqual(len(startup), 11)
                    process.stdin.write(invalid_line)
                    process.stdin.flush()
                    stdout, stderr = process.communicate(timeout=3)
                    self.assertEqual(process.returncode, 7, stderr)
                    self.assertEqual(stdout, "")
                    self.assertIn("input_error=invalid_line", stderr)
                    self.assertIn("final_zero_host_write_complete=true", stderr)
                finally:
                    if process is not None and process.poll() is None:
                        process.kill()
                        process.wait(timeout=2)
                    if process is not None:
                        for stream in (
                            process.stdin,
                            process.stdout,
                            process.stderr,
                        ):
                            if stream is not None:
                                stream.close()
                    if config_path is not None:
                        os.unlink(config_path)
                    fixture.close()


if __name__ == "__main__":
    unittest.main()
