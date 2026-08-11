#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Small branch policy check; it never opens a device."""

from pathlib import Path
import sys
from xml.etree import ElementTree


def message_fields(path: Path):
    fields = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            fields.append(line)
    return fields


def main() -> int:
    package = Path(__file__).resolve().parents[1] / "wheeltec_vcu_serial_ros1"
    source_suffixes = {
        ".cpp", ".hpp", ".h", ".launch", ".msg", ".srv", ".test"
    }
    failures = []
    for path in sorted(package.rglob("*")):
        if not path.is_file() or path.suffix not in source_suffixes:
            continue
        text = path.read_text(encoding="utf-8")
        if "SPDX-License-Identifier: Apache-2.0" not in text:
            failures.append(f"missing SPDX identifier: {path}")
        if "/cmd" + "_vel" in text:
            failures.append(f"forbidden command subscription contract: {path}")
        if "/dev/wheeltec" + "_controller" in text:
            failures.append(f"real hardware alias embedded in source: {path}")

    expected_feedback = [
        "time receipt_time",
        "uint8 composite_stop_flag_raw",
        "bool control_allowed",
        "bool control_inhibited",
        "float64 linear_speed_mps",
        "float64 lateral_speed_mps",
        "float64 yaw_rate_radps",
        "float64[3] linear_acceleration_mps2",
        "float64[3] angular_velocity_radps",
        "float64 supply_voltage_v",
        "bool vcu_ack_available",
        "bool source_time_available",
    ]
    expected_state = [
        "time receipt_time",
        "string session_state",
        "bool actuation_enabled",
        "bool connected",
        "uint64 connection_generation",
        "bool authorized",
        "bool software_estop_latched",
        "bool vcu_ack_available",
        "bool source_time_available",
    ]
    if message_fields(package / "msg/Feedback.msg") != expected_feedback:
        failures.append("Feedback.msg does not match the maintained contract")
    if message_fields(package / "msg/AdapterState.msg") != expected_state:
        failures.append("AdapterState.msg does not match the maintained contract")

    launch_root = ElementTree.parse(package / "launch/adapter.launch").getroot()
    launch_args = {
        item.attrib.get("name"): item.attrib.get("default")
        for item in launch_root.findall("arg")
    }
    expected_offline_args = {
        "acknowledge_unverified_protocol": "false",
        "enable_actuation": "false",
        "operator_confirmation": "",
    }
    for name, expected in expected_offline_args.items():
        if launch_args.get(name) != expected:
            failures.append(
                f"default launch must keep {name}={expected!r} for offline mode"
            )
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print("ROS 1 boundary policy checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
