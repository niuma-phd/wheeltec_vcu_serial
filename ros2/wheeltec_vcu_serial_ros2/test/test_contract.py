#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Static contract checks for the thin ROS 2 boundary."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(sys.argv[1]).resolve()
sys.argv = [sys.argv[0]]


class ContractTest(unittest.TestCase):
    def read(self, relative):
        return (PACKAGE_ROOT / relative).read_text(encoding="utf-8")

    def message_fields(self, relative):
        return [
            line.strip()
            for line in self.read(relative).splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]

    def test_drive_contract_is_vehicle_specific_and_receipt_relative(self):
        message = self.read("msg/DriveCommand.msg")
        self.assertIn("uint64 sequence_id", message)
        self.assertIn("float64 linear_speed_mps", message)
        self.assertIn("float64 yaw_rate_radps", message)
        self.assertIn("builtin_interfaces/Duration valid_for", message)
        self.assertIn("callback receipt", message)

    def test_feedback_marks_protocol_absences_explicitly(self):
        self.assertEqual(
            self.message_fields("msg/Feedback.msg"),
            [
                "builtin_interfaces/Time receipt_time",
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
            ],
        )
        source = self.read("src/adapter_node.cpp")
        self.assertIn(
            "message.composite_stop_flag_raw = "
            "frame.feedback.composite_stop_flag_raw",
            source,
        )
        self.assertIn("message.vcu_ack_available = false", source)
        self.assertIn("message.source_time_available = false", source)

    def test_adapter_state_contract_is_complete_and_ordered(self):
        self.assertEqual(
            self.message_fields("msg/AdapterState.msg"),
            [
                "builtin_interfaces/Time receipt_time",
                "string session_state",
                "bool actuation_enabled",
                "bool connected",
                "uint64 connection_generation",
                "bool authorized",
                "bool software_estop_latched",
                "bool vcu_ack_available",
                "bool source_time_available",
            ],
        )
        source = self.read("src/adapter_node.cpp")
        self.assertIn("state.vcu_ack_available = false", source)
        self.assertIn("state.source_time_available = false", source)

    def test_source_has_no_generic_velocity_subscription(self):
        source_tree = "\n".join(
            path.read_text(encoding="utf-8")
            for directory in ("src", "include", "launch")
            for path in sorted((PACKAGE_ROOT / directory).rglob("*"))
            if path.is_file()
        )
        forbidden_topic = "/cmd" + "_vel"
        self.assertNotIn(forbidden_topic, source_tree)
        self.assertNotIn("geometry_msgs", source_tree)
        self.assertIn('"~/drive_command"', source_tree)

    def test_launch_is_offline_and_device_free_by_default(self):
        launch = self.read("launch/wheeltec_vcu_serial.launch.py")
        self.assertIn('default_value="false"', launch)
        self.assertIn('default_value=""', launch)
        config = self.read("config/offline.ini")
        self.assertRegex(config, r"(?m)^device\s*=\s*$")

    def test_executor_and_core_boundary_are_explicit(self):
        main = self.read("src/main.cpp")
        cmake = self.read("CMakeLists.txt")
        self.assertIn("SingleThreadedExecutor", main)
        self.assertIn(
            "find_package(wheeltec_vcu_serial 0.1 CONFIG REQUIRED)", cmake
        )
        self.assertIn(
            "wheeltec_vcu_serial::wheeltec_vcu_serial", cmake
        )

    def test_device_open_is_after_all_ros_entities_and_has_cleanup(self):
        source = self.read("src/adapter_node.cpp")
        open_stage = source.index("  openActiveTransport();")
        self.assertLess(source.index("feedback_publisher_ ="), open_stage)
        self.assertLess(source.index("reset_estop_service_ ="), open_stage)
        self.assertLess(source.index("io_timer_ ="), open_stage)
        self.assertLess(
            source.index("publishState(core::monotonicNowNs(), true)"),
            open_stage,
        )
        open_method = source[
            source.index("void AdapterNode::openActiveTransport()") :
            source.index("void AdapterNode::onDriveCommand")
        ]
        self.assertIn("catch (...)", open_method)
        self.assertIn("(void)shutdownSafely();", open_method)
        self.assertIn("throw;", open_method)

    def test_shutdown_failure_is_observable_and_has_one_attempt_budget(self):
        source = self.read("src/adapter_node.cpp")
        shutdown = source[source.index("bool AdapterNode::shutdownSafely()") :]
        self.assertIn("runBoundedZeroAttempts(", shutdown)
        self.assertNotIn("session_->tick", shutdown)
        main = self.read("src/main.cpp")
        self.assertIn("if (!node->shutdownSafely())", main)
        self.assertIn("result = 9;", main)

    def test_offline_fail_safe_service_contract(self):
        source = self.read("src/adapter_node.cpp")
        stop = source[
            source.index("void AdapterNode::onStop") :
            source.index("void AdapterNode::onEmergencyStop")
        ]
        self.assertIn("response->success = true", stop)
        self.assertIn("local_stop_accepted_offline", stop)

        estop = source[
            source.index("void AdapterNode::onEmergencyStop") :
            source.index("void AdapterNode::onResetEstop")
        ]
        self.assertIn("local_software_estop_latched_ = true", estop)
        self.assertIn("response->success = true", estop)
        self.assertIn("software_emergency_stop_latched_offline_locally", estop)

        reset = source[
            source.index("void AdapterNode::onResetEstop") :
            source.index("void AdapterNode::tryReconnect")
        ]
        self.assertIn('response->reason = "adapter_offline"', reset)
        self.assertIn("response->accepted = false", reset)
        self.assertIn("local_software_estop_latched_ = false", reset)

    def test_graph_names_are_private_and_token_is_not_authentication(self):
        source = self.read("src/adapter_node.cpp")
        for name in (
            "feedback",
            "adapter_state",
            "drive_command",
            "authorize",
            "stop",
            "emergency_stop",
            "reset_emergency_stop",
        ):
            self.assertIn(f'"~/{name}"', source)
        readme = self.read("README.md")
        normalized_readme = " ".join(readme.split())
        self.assertIn("not a credential", normalized_readme)
        self.assertIn("trusted and isolated", normalized_readme)
        self.assertIn("deployment security", normalized_readme)


if __name__ == "__main__":
    unittest.main(verbosity=2)
