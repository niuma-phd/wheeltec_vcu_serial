#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Runtime contract test for the no-device, all-gates-disabled mode."""

import threading
import unittest

import rospy
import rostest

from wheeltec_vcu_serial_ros1.msg import AdapterState, DriveCommand
from wheeltec_vcu_serial_ros1.srv import Authorize, AuthorizeRequest


class OfflineNodeContractTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._state_event = threading.Event()
        self._latest_state = None
        self._state_count = 0
        self._state_subscriber = rospy.Subscriber(
            "/offline_adapter_under_test/adapter_state",
            AdapterState,
            self._on_state,
            queue_size=10,
        )
        self._command_publisher = rospy.Publisher(
            "/offline_adapter_under_test/drive_command",
            DriveCommand,
            queue_size=1,
        )

    def _on_state(self, message):
        with self._lock:
            self._latest_state = message
            self._state_count += 1
        self._state_event.set()

    def _snapshot(self):
        with self._lock:
            return self._latest_state, self._state_count

    def test_default_node_stays_offline_and_rejects_authorization(self):
        self.assertTrue(self._state_event.wait(5.0), "no adapter state received")
        first_state, first_count = self._snapshot()
        self.assertIsNotNone(first_state)
        self.assertEqual("offline", first_state.session_state)
        self.assertFalse(first_state.actuation_enabled)
        self.assertFalse(first_state.connected)
        self.assertEqual(0, first_state.connection_generation)
        self.assertFalse(first_state.authorized)
        self.assertFalse(first_state.software_estop_latched)
        self.assertFalse(first_state.vcu_ack_available)
        self.assertFalse(first_state.source_time_available)

        service_name = "/offline_adapter_under_test/authorize"
        rospy.wait_for_service(service_name, timeout=5.0)
        authorize = rospy.ServiceProxy(service_name, Authorize)
        response = authorize(AuthorizeRequest(token=1))
        self.assertFalse(response.accepted)
        self.assertEqual("actuation_disabled", response.reason)

        command = DriveCommand()
        command.sequence_id = 1
        command.linear_speed_mps = 0.1
        command.yaw_rate_radps = 0.0
        command.valid_for = rospy.Duration.from_sec(0.1)
        self._command_publisher.publish(command)

        rospy.sleep(1.2)
        later_state, later_count = self._snapshot()
        self.assertGreater(later_count, first_count)
        self.assertEqual("offline", later_state.session_state)
        self.assertFalse(later_state.actuation_enabled)
        self.assertFalse(later_state.connected)
        self.assertEqual(0, later_state.connection_generation)
        self.assertFalse(later_state.authorized)


if __name__ == "__main__":
    rospy.init_node("offline_node_contract_test", anonymous=True)
    rostest.rosrun(
        "wheeltec_vcu_serial_ros1",
        "offline_node_contract",
        OfflineNodeContractTest,
    )
