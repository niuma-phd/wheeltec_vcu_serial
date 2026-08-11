// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <ros/duration.h>
#include <ros/time.h>

#include "wheeltec_vcu_serial/config.hpp"
#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial_ros1/AdapterState.h"
#include "wheeltec_vcu_serial_ros1/DriveCommand.h"
#include "wheeltec_vcu_serial_ros1/adapter_support.hpp"

namespace adapter = wheeltec_vcu_serial_ros1;
namespace core = wheeltec_vcu_serial;

TEST(ActuationGate, AllDisabledIsOfflineAndAllEnabledIsActuation) {
  adapter::ActuationGate gate;
  std::string reason;
  EXPECT_EQ(adapter::ActuationGateMode::kOffline,
            adapter::classifyActuationGate(gate, &reason));
  EXPECT_EQ("offline", reason);

  gate.acknowledge_unverified_protocol = true;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate, &reason));

  gate.enable_actuation = true;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate, &reason));

  gate.operator_confirmation = adapter::kActuationConfirmation;
  EXPECT_EQ(adapter::ActuationGateMode::kActuation,
            adapter::classifyActuationGate(gate, &reason));
  EXPECT_EQ("actuation", reason);
}

TEST(ActuationGate, EveryPartialOrMismatchedCombinationIsInvalid) {
  adapter::ActuationGate gate;
  gate.operator_confirmation = adapter::kActuationConfirmation;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate));

  gate.operator_confirmation.clear();
  gate.enable_actuation = true;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate));

  gate.acknowledge_unverified_protocol = true;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate));

  gate.operator_confirmation = "WRONG_CONFIRMATION";
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate));

  gate.enable_actuation = false;
  gate.operator_confirmation = adapter::kActuationConfirmation;
  EXPECT_EQ(adapter::ActuationGateMode::kInvalid,
            adapter::classifyActuationGate(gate));
}

TEST(DriveConversion, UsesReceiptTimeAndDuration) {
  adapter::DriveCommand message;
  message.sequence_id = 41U;
  message.linear_speed_mps = -0.2;
  message.yaw_rate_radps = 0.3;
  message.valid_for = ros::Duration(0, 80000000);

  const adapter::DriveConversionResult result =
      adapter::convertDriveCommand(message, 1000000000LL, 250000000LL);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(41U, result.command.sequence_id);
  EXPECT_EQ(1000000000LL, result.command.created_monotonic_ns);
  EXPECT_EQ(1080000000LL, result.command.deadline_monotonic_ns);
  EXPECT_DOUBLE_EQ(-0.2, result.command.motion.linear_speed_mps);
  EXPECT_DOUBLE_EQ(0.0, result.command.motion.lateral_speed_mps);
  EXPECT_DOUBLE_EQ(0.3, result.command.motion.yaw_rate_radps);
}

TEST(DriveConversion, RejectsInvalidInputsFailClosed) {
  adapter::DriveCommand message;
  message.sequence_id = 1U;
  message.valid_for = ros::Duration(0, 250000001);
  EXPECT_FALSE(adapter::convertDriveCommand(
                   message, 10LL, 250000000LL).valid);

  message.valid_for = ros::Duration(0, 1);
  message.sequence_id = 0U;
  EXPECT_FALSE(adapter::convertDriveCommand(message, 10LL, 10LL).valid);

  message.sequence_id = 2U;
  message.linear_speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::convertDriveCommand(message, 10LL, 10LL).valid);

  message.linear_speed_mps = 0.0;
  message.yaw_rate_radps = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(adapter::convertDriveCommand(message, 10LL, 10LL).valid);
}

TEST(Configuration, OfflineExampleIsValidButSelectsNoDevice) {
  const std::string path =
      std::string(WHEELTEC_ROS1_PACKAGE_DIR) + "/config/offline.ini";
  const core::ConfigResult loaded = core::loadRuntimeConfig(path, false);
  ASSERT_TRUE(loaded.ok()) << loaded.message;
  EXPECT_TRUE(loaded.config.device_path.empty());
  EXPECT_TRUE(core::isValidMaxLinearSpeed(
      loaded.config.max_linear_speed_mps));
  EXPECT_FALSE(core::loadRuntimeConfig(path, true).ok());
}

TEST(FeedbackConversion, MarksAbsentControllerMetadataExplicitly) {
  core::FeedbackData feedback;
  feedback.composite_stop_flag_raw = 0U;
  feedback.control_allowed = true;
  feedback.control_inhibited = false;
  feedback.linear_speed_mps = -0.125;
  feedback.lateral_speed_mps = 0.0;
  feedback.yaw_rate_radps = 0.25;
  feedback.linear_acceleration_mps2 = {{1.0, 2.0, 3.0}};
  feedback.angular_velocity_radps = {{4.0, 5.0, 6.0}};
  feedback.supply_voltage_v = 24.1;

  const adapter::Feedback message =
      adapter::makeFeedbackMessage(feedback, ros::Time(12U, 34U));
  EXPECT_EQ(ros::Time(12U, 34U), message.receipt_time);
  EXPECT_TRUE(message.control_allowed);
  EXPECT_FALSE(message.control_inhibited);
  EXPECT_DOUBLE_EQ(-0.125, message.linear_speed_mps);
  EXPECT_DOUBLE_EQ(3.0, message.linear_acceleration_mps2[2]);
  EXPECT_DOUBLE_EQ(6.0, message.angular_velocity_radps[2]);
  EXPECT_FALSE(message.vcu_ack_available);
  EXPECT_FALSE(message.source_time_available);
}

TEST(AdapterStateContract, ExposesUnifiedSafetyAndMetadataFields) {
  adapter::AdapterState message;
  message.receipt_time = ros::Time(1U, 2U);
  message.session_state = "offline";
  message.actuation_enabled = false;
  message.connected = false;
  message.connection_generation = 0U;
  message.authorized = false;
  message.software_estop_latched = true;
  message.vcu_ack_available = false;
  message.source_time_available = false;

  EXPECT_EQ("offline", message.session_state);
  EXPECT_FALSE(message.actuation_enabled);
  EXPECT_TRUE(message.software_estop_latched);
  EXPECT_FALSE(message.vcu_ack_available);
  EXPECT_FALSE(message.source_time_available);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
