// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>

#include <ros/time.h>

#include "wheeltec_vcu_serial/config.hpp"
#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial/safety_session.hpp"
#include "wheeltec_vcu_serial/transport.hpp"
#include "wheeltec_vcu_serial_ros1/DriveCommand.h"
#include "wheeltec_vcu_serial_ros1/Feedback.h"

namespace wheeltec_vcu_serial_ros1 {

constexpr const char* kActuationConfirmation =
    "I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK";

struct ActuationGate {
  bool acknowledge_unverified_protocol{false};
  bool enable_actuation{false};
  std::string operator_confirmation;
};

enum class ActuationGateMode : std::uint8_t {
  kOffline = 0,
  kActuation,
  kInvalid,
};

// Exactly all-disabled with an empty confirmation is a deliberate offline
// mode. Exactly all-enabled with the confirmation phrase permits actuation.
// Every partial or mismatched combination is invalid and must fail closed.
ActuationGateMode classifyActuationGate(const ActuationGate& gate,
                                        std::string* reason = nullptr);

wheeltec_vcu_serial::SafetyConfig makeSafetyConfig(
    const wheeltec_vcu_serial::RuntimeConfig& config);

struct DriveConversionResult {
  bool valid{false};
  wheeltec_vcu_serial::TimedMotionCommand command{};
  std::string reason;
};

// Receipt time is CLOCK_MONOTONIC. valid_for starts at receipt and may not
// exceed the configured command watchdog. No ROS or sender time enters the
// core safety clock domain.
DriveConversionResult convertDriveCommand(
    const DriveCommand& message,
    std::int64_t receipt_monotonic_ns,
    std::int64_t maximum_valid_for_ns);

Feedback makeFeedbackMessage(
    const wheeltec_vcu_serial::FeedbackData& feedback,
    const ros::Time& receipt_time);

const char* authorizationStatusName(
    wheeltec_vcu_serial::AuthorizationStatus status) noexcept;
const char* submissionStatusName(
    wheeltec_vcu_serial::SubmissionStatus status) noexcept;
const char* sessionStateName(
    wheeltec_vcu_serial::SessionState state) noexcept;
const char* transportStatusName(
    wheeltec_vcu_serial::TransportStatus status) noexcept;
const char* cycleActionName(
    wheeltec_vcu_serial::CycleAction action) noexcept;

}  // namespace wheeltec_vcu_serial_ros1
