// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial_ros1/adapter_support.hpp"

#include <cmath>
#include <limits>

namespace core = wheeltec_vcu_serial;

namespace wheeltec_vcu_serial_ros1 {
namespace {

std::int64_t millisecondsToNanoseconds(std::int64_t milliseconds) {
  return milliseconds * 1000000LL;
}

}  // namespace

ActuationGateMode classifyActuationGate(const ActuationGate& gate,
                                        std::string* reason) {
  const bool all_disabled = !gate.acknowledge_unverified_protocol &&
                            !gate.enable_actuation &&
                            gate.operator_confirmation.empty();
  if (all_disabled) {
    if (reason != nullptr) {
      *reason = "offline";
    }
    return ActuationGateMode::kOffline;
  }

  const bool all_enabled = gate.acknowledge_unverified_protocol &&
                           gate.enable_actuation &&
                           gate.operator_confirmation ==
                               kActuationConfirmation;
  if (all_enabled) {
    if (reason != nullptr) {
      *reason = "actuation";
    }
    return ActuationGateMode::kActuation;
  }

  if (reason != nullptr) {
    *reason = "partial gates or invalid operator_confirmation";
  }
  return ActuationGateMode::kInvalid;
}

core::SafetyConfig makeSafetyConfig(const core::RuntimeConfig& config) {
  core::SafetyConfig safety;
  safety.max_linear_speed_mps = config.max_linear_speed_mps;
  safety.max_abs_yaw_rate_radps = config.max_abs_yaw_rate_radps;
  safety.max_command_age_ns =
      millisecondsToNanoseconds(config.command_timeout_ms);
  safety.feedback_timeout_ns =
      millisecondsToNanoseconds(config.feedback_timeout_ms);
  safety.transmit_period_ns =
      millisecondsToNanoseconds(config.transmit_period_ms);
  safety.write_timeout_ns =
      millisecondsToNanoseconds(config.write_timeout_ms);
  safety.zero_retry_interval_ns =
      millisecondsToNanoseconds(config.zero_retry_interval_ms);
  safety.maximum_zero_attempts = config.zero_retry_attempts;
  safety.fresh_commands_required = config.fresh_commands_required;
  return safety;
}

DriveConversionResult convertDriveCommand(
    const DriveCommand& message, std::int64_t receipt_monotonic_ns,
    std::int64_t maximum_valid_for_ns) {
  DriveConversionResult result;
  if (receipt_monotonic_ns <= 0 || maximum_valid_for_ns <= 0) {
    result.reason = "invalid monotonic receipt or watchdog duration";
    return result;
  }
  if (message.sequence_id == 0U) {
    result.reason = "sequence_id must be nonzero";
    return result;
  }
  const std::int64_t valid_for_ns = message.valid_for.toNSec();
  if (valid_for_ns <= 0 || valid_for_ns > maximum_valid_for_ns) {
    result.reason = "valid_for must be positive and no greater than command_timeout_ms";
    return result;
  }
  if (receipt_monotonic_ns >
      std::numeric_limits<std::int64_t>::max() - valid_for_ns) {
    result.reason = "command deadline overflows monotonic nanoseconds";
    return result;
  }
  if (!std::isfinite(message.linear_speed_mps) ||
      !std::isfinite(message.yaw_rate_radps)) {
    result.reason = "motion fields must be finite";
    return result;
  }

  result.command.sequence_id = message.sequence_id;
  result.command.created_monotonic_ns = receipt_monotonic_ns;
  result.command.deadline_monotonic_ns = receipt_monotonic_ns + valid_for_ns;
  result.command.motion.linear_speed_mps = message.linear_speed_mps;
  result.command.motion.lateral_speed_mps = 0.0;
  result.command.motion.yaw_rate_radps = message.yaw_rate_radps;
  result.valid = true;
  result.reason = "accepted";
  return result;
}

Feedback makeFeedbackMessage(const core::FeedbackData& feedback,
                             const ros::Time& receipt_time) {
  Feedback message;
  message.receipt_time = receipt_time;
  message.composite_stop_flag_raw = feedback.composite_stop_flag_raw;
  message.control_allowed = feedback.control_allowed;
  message.control_inhibited = feedback.control_inhibited;
  message.linear_speed_mps = feedback.linear_speed_mps;
  message.lateral_speed_mps = feedback.lateral_speed_mps;
  message.yaw_rate_radps = feedback.yaw_rate_radps;
  for (std::size_t index = 0U; index < 3U; ++index) {
    message.linear_acceleration_mps2[index] =
        feedback.linear_acceleration_mps2[index];
    message.angular_velocity_radps[index] =
        feedback.angular_velocity_radps[index];
  }
  message.supply_voltage_v = feedback.supply_voltage_v;
  message.vcu_ack_available = false;
  message.source_time_available = false;
  return message;
}

const char* authorizationStatusName(core::AuthorizationStatus status) noexcept {
  switch (status) {
    case core::AuthorizationStatus::kAuthorized:
      return "authorized";
    case core::AuthorizationStatus::kConfigurationInvalid:
      return "configuration_invalid";
    case core::AuthorizationStatus::kDisconnected:
      return "disconnected";
    case core::AuthorizationStatus::kInitialZeroPending:
      return "initial_zero_pending";
    case core::AuthorizationStatus::kFeedbackUnavailable:
      return "feedback_unavailable";
    case core::AuthorizationStatus::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case core::AuthorizationStatus::kFaulted:
      return "faulted";
    case core::AuthorizationStatus::kTokenInvalid:
      return "token_invalid";
    case core::AuthorizationStatus::kTimestampInvalid:
      return "timestamp_invalid";
  }
  return "unknown";
}

const char* submissionStatusName(core::SubmissionStatus status) noexcept {
  switch (status) {
    case core::SubmissionStatus::kAccepted:
      return "accepted";
    case core::SubmissionStatus::kRecoveryPending:
      return "recovery_pending";
    case core::SubmissionStatus::kStopAccepted:
      return "stop_accepted";
    case core::SubmissionStatus::kConfigurationInvalid:
      return "configuration_invalid";
    case core::SubmissionStatus::kDisconnected:
      return "disconnected";
    case core::SubmissionStatus::kNotAuthorized:
      return "not_authorized";
    case core::SubmissionStatus::kSequenceInvalid:
      return "sequence_invalid";
    case core::SubmissionStatus::kTimestampInvalid:
      return "timestamp_invalid";
    case core::SubmissionStatus::kStale:
      return "stale";
    case core::SubmissionStatus::kPredatesAuthorization:
      return "predates_authorization";
    case core::SubmissionStatus::kFeedbackUnavailable:
      return "feedback_unavailable";
    case core::SubmissionStatus::kEncodingRejected:
      return "encoding_rejected";
    case core::SubmissionStatus::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case core::SubmissionStatus::kFaulted:
      return "faulted";
  }
  return "unknown";
}

const char* sessionStateName(core::SessionState state) noexcept {
  switch (state) {
    case core::SessionState::kConfigurationInvalid:
      return "configuration_invalid";
    case core::SessionState::kDisconnected:
      return "disconnected";
    case core::SessionState::kConnectedInhibited:
      return "connected_inhibited";
    case core::SessionState::kRecovering:
      return "recovering";
    case core::SessionState::kActive:
      return "active";
    case core::SessionState::kStopPending:
      return "stop_pending";
    case core::SessionState::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case core::SessionState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

const char* transportStatusName(core::TransportStatus status) noexcept {
  switch (status) {
    case core::TransportStatus::kOk:
      return "ok";
    case core::TransportStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case core::TransportStatus::kDisconnected:
      return "disconnected";
    case core::TransportStatus::kWriteDisabled:
      return "write_disabled";
    case core::TransportStatus::kInvalidArgument:
      return "invalid_argument";
    case core::TransportStatus::kConfigurationError:
      return "configuration_error";
    case core::TransportStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

const char* cycleActionName(core::CycleAction action) noexcept {
  switch (action) {
    case core::CycleAction::kNoAction:
      return "no_action";
    case core::CycleAction::kTransmitRateLimited:
      return "transmit_rate_limited";
    case core::CycleAction::kMotionHostWriteComplete:
      return "motion_host_write_complete";
    case core::CycleAction::kZeroHostWriteComplete:
      return "zero_host_write_complete";
    case core::CycleAction::kZeroHostWriteFailed:
      return "zero_host_write_failed";
    case core::CycleAction::kZeroRetryPending:
      return "zero_retry_pending";
    case core::CycleAction::kZeroRetriesExhausted:
      return "zero_retries_exhausted";
    case core::CycleAction::kDisconnected:
      return "disconnected";
    case core::CycleAction::kConfigurationInvalid:
      return "configuration_invalid";
    case core::CycleAction::kClockInvalid:
      return "clock_invalid";
  }
  return "unknown";
}

}  // namespace wheeltec_vcu_serial_ros1
