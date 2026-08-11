// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial_ros2/adapter_logic.hpp"

#include <limits>

namespace wheeltec_vcu_serial_ros2 {

GateDecision evaluateActuationGates(const ActuationGates& gates) noexcept {
  const bool confirmation_empty = gates.operator_confirmation.empty();
  if (!gates.acknowledge_unverified_protocol && !gates.enable_actuation &&
      confirmation_empty) {
    return GateDecision::kOffline;
  }
  if (gates.acknowledge_unverified_protocol && gates.enable_actuation &&
      gates.operator_confirmation == kActuationConfirmation) {
    return GateDecision::kEnabled;
  }
  return GateDecision::kInvalidPartialConfiguration;
}

bool boundedDurationNanoseconds(std::int32_t sec, std::uint32_t nanosec,
                                std::int64_t maximum_ns,
                                std::int64_t* result_ns) noexcept {
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  if (result_ns == nullptr || sec < 0 || nanosec >= 1000000000U ||
      maximum_ns <= 0) {
    return false;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(sec);
  if (seconds >
      (std::numeric_limits<std::int64_t>::max() -
       static_cast<std::int64_t>(nanosec)) /
          kNanosecondsPerSecond) {
    return false;
  }
  const std::int64_t duration_ns =
      seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(nanosec);
  if (duration_ns <= 0 || duration_ns > maximum_ns) {
    return false;
  }
  *result_ns = duration_ns;
  return true;
}

bool checkedAddNanoseconds(std::int64_t receipt_ns,
                           std::int64_t duration_ns,
                           std::int64_t* deadline_ns) noexcept {
  if (deadline_ns == nullptr || receipt_ns <= 0 || duration_ns <= 0 ||
      receipt_ns > std::numeric_limits<std::int64_t>::max() - duration_ns) {
    return false;
  }
  *deadline_ns = receipt_ns + duration_ns;
  return true;
}

bool submissionWasAccepted(
    wheeltec_vcu_serial::SubmissionStatus status) noexcept {
  return status == wheeltec_vcu_serial::SubmissionStatus::kAccepted ||
         status == wheeltec_vcu_serial::SubmissionStatus::kRecoveryPending ||
         status == wheeltec_vcu_serial::SubmissionStatus::kStopAccepted;
}

BoundedZeroResult runBoundedZeroAttempts(
    std::uint32_t maximum_attempts,
    const std::function<FinalZeroAttemptDisposition()>& attempt,
    const std::function<void()>& between_attempts) {
  BoundedZeroResult result;
  for (std::uint32_t attempt_number = 0U;
       attempt_number < maximum_attempts; ++attempt_number) {
    ++result.attempts_made;
    const FinalZeroAttemptDisposition disposition = attempt();
    if (disposition == FinalZeroAttemptDisposition::kHostWriteComplete) {
      result.host_write_complete = true;
      break;
    }
    if (disposition == FinalZeroAttemptDisposition::kAbort ||
        attempt_number + 1U == maximum_attempts) {
      break;
    }
    between_attempts();
  }
  return result;
}

}  // namespace wheeltec_vcu_serial_ros2
