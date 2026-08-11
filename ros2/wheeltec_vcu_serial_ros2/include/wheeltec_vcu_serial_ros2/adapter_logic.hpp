// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "wheeltec_vcu_serial/safety_session.hpp"

namespace wheeltec_vcu_serial_ros2 {

constexpr const char* kActuationConfirmation =
    "I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK";

struct ActuationGates {
  bool acknowledge_unverified_protocol{false};
  bool enable_actuation{false};
  std::string operator_confirmation;
};

enum class GateDecision : std::uint8_t {
  kOffline = 0,
  kEnabled,
  kInvalidPartialConfiguration,
};

GateDecision evaluateActuationGates(const ActuationGates& gates) noexcept;

// Converts a builtin_interfaces/Duration representation to a positive bounded
// nanosecond interval. nanosec must be canonical (< 1e9).
bool boundedDurationNanoseconds(std::int32_t sec, std::uint32_t nanosec,
                                std::int64_t maximum_ns,
                                std::int64_t* result_ns) noexcept;

// Saturating-free addition used to construct a monotonic command deadline.
bool checkedAddNanoseconds(std::int64_t receipt_ns,
                           std::int64_t duration_ns,
                           std::int64_t* deadline_ns) noexcept;

// Recovery-pending and stop-accepted are successful state-machine outcomes,
// not rejected commands. This helper keeps middleware logging consistent with
// the core contract.
bool submissionWasAccepted(
    wheeltec_vcu_serial::SubmissionStatus status) noexcept;

enum class FinalZeroAttemptDisposition : std::uint8_t {
  kHostWriteComplete = 0,
  kRetry,
  kAbort,
};

struct BoundedZeroResult {
  bool host_write_complete{false};
  std::uint32_t attempts_made{0U};
};

// Runs at most maximum_attempts. between_attempts is called only before an
// actual retry, never after success, abort, or the last permitted attempt.
BoundedZeroResult runBoundedZeroAttempts(
    std::uint32_t maximum_attempts,
    const std::function<FinalZeroAttemptDisposition()>& attempt,
    const std::function<void()>& between_attempts);

}  // namespace wheeltec_vcu_serial_ros2
