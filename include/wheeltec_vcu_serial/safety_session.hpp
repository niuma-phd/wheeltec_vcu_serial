// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <limits>

#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial/transport.hpp"

namespace wheeltec_vcu_serial {

struct SafetyConfig {
  // Deliberately invalid until supplied by the application.  The accepted
  // domain is finite and 0 < max_linear_speed_mps < 6.0.
  double max_linear_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double max_abs_yaw_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  std::int64_t max_command_age_ns{100000000LL};
  std::int64_t feedback_timeout_ns{150000000LL};
  std::int64_t transmit_period_ns{20000000LL};
  std::int64_t write_timeout_ns{10000000LL};
  std::int64_t zero_retry_interval_ns{20000000LL};
  std::uint32_t maximum_zero_attempts{3U};
  std::uint32_t fresh_commands_required{3U};
};

bool safetyConfigIsValid(const SafetyConfig& config) noexcept;

struct TimedMotionCommand {
  std::uint64_t sequence_id{0U};
  std::int64_t created_monotonic_ns{0};
  std::int64_t deadline_monotonic_ns{0};
  MotionCommand motion{};
};

enum class SessionState : std::uint8_t {
  kConfigurationInvalid = 0,
  kDisconnected,
  kConnectedInhibited,
  kRecovering,
  kActive,
  kStopPending,
  kEmergencyStopLatched,
  // Zero retry budget is exhausted for this transport generation.  Only a
  // new generation can start another initial-zero episode.
  kFaulted,
};

enum class AuthorizationStatus : std::uint8_t {
  kAuthorized = 0,
  kConfigurationInvalid,
  kDisconnected,
  kInitialZeroPending,
  kFeedbackUnavailable,
  kEmergencyStopLatched,
  kFaulted,
  kTokenInvalid,
  kTimestampInvalid,
};

enum class SubmissionStatus : std::uint8_t {
  kAccepted = 0,
  kRecoveryPending,
  kStopAccepted,
  kConfigurationInvalid,
  kDisconnected,
  kNotAuthorized,
  kSequenceInvalid,
  kTimestampInvalid,
  kStale,
  kPredatesAuthorization,
  kFeedbackUnavailable,
  kEncodingRejected,
  kEmergencyStopLatched,
  kFaulted,
};

enum class FeedbackObservationStatus : std::uint8_t {
  kAcceptedControlAllowed = 0,
  kAcceptedControlInhibited,
  kFrameInvalid,
  kTimestampInvalid,
  kDisconnected,
  kConfigurationInvalid,
};

struct FeedbackObservationResult {
  FeedbackObservationStatus status{
      FeedbackObservationStatus::kConfigurationInvalid};
  bool valid_frame{false};
  bool control_allowed{false};
};

enum class CycleAction : std::uint8_t {
  kNoAction = 0,
  kTransmitRateLimited,
  kMotionHostWriteComplete,
  kZeroHostWriteComplete,
  kZeroHostWriteFailed,
  kZeroRetryPending,
  kZeroRetriesExhausted,
  kDisconnected,
  kConfigurationInvalid,
  kClockInvalid,
};

struct CycleResult {
  CycleAction action{CycleAction::kNoAction};
  SessionState state{SessionState::kConfigurationInvalid};
  TransportStatus transport_status{TransportStatus::kOk};
  bool motion_write_attempted{false};
  bool motion_host_write_complete{false};
  bool zero_write_attempted{false};
  bool zero_host_write_complete{false};
  bool outcome_uncertain{false};
  std::uint32_t zero_attempts{0U};
};

// Single-threaded safety state machine.  Callers must serialize every method.
// Destruction closes no transport and makes no claim that a zero frame reached
// or was accepted by the VCU; orderly shutdown must request stop/disarm and
// continue tick() through the bounded zero result.
class SafetySession final {
 public:
  SafetySession(const SafetyConfig& config, ITransport* transport);
  ~SafetySession() = default;

  SafetySession(const SafetySession&) = delete;
  SafetySession& operator=(const SafetySession&) = delete;

  AuthorizationStatus authorize(std::uint64_t authorization_token,
                                std::int64_t authorization_monotonic_ns);
  bool disarm(std::int64_t now_monotonic_ns);
  bool requestStop(std::int64_t now_monotonic_ns);

  SubmissionStatus submit(const TimedMotionCommand& command,
                          std::int64_t receipt_monotonic_ns);
  FeedbackObservationResult observeFeedback(
      const FeedbackFrame& frame, std::int64_t receipt_monotonic_ns);

  void latchEmergencyStop(std::int64_t now_monotonic_ns);
  bool resetEmergencyStop(std::uint64_t reset_token,
                          std::int64_t now_monotonic_ns);

  CycleResult tick(std::int64_t now_monotonic_ns);

  bool configurationValid() const noexcept { return configuration_valid_; }
  SessionState state() const noexcept { return state_; }
  bool authorized() const noexcept;
  bool emergencyStopLatched() const noexcept {
    return emergency_stop_latched_;
  }
  std::uint64_t connectionGeneration() const noexcept {
    return connection_generation_;
  }
  std::uint64_t highestObservedSequence() const noexcept {
    return highest_sequence_;
  }
  std::uint64_t highestAuthorizationToken() const noexcept {
    return highest_authorization_token_;
  }
  std::uint32_t consecutiveFreshCommands() const noexcept {
    return consecutive_fresh_commands_;
  }

 private:
  enum class ConnectionSync : std::uint8_t {
    kUnchanged = 0,
    kNewGeneration,
    kDisconnected,
  };

  ConnectionSync synchronizeConnection();
  bool observeMonotonicTime(std::int64_t now_monotonic_ns);
  void handleClockFailure();
  void handleDisconnect();
  void handleNewConnection(std::uint64_t generation);
  void revokeAuthorization();
  void clearMotionIntent();
  void beginZeroEpisode(SessionState state_after_zero,
                        std::int64_t earliest_attempt_ns);
  CycleResult attemptZero(std::int64_t now_monotonic_ns,
                          CycleResult result);
  bool feedbackIsFresh(std::int64_t now_monotonic_ns) const noexcept;
  bool commandIsFresh(std::int64_t now_monotonic_ns) const noexcept;
  std::int64_t boundedWriteDeadline(
      std::int64_t now_monotonic_ns,
      std::int64_t command_deadline_ns) const noexcept;

  SafetyConfig config_;
  ITransport* transport_{nullptr};
  bool configuration_valid_{false};
  SessionState state_{SessionState::kConfigurationInvalid};

  bool connection_known_{false};
  std::uint64_t connection_generation_{0U};
  std::int64_t last_observed_monotonic_ns_{0};

  bool authorization_active_{false};
  std::uint64_t highest_authorization_token_{0U};
  std::uint64_t authorized_generation_{0U};
  std::int64_t authorization_monotonic_ns_{0};

  std::uint64_t highest_sequence_{0U};
  std::uint32_t consecutive_fresh_commands_{0U};
  bool latest_command_valid_{false};
  TimedMotionCommand latest_command_{};
  CommandFrame latest_frame_{};
  std::int64_t latest_command_receipt_ns_{0};
  std::int64_t last_motion_host_write_completion_ns_{0};

  bool feedback_allowed_seen_{false};
  std::int64_t latest_allowed_feedback_receipt_ns_{0};

  bool zero_episode_active_{false};
  SessionState state_after_zero_{SessionState::kConnectedInhibited};
  std::uint32_t zero_attempts_{0U};
  std::int64_t next_zero_attempt_ns_{0};

  bool emergency_stop_latched_{false};
  std::uint64_t emergency_stop_latch_generation_{0U};
  std::uint64_t highest_estop_reset_token_{0U};
};

}  // namespace wheeltec_vcu_serial
