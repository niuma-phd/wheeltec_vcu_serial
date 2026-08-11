// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/safety_session.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>

namespace wheeltec_vcu_serial {
namespace {

bool commandIsExactZero(const MotionCommand& command) noexcept {
  return command.linear_speed_mps == 0.0 &&
         command.lateral_speed_mps == 0.0 &&
         command.yaw_rate_radps == 0.0;
}

std::int64_t saturatingAdd(std::int64_t value,
                           std::int64_t positive_delta) noexcept {
  if (positive_delta <= 0) {
    return value;
  }
  if (value > std::numeric_limits<std::int64_t>::max() - positive_delta) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + positive_delta;
}

IoResult exceptionIoResult(std::int64_t now_monotonic_ns) {
  IoResult result;
  result.status = TransportStatus::kIoError;
  result.os_error = EIO;
  result.outcome_uncertain = true;
  result.completion_monotonic_ns = now_monotonic_ns;
  return result;
}

}  // namespace

bool safetyConfigIsValid(const SafetyConfig& config) noexcept {
  constexpr std::int64_t kMaximumDurationNs = 60000000000LL;
  return isValidMaxLinearSpeed(config.max_linear_speed_mps) &&
         std::isfinite(config.max_abs_yaw_rate_radps) &&
         config.max_abs_yaw_rate_radps > 0.0 &&
         config.max_abs_yaw_rate_radps <= 32.767 &&
         config.max_command_age_ns > 0 &&
         config.max_command_age_ns <= kMaximumDurationNs &&
         config.feedback_timeout_ns > 0 &&
         config.feedback_timeout_ns <= kMaximumDurationNs &&
         config.transmit_period_ns > 0 && config.write_timeout_ns > 0 &&
         config.transmit_period_ns <= kMaximumDurationNs &&
         config.write_timeout_ns <= kMaximumDurationNs &&
         config.zero_retry_interval_ns > 0 &&
         config.zero_retry_interval_ns <= kMaximumDurationNs &&
         config.write_timeout_ns <= config.transmit_period_ns &&
         config.transmit_period_ns < config.max_command_age_ns &&
         config.transmit_period_ns < config.feedback_timeout_ns &&
         config.maximum_zero_attempts > 0U &&
         config.maximum_zero_attempts <= 20U &&
         config.fresh_commands_required > 0U &&
         config.fresh_commands_required <= 100U;
}

SafetySession::SafetySession(const SafetyConfig& config,
                             ITransport* transport)
    : config_(config),
      transport_(transport),
      configuration_valid_(transport != nullptr &&
                           safetyConfigIsValid(config)) {
  if (!configuration_valid_) {
    state_ = SessionState::kConfigurationInvalid;
    return;
  }
  (void)synchronizeConnection();
}

AuthorizationStatus SafetySession::authorize(
    std::uint64_t authorization_token,
    std::int64_t authorization_monotonic_ns) {
  if (!configuration_valid_) {
    return AuthorizationStatus::kConfigurationInvalid;
  }
  if (!observeMonotonicTime(authorization_monotonic_ns)) {
    handleClockFailure();
    return AuthorizationStatus::kTimestampInvalid;
  }
  const ConnectionSync connection = synchronizeConnection();
  if (connection == ConnectionSync::kDisconnected) {
    return AuthorizationStatus::kDisconnected;
  }
  if (emergency_stop_latched_) {
    return AuthorizationStatus::kEmergencyStopLatched;
  }
  if (state_ == SessionState::kFaulted) {
    return AuthorizationStatus::kFaulted;
  }
  if (zero_episode_active_ || connection == ConnectionSync::kNewGeneration) {
    return AuthorizationStatus::kInitialZeroPending;
  }
  if (authorization_token == 0U ||
      authorization_token <= highest_authorization_token_) {
    return AuthorizationStatus::kTokenInvalid;
  }
  if (!feedbackIsFresh(authorization_monotonic_ns)) {
    return AuthorizationStatus::kFeedbackUnavailable;
  }

  highest_authorization_token_ = authorization_token;
  authorization_active_ = true;
  authorized_generation_ = connection_generation_;
  authorization_monotonic_ns_ = authorization_monotonic_ns;
  clearMotionIntent();
  state_ = SessionState::kRecovering;
  return AuthorizationStatus::kAuthorized;
}

bool SafetySession::disarm(std::int64_t now_monotonic_ns) {
  if (!configuration_valid_) {
    return false;
  }
  if (!observeMonotonicTime(now_monotonic_ns)) {
    handleClockFailure();
    return false;
  }
  const ConnectionSync connection = synchronizeConnection();
  if (connection == ConnectionSync::kDisconnected) {
    // The local disarm intent is already enforced by handleDisconnect().  No
    // zero delivery can be claimed while the transport is absent.
    return true;
  }
  if (state_ == SessionState::kFaulted) {
    return false;
  }
  revokeAuthorization();
  clearMotionIntent();
  beginZeroEpisode(emergency_stop_latched_
                       ? SessionState::kEmergencyStopLatched
                       : SessionState::kConnectedInhibited,
                   now_monotonic_ns);
  return true;
}

bool SafetySession::requestStop(std::int64_t now_monotonic_ns) {
  if (!configuration_valid_) {
    return false;
  }
  if (!observeMonotonicTime(now_monotonic_ns)) {
    handleClockFailure();
    return false;
  }
  const ConnectionSync connection = synchronizeConnection();
  if (connection == ConnectionSync::kDisconnected) {
    // STOP is a valid local fail-safe intent even when no zero can currently
    // be written.  The next connection generation still starts with zero.
    return true;
  }
  if (state_ == SessionState::kFaulted) {
    return false;
  }
  clearMotionIntent();
  const SessionState after_zero =
      emergency_stop_latched_
          ? SessionState::kEmergencyStopLatched
          : (authorized() ? SessionState::kRecovering
                          : SessionState::kConnectedInhibited);
  beginZeroEpisode(after_zero, now_monotonic_ns);
  return true;
}

SubmissionStatus SafetySession::submit(
    const TimedMotionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  if (!configuration_valid_) {
    return SubmissionStatus::kConfigurationInvalid;
  }
  if (!observeMonotonicTime(receipt_monotonic_ns)) {
    handleClockFailure();
    return SubmissionStatus::kTimestampInvalid;
  }
  const ConnectionSync connection = synchronizeConnection();
  if (connection == ConnectionSync::kDisconnected) {
    return SubmissionStatus::kDisconnected;
  }
  if (emergency_stop_latched_) {
    return SubmissionStatus::kEmergencyStopLatched;
  }
  if (state_ == SessionState::kFaulted) {
    return SubmissionStatus::kFaulted;
  }
  if (command.sequence_id == 0U ||
      command.sequence_id <= highest_sequence_) {
    revokeAuthorization();
    clearMotionIntent();
    beginZeroEpisode(SessionState::kConnectedInhibited,
                     receipt_monotonic_ns);
    return SubmissionStatus::kSequenceInvalid;
  }
  // Consume every new sequence before later validation so a rejected command
  // cannot be replayed after reconnect or reauthorization.
  highest_sequence_ = command.sequence_id;

  if (command.created_monotonic_ns <= 0 ||
      command.created_monotonic_ns > receipt_monotonic_ns ||
      command.deadline_monotonic_ns <= command.created_monotonic_ns) {
    revokeAuthorization();
    clearMotionIntent();
    beginZeroEpisode(SessionState::kConnectedInhibited,
                     receipt_monotonic_ns);
    return SubmissionStatus::kTimestampInvalid;
  }
  if (receipt_monotonic_ns > command.deadline_monotonic_ns ||
      receipt_monotonic_ns - command.created_monotonic_ns >
          config_.max_command_age_ns) {
    clearMotionIntent();
    beginZeroEpisode(authorized() ? SessionState::kRecovering
                                  : SessionState::kConnectedInhibited,
                     receipt_monotonic_ns);
    return SubmissionStatus::kStale;
  }
  if (!authorized() || zero_episode_active_ ||
      connection == ConnectionSync::kNewGeneration) {
    clearMotionIntent();
    return SubmissionStatus::kNotAuthorized;
  }
  if (command.created_monotonic_ns <= authorization_monotonic_ns_) {
    clearMotionIntent();
    beginZeroEpisode(SessionState::kRecovering, receipt_monotonic_ns);
    return SubmissionStatus::kPredatesAuthorization;
  }

  if (!std::isfinite(command.motion.yaw_rate_radps) ||
      std::abs(command.motion.yaw_rate_radps) >
          config_.max_abs_yaw_rate_radps ||
      command.motion.lateral_speed_mps != 0.0) {
    revokeAuthorization();
    clearMotionIntent();
    beginZeroEpisode(SessionState::kConnectedInhibited,
                     receipt_monotonic_ns);
    return SubmissionStatus::kEncodingRejected;
  }

  const EncodeResult encoded =
      encodeCommand(command.motion, config_.max_linear_speed_mps);
  if (!encoded.ok()) {
    revokeAuthorization();
    clearMotionIntent();
    beginZeroEpisode(SessionState::kConnectedInhibited,
                     receipt_monotonic_ns);
    return SubmissionStatus::kEncodingRejected;
  }
  if (commandIsExactZero(command.motion)) {
    clearMotionIntent();
    beginZeroEpisode(SessionState::kRecovering, receipt_monotonic_ns);
    return SubmissionStatus::kStopAccepted;
  }
  if (!feedbackIsFresh(receipt_monotonic_ns)) {
    clearMotionIntent();
    beginZeroEpisode(SessionState::kRecovering, receipt_monotonic_ns);
    return SubmissionStatus::kFeedbackUnavailable;
  }

  if (latest_command_valid_) {
    const bool previous_is_stale =
        receipt_monotonic_ns > latest_command_.deadline_monotonic_ns ||
        receipt_monotonic_ns - latest_command_receipt_ns_ >
            config_.max_command_age_ns;
    if (previous_is_stale) {
      clearMotionIntent();
    } else if (command.created_monotonic_ns <=
               latest_command_.created_monotonic_ns) {
      revokeAuthorization();
      clearMotionIntent();
      beginZeroEpisode(SessionState::kConnectedInhibited,
                       receipt_monotonic_ns);
      return SubmissionStatus::kTimestampInvalid;
    }
  }

  latest_command_ = command;
  latest_frame_ = encoded.frame;
  latest_command_valid_ = true;
  latest_command_receipt_ns_ = receipt_monotonic_ns;
  if (consecutive_fresh_commands_ < config_.fresh_commands_required) {
    ++consecutive_fresh_commands_;
  }
  if (consecutive_fresh_commands_ >= config_.fresh_commands_required) {
    state_ = SessionState::kActive;
    return SubmissionStatus::kAccepted;
  }
  state_ = SessionState::kRecovering;
  return SubmissionStatus::kRecoveryPending;
}

FeedbackObservationResult SafetySession::observeFeedback(
    const FeedbackFrame& frame, std::int64_t receipt_monotonic_ns) {
  if (!configuration_valid_) {
    return {FeedbackObservationStatus::kConfigurationInvalid, false, false};
  }
  if (!observeMonotonicTime(receipt_monotonic_ns)) {
    handleClockFailure();
    return {FeedbackObservationStatus::kTimestampInvalid, false, false};
  }
  if (synchronizeConnection() == ConnectionSync::kDisconnected) {
    return {FeedbackObservationStatus::kDisconnected, false, false};
  }

  const FeedbackDecodeResult decoded = decodeFeedbackFrame(frame);
  if (!decoded.ok()) {
    return {FeedbackObservationStatus::kFrameInvalid, false, false};
  }
  if (decoded.feedback.control_allowed) {
    feedback_allowed_seen_ = true;
    latest_allowed_feedback_receipt_ns_ = receipt_monotonic_ns;
    return {FeedbackObservationStatus::kAcceptedControlAllowed, true, true};
  }

  feedback_allowed_seen_ = false;
  latest_allowed_feedback_receipt_ns_ = 0;
  revokeAuthorization();
  clearMotionIntent();
  beginZeroEpisode(emergency_stop_latched_
                       ? SessionState::kEmergencyStopLatched
                       : SessionState::kConnectedInhibited,
                   receipt_monotonic_ns);
  return {FeedbackObservationStatus::kAcceptedControlInhibited, true, false};
}

void SafetySession::latchEmergencyStop(std::int64_t now_monotonic_ns) {
  emergency_stop_latched_ = true;
  if (emergency_stop_latch_generation_ !=
      std::numeric_limits<std::uint64_t>::max()) {
    ++emergency_stop_latch_generation_;
  }
  revokeAuthorization();
  clearMotionIntent();
  feedback_allowed_seen_ = false;
  latest_allowed_feedback_receipt_ns_ = 0;

  if (!configuration_valid_) {
    state_ = SessionState::kConfigurationInvalid;
    return;
  }
  const bool time_valid = observeMonotonicTime(now_monotonic_ns);
  if (synchronizeConnection() == ConnectionSync::kDisconnected) {
    return;
  }
  if (state_ == SessionState::kFaulted) {
    return;
  }
  beginZeroEpisode(SessionState::kEmergencyStopLatched,
                   time_valid ? now_monotonic_ns
                              : last_observed_monotonic_ns_);
}

bool SafetySession::resetEmergencyStop(
    std::uint64_t reset_token, std::int64_t now_monotonic_ns) {
  if (!configuration_valid_ || !emergency_stop_latched_) {
    return false;
  }
  if (!observeMonotonicTime(now_monotonic_ns)) {
    handleClockFailure();
    return false;
  }
  if (synchronizeConnection() != ConnectionSync::kUnchanged ||
      zero_episode_active_ || state_ == SessionState::kFaulted ||
      reset_token == 0U || reset_token <= highest_estop_reset_token_ ||
      !feedbackIsFresh(now_monotonic_ns)) {
    return false;
  }

  highest_estop_reset_token_ = reset_token;
  emergency_stop_latched_ = false;
  revokeAuthorization();
  clearMotionIntent();
  state_ = SessionState::kConnectedInhibited;
  return true;
}

CycleResult SafetySession::tick(std::int64_t now_monotonic_ns) {
  CycleResult result;
  result.state = state_;
  if (!configuration_valid_) {
    result.action = CycleAction::kConfigurationInvalid;
    result.state = SessionState::kConfigurationInvalid;
    result.transport_status = TransportStatus::kInvalidArgument;
    return result;
  }
  if (!observeMonotonicTime(now_monotonic_ns)) {
    handleClockFailure();
    result.action = CycleAction::kClockInvalid;
    result.state = state_;
    result.transport_status = TransportStatus::kInvalidArgument;
    result.outcome_uncertain = true;
    return result;
  }

  const ConnectionSync connection = synchronizeConnection();
  if (connection == ConnectionSync::kDisconnected) {
    result.action = CycleAction::kDisconnected;
    result.state = state_;
    result.transport_status = TransportStatus::kDisconnected;
    result.outcome_uncertain = true;
    return result;
  }
  if (zero_episode_active_) {
    return attemptZero(now_monotonic_ns, result);
  }
  if (state_ == SessionState::kFaulted) {
    result.state = state_;
    return result;
  }
  if (emergency_stop_latched_) {
    state_ = SessionState::kEmergencyStopLatched;
    result.state = state_;
    return result;
  }
  if (state_ != SessionState::kActive) {
    result.state = state_;
    return result;
  }

  if (!authorized()) {
    clearMotionIntent();
    beginZeroEpisode(SessionState::kConnectedInhibited,
                     now_monotonic_ns);
    return attemptZero(now_monotonic_ns, result);
  }
  if (!feedbackIsFresh(now_monotonic_ns) ||
      !commandIsFresh(now_monotonic_ns)) {
    clearMotionIntent();
    beginZeroEpisode(SessionState::kRecovering, now_monotonic_ns);
    return attemptZero(now_monotonic_ns, result);
  }
  if (last_motion_host_write_completion_ns_ > 0 &&
      now_monotonic_ns - last_motion_host_write_completion_ns_ <
          config_.transmit_period_ns) {
    result.action = CycleAction::kTransmitRateLimited;
    result.state = state_;
    return result;
  }

  result.motion_write_attempted = true;
  IoResult write_result;
  try {
    write_result = transport_->writeAll(
        latest_frame_.data(), latest_frame_.size(),
        boundedWriteDeadline(now_monotonic_ns,
                             latest_command_.deadline_monotonic_ns));
  } catch (...) {
    write_result = exceptionIoResult(now_monotonic_ns);
  }
  result.transport_status = write_result.status;
  result.motion_host_write_complete = write_result.host_write_complete;
  result.outcome_uncertain = write_result.outcome_uncertain;
  if (write_result.status == TransportStatus::kOk &&
      write_result.host_write_complete &&
      write_result.bytes_transferred == latest_frame_.size()) {
    last_motion_host_write_completion_ns_ =
        write_result.completion_monotonic_ns > 0
            ? write_result.completion_monotonic_ns
            : now_monotonic_ns;
    last_observed_monotonic_ns_ =
        std::max(last_observed_monotonic_ns_,
                 last_motion_host_write_completion_ns_);
    result.action = CycleAction::kMotionHostWriteComplete;
    result.state = state_;
    return result;
  }

  revokeAuthorization();
  clearMotionIntent();
  if (write_result.status == TransportStatus::kDisconnected ||
      !transport_->connected()) {
    handleDisconnect();
    result.action = CycleAction::kDisconnected;
    result.state = state_;
    result.transport_status = TransportStatus::kDisconnected;
    result.outcome_uncertain = true;
    return result;
  }

  const std::int64_t zero_start_ns =
      std::max(now_monotonic_ns, write_result.completion_monotonic_ns);
  beginZeroEpisode(SessionState::kConnectedInhibited, zero_start_ns);
  return attemptZero(zero_start_ns, result);
}

bool SafetySession::authorized() const noexcept {
  return configuration_valid_ && connection_known_ &&
         authorization_active_ && !emergency_stop_latched_ &&
         authorized_generation_ == connection_generation_ &&
         authorization_monotonic_ns_ > 0 &&
         state_ != SessionState::kFaulted;
}

SafetySession::ConnectionSync SafetySession::synchronizeConnection() {
  if (transport_ == nullptr || !transport_->connected() ||
      transport_->generation() == 0U) {
    handleDisconnect();
    return ConnectionSync::kDisconnected;
  }
  const std::uint64_t observed_generation = transport_->generation();
  if (!connection_known_ || observed_generation != connection_generation_) {
    handleNewConnection(observed_generation);
    return ConnectionSync::kNewGeneration;
  }
  return ConnectionSync::kUnchanged;
}

bool SafetySession::observeMonotonicTime(
    std::int64_t now_monotonic_ns) {
  if (now_monotonic_ns <= 0 ||
      (last_observed_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_observed_monotonic_ns_)) {
    return false;
  }
  if (now_monotonic_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = now_monotonic_ns;
  }
  return true;
}

void SafetySession::handleClockFailure() {
  revokeAuthorization();
  clearMotionIntent();
  feedback_allowed_seen_ = false;
  latest_allowed_feedback_receipt_ns_ = 0;
  if (connection_known_ && transport_ != nullptr && transport_->connected()) {
    beginZeroEpisode(emergency_stop_latched_
                         ? SessionState::kEmergencyStopLatched
                         : SessionState::kConnectedInhibited,
                     last_observed_monotonic_ns_);
  }
}

void SafetySession::handleDisconnect() {
  connection_known_ = false;
  revokeAuthorization();
  clearMotionIntent();
  feedback_allowed_seen_ = false;
  latest_allowed_feedback_receipt_ns_ = 0;
  zero_episode_active_ = false;
  zero_attempts_ = 0U;
  next_zero_attempt_ns_ = 0;
  last_motion_host_write_completion_ns_ = 0;
  if (configuration_valid_) {
    state_ = SessionState::kDisconnected;
  }
}

void SafetySession::handleNewConnection(std::uint64_t generation) {
  connection_known_ = true;
  connection_generation_ = generation;
  revokeAuthorization();
  clearMotionIntent();
  feedback_allowed_seen_ = false;
  latest_allowed_feedback_receipt_ns_ = 0;
  zero_episode_active_ = false;
  zero_attempts_ = 0U;
  next_zero_attempt_ns_ = 0;
  last_motion_host_write_completion_ns_ = 0;
  // A new descriptor generation is the only implicit recovery from a
  // zero-write fault.  It must still complete the initial-zero episode before
  // authorization can succeed.
  state_ = SessionState::kConnectedInhibited;
  beginZeroEpisode(emergency_stop_latched_
                       ? SessionState::kEmergencyStopLatched
                       : SessionState::kConnectedInhibited,
                   0);
}

void SafetySession::revokeAuthorization() {
  authorization_active_ = false;
  authorized_generation_ = 0U;
  authorization_monotonic_ns_ = 0;
}

void SafetySession::clearMotionIntent() {
  latest_command_valid_ = false;
  latest_command_ = TimedMotionCommand{};
  latest_frame_ = CommandFrame{};
  latest_command_receipt_ns_ = 0;
  consecutive_fresh_commands_ = 0U;
}

void SafetySession::beginZeroEpisode(SessionState state_after_zero,
                                     std::int64_t earliest_attempt_ns) {
  if (!connection_known_ || transport_ == nullptr ||
      !transport_->connected()) {
    state_ = SessionState::kDisconnected;
    return;
  }
  if (state_ == SessionState::kFaulted) {
    return;
  }
  if (zero_episode_active_) {
    if (state_after_zero == SessionState::kEmergencyStopLatched ||
        (state_after_zero == SessionState::kConnectedInhibited &&
         state_after_zero_ == SessionState::kRecovering)) {
      state_after_zero_ = state_after_zero;
    }
    return;
  }
  zero_episode_active_ = true;
  state_after_zero_ = state_after_zero;
  zero_attempts_ = 0U;
  next_zero_attempt_ns_ = std::max<std::int64_t>(0, earliest_attempt_ns);
  state_ = SessionState::kStopPending;
}

CycleResult SafetySession::attemptZero(std::int64_t now_monotonic_ns,
                                       CycleResult result) {
  result.state = state_;
  if (!zero_episode_active_) {
    return result;
  }
  if (now_monotonic_ns < next_zero_attempt_ns_) {
    result.action = CycleAction::kZeroRetryPending;
    result.zero_attempts = zero_attempts_;
    return result;
  }

  const CommandFrame zero = makeZeroCommandFrame();
  result.zero_write_attempted = true;
  ++zero_attempts_;
  IoResult zero_result;
  try {
    zero_result = transport_->writeAll(
        zero.data(), zero.size(),
        boundedWriteDeadline(now_monotonic_ns,
                             std::numeric_limits<std::int64_t>::max()));
  } catch (...) {
    zero_result = exceptionIoResult(now_monotonic_ns);
  }
  result.zero_attempts = zero_attempts_;
  result.zero_host_write_complete = zero_result.host_write_complete;
  result.outcome_uncertain =
      result.outcome_uncertain || zero_result.outcome_uncertain;
  result.transport_status = zero_result.status;
  if (zero_result.completion_monotonic_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = zero_result.completion_monotonic_ns;
  }

  if (zero_result.status == TransportStatus::kDisconnected ||
      !transport_->connected()) {
    handleDisconnect();
    result.action = CycleAction::kDisconnected;
    result.state = state_;
    result.transport_status = TransportStatus::kDisconnected;
    result.outcome_uncertain = true;
    return result;
  }
  if (zero_result.status == TransportStatus::kOk &&
      zero_result.host_write_complete &&
      zero_result.bytes_transferred == zero.size()) {
    zero_episode_active_ = false;
    zero_attempts_ = 0U;
    next_zero_attempt_ns_ = 0;
    state_ = emergency_stop_latched_
                 ? SessionState::kEmergencyStopLatched
                 : state_after_zero_;
    result.action = CycleAction::kZeroHostWriteComplete;
    result.state = state_;
    return result;
  }

  if (zero_attempts_ >= config_.maximum_zero_attempts) {
    zero_episode_active_ = false;
    state_ = SessionState::kFaulted;
    result.action = CycleAction::kZeroRetriesExhausted;
    result.state = state_;
    result.outcome_uncertain = true;
    return result;
  }

  const std::int64_t retry_base =
      std::max(now_monotonic_ns, zero_result.completion_monotonic_ns);
  next_zero_attempt_ns_ =
      saturatingAdd(retry_base, config_.zero_retry_interval_ns);
  result.action = CycleAction::kZeroHostWriteFailed;
  result.state = state_;
  return result;
}

bool SafetySession::feedbackIsFresh(
    std::int64_t now_monotonic_ns) const noexcept {
  return feedback_allowed_seen_ && latest_allowed_feedback_receipt_ns_ > 0 &&
         latest_allowed_feedback_receipt_ns_ <= now_monotonic_ns &&
         now_monotonic_ns - latest_allowed_feedback_receipt_ns_ <=
             config_.feedback_timeout_ns;
}

bool SafetySession::commandIsFresh(
    std::int64_t now_monotonic_ns) const noexcept {
  return latest_command_valid_ && latest_command_receipt_ns_ > 0 &&
         latest_command_.created_monotonic_ns <= now_monotonic_ns &&
         latest_command_receipt_ns_ <= now_monotonic_ns &&
         now_monotonic_ns <= latest_command_.deadline_monotonic_ns &&
         now_monotonic_ns - latest_command_.created_monotonic_ns <=
             config_.max_command_age_ns &&
         now_monotonic_ns - latest_command_receipt_ns_ <=
             config_.max_command_age_ns;
}

std::int64_t SafetySession::boundedWriteDeadline(
    std::int64_t now_monotonic_ns,
    std::int64_t command_deadline_ns) const noexcept {
  return std::min(saturatingAdd(now_monotonic_ns, config_.write_timeout_ns),
                  command_deadline_ns);
}

}  // namespace wheeltec_vcu_serial
