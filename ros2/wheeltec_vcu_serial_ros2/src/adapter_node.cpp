// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial_ros2/adapter_node.hpp"

#include "wheeltec_vcu_serial_ros2/adapter_logic.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace core = wheeltec_vcu_serial;

namespace wheeltec_vcu_serial_ros2 {
namespace {

constexpr std::int64_t kNanosecondsPerMillisecond = 1000000LL;

std::int64_t millisecondsToNanoseconds(std::int64_t milliseconds) noexcept {
  return milliseconds * kNanosecondsPerMillisecond;
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
  safety.write_timeout_ns = millisecondsToNanoseconds(config.write_timeout_ms);
  safety.zero_retry_interval_ns =
      millisecondsToNanoseconds(config.zero_retry_interval_ms);
  safety.maximum_zero_attempts = config.zero_retry_attempts;
  safety.fresh_commands_required = config.fresh_commands_required;
  return safety;
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

const char* authorizationName(core::AuthorizationStatus status) noexcept {
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

const char* submissionName(core::SubmissionStatus status) noexcept {
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

const char* transportName(core::TransportStatus status) noexcept {
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

}  // namespace

AdapterNode::AdapterNode(const rclcpp::NodeOptions& options)
    : Node("wheeltec_vcu_serial_adapter", options) {
  const std::string config_file =
      declare_parameter<std::string>("config_file", "");
  ActuationGates gates;
  gates.acknowledge_unverified_protocol =
      declare_parameter<bool>("acknowledge_unverified_protocol", false);
  gates.enable_actuation =
      declare_parameter<bool>("enable_actuation", false);
  gates.operator_confirmation =
      declare_parameter<std::string>("operator_confirmation", "");

  if (config_file.empty()) {
    throw std::invalid_argument("config_file is required");
  }
  const GateDecision gate_decision = evaluateActuationGates(gates);
  if (gate_decision == GateDecision::kInvalidPartialConfiguration) {
    throw std::invalid_argument(
        "actuation gates must be either all disabled or all explicitly enabled");
  }
  actuation_enabled_ = gate_decision == GateDecision::kEnabled;

  const core::ConfigResult loaded =
      core::loadRuntimeConfig(config_file, actuation_enabled_);
  if (!loaded.ok()) {
    throw std::invalid_argument("invalid configuration: " + loaded.message);
  }
  runtime_config_ = loaded.config;

  // Allocate and validate every active-mode object before any device is
  // opened. If later ROS entity construction throws, there is no live serial
  // descriptor that would require a final-zero cleanup.
  if (actuation_enabled_) {
    prepareActiveTransport();
  }

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  const auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  feedback_publisher_ = create_publisher<Feedback>("~/feedback", feedback_qos);
  state_publisher_ =
      create_publisher<AdapterState>("~/adapter_state", state_qos);
  command_subscription_ = create_subscription<DriveCommand>(
      "~/drive_command", command_qos,
      std::bind(&AdapterNode::onDriveCommand, this, std::placeholders::_1));
  authorize_service_ = create_service<Authorize>(
      "~/authorize",
      std::bind(&AdapterNode::onAuthorize, this, std::placeholders::_1,
                std::placeholders::_2));
  stop_service_ = create_service<Trigger>(
      "~/stop", std::bind(&AdapterNode::onStop, this, std::placeholders::_1,
                          std::placeholders::_2));
  estop_service_ = create_service<Trigger>(
      "~/emergency_stop",
      std::bind(&AdapterNode::onEmergencyStop, this, std::placeholders::_1,
                std::placeholders::_2));
  reset_estop_service_ = create_service<ResetEstop>(
      "~/reset_emergency_stop",
      std::bind(&AdapterNode::onResetEstop, this, std::placeholders::_1,
                std::placeholders::_2));

  io_timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&AdapterNode::onIoTimer, this));
  publishState(core::monotonicNowNs(), true);

  if (!actuation_enabled_) {
    RCLCPP_WARN(get_logger(),
                "adapter is offline: actuation gates are disabled and no "
                "serial device was opened");
    return;
  }

  // This is deliberately the last active-mode construction stage. All ROS
  // entities and timers already exist. openActiveTransport() catches any
  // exception after a successful open, attempts the bounded final zero, closes
  // the descriptor, and then rethrows.
  openActiveTransport();
}

AdapterNode::~AdapterNode() { (void)shutdownSafely(); }

void AdapterNode::prepareActiveTransport() {
  transport_.reset(new core::PosixSerialTransport(
      runtime_config_.device_path, core::SerialAccess::kReadWrite));
  session_.reset(
      new core::SafetySession(makeSafetyConfig(runtime_config_), transport_.get()));
  if (!session_->configurationValid()) {
    throw std::invalid_argument("core safety configuration is invalid");
  }
  last_reported_state_ = session_->state();
}

void AdapterNode::openActiveTransport() {
  try {
    const core::SerialOpenResult opened = transport_->open();
    if (opened.ok()) {
      RCLCPP_WARN(get_logger(),
                  "serial connection opened (generation=%llu); startup zero "
                  "and fresh authorization are required; controller ACK is "
                  "unavailable",
                  static_cast<unsigned long long>(opened.generation));
    } else {
      next_reconnect_ns_ = core::monotonicNowNs() +
                           millisecondsToNanoseconds(
                               runtime_config_.reconnect_interval_ms);
      RCLCPP_ERROR(get_logger(),
                   "initial serial open failed (status=%s, os_error=%d); "
                   "remaining fail-closed and retrying",
                   transportName(opened.status), opened.os_error);
    }
  } catch (...) {
    // A derived destructor is not run when its constructor throws. Explicitly
    // cover every exception that could occur after open() acquired the device.
    (void)shutdownSafely();
    throw;
  }
}

void AdapterNode::onDriveCommand(DriveCommand::ConstSharedPtr message) {
  if (!actuation_enabled_ || session_ == nullptr || message == nullptr) {
    RCLCPP_WARN(get_logger(), "drive command rejected: adapter is offline");
    return;
  }
  const std::int64_t receipt_ns = core::monotonicNowNs();
  const std::int64_t maximum_ns =
      millisecondsToNanoseconds(runtime_config_.command_timeout_ms);
  std::int64_t valid_for_ns = 0;
  std::int64_t deadline_ns = 0;
  if (!std::isfinite(message->linear_speed_mps) ||
      !std::isfinite(message->yaw_rate_radps) ||
      !boundedDurationNanoseconds(message->valid_for.sec,
                                  message->valid_for.nanosec, maximum_ns,
                                  &valid_for_ns) ||
      !checkedAddNanoseconds(receipt_ns, valid_for_ns, &deadline_ns)) {
    (void)session_->disarm(receipt_ns);
    RCLCPP_ERROR(get_logger(),
                 "drive command rejected: non-finite motion or invalid "
                 "receipt-relative valid_for; motion disarmed");
    return;
  }

  core::TimedMotionCommand command;
  command.sequence_id = message->sequence_id;
  command.created_monotonic_ns = receipt_ns;
  command.deadline_monotonic_ns = deadline_ns;
  command.motion.linear_speed_mps = message->linear_speed_mps;
  command.motion.lateral_speed_mps = 0.0;
  command.motion.yaw_rate_radps = message->yaw_rate_radps;
  const core::SubmissionStatus status = session_->submit(command, receipt_ns);
  if (!submissionWasAccepted(status)) {
    RCLCPP_WARN(get_logger(), "drive command sequence=%llu rejected (%s)",
                static_cast<unsigned long long>(message->sequence_id),
                submissionName(status));
  }
}

void AdapterNode::onAuthorize(const Authorize::Request::SharedPtr request,
                              Authorize::Response::SharedPtr response) {
  if (request == nullptr || response == nullptr) {
    return;
  }
  if (!actuation_enabled_ || session_ == nullptr) {
    response->accepted = false;
    response->reason = "adapter_offline";
    return;
  }
  const core::AuthorizationStatus status =
      session_->authorize(request->token, core::monotonicNowNs());
  response->accepted = status == core::AuthorizationStatus::kAuthorized;
  response->reason = authorizationName(status);
  publishState(core::monotonicNowNs(), true);
}

void AdapterNode::onStop(const Trigger::Request::SharedPtr,
                         Trigger::Response::SharedPtr response) {
  if (response == nullptr) {
    return;
  }
  if (!actuation_enabled_ || session_ == nullptr) {
    // Stop is an idempotent local fail-safe intent. Offline already inhibits
    // actuation; success makes no VCU delivery or acknowledgement claim.
    response->success = true;
    response->message = "local_stop_accepted_offline_vcu_delivery_unavailable";
    publishState(core::monotonicNowNs(), true);
    return;
  }
  response->success = session_->requestStop(core::monotonicNowNs());
  response->message = response->success ? "stop_requested"
                                        : "stop_zero_episode_not_started";
  publishState(core::monotonicNowNs(), true);
}

void AdapterNode::onEmergencyStop(const Trigger::Request::SharedPtr,
                                  Trigger::Response::SharedPtr response) {
  if (response == nullptr) {
    return;
  }
  const std::int64_t now_ns = core::monotonicNowNs();
  local_software_estop_latched_ = true;
  if (actuation_enabled_ && session_ != nullptr) {
    session_->latchEmergencyStop(now_ns);
  }
  response->success = true;
  response->message = !actuation_enabled_ || session_ == nullptr
                          ? "software_emergency_stop_latched_offline_locally"
                          : "software_emergency_stop_latched";
  publishState(now_ns, true);
}

void AdapterNode::onResetEstop(
    const ResetEstop::Request::SharedPtr request,
    ResetEstop::Response::SharedPtr response) {
  if (request == nullptr || response == nullptr) {
    return;
  }
  if (!actuation_enabled_ || session_ == nullptr) {
    response->accepted = false;
    response->reason = "adapter_offline";
    return;
  }
  response->accepted =
      session_->resetEmergencyStop(request->token, core::monotonicNowNs());
  if (response->accepted) {
    local_software_estop_latched_ = false;
  }
  response->reason = response->accepted ? "reset_accepted" : "reset_rejected";
  publishState(core::monotonicNowNs(), true);
}

void AdapterNode::tryReconnect(std::int64_t now_monotonic_ns) {
  if (transport_ == nullptr || transport_->connected() ||
      now_monotonic_ns < next_reconnect_ns_) {
    return;
  }
  const core::SerialOpenResult reopened = transport_->reopen();
  next_reconnect_ns_ = now_monotonic_ns + millisecondsToNanoseconds(
                                               runtime_config_.reconnect_interval_ms);
  if (reopened.ok()) {
    parser_.reset();
    RCLCPP_WARN(get_logger(),
                "serial connection reopened (generation=%llu); startup zero "
                "and new authorization are required",
                static_cast<unsigned long long>(reopened.generation));
  }
}

void AdapterNode::readFeedback(std::int64_t now_monotonic_ns) {
  if (transport_ == nullptr || session_ == nullptr ||
      !transport_->connected()) {
    return;
  }
  std::array<std::uint8_t, 512U> buffer{};
  const core::IoResult read = transport_->readSome(
      buffer.data(), buffer.size(), now_monotonic_ns + kNanosecondsPerMillisecond);
  if (read.status == core::TransportStatus::kDeadlineExceeded ||
      read.status == core::TransportStatus::kDisconnected) {
    return;
  }
  if (read.status != core::TransportStatus::kOk) {
    session_->latchEmergencyStop(now_monotonic_ns);
    RCLCPP_ERROR(get_logger(), "serial read failed (%s); software emergency "
                               "stop latched",
                 transportName(read.status));
    return;
  }

  const std::int64_t receipt_ns = read.completion_monotonic_ns > 0
                                      ? read.completion_monotonic_ns
                                      : core::monotonicNowNs();
  const auto parsed = parser_.consume(buffer.data(), read.bytes_transferred);
  for (const auto& frame : parsed) {
    const core::FeedbackObservationResult observation =
        session_->observeFeedback(frame.frame, receipt_ns);
    if (!observation.valid_frame) {
      continue;
    }
    Feedback message;
    message.receipt_time = get_clock()->now();
    message.composite_stop_flag_raw = frame.feedback.composite_stop_flag_raw;
    message.control_allowed = frame.feedback.control_allowed;
    message.control_inhibited = frame.feedback.control_inhibited;
    message.linear_speed_mps = frame.feedback.linear_speed_mps;
    message.lateral_speed_mps = frame.feedback.lateral_speed_mps;
    message.yaw_rate_radps = frame.feedback.yaw_rate_radps;
    message.linear_acceleration_mps2 = frame.feedback.linear_acceleration_mps2;
    message.angular_velocity_radps = frame.feedback.angular_velocity_radps;
    message.supply_voltage_v = frame.feedback.supply_voltage_v;
    message.vcu_ack_available = false;
    message.source_time_available = false;
    feedback_publisher_->publish(message);
  }
}

void AdapterNode::reportCycle(const core::CycleResult& cycle) {
  if (cycle.state != last_reported_state_) {
    RCLCPP_INFO(get_logger(), "session state=%s",
                sessionStateName(cycle.state));
    last_reported_state_ = cycle.state;
  }
  if (cycle.action == core::CycleAction::kZeroHostWriteComplete) {
    RCLCPP_INFO(get_logger(),
                "zero frame completed at host; controller acknowledgment is "
                "unavailable");
  } else if (cycle.action == core::CycleAction::kZeroHostWriteFailed ||
             cycle.action == core::CycleAction::kZeroRetriesExhausted) {
    RCLCPP_ERROR(get_logger(),
                 "zero host write failed (status=%s, attempts=%u, "
                 "outcome_uncertain=%s)",
                 transportName(cycle.transport_status), cycle.zero_attempts,
                 cycle.outcome_uncertain ? "true" : "false");
  }
}

void AdapterNode::publishState(std::int64_t now_monotonic_ns, bool force) {
  if (!force && now_monotonic_ns > 0 &&
      now_monotonic_ns < next_state_publish_ns_) {
    return;
  }
  AdapterState state;
  state.receipt_time = get_clock()->now();
  state.session_state = session_ == nullptr
                            ? "offline"
                            : sessionStateName(session_->state());
  state.actuation_enabled = actuation_enabled_;
  state.connected = transport_ != nullptr && transport_->connected();
  state.connection_generation =
      transport_ == nullptr ? 0U : transport_->generation();
  state.authorized = session_ != nullptr && session_->authorized();
  state.software_estop_latched =
      local_software_estop_latched_ ||
      (session_ != nullptr && session_->emergencyStopLatched());
  state.vcu_ack_available = false;
  state.source_time_available = false;
  state_publisher_->publish(state);
  if (now_monotonic_ns > 0) {
    next_state_publish_ns_ = now_monotonic_ns + 1000000000LL;
  }
}

void AdapterNode::onIoTimer() {
  const std::int64_t now_ns = core::monotonicNowNs();
  if (!actuation_enabled_ || session_ == nullptr || transport_ == nullptr) {
    publishState(now_ns, false);
    return;
  }
  if (now_ns <= 0) {
    session_->latchEmergencyStop(now_ns);
    return;
  }
  tryReconnect(now_ns);
  readFeedback(now_ns);
  const core::CycleResult cycle = session_->tick(core::monotonicNowNs());
  const bool state_changed = cycle.state != last_reported_state_;
  reportCycle(cycle);
  publishState(now_ns, state_changed);
}

bool AdapterNode::shutdownSafely() noexcept {
  if (shutdown_started_) {
    return shutdown_succeeded_;
  }
  shutdown_started_ = true;
  if (!actuation_enabled_ || transport_ == nullptr) {
    shutdown_succeeded_ = true;
    return true;
  }

  shutdown_succeeded_ = false;
  try {
    const std::int64_t stop_ns = core::monotonicNowNs();
    if (session_ != nullptr && stop_ns > 0) {
      // Do not tick here: tick() could consume one zero write in addition to
      // the explicit shutdown budget below. Disarm still clears motion intent.
      (void)session_->disarm(stop_ns);
    }

    if (transport_->connected()) {
      const core::CommandFrame zero = core::makeZeroCommandFrame();
      const BoundedZeroResult zero_result = runBoundedZeroAttempts(
          runtime_config_.zero_retry_attempts,
          [this, &zero]() {
            const std::int64_t now_ns = core::monotonicNowNs();
            if (now_ns <= 0) {
              return FinalZeroAttemptDisposition::kAbort;
            }
            const core::IoResult result = transport_->writeAll(
                zero.data(), zero.size(),
                now_ns + millisecondsToNanoseconds(
                             runtime_config_.write_timeout_ms));
            if (result.status == core::TransportStatus::kOk &&
                result.host_write_complete &&
                result.bytes_transferred == zero.size()) {
              return FinalZeroAttemptDisposition::kHostWriteComplete;
            }
            return transport_->connected()
                       ? FinalZeroAttemptDisposition::kRetry
                       : FinalZeroAttemptDisposition::kAbort;
          },
          [this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                runtime_config_.zero_retry_interval_ms));
          });
      shutdown_succeeded_ = zero_result.host_write_complete;
    }

    if (shutdown_succeeded_) {
      RCLCPP_WARN(
          get_logger(),
          "final zero completed at host; this is not a controller ACK");
    } else {
      RCLCPP_ERROR(get_logger(),
                   "final zero was not completed at host within the configured "
                   "attempt budget; outcome may be uncertain and no controller "
                   "ACK is available");
    }
  } catch (...) {
    // Keep this path noexcept. The caller receives failure and the descriptor
    // is still closed below even if a clock, transport seam, or logger throws.
    shutdown_succeeded_ = false;
  }
  transport_->close();
  return shutdown_succeeded_;
}

}  // namespace wheeltec_vcu_serial_ros2
