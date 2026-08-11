// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "wheeltec_vcu_serial/config.hpp"
#include "wheeltec_vcu_serial/feedback_parser.hpp"
#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial/safety_session.hpp"
#include "wheeltec_vcu_serial/transport.hpp"
#include "wheeltec_vcu_serial_ros1/AdapterState.h"
#include "wheeltec_vcu_serial_ros1/Authorize.h"
#include "wheeltec_vcu_serial_ros1/DriveCommand.h"
#include "wheeltec_vcu_serial_ros1/Feedback.h"
#include "wheeltec_vcu_serial_ros1/ResetEstop.h"
#include "wheeltec_vcu_serial_ros1/adapter_support.hpp"

namespace core = wheeltec_vcu_serial;
namespace adapter = wheeltec_vcu_serial_ros1;

namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;

void handleSignal(int) { g_shutdown_requested = 1; }

std::int64_t millisecondsToNanoseconds(std::int64_t milliseconds) {
  return milliseconds * 1000000LL;
}

std::int64_t saturatingAdd(std::int64_t value,
                           std::int64_t positive_delta) {
  if (positive_delta <= 0) {
    return value;
  }
  if (value > std::numeric_limits<std::int64_t>::max() - positive_delta) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + positive_delta;
}

class RosAdapter final {
 public:
  RosAdapter(ros::NodeHandle private_node,
             const core::RuntimeConfig& runtime_config,
             bool actuation_enabled)
      : private_node_(std::move(private_node)),
        runtime_config_(runtime_config),
        actuation_enabled_(actuation_enabled),
        transport_(new core::PosixSerialTransport(
            runtime_config.device_path,
            actuation_enabled ? core::SerialAccess::kReadWrite
                              : core::SerialAccess::kReadOnly)),
        session_(new core::SafetySession(
            adapter::makeSafetyConfig(runtime_config), transport_.get())) {
    feedback_publisher_ =
        private_node_.advertise<adapter::Feedback>("feedback", 10U);
    state_publisher_ =
        private_node_.advertise<adapter::AdapterState>("adapter_state", 10U,
                                                       true);
    command_subscriber_ = private_node_.subscribe(
        "drive_command", 1U, &RosAdapter::onDriveCommand, this);
    authorize_service_ = private_node_.advertiseService(
        "authorize", &RosAdapter::onAuthorize, this);
    stop_service_ = private_node_.advertiseService(
        "stop", &RosAdapter::onStop, this);
    emergency_stop_service_ = private_node_.advertiseService(
        "emergency_stop", &RosAdapter::onEmergencyStop, this);
    reset_emergency_stop_service_ = private_node_.advertiseService(
        "reset_emergency_stop", &RosAdapter::onResetEmergencyStop, this);
  }

  ~RosAdapter() noexcept {
    // If an exception escapes any runtime path after a successful open, make
    // one last bounded attempt before the transport destructor closes the fd.
    // This still cannot prove delivery or acknowledgement.
    if (!finalization_performed_ && transport_ && transport_->connected()) {
      try {
        (void)finishWithBoundedZero();
      } catch (...) {
        // Destructors must not throw. The outcome remains explicitly unknown.
      }
    }
    if (transport_) {
      transport_->close();
    }
  }

  int run() {
    if (!session_->configurationValid()) {
      ROS_FATAL("Safety configuration is invalid; serial was not opened");
      return 5;
    }

    const std::int64_t start_ns = core::monotonicNowNs();
    if (start_ns <= 0) {
      ROS_FATAL("CLOCK_MONOTONIC is unavailable; serial was not opened");
      return 6;
    }
    if (!actuation_enabled_) {
      return runOffline(start_ns);
    }
    bool unexpected_exception = false;
    try {
      const core::SerialOpenResult opened = transport_->open();
      if (opened.ok()) {
        ROS_WARN_STREAM(
            "Serial opened for actuation; generation=" << opened.generation
            << ". Reauthorization and allowed feedback are required. "
               "A complete host write is not a VCU ACK.");
      } else {
        ROS_ERROR_STREAM(
            "Initial serial open failed: status="
            << adapter::transportStatusName(opened.status)
            << " os_error=" << opened.os_error
            << ". Reconnect will be attempted; no motion is authorized.");
      }
      next_reconnect_ns_ = saturatingAdd(
          start_ns,
          millisecondsToNanoseconds(runtime_config_.reconnect_interval_ms));

      ros::WallRate loop_rate(200.0);
      publishState(true, start_ns);
      while (ros::ok() && g_shutdown_requested == 0) {
        // All ROS callbacks, receive processing, and SafetySession calls are
        // serialized on this thread. AsyncSpinner is intentionally unsupported.
        ros::spinOnce();

        std::int64_t now_ns = core::monotonicNowNs();
        if (now_ns <= 0) {
          session_->latchEmergencyStop(now_ns);
          ROS_FATAL("CLOCK_MONOTONIC failed; software E-stop latched");
          break;
        }

        maintainConnection(now_ns);
        receiveFeedback(now_ns);

        now_ns = core::monotonicNowNs();
        if (now_ns <= 0) {
          session_->latchEmergencyStop(now_ns);
          ROS_FATAL("CLOCK_MONOTONIC failed; software E-stop latched");
          break;
        }
        const core::CycleResult cycle = session_->tick(now_ns);
        reportCycle(cycle);
        publishState(false, now_ns);
        loop_rate.sleep();
      }
    } catch (const std::exception& error) {
      unexpected_exception = true;
      const std::int64_t now_ns = core::monotonicNowNs();
      session_->latchEmergencyStop(now_ns);
      ROS_FATAL_STREAM("Unhandled adapter exception; motion disarmed and "
                       "bounded final zero will be attempted: " << error.what());
    } catch (...) {
      unexpected_exception = true;
      const std::int64_t now_ns = core::monotonicNowNs();
      session_->latchEmergencyStop(now_ns);
      ROS_FATAL("Unknown adapter exception; motion disarmed and bounded final "
                "zero will be attempted");
    }

    const bool final_zero_complete = finishWithBoundedZero();
    finalization_performed_ = true;
    transport_->close();
    publishState(true, core::monotonicNowNs());
    if (!final_zero_complete) {
      ROS_ERROR("Final zero host write was not completed; outcome is uncertain. "
                "No VCU ACK is available.");
      return 9;
    }
    if (unexpected_exception) {
      return 10;
    }
    return g_shutdown_requested == 0 ? 0 : 130;
  }

 private:
  int runOffline(std::int64_t start_ns) {
    ROS_INFO("Offline mode active: all actuation gates are disabled and no "
             "serial open or reconnect will be attempted");
    publishState(true, start_ns);
    ros::WallRate loop_rate(20.0);
    while (ros::ok() && g_shutdown_requested == 0) {
      ros::spinOnce();
      const std::int64_t now_ns = core::monotonicNowNs();
      if (now_ns <= 0) {
        ROS_FATAL("CLOCK_MONOTONIC failed in offline mode");
        return 6;
      }
      publishState(false, now_ns);
      loop_rate.sleep();
    }
    return g_shutdown_requested == 0 ? 0 : 130;
  }

  void onDriveCommand(const adapter::DriveCommand::ConstPtr& message) {
    if (!actuation_enabled_) {
      ROS_WARN_THROTTLE(2.0,
                        "Drive command ignored: adapter is in offline mode");
      return;
    }
    const std::int64_t receipt_ns = core::monotonicNowNs();
    if (!message || receipt_ns <= 0) {
      session_->latchEmergencyStop(receipt_ns);
      ROS_ERROR("Invalid command callback or monotonic clock; E-stop latched");
      return;
    }
    const adapter::DriveConversionResult converted =
        adapter::convertDriveCommand(
            *message, receipt_ns,
            millisecondsToNanoseconds(runtime_config_.command_timeout_ms));
    if (!converted.valid) {
      (void)session_->disarm(receipt_ns);
      ROS_WARN_STREAM("Drive command rejected and authorization revoked: "
                      << converted.reason);
      return;
    }

    const core::SubmissionStatus status =
        session_->submit(converted.command, receipt_ns);
    if (status != core::SubmissionStatus::kAccepted &&
        status != core::SubmissionStatus::kRecoveryPending &&
        status != core::SubmissionStatus::kStopAccepted) {
      ROS_WARN_STREAM("Drive command rejected: sequence="
                      << message->sequence_id
                      << " status=" << adapter::submissionStatusName(status));
    }
  }

  bool onAuthorize(adapter::Authorize::Request& request,
                   adapter::Authorize::Response& response) {
    if (!actuation_enabled_) {
      response.accepted = false;
      response.reason = "actuation_disabled";
      return true;
    }
    const std::int64_t now_ns = core::monotonicNowNs();
    const core::AuthorizationStatus status =
        session_->authorize(request.token, now_ns);
    response.accepted = status == core::AuthorizationStatus::kAuthorized;
    response.reason = adapter::authorizationStatusName(status);
    if (response.accepted) {
      ROS_WARN_STREAM("Local motion authorization accepted for generation="
                      << transport_->generation()
                      << "; this is not controller acknowledgement");
    } else {
      ROS_WARN_STREAM("Local motion authorization rejected: "
                      << response.reason);
    }
    return true;
  }

  bool onStop(std_srvs::Trigger::Request&,
              std_srvs::Trigger::Response& response) {
    if (!actuation_enabled_) {
      response.success = true;
      response.message = "offline; no serial device was opened";
      return true;
    }
    const std::int64_t now_ns = core::monotonicNowNs();
    response.success = session_->requestStop(now_ns);
    response.message = response.success
                           ? "local stop requested; VCU ACK unavailable"
                           : "local stop could not start a zero episode; outcome uncertain";
    return true;
  }

  bool onEmergencyStop(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    const std::int64_t now_ns = core::monotonicNowNs();
    session_->latchEmergencyStop(now_ns);
    response.success = true;
    response.message =
        "software E-stop latched locally; zero delivery is not acknowledged";
    ROS_ERROR("Software E-stop latched; physical E-stop remains independent");
    return true;
  }

  bool onResetEmergencyStop(adapter::ResetEstop::Request& request,
                            adapter::ResetEstop::Response& response) {
    const std::int64_t now_ns = core::monotonicNowNs();
    response.accepted = session_->resetEmergencyStop(request.token, now_ns);
    response.reason = response.accepted
                          ? "reset accepted; reauthorization required"
                          : "reset rejected; check token, feedback, zero, and connection";
    if (response.accepted) {
      ROS_WARN("Software E-stop reset; motion remains unauthorized");
    }
    return true;
  }

  void maintainConnection(std::int64_t now_ns) {
    if (transport_->connected() || now_ns < next_reconnect_ns_) {
      return;
    }
    const core::SerialOpenResult reopened = transport_->reopen();
    next_reconnect_ns_ = saturatingAdd(
        now_ns,
        millisecondsToNanoseconds(runtime_config_.reconnect_interval_ms));
    if (reopened.ok()) {
      feedback_parser_.reset();
      ROS_WARN_STREAM(
          "Serial reconnected; generation=" << reopened.generation
          << ". Initial zero, fresh allowed feedback, and a new higher "
             "authorization token are required. Host write is not a VCU ACK.");
    } else {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "Serial reconnect failed: status="
                   << adapter::transportStatusName(reopened.status)
                   << " os_error=" << reopened.os_error);
    }
  }

  void receiveFeedback(std::int64_t now_ns) {
    if (!transport_->connected()) {
      return;
    }
    std::uint8_t buffer[512U]{};
    const core::IoResult read = transport_->readSome(
        buffer, sizeof(buffer), saturatingAdd(now_ns, 1000000LL));
    if (read.status == core::TransportStatus::kDeadlineExceeded ||
        read.status == core::TransportStatus::kDisconnected) {
      return;
    }
    if (read.status != core::TransportStatus::kOk) {
      session_->latchEmergencyStop(now_ns);
      ROS_ERROR_STREAM("Serial read failed; software E-stop latched: status="
                       << adapter::transportStatusName(read.status));
      return;
    }

    const auto frames = feedback_parser_.consume(buffer, read.bytes_transferred);
    const std::int64_t receipt_ns =
        read.completion_monotonic_ns > 0 ? read.completion_monotonic_ns
                                         : core::monotonicNowNs();
    const ros::Time receipt_ros_time = ros::Time::now();
    for (const auto& parsed : frames) {
      const core::FeedbackObservationResult observation =
          session_->observeFeedback(parsed.frame, receipt_ns);
      feedback_publisher_.publish(
          adapter::makeFeedbackMessage(parsed.feedback, receipt_ros_time));
      if (!observation.valid_frame) {
        ROS_ERROR("Validated parser frame was rejected by the safety session");
      }
    }
  }

  void reportCycle(const core::CycleResult& cycle) {
    if (cycle.state != previous_state_) {
      ROS_INFO_STREAM("Safety session state="
                      << adapter::sessionStateName(cycle.state));
      previous_state_ = cycle.state;
    }
    if (cycle.action == core::CycleAction::kMotionHostWriteComplete) {
      ROS_DEBUG("Motion host write complete; VCU ACK unavailable");
    } else if (cycle.action == core::CycleAction::kZeroHostWriteComplete) {
      ROS_WARN("Zero host write complete; VCU ACK unavailable");
    } else if (cycle.action == core::CycleAction::kZeroHostWriteFailed ||
               cycle.action == core::CycleAction::kZeroRetriesExhausted) {
      ROS_ERROR_STREAM("Zero write did not complete: action="
                       << adapter::cycleActionName(cycle.action)
                       << " transport="
                       << adapter::transportStatusName(cycle.transport_status)
                       << " outcome_uncertain="
                       << (cycle.outcome_uncertain ? "true" : "false"));
    }
  }

  void publishState(bool force, std::int64_t now_ns) {
    const core::SessionState state = session_->state();
    const std::uint64_t generation = transport_->generation();
    const bool connected = transport_->connected();
    const bool authorized = session_->authorized();
    const bool estop = session_->emergencyStopLatched();
    const bool changed = state != last_published_state_ ||
                         generation != last_published_generation_ ||
                         connected != last_published_connected_ ||
                         authorized != last_published_authorized_ ||
                         estop != last_published_estop_;
    if (!force && !changed && last_state_publish_ns_ > 0 && now_ns > 0 &&
        now_ns - last_state_publish_ns_ < 1000000000LL) {
      return;
    }

    adapter::AdapterState message;
    message.receipt_time = ros::Time::now();
    message.session_state =
        actuation_enabled_ ? adapter::sessionStateName(state) : "offline";
    message.actuation_enabled = actuation_enabled_;
    message.connected = connected;
    message.connection_generation = generation;
    message.authorized = authorized;
    message.software_estop_latched = estop;
    message.vcu_ack_available = false;
    message.source_time_available = false;
    state_publisher_.publish(message);

    last_published_state_ = state;
    last_published_generation_ = generation;
    last_published_connected_ = connected;
    last_published_authorized_ = authorized;
    last_published_estop_ = estop;
    last_state_publish_ns_ = now_ns;
  }

  bool finishWithBoundedZero() {
    const std::int64_t disarm_ns = core::monotonicNowNs();
    if (disarm_ns > 0) {
      (void)session_->disarm(disarm_ns);
    }
    if (!transport_->connected()) {
      return false;
    }

    const core::CommandFrame zero = core::makeZeroCommandFrame();
    for (std::uint32_t attempt = 1U;
         attempt <= runtime_config_.zero_retry_attempts; ++attempt) {
      const std::int64_t now_ns = core::monotonicNowNs();
      if (now_ns <= 0) {
        break;
      }
      const core::IoResult write = transport_->writeAll(
          zero.data(), zero.size(),
          saturatingAdd(
              now_ns,
              millisecondsToNanoseconds(runtime_config_.write_timeout_ms)));
      if (write.status == core::TransportStatus::kOk &&
          write.host_write_complete &&
          write.bytes_transferred == zero.size()) {
        ROS_WARN_STREAM("Final zero host write complete on attempt=" << attempt
                        << "; VCU ACK unavailable");
        return true;
      }
      ROS_ERROR_STREAM("Final zero attempt=" << attempt
                       << " status="
                       << adapter::transportStatusName(write.status)
                       << " outcome_uncertain="
                       << (write.outcome_uncertain ? "true" : "false"));
      if (!transport_->connected()) {
        break;
      }
      ros::WallDuration(
          static_cast<double>(runtime_config_.zero_retry_interval_ms) /
          1000.0).sleep();
    }
    return false;
  }

  ros::NodeHandle private_node_;
  core::RuntimeConfig runtime_config_;
  bool actuation_enabled_{false};
  std::unique_ptr<core::PosixSerialTransport> transport_;
  std::unique_ptr<core::SafetySession> session_;
  core::FeedbackParser feedback_parser_;

  ros::Publisher feedback_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber command_subscriber_;
  ros::ServiceServer authorize_service_;
  ros::ServiceServer stop_service_;
  ros::ServiceServer emergency_stop_service_;
  ros::ServiceServer reset_emergency_stop_service_;

  std::int64_t next_reconnect_ns_{0};
  std::int64_t last_state_publish_ns_{0};
  core::SessionState previous_state_{core::SessionState::kConfigurationInvalid};
  core::SessionState last_published_state_{
      core::SessionState::kConfigurationInvalid};
  std::uint64_t last_published_generation_{0U};
  bool last_published_connected_{false};
  bool last_published_authorized_{false};
  bool last_published_estop_{false};
  bool finalization_performed_{false};
};

bool loadActuationGate(const ros::NodeHandle& private_node,
                       adapter::ActuationGate* gate,
                       std::string* reason) {
  if (gate == nullptr || reason == nullptr) {
    return false;
  }
  const bool has_ack = private_node.getParam(
      "acknowledge_unverified_protocol",
      gate->acknowledge_unverified_protocol);
  const bool has_enable =
      private_node.getParam("enable_actuation", gate->enable_actuation);
  const bool has_confirmation = private_node.getParam(
      "operator_confirmation", gate->operator_confirmation);
  if (!has_ack || !has_enable || !has_confirmation) {
    *reason = "all three actuation gate parameters must be present";
    return false;
  }
  *reason = "loaded";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheeltec_vcu_serial_adapter",
            ros::init_options::NoSigintHandler);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);

  ros::NodeHandle private_node("~");
  std::string config_file;
  if (!private_node.getParam("config_file", config_file) ||
      config_file.empty()) {
    ROS_FATAL("Required private parameter ~config_file is missing or empty; "
              "serial was not opened");
    return 2;
  }

  const core::ConfigResult loaded =
      core::loadRuntimeConfig(config_file, false);
  if (!loaded.ok()) {
    ROS_FATAL_STREAM("Configuration rejected before serial open: "
                     << core::configErrorName(loaded.error)
                     << " message=" << loaded.message);
    return 3;
  }

  adapter::ActuationGate gate;
  std::string gate_reason;
  if (!loadActuationGate(private_node, &gate, &gate_reason)) {
    ROS_FATAL_STREAM("Actuation gate parameters are invalid; serial was not opened: "
                     << gate_reason);
    return 4;
  }
  const adapter::ActuationGateMode gate_mode =
      adapter::classifyActuationGate(gate, &gate_reason);
  if (gate_mode == adapter::ActuationGateMode::kInvalid) {
    ROS_FATAL_STREAM("Actuation gates fail closed; serial was not opened: "
                     << gate_reason);
    return 4;
  }
  const bool actuation_enabled =
      gate_mode == adapter::ActuationGateMode::kActuation;
  if (actuation_enabled) {
    const core::ConfigResult actuation_config =
        core::validateRuntimeConfig(loaded.config, true);
    if (!actuation_config.ok()) {
      ROS_FATAL_STREAM("Actuation configuration rejected before serial open: "
                       << core::configErrorName(actuation_config.error)
                       << " message=" << actuation_config.message);
      return 3;
    }
  }

  try {
    if (actuation_enabled) {
      ROS_WARN("All host actuation gates are open. The serial protocol remains "
               "unverified on this vehicle, and host write completion is not a "
               "VCU ACK. Keep an independent physical E-stop available.");
    } else {
      ROS_INFO("All actuation gates are disabled: starting persistent offline "
               "mode without opening a serial device");
    }
    RosAdapter node(private_node, loaded.config, actuation_enabled);
    const int result = node.run();
    ros::shutdown();
    return result;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Adapter construction/finalization failed: "
                     << error.what());
  } catch (...) {
    ROS_FATAL("Adapter construction/finalization failed with unknown exception");
  }
  ros::shutdown();
  return 11;
}
