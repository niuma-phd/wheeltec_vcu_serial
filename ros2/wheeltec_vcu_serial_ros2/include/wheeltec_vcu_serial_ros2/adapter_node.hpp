// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wheeltec_vcu_serial/config.hpp"
#include "wheeltec_vcu_serial/feedback_parser.hpp"
#include "wheeltec_vcu_serial/safety_session.hpp"
#include "wheeltec_vcu_serial/transport.hpp"
#include "wheeltec_vcu_serial_ros2/msg/adapter_state.hpp"
#include "wheeltec_vcu_serial_ros2/msg/drive_command.hpp"
#include "wheeltec_vcu_serial_ros2/msg/feedback.hpp"
#include "wheeltec_vcu_serial_ros2/srv/authorize.hpp"
#include "wheeltec_vcu_serial_ros2/srv/reset_estop.hpp"

namespace wheeltec_vcu_serial_ros2 {

class AdapterNode final : public rclcpp::Node {
 public:
  explicit AdapterNode(const rclcpp::NodeOptions& options =
                           rclcpp::NodeOptions());
  ~AdapterNode() override;

  AdapterNode(const AdapterNode&) = delete;
  AdapterNode& operator=(const AdapterNode&) = delete;

  // Idempotent, bounded best-effort zero sequence for an orderly shutdown.
  // Completion is only a host I/O observation, never a controller ACK.
  // False means the process must report a failed/uncertain final zero.
  bool shutdownSafely() noexcept;

 private:
  using DriveCommand = msg::DriveCommand;
  using Feedback = msg::Feedback;
  using AdapterState = msg::AdapterState;
  using Authorize = srv::Authorize;
  using ResetEstop = srv::ResetEstop;
  using Trigger = std_srvs::srv::Trigger;

  void prepareActiveTransport();
  void openActiveTransport();
  void onDriveCommand(DriveCommand::ConstSharedPtr message);
  void onAuthorize(const Authorize::Request::SharedPtr request,
                   Authorize::Response::SharedPtr response);
  void onStop(const Trigger::Request::SharedPtr request,
              Trigger::Response::SharedPtr response);
  void onEmergencyStop(const Trigger::Request::SharedPtr request,
                       Trigger::Response::SharedPtr response);
  void onResetEstop(const ResetEstop::Request::SharedPtr request,
                    ResetEstop::Response::SharedPtr response);
  void onIoTimer();
  void readFeedback(std::int64_t now_monotonic_ns);
  void tryReconnect(std::int64_t now_monotonic_ns);
  void publishState(std::int64_t now_monotonic_ns, bool force);
  void reportCycle(const wheeltec_vcu_serial::CycleResult& cycle);

  wheeltec_vcu_serial::RuntimeConfig runtime_config_{};
  bool actuation_enabled_{false};
  bool shutdown_started_{false};
  bool shutdown_succeeded_{true};
  bool local_software_estop_latched_{false};
  std::unique_ptr<wheeltec_vcu_serial::PosixSerialTransport> transport_;
  std::unique_ptr<wheeltec_vcu_serial::SafetySession> session_;
  wheeltec_vcu_serial::FeedbackParser parser_;

  rclcpp::Subscription<DriveCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<AdapterState>::SharedPtr state_publisher_;
  rclcpp::Service<Authorize>::SharedPtr authorize_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr estop_service_;
  rclcpp::Service<ResetEstop>::SharedPtr reset_estop_service_;
  rclcpp::TimerBase::SharedPtr io_timer_;

  std::int64_t next_reconnect_ns_{0};
  std::int64_t next_state_publish_ns_{0};
  wheeltec_vcu_serial::SessionState last_reported_state_{
      wheeltec_vcu_serial::SessionState::kConfigurationInvalid};
};

}  // namespace wheeltec_vcu_serial_ros2
