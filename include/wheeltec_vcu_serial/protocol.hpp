// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wheeltec_vcu_serial {

constexpr std::uint8_t kFrameHeader = 0x7BU;
constexpr std::uint8_t kFrameTail = 0x7DU;
constexpr std::size_t kCommandFrameSize = 11U;
constexpr std::size_t kFeedbackFrameSize = 24U;

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;
using FeedbackFrame = std::array<std::uint8_t, kFeedbackFrameSize>;

struct MotionCommand {
  double linear_speed_mps{0.0};
  double lateral_speed_mps{0.0};
  double yaw_rate_radps{0.0};
};

enum class EncodeError : std::uint8_t {
  kNone = 0,
  kInvalidMaxLinearSpeed,
  kNonFiniteLinearSpeed,
  kNonFiniteLateralSpeed,
  kNonFiniteYawRate,
  kLinearWireRange,
  kLateralWireRange,
  kYawRateWireRange,
  kLinearSpeedLimitExceeded,
};

struct EncodeResult {
  CommandFrame frame{};
  EncodeError error{EncodeError::kInvalidMaxLinearSpeed};

  bool ok() const noexcept { return error == EncodeError::kNone; }
};

bool isValidMaxLinearSpeed(double value_mps) noexcept;

// The limit is mandatory on every motion-encoding call.  There is no library
// fallback that can silently authorize a nonzero command.
EncodeResult encodeCommand(
    const MotionCommand& command,
    double explicit_max_linear_speed_mps) noexcept;

// Emergency-stop and shutdown paths can always construct the protocol's exact
// zero frame, even when motion configuration is absent or invalid.
CommandFrame makeZeroCommandFrame() noexcept;

struct FeedbackData {
  // Defaults are deliberately inhibited.  A successful decode is required
  // before control_allowed can become true.
  std::uint8_t composite_stop_flag_raw{1U};
  bool control_allowed{false};
  bool control_inhibited{true};
  double linear_speed_mps{0.0};
  double lateral_speed_mps{0.0};
  double yaw_rate_radps{0.0};
  std::array<double, 3U> linear_acceleration_mps2{};
  std::array<double, 3U> angular_velocity_radps{};
  double supply_voltage_v{0.0};
  bool vcu_ack_available{false};
  bool source_time_available{false};
};

enum class FeedbackDecodeError : std::uint8_t {
  kNone = 0,
  kInvalidFrameSize,
  kInvalidHeader,
  kInvalidTail,
  kChecksumMismatch,
  kInvalidCompositeStopFlag,
};

struct FeedbackDecodeResult {
  FeedbackData feedback{};
  FeedbackDecodeError error{FeedbackDecodeError::kInvalidFrameSize};

  bool ok() const noexcept { return error == FeedbackDecodeError::kNone; }
};

FeedbackDecodeResult decodeFeedbackFrame(const std::uint8_t* data,
                                         std::size_t size) noexcept;

FeedbackDecodeResult decodeFeedbackFrame(const FeedbackFrame& frame) noexcept;

}  // namespace wheeltec_vcu_serial
