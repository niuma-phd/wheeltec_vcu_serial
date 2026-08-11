// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/protocol.hpp"

#include <cmath>
#include <limits>

namespace wheeltec_vcu_serial {
namespace {

constexpr double kMotionUnitsPerWireUnit = 1000.0;
constexpr double kAccelerometerDivisor = 1671.84;
constexpr double kGyroscopeMultiplier = 0.00026644;

std::uint8_t xorRange(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint8_t value = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    value = static_cast<std::uint8_t>(value ^ data[index]);
  }
  return value;
}

bool quantizeWireValue(double value, std::int16_t* output) noexcept {
  const double scaled = std::trunc(value * kMotionUnitsPerWireUnit);
  if (!std::isfinite(scaled) ||
      scaled < static_cast<double>(std::numeric_limits<std::int16_t>::min()) ||
      scaled > static_cast<double>(std::numeric_limits<std::int16_t>::max())) {
    return false;
  }
  *output = static_cast<std::int16_t>(scaled);
  return true;
}

void storeSignedBigEndian(std::int16_t value,
                          std::uint8_t* destination) noexcept {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  destination[0U] = static_cast<std::uint8_t>(bits >> 8U);
  destination[1U] = static_cast<std::uint8_t>(bits & 0xFFU);
}

std::int16_t loadSignedBigEndian(const std::uint8_t* source) noexcept {
  const std::uint16_t bits = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(source[0U]) << 8U) |
      static_cast<std::uint16_t>(source[1U]));
  std::int32_t widened = static_cast<std::int32_t>(bits);
  if ((bits & 0x8000U) != 0U) {
    widened -= 0x10000;
  }
  return static_cast<std::int16_t>(widened);
}

std::uint16_t loadUnsignedBigEndian(const std::uint8_t* source) noexcept {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(source[0U]) << 8U) |
      static_cast<std::uint16_t>(source[1U]));
}

EncodeResult encodeFailure(EncodeError error) noexcept {
  EncodeResult result;
  result.error = error;
  return result;
}

}  // namespace

bool isValidMaxLinearSpeed(double value_mps) noexcept {
  return std::isfinite(value_mps) && value_mps > 0.0 && value_mps < 6.0;
}

EncodeResult encodeCommand(
    const MotionCommand& command,
    double explicit_max_linear_speed_mps) noexcept {
  if (!isValidMaxLinearSpeed(explicit_max_linear_speed_mps)) {
    return encodeFailure(EncodeError::kInvalidMaxLinearSpeed);
  }
  if (!std::isfinite(command.linear_speed_mps)) {
    return encodeFailure(EncodeError::kNonFiniteLinearSpeed);
  }
  if (!std::isfinite(command.lateral_speed_mps)) {
    return encodeFailure(EncodeError::kNonFiniteLateralSpeed);
  }
  if (!std::isfinite(command.yaw_rate_radps)) {
    return encodeFailure(EncodeError::kNonFiniteYawRate);
  }

  std::int16_t linear_wire = 0;
  std::int16_t lateral_wire = 0;
  std::int16_t yaw_wire = 0;
  if (!quantizeWireValue(command.linear_speed_mps, &linear_wire)) {
    return encodeFailure(EncodeError::kLinearWireRange);
  }
  if (!quantizeWireValue(command.lateral_speed_mps, &lateral_wire)) {
    return encodeFailure(EncodeError::kLateralWireRange);
  }
  if (!quantizeWireValue(command.yaw_rate_radps, &yaw_wire)) {
    return encodeFailure(EncodeError::kYawRateWireRange);
  }
  if (std::fabs(command.linear_speed_mps) >
      explicit_max_linear_speed_mps) {
    return encodeFailure(EncodeError::kLinearSpeedLimitExceeded);
  }

  EncodeResult result;
  result.frame[0U] = kFrameHeader;
  result.frame[1U] = 0U;
  result.frame[2U] = 0U;
  storeSignedBigEndian(linear_wire, &result.frame[3U]);
  storeSignedBigEndian(lateral_wire, &result.frame[5U]);
  storeSignedBigEndian(yaw_wire, &result.frame[7U]);
  result.frame[9U] = xorRange(result.frame.data(), 9U);
  result.frame[10U] = kFrameTail;
  result.error = EncodeError::kNone;
  return result;
}

CommandFrame makeZeroCommandFrame() noexcept {
  CommandFrame frame{};
  frame[0U] = kFrameHeader;
  frame[9U] = kFrameHeader;
  frame[10U] = kFrameTail;
  return frame;
}

FeedbackDecodeResult decodeFeedbackFrame(const std::uint8_t* data,
                                         std::size_t size) noexcept {
  FeedbackDecodeResult result;
  if (data == nullptr || size != kFeedbackFrameSize) {
    result.error = FeedbackDecodeError::kInvalidFrameSize;
    return result;
  }
  if (data[0U] != kFrameHeader) {
    result.error = FeedbackDecodeError::kInvalidHeader;
    return result;
  }
  if (data[23U] != kFrameTail) {
    result.error = FeedbackDecodeError::kInvalidTail;
    return result;
  }
  if (xorRange(data, 22U) != data[22U]) {
    result.error = FeedbackDecodeError::kChecksumMismatch;
    return result;
  }
  if (data[1U] > 1U) {
    result.error = FeedbackDecodeError::kInvalidCompositeStopFlag;
    return result;
  }

  result.feedback.composite_stop_flag_raw = data[1U];
  result.feedback.control_allowed = data[1U] == 0U;
  result.feedback.control_inhibited = data[1U] == 1U;
  result.feedback.linear_speed_mps =
      static_cast<double>(loadSignedBigEndian(&data[2U])) /
      kMotionUnitsPerWireUnit;
  result.feedback.lateral_speed_mps =
      static_cast<double>(loadSignedBigEndian(&data[4U])) /
      kMotionUnitsPerWireUnit;
  result.feedback.yaw_rate_radps =
      static_cast<double>(loadSignedBigEndian(&data[6U])) /
      kMotionUnitsPerWireUnit;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    result.feedback.linear_acceleration_mps2[axis] =
        static_cast<double>(loadSignedBigEndian(&data[8U + axis * 2U])) /
        kAccelerometerDivisor;
    result.feedback.angular_velocity_radps[axis] =
        static_cast<double>(loadSignedBigEndian(&data[14U + axis * 2U])) *
        kGyroscopeMultiplier;
  }
  result.feedback.supply_voltage_v =
      static_cast<double>(loadUnsignedBigEndian(&data[20U])) /
      kMotionUnitsPerWireUnit;
  result.feedback.vcu_ack_available = false;
  result.feedback.source_time_available = false;
  result.error = FeedbackDecodeError::kNone;
  return result;
}

FeedbackDecodeResult decodeFeedbackFrame(const FeedbackFrame& frame) noexcept {
  return decodeFeedbackFrame(frame.data(), frame.size());
}

}  // namespace wheeltec_vcu_serial
