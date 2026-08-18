// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace protocol = wheeltec_vcu_serial;

namespace {

int failures = 0;

void check(bool condition, const char* description) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", description);
    ++failures;
  }
}

void checkNear(double actual, double expected, double tolerance,
               const char* description) {
  check(std::fabs(actual - expected) <= tolerance, description);
}

std::uint8_t checksum(const std::uint8_t* bytes, std::size_t size) {
  std::uint8_t result = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    result = static_cast<std::uint8_t>(result ^ bytes[index]);
  }
  return result;
}

void putSigned(std::int16_t value, std::uint8_t* destination) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  destination[0U] = static_cast<std::uint8_t>(bits >> 8U);
  destination[1U] = static_cast<std::uint8_t>(bits & 0xFFU);
}

protocol::FeedbackFrame syntheticFeedback(std::uint8_t stop_flag) {
  protocol::FeedbackFrame frame{};
  frame[0U] = protocol::kFrameHeader;
  frame[1U] = stop_flag;
  putSigned(1234, &frame[2U]);
  putSigned(-321, &frame[4U]);
  putSigned(42, &frame[6U]);
  putSigned(2048, &frame[8U]);
  putSigned(-1024, &frame[10U]);
  putSigned(17, &frame[12U]);
  putSigned(-750, &frame[14U]);
  putSigned(333, &frame[16U]);
  putSigned(-1, &frame[18U]);
  frame[20U] = 0x62U;
  frame[21U] = 0x23U;
  frame[22U] = checksum(frame.data(), 22U);
  frame[23U] = protocol::kFrameTail;
  return frame;
}

void testZeroFrame() {
  const protocol::CommandFrame expected{
      {0x7BU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
       0x00U, 0x00U, 0x00U, 0x7BU, 0x7DU}};
  check(protocol::makeZeroCommandFrame() == expected,
        "zero frame is available without a motion limit");
}

void testExplicitLimitValidation() {
  check(protocol::isValidMaxLinearSpeed(0.001),
        "a positive finite limit is valid");
  check(protocol::isValidMaxLinearSpeed(6.0),
        "the configured six metre-per-second ceiling is valid");
  check(!protocol::isValidMaxLinearSpeed(0.0), "zero limit is invalid");
  check(!protocol::isValidMaxLinearSpeed(-0.1), "negative limit is invalid");
  check(!protocol::isValidMaxLinearSpeed(6.001),
        "a limit above the configuration bound is invalid");
  check(!protocol::isValidMaxLinearSpeed(
            std::numeric_limits<double>::quiet_NaN()),
        "NaN limit is invalid");
  check(!protocol::isValidMaxLinearSpeed(
            std::numeric_limits<double>::infinity()),
        "infinite limit is invalid");

  const protocol::MotionCommand stopped{};
  check(protocol::encodeCommand(stopped, 0.0).error ==
            protocol::EncodeError::kInvalidMaxLinearSpeed,
        "normal encoder fails closed when the explicit limit is absent");
}

void testSignedEncodingAndTruncation() {
  const protocol::MotionCommand positive{1.2349, -0.4569, 0.0789};
  const protocol::CommandFrame positive_expected{
      {0x7BU, 0x00U, 0x00U, 0x04U, 0xD2U, 0xFEU,
       0x38U, 0x00U, 0x4EU, 0x25U, 0x7DU}};
  const protocol::EncodeResult positive_result =
      protocol::encodeCommand(positive, 2.0);
  check(positive_result.ok() && positive_result.frame == positive_expected,
        "positive command fields use signed big-endian truncation");

  const protocol::MotionCommand reverse{-0.8759, 0.0019, -0.2229};
  const protocol::CommandFrame reverse_expected{
      {0x7BU, 0x00U, 0x00U, 0xFCU, 0x95U, 0x00U,
       0x01U, 0xFFU, 0x22U, 0xCEU, 0x7DU}};
  const protocol::EncodeResult reverse_result =
      protocol::encodeCommand(reverse, 1.0);
  check(reverse_result.ok() && reverse_result.frame == reverse_expected,
        "negative linear and yaw values retain their signs");

  check(protocol::encodeCommand(protocol::MotionCommand{2.0, 0.0, 0.0}, 2.0)
            .ok(),
        "linear speed equal to the explicit limit is accepted");
  check(protocol::encodeCommand(protocol::MotionCommand{6.0, 0.0, 0.0}, 6.0)
            .ok(),
        "six metres per second fits the signed wire field and configured limit");
  check(protocol::encodeCommand(protocol::MotionCommand{2.001, 0.0, 0.0}, 2.0)
            .error == protocol::EncodeError::kLinearSpeedLimitExceeded,
        "positive speed above the explicit limit is rejected");
  check(protocol::encodeCommand(protocol::MotionCommand{-2.001, 0.0, 0.0}, 2.0)
            .error == protocol::EncodeError::kLinearSpeedLimitExceeded,
        "reverse speed uses the same absolute limit");
}

void testNonFiniteAndWireBounds() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  check(protocol::encodeCommand(protocol::MotionCommand{nan, 0.0, 0.0}, 1.0)
            .error == protocol::EncodeError::kNonFiniteLinearSpeed,
        "non-finite linear speed has a named error");
  check(protocol::encodeCommand(protocol::MotionCommand{0.0, infinity, 0.0}, 1.0)
            .error == protocol::EncodeError::kNonFiniteLateralSpeed,
        "non-finite lateral speed has a named error");
  check(protocol::encodeCommand(protocol::MotionCommand{0.0, 0.0, -infinity}, 1.0)
            .error == protocol::EncodeError::kNonFiniteYawRate,
        "non-finite yaw rate has a named error");

  check(protocol::encodeCommand(protocol::MotionCommand{33.0, 0.0, 0.0}, 5.0)
            .error == protocol::EncodeError::kLinearWireRange,
        "linear wire range is checked independently");
  check(protocol::encodeCommand(protocol::MotionCommand{0.0, 32.768, 0.0}, 1.0)
            .error == protocol::EncodeError::kLateralWireRange,
        "positive lateral wire overflow is rejected");
  check(protocol::encodeCommand(protocol::MotionCommand{0.0, -32.769, 0.0}, 1.0)
            .error == protocol::EncodeError::kLateralWireRange,
        "negative lateral wire overflow is rejected");
  check(protocol::encodeCommand(protocol::MotionCommand{0.0, 0.0, 32.768}, 1.0)
            .error == protocol::EncodeError::kYawRateWireRange,
        "yaw wire overflow is rejected");

  const protocol::EncodeResult positive_edge = protocol::encodeCommand(
      protocol::MotionCommand{0.0, 32.7679, 0.0}, 1.0);
  check(positive_edge.ok() && positive_edge.frame[5U] == 0x7FU &&
            positive_edge.frame[6U] == 0xFFU,
        "largest positive int16 result remains representable after truncation");
  const protocol::EncodeResult negative_edge = protocol::encodeCommand(
      protocol::MotionCommand{0.0, -32.7689, 0.0}, 1.0);
  check(negative_edge.ok() && negative_edge.frame[5U] == 0x80U &&
            negative_edge.frame[6U] == 0x00U,
        "smallest negative int16 result remains representable after truncation");
}

void testFeedbackDecode() {
  const protocol::FeedbackFrame frame = syntheticFeedback(0U);
  const protocol::FeedbackDecodeResult decoded =
      protocol::decodeFeedbackFrame(frame);
  check(decoded.ok(), "synthetic feedback frame decodes");
  check(decoded.feedback.composite_stop_flag_raw == 0U &&
            decoded.feedback.control_allowed &&
            !decoded.feedback.control_inhibited,
        "zero composite stop flag derives an allowed state");
  checkNear(decoded.feedback.linear_speed_mps, 1.234, 1e-12,
            "linear feedback uses signed millimetre scaling");
  checkNear(decoded.feedback.lateral_speed_mps, -0.321, 1e-12,
            "lateral feedback remains signed");
  checkNear(decoded.feedback.yaw_rate_radps, 0.042, 1e-12,
            "yaw feedback uses signed milliradian scaling");
  checkNear(decoded.feedback.linear_acceleration_mps2[0U],
            2048.0 / 1671.84, 1e-12,
            "accelerometer conversion is applied");
  checkNear(decoded.feedback.angular_velocity_radps[0U],
            -750.0 * 0.00026644, 1e-12,
            "gyroscope conversion is applied");
  checkNear(decoded.feedback.supply_voltage_v, 25.123, 1e-12,
            "voltage decodes from an unsigned big-endian millivolt field");
  check(!decoded.feedback.vcu_ack_available &&
            !decoded.feedback.source_time_available,
        "feedback explicitly exposes neither ACK nor source time");

  const protocol::FeedbackDecodeResult inhibited =
      protocol::decodeFeedbackFrame(syntheticFeedback(1U));
  check(inhibited.ok() && !inhibited.feedback.control_allowed &&
            inhibited.feedback.control_inhibited,
        "one composite stop flag is a valid inhibited frame");

  protocol::FeedbackFrame invalid_flag = syntheticFeedback(2U);
  check(protocol::decodeFeedbackFrame(invalid_flag).error ==
            protocol::FeedbackDecodeError::kInvalidCompositeStopFlag,
        "a checksum-consistent stop flag outside the binary domain is rejected");

  protocol::FeedbackFrame corrupt = frame;
  corrupt[22U] ^= 0x40U;
  check(protocol::decodeFeedbackFrame(corrupt).error ==
            protocol::FeedbackDecodeError::kChecksumMismatch,
        "feedback XOR is verified");
  corrupt = frame;
  corrupt[0U] = 0U;
  check(protocol::decodeFeedbackFrame(corrupt).error ==
            protocol::FeedbackDecodeError::kInvalidHeader,
        "feedback header is verified");
  corrupt = frame;
  corrupt[23U] = 0U;
  check(protocol::decodeFeedbackFrame(corrupt).error ==
            protocol::FeedbackDecodeError::kInvalidTail,
        "feedback tail is verified");
  check(protocol::decodeFeedbackFrame(frame.data(), frame.size() - 1U).error ==
            protocol::FeedbackDecodeError::kInvalidFrameSize,
        "partial fixed frame is rejected by the decoder");
  check(protocol::decodeFeedbackFrame(nullptr, frame.size()).error ==
            protocol::FeedbackDecodeError::kInvalidFrameSize,
        "null feedback input is rejected");
}

}  // namespace

int main() {
  testZeroFrame();
  testExplicitLimitValidation();
  testSignedEncodingAndTruncation();
  testNonFiniteAndWireBounds();
  testFeedbackDecode();
  if (failures != 0) {
    std::fprintf(stderr, "%d protocol assertion(s) failed\n", failures);
    return 1;
  }
  std::puts("PASS: protocol codec");
  return 0;
}
