// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/feedback_parser.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace protocol = wheeltec_vcu_serial;

namespace {

int failures = 0;

void check(bool condition, const char* description) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", description);
    ++failures;
  }
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

protocol::FeedbackFrame makeFrame(std::int16_t linear,
                                  std::uint8_t stop_flag = 0U) {
  protocol::FeedbackFrame frame{};
  frame[0U] = protocol::kFrameHeader;
  frame[1U] = stop_flag;
  putSigned(linear, &frame[2U]);
  putSigned(-50, &frame[4U]);
  putSigned(25, &frame[6U]);
  putSigned(300, &frame[8U]);
  putSigned(-200, &frame[10U]);
  putSigned(100, &frame[12U]);
  putSigned(-60, &frame[14U]);
  putSigned(40, &frame[16U]);
  putSigned(-20, &frame[18U]);
  frame[20U] = 0x30U;
  frame[21U] = 0x39U;
  frame[22U] = checksum(frame.data(), 22U);
  frame[23U] = protocol::kFrameTail;
  return frame;
}

void testEveryTwoChunkBoundary() {
  const protocol::FeedbackFrame frame = makeFrame(731);
  for (std::size_t split = 0U; split <= frame.size(); ++split) {
    protocol::FeedbackParser parser;
    std::vector<protocol::ParsedFeedbackFrame> first =
        parser.consume(frame.data(), split);
    std::vector<protocol::ParsedFeedbackFrame> second =
        parser.consume(frame.data() + split, frame.size() - split);
    check(first.size() + second.size() == 1U,
          "each possible two-chunk split produces one frame");
    check(parser.statistics().valid_frames == 1U &&
              parser.retainedByteCount() == 0U,
          "each split leaves an empty parser after the complete frame");
  }

  protocol::FeedbackParser bytewise_parser;
  std::size_t produced = 0U;
  for (const std::uint8_t byte : frame) {
    produced += bytewise_parser.consume(&byte, 1U).size();
    check(bytewise_parser.retainedByteCount() <= 23U,
          "bytewise parsing never retains a complete candidate");
  }
  check(produced == 1U, "one-byte chunks reconstruct a frame");
}

void testNoiseAndBackToBackFrames() {
  const protocol::FeedbackFrame first = makeFrame(101);
  const protocol::FeedbackFrame second = makeFrame(-202, 1U);
  std::vector<std::uint8_t> stream{0x00U, 0x55U, 0xAAU, 0x7DU};
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());

  protocol::FeedbackParser parser;
  const std::vector<protocol::ParsedFeedbackFrame> parsed =
      parser.consume(stream.data(), stream.size());
  check(parsed.size() == 2U, "noise and back-to-back frames are separated");
  check(parsed[0U].feedback.linear_speed_mps == 0.101 &&
            parsed[1U].feedback.linear_speed_mps == -0.202,
        "frame order and signed values survive stream parsing");
  check(parsed[1U].feedback.control_inhibited,
        "a parsed FlagStop one frame remains inhibited");
  check(parser.statistics().valid_frames == 2U &&
            parser.statistics().discarded_bytes == 4U,
        "leading noise is counted without affecting valid frames");
}

void testCorruptCandidatesResynchronize() {
  protocol::FeedbackFrame bad_xor = makeFrame(303);
  bad_xor[22U] ^= 0x80U;
  protocol::FeedbackFrame bad_flag = makeFrame(404, 2U);
  const protocol::FeedbackFrame valid = makeFrame(505);

  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), bad_xor.begin(), bad_xor.end());
  stream.insert(stream.end(), bad_flag.begin(), bad_flag.end());
  stream.insert(stream.end(), valid.begin(), valid.end());

  protocol::FeedbackParser parser;
  const std::vector<protocol::ParsedFeedbackFrame> parsed =
      parser.consume(stream.data(), stream.size());
  check(parsed.size() == 1U &&
            parsed[0U].feedback.linear_speed_mps == 0.505,
        "bad XOR and bad FlagStop candidates do not hide a later frame");
  check(parser.statistics().checksum_failures >= 1U,
        "checksum failures are counted");
  check(parser.statistics().framing_failures >= 1U,
        "non-checksum candidate failures are counted");
  check(parser.retainedByteCount() <= 23U,
        "corruption recovery preserves the fixed retention bound");
}

void testEmbeddedHeaderRecovery() {
  const protocol::FeedbackFrame valid = makeFrame(-616);
  std::vector<std::uint8_t> stream{
      protocol::kFrameHeader, 0x10U, 0x20U, 0x30U, 0x40U, 0x50U};
  stream.insert(stream.end(), valid.begin(), valid.end());

  protocol::FeedbackParser parser;
  const std::vector<protocol::ParsedFeedbackFrame> parsed =
      parser.consume(stream.data(), stream.size());
  check(parsed.size() == 1U && parsed[0U].frame == valid,
        "an embedded header becomes the next candidate after framing failure");
  check(parser.statistics().framing_failures >= 1U &&
            parser.statistics().discarded_bytes >= 6U,
        "bytes before the embedded header are accounted for");
}

void testOneMiBNoiseUsesBoundedState() {
  std::array<std::uint8_t, 4096U> noise{};
  // A header storm is the adversarial case: every invalid 24-byte candidate
  // has another possible start one byte later.  Reuse one fixed chunk so the
  // test itself does not allocate a one-MiB input buffer.
  noise.fill(protocol::kFrameHeader);
  protocol::FeedbackParser parser;
  std::size_t outputs = 0U;
  constexpr std::size_t kChunks = (1024U * 1024U) / noise.size();
  for (std::size_t index = 0U; index < kChunks; ++index) {
    outputs += parser.consume(noise.data(), noise.size()).size();
    check(parser.retainedByteCount() <= 23U,
          "large-noise parsing keeps at most a partial frame");
  }
  constexpr std::uint64_t kNoiseBytes = 1024U * 1024U;
  check(outputs == 0U, "one MiB of adversarial header noise emits no frame");
  check(parser.retainedByteCount() == 23U &&
            parser.statistics().discarded_bytes == kNoiseBytes - 23U &&
            parser.statistics().framing_failures == kNoiseBytes - 23U,
        "one MiB header storm retains only the final 23-byte prefix");
}

void testResetAndNullInput() {
  const protocol::FeedbackFrame frame = makeFrame(77);
  protocol::FeedbackParser parser;
  parser.consume(frame.data(), 9U);
  check(parser.retainedByteCount() == 9U,
        "partial state is visible before reset");
  parser.reset();
  check(parser.retainedByteCount() == 0U &&
            parser.statistics().discarded_bytes == 0U,
        "reset clears state and statistics");
  check(parser.consume(nullptr, 5U).empty() &&
            parser.statistics().framing_failures == 1U &&
            parser.statistics().discarded_bytes == 5U,
        "null non-empty input fails closed without dereferencing");
}

}  // namespace

int main() {
  testEveryTwoChunkBoundary();
  testNoiseAndBackToBackFrames();
  testCorruptCandidatesResynchronize();
  testEmbeddedHeaderRecovery();
  testOneMiBNoiseUsesBoundedState();
  testResetAndNullInput();
  if (failures != 0) {
    std::fprintf(stderr, "%d parser assertion(s) failed\n", failures);
    return 1;
  }
  std::puts("PASS: feedback stream parser");
  return 0;
}
