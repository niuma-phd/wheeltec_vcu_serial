// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/feedback_parser.hpp"

#include <algorithm>

namespace wheeltec_vcu_serial {

std::vector<ParsedFeedbackFrame> FeedbackParser::consume(
    const std::uint8_t* data, std::size_t size) {
  std::vector<ParsedFeedbackFrame> parsed_frames;
  if (size == 0U) {
    return parsed_frames;
  }
  if (data == nullptr) {
    ++statistics_.framing_failures;
    statistics_.discarded_bytes += size;
    return parsed_frames;
  }

  for (std::size_t input_index = 0U; input_index < size; ++input_index) {
    const std::uint8_t byte = data[input_index];
    if (retained_size_ == 0U && byte != kFrameHeader) {
      ++statistics_.discarded_bytes;
      continue;
    }

    retained_[retained_size_] = byte;
    ++retained_size_;
    if (retained_size_ < kFeedbackFrameSize) {
      continue;
    }

    const FeedbackDecodeResult decoded = decodeFeedbackFrame(retained_);
    if (decoded.ok()) {
      ParsedFeedbackFrame parsed;
      parsed.frame = retained_;
      parsed.feedback = decoded.feedback;
      // Consume the candidate before growing the caller-visible result.  If
      // vector allocation throws, the parser still preserves its fixed-state
      // invariant and never resumes with retained_size_ == frame capacity.
      retained_size_ = 0U;
      parsed_frames.push_back(parsed);
      ++statistics_.valid_frames;
      continue;
    }

    if (decoded.error == FeedbackDecodeError::kChecksumMismatch) {
      ++statistics_.checksum_failures;
    } else {
      ++statistics_.framing_failures;
    }
    resynchronizeAfterInvalidCandidate();
  }
  return parsed_frames;
}

void FeedbackParser::reset() noexcept {
  retained_.fill(0U);
  retained_size_ = 0U;
  statistics_ = FeedbackParserStatistics{};
}

void FeedbackParser::resynchronizeAfterInvalidCandidate() noexcept {
  const auto next_header =
      std::find(retained_.begin() + 1, retained_.end(), kFrameHeader);
  const std::size_t discard_count = static_cast<std::size_t>(
      std::distance(retained_.begin(), next_header));
  statistics_.discarded_bytes += discard_count;
  if (next_header == retained_.end()) {
    retained_size_ = 0U;
    return;
  }

  const std::size_t suffix_size = kFeedbackFrameSize - discard_count;
  std::move(next_header, retained_.end(), retained_.begin());
  retained_size_ = suffix_size;
}

}  // namespace wheeltec_vcu_serial
