// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "wheeltec_vcu_serial/protocol.hpp"

namespace wheeltec_vcu_serial {

struct ParsedFeedbackFrame {
  FeedbackFrame frame{};
  FeedbackData feedback{};
};

struct FeedbackParserStatistics {
  std::uint64_t valid_frames{0U};
  std::uint64_t checksum_failures{0U};
  std::uint64_t framing_failures{0U};
  std::uint64_t discarded_bytes{0U};
};

class FeedbackParser {
 public:
  // Persistent parser storage is fixed.  Between calls only a possible frame
  // prefix (at most 23 bytes) is retained; the returned vector grows only for
  // fully validated feedback frames, never in proportion to discarded noise.
  std::vector<ParsedFeedbackFrame> consume(const std::uint8_t* data,
                                           std::size_t size);

  void reset() noexcept;

  std::size_t retainedByteCount() const noexcept { return retained_size_; }

  const FeedbackParserStatistics& statistics() const noexcept {
    return statistics_;
  }

 private:
  void resynchronizeAfterInvalidCandidate() noexcept;

  FeedbackFrame retained_{};
  std::size_t retained_size_{0U};
  FeedbackParserStatistics statistics_{};
};

}  // namespace wheeltec_vcu_serial
