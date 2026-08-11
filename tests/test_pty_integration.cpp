// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/feedback_parser.hpp"
#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial/transport.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace vcu = wheeltec_vcu_serial;

namespace {

int failures = 0;

void check(bool condition, const char* message) {
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

struct Pty {
  int master{-1};
  std::string slave_path;

  ~Pty() {
    if (master >= 0) {
      ::close(master);
    }
  }
};

bool openPty(Pty* pty) {
  if (pty == nullptr) {
    return false;
  }
  pty->master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (pty->master < 0 || ::grantpt(pty->master) != 0 ||
      ::unlockpt(pty->master) != 0) {
    return false;
  }
  char* name = ::ptsname(pty->master);
  if (name == nullptr) {
    return false;
  }
  pty->slave_path = name;
  return true;
}

void storeSigned(std::int16_t value, std::uint8_t* output) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  output[0U] = static_cast<std::uint8_t>(bits >> 8U);
  output[1U] = static_cast<std::uint8_t>(bits & 0xFFU);
}

vcu::FeedbackFrame syntheticFeedback() {
  vcu::FeedbackFrame frame{};
  frame[0U] = vcu::kFrameHeader;
  frame[1U] = 0U;
  storeSigned(-321, &frame[2U]);
  storeSigned(17, &frame[4U]);
  storeSigned(125, &frame[6U]);
  storeSigned(333, &frame[8U]);
  storeSigned(-444, &frame[10U]);
  storeSigned(555, &frame[12U]);
  storeSigned(101, &frame[14U]);
  storeSigned(-202, &frame[16U]);
  storeSigned(303, &frame[18U]);
  frame[20U] = 0x30U;
  frame[21U] = 0x39U;
  std::uint8_t checksum = 0U;
  for (std::size_t index = 0U; index < 22U; ++index) {
    checksum = static_cast<std::uint8_t>(checksum ^ frame[index]);
  }
  frame[22U] = checksum;
  frame[23U] = vcu::kFrameTail;
  return frame;
}

void testReadOnlyFeedbackAndWriteGate() {
  Pty pty;
  check(openPty(&pty), "PTY fixture opens");
  if (pty.master < 0) {
    return;
  }

  vcu::PosixSerialTransport transport(pty.slave_path,
                                      vcu::SerialAccess::kReadOnly);
  const vcu::SerialOpenResult opened = transport.open();
  check(opened.ok() && opened.generation == 1U,
        "read-only serial transport opens generation one");
  if (!opened.ok()) {
    return;
  }

  const vcu::CommandFrame zero = vcu::makeZeroCommandFrame();
  const vcu::IoResult blocked = transport.writeAll(
      zero.data(), zero.size(), vcu::monotonicNowNs() + 100000000LL);
  check(blocked.status == vcu::TransportStatus::kWriteDisabled &&
            blocked.bytes_transferred == 0U &&
            !blocked.host_write_complete && !blocked.outcome_uncertain,
        "read-only transport rejects a nonempty write before the fd");
  const vcu::IoResult blocked_empty = transport.writeAll(
      nullptr, 0U, vcu::monotonicNowNs() + 100000000LL);
  check(blocked_empty.status == vcu::TransportStatus::kWriteDisabled,
        "read-only transport also rejects an empty write");
  std::uint8_t outbound_probe[8U]{};
  errno = 0;
  const ssize_t outbound_count =
      ::read(pty.master, outbound_probe, sizeof(outbound_probe));
  check(outbound_count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
        "PTY peer observes no bytes from the read-only transport");

  const vcu::FeedbackFrame frame = syntheticFeedback();
  check(::write(pty.master, frame.data(), 9U) == 9,
        "PTY peer writes the first synthetic fragment");
  std::uint8_t input[64U]{};
  vcu::IoResult received = transport.readSome(
      input, sizeof(input), vcu::monotonicNowNs() + 100000000LL);
  check(received.status == vcu::TransportStatus::kOk &&
            received.bytes_transferred == 9U,
        "transport reads the first fragment");
  vcu::FeedbackParser parser;
  auto parsed = parser.consume(input, received.bytes_transferred);
  check(parsed.empty() && parser.retainedByteCount() == 9U,
        "parser retains the incomplete PTY fragment");

  const std::size_t suffix = frame.size() - 9U;
  check(::write(pty.master, frame.data() + 9U, suffix) ==
            static_cast<ssize_t>(suffix),
        "PTY peer writes the remaining synthetic bytes");
  received = transport.readSome(
      input, sizeof(input), vcu::monotonicNowNs() + 100000000LL);
  parsed = parser.consume(input, received.bytes_transferred);
  check(received.status == vcu::TransportStatus::kOk && parsed.size() == 1U,
        "fragmented PTY feedback becomes one valid frame");
  if (parsed.size() == 1U) {
    check(std::fabs(parsed[0U].feedback.linear_speed_mps + 0.321) < 1e-12 &&
              std::fabs(parsed[0U].feedback.yaw_rate_radps - 0.125) < 1e-12 &&
              !parsed[0U].feedback.vcu_ack_available,
          "decoded PTY values preserve signed scaling without inventing an ACK");
  }
  transport.close();
}

void testReadWriteCommandAndGeneration() {
  Pty pty;
  check(openPty(&pty), "second PTY fixture opens");
  if (pty.master < 0) {
    return;
  }
  vcu::PosixSerialTransport transport(pty.slave_path,
                                      vcu::SerialAccess::kReadWrite);
  check(transport.open().ok(), "read-write PTY transport opens explicitly");
  const vcu::EncodeResult encoded =
      vcu::encodeCommand(vcu::MotionCommand{-0.2, 0.0, 0.1}, 0.75);
  check(encoded.ok(), "synthetic reverse command encodes for PTY validation");
  if (!encoded.ok()) {
    return;
  }
  const vcu::IoResult written = transport.writeAll(
      encoded.frame.data(), encoded.frame.size(),
      vcu::monotonicNowNs() + 100000000LL);
  check(written.status == vcu::TransportStatus::kOk &&
            written.host_write_complete &&
            written.bytes_transferred == encoded.frame.size(),
        "read-write transport completes the synthetic host write");
  std::uint8_t observed[vcu::kCommandFrameSize]{};
  const ssize_t count = ::read(pty.master, observed, sizeof(observed));
  check(count == static_cast<ssize_t>(sizeof(observed)) &&
            std::memcmp(observed, encoded.frame.data(), sizeof(observed)) == 0,
        "PTY peer observes the complete command frame");

  const vcu::SerialOpenResult reopened = transport.reopen();
  check(reopened.ok() && reopened.generation == 2U,
        "a successful reopen advances the connection generation");
  transport.close();
}

}  // namespace

int main() {
  testReadOnlyFeedbackAndWriteGate();
  testReadWriteCommandAndGeneration();
  if (failures != 0) {
    std::fprintf(stderr, "%d PTY integration checks failed\n", failures);
    return EXIT_FAILURE;
  }
  std::puts("PTY integration checks passed");
  return EXIT_SUCCESS;
}
