// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/transport.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace wvs = wheeltec_vcu_serial;

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

struct ScriptedIo {
  std::int64_t now_ns{100};
  std::deque<std::int64_t> clock_values;
  std::deque<ssize_t> write_results;
  std::deque<int> write_errors;
  std::deque<ssize_t> read_results;
  std::deque<int> read_errors;
  std::deque<int> poll_results;
  std::deque<int> poll_errors;
  std::deque<short> poll_revents;
  std::vector<std::uint8_t> read_payload;
  std::size_t write_calls{0U};
  std::size_t read_calls{0U};
  std::size_t poll_calls{0U};

  wvs::IoOperations operations() {
    wvs::IoOperations result;
    result.monotonic_now_ns = [this]() {
      if (clock_values.empty()) {
        return now_ns;
      }
      const std::int64_t value = clock_values.front();
      clock_values.pop_front();
      now_ns = value;
      return value;
    };
    result.write_bytes =
        [this](int, const std::uint8_t*, std::size_t size) -> ssize_t {
      ++write_calls;
      if (write_results.empty()) {
        return static_cast<ssize_t>(size);
      }
      const ssize_t value = write_results.front();
      write_results.pop_front();
      if (value < 0) {
        errno = write_errors.front();
        write_errors.pop_front();
      }
      return value;
    };
    result.read_bytes =
        [this](int, std::uint8_t* data, std::size_t capacity) -> ssize_t {
      ++read_calls;
      if (read_results.empty()) {
        errno = EAGAIN;
        return -1;
      }
      const ssize_t value = read_results.front();
      read_results.pop_front();
      if (value < 0) {
        errno = read_errors.front();
        read_errors.pop_front();
        return value;
      }
      const std::size_t count = static_cast<std::size_t>(value);
      if (count <= capacity && count <= read_payload.size()) {
        std::memcpy(data, read_payload.data(), count);
      }
      return value;
    };
    result.poll_one = [this](int, short requested, int, short* revents) {
      ++poll_calls;
      if (poll_results.empty()) {
        if (revents != nullptr) {
          *revents = requested;
        }
        return 1;
      }
      const int value = poll_results.front();
      poll_results.pop_front();
      if (value < 0) {
        errno = poll_errors.front();
        poll_errors.pop_front();
      }
      if (revents != nullptr) {
        *revents = poll_revents.empty() ? 0 : poll_revents.front();
      }
      if (!poll_revents.empty()) {
        poll_revents.pop_front();
      }
      return value;
    };
    return result;
  }
};

void testWriteCompletesAcrossAllRetryClasses() {
  ScriptedIo script;
  script.write_results = {-1, 2, -1, 3};
  script.write_errors = {EINTR, EAGAIN};
  script.poll_results = {-1, 1};
  script.poll_errors = {EINTR};
  script.poll_revents = {0, POLLOUT};
  const std::uint8_t bytes[] = {1U, 2U, 3U, 4U, 5U};
  const wvs::IoResult result = wvs::writeAllWithOperations(
      7, bytes, sizeof(bytes), 200, script.operations());
  expect(result.status == wvs::TransportStatus::kOk &&
             result.host_write_complete &&
             result.bytes_transferred == sizeof(bytes) &&
             !result.outcome_uncertain && script.write_calls == 4U &&
             script.poll_calls == 2U,
         "short write, write EINTR/EAGAIN, and poll EINTR all recover");
}

void testWriteFailuresAndDeadlineSemantics() {
  const std::uint8_t bytes[] = {1U, 2U, 3U};
  {
    ScriptedIo script;
    script.write_results = {0};
    const wvs::IoResult result = wvs::writeAllWithOperations(
        7, bytes, sizeof(bytes), 200, script.operations());
    expect(result.status == wvs::TransportStatus::kDisconnected &&
               !result.host_write_complete && result.outcome_uncertain,
           "write returning zero is a disconnected uncertain outcome");
  }
  {
    ScriptedIo script;
    script.write_results = {1, -1};
    script.write_errors = {EPIPE};
    const wvs::IoResult result = wvs::writeAllWithOperations(
        7, bytes, sizeof(bytes), 200, script.operations());
    expect(result.status == wvs::TransportStatus::kDisconnected &&
               result.bytes_transferred == 1U &&
               !result.host_write_complete && result.outcome_uncertain,
           "disconnect after partial progress preserves uncertainty");
  }
  {
    ScriptedIo script;
    script.now_ns = 200;
    const wvs::IoResult result = wvs::writeAllWithOperations(
        7, bytes, sizeof(bytes), 200, script.operations());
    expect(result.status == wvs::TransportStatus::kDeadlineExceeded &&
               script.write_calls == 0U && !result.host_write_complete,
           "an expired absolute deadline prevents the first write syscall");
  }
  {
    ScriptedIo script;
    script.clock_values = {100, 201};
    script.write_results = {3};
    const wvs::IoResult result = wvs::writeAllWithOperations(
        7, bytes, sizeof(bytes), 200, script.operations());
    expect(result.status == wvs::TransportStatus::kDeadlineExceeded &&
               result.bytes_transferred == sizeof(bytes) &&
               result.host_write_complete && result.outcome_uncertain &&
               result.completion_monotonic_ns == 201,
           "a full host write discovered late remains a deadline failure");
  }
  {
    ScriptedIo script;
    script.write_results = {-1};
    script.write_errors = {EAGAIN};
    script.poll_results = {0};
    script.poll_revents = {0};
    const wvs::IoResult result = wvs::writeAllWithOperations(
        7, bytes, sizeof(bytes), 200, script.operations());
    expect(result.status == wvs::TransportStatus::kDeadlineExceeded &&
               !result.host_write_complete && script.poll_calls == 1U,
           "would-block exits when poll reaches the absolute deadline");
  }
}

void testReadRetriesAndDeadline() {
  ScriptedIo script;
  script.read_results = {-1, -1, 3};
  script.read_errors = {EINTR, EAGAIN};
  script.poll_results = {-1, 1};
  script.poll_errors = {EINTR};
  script.poll_revents = {0, POLLIN};
  script.read_payload = {9U, 8U, 7U};
  std::uint8_t received[8U]{};
  const wvs::IoResult result = wvs::readSomeWithOperations(
      8, received, sizeof(received), 200, script.operations());
  expect(result.status == wvs::TransportStatus::kOk &&
             result.bytes_transferred == 3U && !result.host_write_complete &&
             std::memcmp(received, script.read_payload.data(), 3U) == 0 &&
             script.read_calls == 3U && script.poll_calls == 2U,
         "read EINTR/EAGAIN and poll EINTR recover without losing bytes");

  ScriptedIo late;
  late.clock_values = {100, 201};
  late.read_results = {1};
  late.read_payload = {0x5AU};
  const wvs::IoResult late_result = wvs::readSomeWithOperations(
      8, received, sizeof(received), 200, late.operations());
  expect(late_result.status == wvs::TransportStatus::kDeadlineExceeded &&
             late_result.bytes_transferred == 1U && received[0U] == 0x5AU,
         "read completion after the absolute deadline is reported as late");
}

struct PtyMaster {
  int fd{-1};
  std::string slave_path;

  ~PtyMaster() {
    if (fd >= 0) {
      (void)::close(fd);
    }
  }
};

bool openPtyMaster(PtyMaster* pty) {
  if (pty == nullptr) {
    return false;
  }
  pty->fd = ::posix_openpt(O_RDWR | O_NONBLOCK | O_NOCTTY | O_CLOEXEC);
  if (pty->fd < 0 || ::grantpt(pty->fd) != 0 ||
      ::unlockpt(pty->fd) != 0) {
    return false;
  }
  char* const name = ::ptsname(pty->fd);
  if (name == nullptr) {
    return false;
  }
  pty->slave_path = name;
  return true;
}

void testPtyReadOnlyAndReadWriteAccess() {
  PtyMaster pty;
  expect(openPtyMaster(&pty), "PTY master opens without a physical device");
  if (pty.fd < 0 || pty.slave_path.empty()) {
    return;
  }

  wvs::PosixSerialTransport receive_only(pty.slave_path);
  const std::uint8_t prohibited = 0xA5U;
  const wvs::IoResult blocked_while_closed = receive_only.writeAll(
      &prohibited, 1U, wvs::monotonicNowNs() + 10000000LL);
  const wvs::IoResult blocked_empty_while_closed = receive_only.writeAll(
      nullptr, 0U, wvs::monotonicNowNs() + 10000000LL);
  expect(blocked_while_closed.status ==
                 wvs::TransportStatus::kWriteDisabled &&
             blocked_empty_while_closed.status ==
                 wvs::TransportStatus::kWriteDisabled,
         "receive-only access rejects writes before descriptor handling");

  const wvs::SerialOpenResult read_open = receive_only.open();
  expect(read_open.ok() && read_open.generation == 1U &&
             receive_only.access() == wvs::SerialAccess::kReadOnly,
         "serial transport defaults to an explicit receive-only generation");
  if (!read_open.ok()) {
    return;
  }
  wvs::PosixSerialTransport competing(
      pty.slave_path, wvs::SerialAccess::kReadWrite);
  const wvs::SerialOpenResult competing_open = competing.open();
  expect(competing_open.status ==
                 wvs::TransportStatus::kConfigurationError &&
             competing_open.os_error == EBUSY && !competing.connected(),
         "an owned TTY rejects a second project transport");
  const wvs::IoResult blocked = receive_only.writeAll(
      &prohibited, 1U, wvs::monotonicNowNs() + 10000000LL);
  const wvs::IoResult blocked_empty = receive_only.writeAll(
      nullptr, 0U, wvs::monotonicNowNs() + 10000000LL);
  expect(blocked.status == wvs::TransportStatus::kWriteDisabled &&
             blocked.bytes_transferred == 0U &&
             blocked_empty.status == wvs::TransportStatus::kWriteDisabled,
         "receive-only transport rejects nonempty and empty writes");
  std::uint8_t unexpected = 0U;
  errno = 0;
  const ssize_t unexpected_count = ::read(pty.fd, &unexpected, 1U);
  expect(unexpected_count < 0 &&
             (errno == EAGAIN || errno == EWOULDBLOCK),
         "receive-only write rejection touches no PTY byte stream");

  const std::uint8_t inbound[] = {4U, 5U, 6U};
  expect(::write(pty.fd, inbound, sizeof(inbound)) ==
             static_cast<ssize_t>(sizeof(inbound)),
         "PTY peer supplies receive-only bytes");
  std::uint8_t read_buffer[8U]{};
  const wvs::IoResult read_result = receive_only.readSome(
      read_buffer, sizeof(read_buffer),
      wvs::monotonicNowNs() + 100000000LL);
  expect(read_result.status == wvs::TransportStatus::kOk &&
             read_result.bytes_transferred == sizeof(inbound) &&
             std::memcmp(read_buffer, inbound, sizeof(inbound)) == 0,
         "receive-only serial transport reads PTY input");
  receive_only.close();

  wvs::PosixSerialTransport read_write(
      pty.slave_path, wvs::SerialAccess::kReadWrite);
  const wvs::SerialOpenResult write_open = read_write.open();
  expect(write_open.ok() && write_open.generation == 1U,
         "read-write access must be selected explicitly");
  if (!write_open.ok()) {
    return;
  }
  const std::uint8_t outbound[] = {7U, 8U, 9U};
  const wvs::IoResult write_result = read_write.writeAll(
      outbound, sizeof(outbound),
      wvs::monotonicNowNs() + 100000000LL);
  std::uint8_t observed[3U]{};
  const ssize_t observed_count = ::read(pty.fd, observed, sizeof(observed));
  expect(write_result.status == wvs::TransportStatus::kOk &&
             write_result.host_write_complete &&
             observed_count == static_cast<ssize_t>(sizeof(observed)) &&
             std::memcmp(observed, outbound, sizeof(outbound)) == 0,
         "explicit read-write PTY transport completes host bytes");

  const wvs::SerialOpenResult reopened = read_write.reopen();
  expect(reopened.ok() && reopened.generation == 2U &&
             read_write.generation() == 2U,
         "each successful reopen increments the connection generation");
}

void testInvalidPathNeverOpens() {
  wvs::PosixSerialTransport transport(
      std::string(), wvs::SerialAccess::kReadWrite);
  const wvs::SerialOpenResult result = transport.open();
  expect(result.status == wvs::TransportStatus::kInvalidArgument &&
             !transport.connected() && transport.generation() == 0U,
         "missing explicit serial path fails before any device open");
}

void testStableUdevSymlinkIsAccepted() {
  PtyMaster pty;
  expect(openPtyMaster(&pty), "PTY for stable udev symlink test opens");
  if (pty.fd < 0 || pty.slave_path.empty()) {
    return;
  }

  char link_path[] = "/tmp/wheeltec_vcu_udev_link_XXXXXX";
  const int placeholder_fd = ::mkstemp(link_path);
  expect(placeholder_fd >= 0, "temporary udev-link name is reserved");
  if (placeholder_fd < 0) {
    return;
  }
  (void)::close(placeholder_fd);
  (void)::unlink(link_path);
  expect(::symlink(pty.slave_path.c_str(), link_path) == 0,
         "stable udev-style symlink is created");

  wvs::PosixSerialTransport transport(
      link_path, wvs::SerialAccess::kReadWrite);
  const wvs::SerialOpenResult result = transport.open();
  expect(result.ok() && transport.connected() && result.generation == 1U,
         "serial transport accepts a stable udev symlink to a TTY");
  transport.close();
  (void)::unlink(link_path);
}

void testRegularFileIsNotAcceptedAsSerial() {
  char path[] = "/tmp/wheeltec_vcu_transport_test_XXXXXX";
  const int file_fd = ::mkstemp(path);
  expect(file_fd >= 0, "temporary regular-file fixture opens");
  if (file_fd < 0) {
    return;
  }
  (void)::close(file_fd);

  // Seed errno to prove that a successful fstat followed by a type mismatch
  // reports ENOTTY deterministically rather than propagating stale state.
  errno = EBUSY;
  wvs::PosixSerialTransport transport(path, wvs::SerialAccess::kReadOnly);
  const wvs::SerialOpenResult result = transport.open();
  expect(result.status == wvs::TransportStatus::kConfigurationError &&
             result.os_error == ENOTTY && !transport.connected() &&
             transport.generation() == 0U,
         "a regular file fails the character-TTY gate with ENOTTY");
  (void)::unlink(path);
}

}  // namespace

int main() {
  testWriteCompletesAcrossAllRetryClasses();
  testWriteFailuresAndDeadlineSemantics();
  testReadRetriesAndDeadline();
  testPtyReadOnlyAndReadWriteAccess();
  testInvalidPathNeverOpens();
  testStableUdevSymlinkIsAccepted();
  testRegularFileIsNotAcceptedAsSerial();
  if (failures != 0) {
    std::fprintf(stderr, "%d transport test(s) failed\n", failures);
    return 1;
  }
  std::puts("transport tests passed");
  return 0;
}
