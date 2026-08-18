// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/transport.hpp"

#include <cerrno>
#include <climits>
#include <cstdint>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <utility>

namespace wheeltec_vcu_serial {
namespace {

bool isDisconnectError(int error_number) noexcept {
  return error_number == EBADF || error_number == EPIPE ||
         error_number == ENODEV || error_number == ENXIO ||
         error_number == EIO || error_number == ENOTCONN;
}

IoResult makeResult(TransportStatus status, std::size_t bytes_transferred,
                    int os_error, bool host_write_complete,
                    bool outcome_uncertain,
                    std::int64_t completion_monotonic_ns) {
  IoResult result;
  result.status = status;
  result.bytes_transferred = bytes_transferred;
  result.os_error = os_error;
  result.host_write_complete = host_write_complete;
  result.outcome_uncertain = outcome_uncertain;
  result.completion_monotonic_ns = completion_monotonic_ns;
  return result;
}

bool writeOperationsAreComplete(const IoOperations& operations) {
  return static_cast<bool>(operations.monotonic_now_ns) &&
         static_cast<bool>(operations.write_bytes) &&
         static_cast<bool>(operations.poll_one);
}

bool readOperationsAreComplete(const IoOperations& operations) {
  return static_cast<bool>(operations.monotonic_now_ns) &&
         static_cast<bool>(operations.read_bytes) &&
         static_cast<bool>(operations.poll_one);
}

int timeoutMilliseconds(std::int64_t remaining_ns) noexcept {
  if (remaining_ns <= 0) {
    return 0;
  }
  std::int64_t milliseconds = remaining_ns / 1000000LL;
  if (remaining_ns % 1000000LL != 0) {
    ++milliseconds;
  }
  if (milliseconds > INT_MAX) {
    return INT_MAX;
  }
  return static_cast<int>(milliseconds);
}

struct WaitResult {
  TransportStatus status{TransportStatus::kIoError};
  int os_error{0};
  std::int64_t completion_monotonic_ns{0};

  bool ready() const noexcept { return status == TransportStatus::kOk; }
};

WaitResult waitForEvent(int fd, short requested_events,
                        std::int64_t absolute_deadline_ns,
                        const IoOperations& operations) {
  for (;;) {
    const std::int64_t before_poll_ns = operations.monotonic_now_ns();
    if (before_poll_ns >= absolute_deadline_ns) {
      return {TransportStatus::kDeadlineExceeded, ETIMEDOUT,
              before_poll_ns};
    }

    short returned_events = 0;
    const int poll_result = operations.poll_one(
        fd, requested_events,
        timeoutMilliseconds(absolute_deadline_ns - before_poll_ns),
        &returned_events);
    const int poll_error = poll_result < 0 ? errno : 0;
    const std::int64_t after_poll_ns = operations.monotonic_now_ns();

    if (poll_result > 0) {
      if ((returned_events & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        const int error_number =
            (returned_events & POLLNVAL) != 0 ? EBADF : EPIPE;
        return {TransportStatus::kDisconnected, error_number,
                after_poll_ns};
      }
      if ((returned_events & requested_events) != 0) {
        if (after_poll_ns > absolute_deadline_ns) {
          return {TransportStatus::kDeadlineExceeded, ETIMEDOUT,
                  after_poll_ns};
        }
        return {TransportStatus::kOk, 0, after_poll_ns};
      }
      continue;
    }
    if (poll_result == 0) {
      return {TransportStatus::kDeadlineExceeded, ETIMEDOUT,
              after_poll_ns};
    }
    if (poll_error == EINTR) {
      continue;
    }
    return {isDisconnectError(poll_error) ? TransportStatus::kDisconnected
                                          : TransportStatus::kIoError,
            poll_error, after_poll_ns};
  }
}

int getFileStatusWithoutInterruption(int fd, struct stat* metadata) {
  int result = -1;
  do {
    result = ::fstat(fd, metadata);
  } while (result < 0 && errno == EINTR);
  return result;
}

int getTermiosWithoutInterruption(int fd, termios* settings) {
  int result = -1;
  do {
    result = ::tcgetattr(fd, settings);
  } while (result < 0 && errno == EINTR);
  return result;
}

int setTermiosWithoutInterruption(int fd, const termios* settings) {
  int result = -1;
  do {
    result = ::tcsetattr(fd, TCSANOW, settings);
  } while (result < 0 && errno == EINTR);
  return result;
}

bool serialSettingsMatch(const termios& settings) noexcept {
  bool hardware_flow_disabled = true;
#ifdef CRTSCTS
  hardware_flow_disabled = (settings.c_cflag & CRTSCTS) == 0;
#endif
  return ::cfgetispeed(&settings) == B115200 &&
         ::cfgetospeed(&settings) == B115200 &&
         (settings.c_cflag & CSIZE) == CS8 &&
         (settings.c_cflag & (PARENB | CSTOPB)) == 0 &&
         (settings.c_cflag & (CLOCAL | CREAD)) == (CLOCAL | CREAD) &&
         hardware_flow_disabled &&
         (settings.c_iflag & (IXON | IXOFF | IXANY)) == 0 &&
         settings.c_cc[VMIN] == 0 && settings.c_cc[VTIME] == 0;
}

bool configureSerialPort(int fd) {
  termios settings{};
  if (getTermiosWithoutInterruption(fd, &settings) != 0) {
    return false;
  }
  ::cfmakeraw(&settings);
  if (::cfsetispeed(&settings, B115200) != 0 ||
      ::cfsetospeed(&settings, B115200) != 0) {
    return false;
  }
  settings.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
  settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
#ifdef CRTSCTS
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  settings.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (setTermiosWithoutInterruption(fd, &settings) != 0) {
    return false;
  }

  termios observed{};
  if (getTermiosWithoutInterruption(fd, &observed) != 0) {
    return false;
  }
  if (!serialSettingsMatch(observed)) {
    // A successful tcgetattr() does not define errno.  Give the caller a
    // deterministic configuration failure instead of leaking a stale value
    // from an earlier, recovered syscall.
    errno = EIO;
    return false;
  }
  return true;
}

bool explicitDevicePathIsValid(const std::string& path) noexcept {
  return !path.empty() && path.front() == '/' && path.back() != '/' &&
         path.find('\0') == std::string::npos;
}

void releaseExclusiveAndClose(int fd) noexcept {
  if (fd < 0) {
    return;
  }
#ifdef TIOCNXCL
  int status = -1;
  do {
    status = ::ioctl(fd, TIOCNXCL);
  } while (status < 0 && errno == EINTR);
  (void)status;
#endif
  (void)::close(fd);
}

}  // namespace

std::int64_t monotonicNowNs() noexcept {
  timespec timestamp{};
  if (::clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
    return 0;
  }
  return static_cast<std::int64_t>(timestamp.tv_sec) * 1000000000LL +
         static_cast<std::int64_t>(timestamp.tv_nsec);
}

IoOperations systemIoOperations() {
  IoOperations operations;
  operations.monotonic_now_ns = []() { return monotonicNowNs(); };
  operations.write_bytes =
      [](int fd, const std::uint8_t* data, std::size_t size) {
        return ::write(fd, data, size);
      };
  operations.read_bytes = [](int fd, std::uint8_t* data, std::size_t size) {
    return ::read(fd, data, size);
  };
  operations.poll_one = [](int fd, short events, int timeout_ms,
                           short* returned_events) {
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = events;
    const int result = ::poll(&descriptor, 1U, timeout_ms);
    if (returned_events != nullptr) {
      *returned_events = descriptor.revents;
    }
    return result;
  };
  return operations;
}

IoResult writeAllWithOperations(int fd, const std::uint8_t* data,
                                std::size_t size,
                                std::int64_t absolute_deadline_ns,
                                const IoOperations& operations) {
  if (fd < 0 || (data == nullptr && size != 0U) ||
      !writeOperationsAreComplete(operations)) {
    return makeResult(TransportStatus::kInvalidArgument, 0U, EINVAL, false,
                      false, 0);
  }
  if (size == 0U) {
    const std::int64_t completion_ns = operations.monotonic_now_ns();
    const bool late = completion_ns > absolute_deadline_ns;
    return makeResult(late ? TransportStatus::kDeadlineExceeded
                           : TransportStatus::kOk,
                      0U, late ? ETIMEDOUT : 0, true, late,
                      completion_ns);
  }

  std::size_t transferred = 0U;
  while (transferred < size) {
    const std::int64_t before_write_ns = operations.monotonic_now_ns();
    if (before_write_ns >= absolute_deadline_ns) {
      return makeResult(TransportStatus::kDeadlineExceeded, transferred,
                        ETIMEDOUT, false, transferred != 0U,
                        before_write_ns);
    }

    const ssize_t write_result = operations.write_bytes(
        fd, data + transferred, size - transferred);
    const int write_error = write_result < 0 ? errno : 0;
    if (write_result > 0) {
      const std::size_t progress = static_cast<std::size_t>(write_result);
      if (progress > size - transferred) {
        return makeResult(TransportStatus::kIoError, transferred, EIO, false,
                          transferred != 0U, before_write_ns);
      }
      transferred += progress;
      const std::int64_t completion_ns = operations.monotonic_now_ns();
      if (transferred == size) {
        const bool late = completion_ns > absolute_deadline_ns;
        return makeResult(late ? TransportStatus::kDeadlineExceeded
                               : TransportStatus::kOk,
                          transferred, late ? ETIMEDOUT : 0, true, late,
                          completion_ns);
      }
      if (completion_ns >= absolute_deadline_ns) {
        return makeResult(TransportStatus::kDeadlineExceeded, transferred,
                          ETIMEDOUT, false, true, completion_ns);
      }
      continue;
    }
    if (write_result == 0) {
      return makeResult(TransportStatus::kDisconnected, transferred, EPIPE,
                        false, true, operations.monotonic_now_ns());
    }
    if (write_error == EINTR) {
      continue;
    }
    if (write_error == EAGAIN || write_error == EWOULDBLOCK) {
      const WaitResult wait = waitForEvent(fd, POLLOUT,
                                           absolute_deadline_ns, operations);
      if (wait.ready()) {
        continue;
      }
      return makeResult(wait.status, transferred, wait.os_error, false,
                        transferred != 0U ||
                            wait.status == TransportStatus::kDisconnected,
                        wait.completion_monotonic_ns);
    }
    const TransportStatus status = isDisconnectError(write_error)
                                       ? TransportStatus::kDisconnected
                                       : TransportStatus::kIoError;
    return makeResult(status, transferred, write_error, false,
                      transferred != 0U ||
                          status == TransportStatus::kDisconnected,
                      operations.monotonic_now_ns());
  }

  return makeResult(TransportStatus::kIoError, transferred, EIO, false, true,
                    operations.monotonic_now_ns());
}

IoResult readSomeWithOperations(int fd, std::uint8_t* data,
                                std::size_t capacity,
                                std::int64_t absolute_deadline_ns,
                                const IoOperations& operations) {
  if (fd < 0 || data == nullptr || capacity == 0U ||
      !readOperationsAreComplete(operations)) {
    return makeResult(TransportStatus::kInvalidArgument, 0U, EINVAL, false,
                      false, 0);
  }

  for (;;) {
    const std::int64_t before_read_ns = operations.monotonic_now_ns();
    if (before_read_ns >= absolute_deadline_ns) {
      return makeResult(TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
                        false, false, before_read_ns);
    }

    const ssize_t read_result = operations.read_bytes(fd, data, capacity);
    const int read_error = read_result < 0 ? errno : 0;
    if (read_result > 0) {
      const std::size_t transferred = static_cast<std::size_t>(read_result);
      const std::int64_t completion_ns = operations.monotonic_now_ns();
      if (transferred > capacity) {
        return makeResult(TransportStatus::kIoError, 0U, EIO, false, false,
                          completion_ns);
      }
      const bool late = completion_ns > absolute_deadline_ns;
      return makeResult(late ? TransportStatus::kDeadlineExceeded
                             : TransportStatus::kOk,
                        transferred, late ? ETIMEDOUT : 0, false, false,
                        completion_ns);
    }
    if (read_result < 0 && read_error == EINTR) {
      continue;
    }
    if (read_result == 0 || read_error == EAGAIN ||
        read_error == EWOULDBLOCK) {
      const WaitResult wait = waitForEvent(fd, POLLIN,
                                           absolute_deadline_ns, operations);
      if (wait.ready()) {
        continue;
      }
      return makeResult(wait.status, 0U, wait.os_error, false,
                        false, wait.completion_monotonic_ns);
    }
    const TransportStatus status = isDisconnectError(read_error)
                                       ? TransportStatus::kDisconnected
                                       : TransportStatus::kIoError;
    return makeResult(status, 0U, read_error, false, false,
                      operations.monotonic_now_ns());
  }
}

PosixSerialTransport::PosixSerialTransport(std::string device_path,
                                           SerialAccess access)
    : PosixSerialTransport(std::move(device_path), access,
                           systemIoOperations()) {}

PosixSerialTransport::PosixSerialTransport(std::string device_path,
                                           SerialAccess access,
                                           IoOperations operations)
    : device_path_(std::move(device_path)),
      access_(access),
      operations_(std::move(operations)) {}

PosixSerialTransport::~PosixSerialTransport() { close(); }

SerialOpenResult PosixSerialTransport::open() {
  if (connected_) {
    return {TransportStatus::kOk, 0, generation_};
  }
  return openNewGeneration();
}

SerialOpenResult PosixSerialTransport::reopen() {
  close();
  return openNewGeneration();
}

void PosixSerialTransport::close() noexcept {
  connected_ = false;
  if (fd_ >= 0) {
    // Retrying close after EINTR can close an unrelated, reused descriptor on
    // Linux.  The fd is forgotten after this single best-effort close.
    releaseExclusiveAndClose(fd_);
    fd_ = -1;
  }
}

bool PosixSerialTransport::connected() const noexcept { return connected_; }

std::uint64_t PosixSerialTransport::generation() const noexcept {
  return generation_;
}

IoResult PosixSerialTransport::writeAll(
    const std::uint8_t* data, std::size_t size,
    std::int64_t absolute_deadline_ns) {
  // This check deliberately precedes descriptor/connection handling.  A
  // receive-only instance has no code path that invokes write(2), including
  // for an empty buffer.
  if (access_ != SerialAccess::kReadWrite) {
    return makeResult(TransportStatus::kWriteDisabled, 0U, EACCES, false,
                      false, 0);
  }
  if (!connected_ || fd_ < 0) {
    return makeResult(TransportStatus::kDisconnected, 0U, ENOTCONN, false,
                      true, operations_.monotonic_now_ns
                                ? operations_.monotonic_now_ns()
                                : 0);
  }
  IoResult result = writeAllWithOperations(
      fd_, data, size, absolute_deadline_ns, operations_);
  if (result.status == TransportStatus::kDisconnected) {
    markDisconnected();
  }
  return result;
}

IoResult PosixSerialTransport::readSome(
    std::uint8_t* data, std::size_t capacity,
    std::int64_t absolute_deadline_ns) {
  if (!connected_ || fd_ < 0) {
    return makeResult(TransportStatus::kDisconnected, 0U, ENOTCONN, false,
                      false, operations_.monotonic_now_ns
                                 ? operations_.monotonic_now_ns()
                                 : 0);
  }
  IoResult result = readSomeWithOperations(
      fd_, data, capacity, absolute_deadline_ns, operations_);
  if (result.status == TransportStatus::kDisconnected) {
    markDisconnected();
  }
  return result;
}

SerialOpenResult PosixSerialTransport::openNewGeneration() {
  if (!explicitDevicePathIsValid(device_path_)) {
    return {TransportStatus::kInvalidArgument, EINVAL, generation_};
  }
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    return {TransportStatus::kConfigurationError, EOVERFLOW, generation_};
  }

  if (access_ != SerialAccess::kReadOnly &&
      access_ != SerialAccess::kReadWrite) {
    return {TransportStatus::kInvalidArgument, EINVAL, generation_};
  }

  const int requested_access =
      access_ == SerialAccess::kReadOnly ? O_RDONLY : O_RDWR;
  int opened_fd = -1;
  do {
    opened_fd = ::open(device_path_.c_str(),
                       requested_access | O_NONBLOCK | O_NOCTTY |
                           O_CLOEXEC);
  } while (opened_fd < 0 && errno == EINTR);
  if (opened_fd < 0) {
    const int open_error = errno == 0 ? EIO : errno;
    return {isDisconnectError(open_error) || open_error == ENOENT
                ? TransportStatus::kDisconnected
                : (open_error == EBUSY
                       ? TransportStatus::kConfigurationError
                       : TransportStatus::kIoError),
            open_error, generation_};
  }

  struct stat metadata {};
  if (getFileStatusWithoutInterruption(opened_fd, &metadata) != 0) {
    const int stat_error = errno == 0 ? EIO : errno;
    (void)::close(opened_fd);
    return {TransportStatus::kConfigurationError, stat_error, generation_};
  }
  if (!S_ISCHR(metadata.st_mode)) {
    (void)::close(opened_fd);
    return {TransportStatus::kConfigurationError, ENOTTY, generation_};
  }
  errno = 0;
  if (::isatty(opened_fd) != 1) {
    const int tty_error = errno == 0 ? ENOTTY : errno;
    (void)::close(opened_fd);
    return {TransportStatus::kConfigurationError, tty_error, generation_};
  }

  // Prevent later opens through this TTY while the transport owns it.  Linux
  // TIOCEXCL cannot evict or detect a non-cooperating descriptor that was
  // already open, so applications must still enforce external single-owner
  // policy as documented.
  int exclusive_status = -1;
  do {
    exclusive_status = ::ioctl(opened_fd, TIOCEXCL);
  } while (exclusive_status < 0 && errno == EINTR);
  if (exclusive_status != 0) {
    const int exclusive_error = errno == 0 ? EBUSY : errno;
    (void)::close(opened_fd);
    return {TransportStatus::kConfigurationError, exclusive_error,
            generation_};
  }

  int file_flags = -1;
  do {
    file_flags = ::fcntl(opened_fd, F_GETFL);
  } while (file_flags < 0 && errno == EINTR);
  int descriptor_flags = -1;
  do {
    descriptor_flags = ::fcntl(opened_fd, F_GETFD);
  } while (descriptor_flags < 0 && errno == EINTR);
  if (file_flags < 0 || descriptor_flags < 0) {
    const int configuration_error = errno == 0 ? EIO : errno;
    releaseExclusiveAndClose(opened_fd);
    return {TransportStatus::kConfigurationError, configuration_error,
            generation_};
  }
  if ((file_flags & O_NONBLOCK) == 0 ||
      (descriptor_flags & FD_CLOEXEC) == 0) {
    releaseExclusiveAndClose(opened_fd);
    return {TransportStatus::kConfigurationError, EIO, generation_};
  }
  if (!configureSerialPort(opened_fd)) {
    const int configuration_error = errno == 0 ? EIO : errno;
    releaseExclusiveAndClose(opened_fd);
    return {TransportStatus::kConfigurationError, configuration_error,
            generation_};
  }

  fd_ = opened_fd;
  connected_ = true;
  ++generation_;
  return {TransportStatus::kOk, 0, generation_};
}

void PosixSerialTransport::markDisconnected() noexcept { close(); }

}  // namespace wheeltec_vcu_serial
