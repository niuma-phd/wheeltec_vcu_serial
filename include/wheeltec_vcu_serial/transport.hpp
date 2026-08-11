// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <sys/types.h>

namespace wheeltec_vcu_serial {

enum class TransportStatus : std::uint8_t {
  kOk = 0,
  kDeadlineExceeded,
  kDisconnected,
  kWriteDisabled,
  kInvalidArgument,
  kConfigurationError,
  kIoError,
};

// This result describes host-side I/O only.  In particular,
// host_write_complete never means that the VCU acknowledged, accepted, or
// physically executed a command.  outcome_uncertain is set when the host can
// no longer establish a clean, in-deadline byte-stream outcome for this call.
struct IoResult {
  TransportStatus status{TransportStatus::kIoError};
  std::size_t bytes_transferred{0U};
  int os_error{0};
  bool host_write_complete{false};
  bool outcome_uncertain{false};
  std::int64_t completion_monotonic_ns{0};
};

// Injectable seams for deterministic short-write, interruption, would-block,
// polling, and deadline tests.  poll_one follows poll(2): it returns -1, 0, or
// a positive ready count and stores the descriptor revents when non-null.
struct IoOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<ssize_t(int, const std::uint8_t*, std::size_t)> write_bytes;
  std::function<ssize_t(int, std::uint8_t*, std::size_t)> read_bytes;
  std::function<int(int, short, int, short*)> poll_one;
};

IoOperations systemIoOperations();
std::int64_t monotonicNowNs() noexcept;

IoResult writeAllWithOperations(int fd, const std::uint8_t* data,
                                std::size_t size,
                                std::int64_t absolute_deadline_ns,
                                const IoOperations& operations);

IoResult readSomeWithOperations(int fd, std::uint8_t* data,
                                std::size_t capacity,
                                std::int64_t absolute_deadline_ns,
                                const IoOperations& operations);

class ITransport {
 public:
  virtual ~ITransport() = default;

  virtual bool connected() const noexcept = 0;
  virtual std::uint64_t generation() const noexcept = 0;
  virtual IoResult writeAll(const std::uint8_t* data, std::size_t size,
                            std::int64_t absolute_deadline_ns) = 0;
  virtual IoResult readSome(std::uint8_t* data, std::size_t capacity,
                            std::int64_t absolute_deadline_ns) = 0;
};

struct SerialOpenResult {
  TransportStatus status{TransportStatus::kIoError};
  int os_error{0};
  std::uint64_t generation{0U};

  bool ok() const noexcept { return status == TransportStatus::kOk; }
};

enum class SerialAccess : std::uint8_t {
  kReadOnly = 0,
  kReadWrite,
};

// POSIX serial transport for an explicitly supplied path.  Construction does
// not open the path.  Every successful open/reopen creates a new monotonically
// increasing connection generation; failures leave the transport closed.
class PosixSerialTransport final : public ITransport {
 public:
  explicit PosixSerialTransport(
      std::string device_path,
      SerialAccess access = SerialAccess::kReadOnly);
  PosixSerialTransport(std::string device_path, SerialAccess access,
                       IoOperations operations);
  ~PosixSerialTransport() override;

  PosixSerialTransport(const PosixSerialTransport&) = delete;
  PosixSerialTransport& operator=(const PosixSerialTransport&) = delete;
  PosixSerialTransport(PosixSerialTransport&&) = delete;
  PosixSerialTransport& operator=(PosixSerialTransport&&) = delete;

  SerialOpenResult open();
  SerialOpenResult reopen();
  void close() noexcept;

  bool connected() const noexcept override;
  std::uint64_t generation() const noexcept override;
  IoResult writeAll(const std::uint8_t* data, std::size_t size,
                    std::int64_t absolute_deadline_ns) override;
  IoResult readSome(std::uint8_t* data, std::size_t capacity,
                    std::int64_t absolute_deadline_ns) override;

  const std::string& devicePath() const noexcept { return device_path_; }
  SerialAccess access() const noexcept { return access_; }

 private:
  SerialOpenResult openNewGeneration();
  void markDisconnected() noexcept;

  std::string device_path_;
  SerialAccess access_{SerialAccess::kReadOnly};
  IoOperations operations_;
  int fd_{-1};
  bool connected_{false};
  std::uint64_t generation_{0U};
};

}  // namespace wheeltec_vcu_serial
