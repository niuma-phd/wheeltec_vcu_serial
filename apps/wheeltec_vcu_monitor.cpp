// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/feedback_parser.hpp"
#include "wheeltec_vcu_serial/transport.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace vcu = wheeltec_vcu_serial;

namespace {

constexpr const char* kReadOnlyConfirmation =
    "I_UNDERSTAND_OPENING_A_TTY_CAN_AFFECT_THE_DEVICE";
volatile std::sig_atomic_t g_stop_requested = 0;

void handleSignal(int) { g_stop_requested = 1; }

struct Options {
  std::string device;
  std::uint64_t frame_count{0U};
  std::int64_t timeout_ms{0};
  bool read_only_selected{false};
  std::string confirmation;
};

void printUsage(std::ostream& stream) {
  stream
      << "Usage:\n"
      << "  wheeltec_vcu_monitor --read-only --device /dev/DIRECT_TTY "
         "--frames N --timeout-ms MS\\\n\n"
      << "    --operator-confirmation " << kReadOnlyConfirmation << "\n\n"
      << "The monitor opens the selected TTY O_RDONLY and never writes bytes.\n"
      << "Opening/configuring a USB TTY can still affect device line state.\n";
}

bool parsePositiveInt64(const std::string& text, std::int64_t* value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const long long parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno == ERANGE || end == text.c_str() || end == nullptr ||
      *end != '\0' || parsed <= 0) {
    return false;
  }
  *value = static_cast<std::int64_t>(parsed);
  return true;
}

bool directDevicePath(const std::string& path) {
  if (path.size() <= 5U || path.compare(0U, 5U, "/dev/") != 0 ||
      path.back() == '/') {
    return false;
  }
  std::size_t offset = 5U;
  while (offset < path.size()) {
    const std::size_t separator = path.find('/', offset);
    const std::size_t end = separator == std::string::npos ? path.size()
                                                            : separator;
    const std::string component = path.substr(offset, end - offset);
    if (component.empty() || component == "." || component == "..") {
      return false;
    }
    if (separator == std::string::npos) {
      break;
    }
    offset = separator + 1U;
  }
  return true;
}

bool parseOptions(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return false;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      printUsage(std::cout);
      std::exit(EXIT_SUCCESS);
    }
    if (argument == "--read-only") {
      options->read_only_selected = true;
      continue;
    }
    if (index + 1 >= argc) {
      return false;
    }
    const std::string value(argv[++index]);
    if (argument == "--device") {
      options->device = value;
    } else if (argument == "--frames") {
      std::int64_t parsed = 0;
      if (!parsePositiveInt64(value, &parsed)) {
        return false;
      }
      options->frame_count = static_cast<std::uint64_t>(parsed);
    } else if (argument == "--timeout-ms") {
      if (!parsePositiveInt64(value, &options->timeout_ms)) {
        return false;
      }
    } else if (argument == "--operator-confirmation") {
      options->confirmation = value;
    } else {
      return false;
    }
  }
  return options->read_only_selected && directDevicePath(options->device) &&
         options->frame_count > 0U && options->frame_count <= 100000U &&
         options->timeout_ms >= 100 && options->timeout_ms <= 3600000 &&
         options->confirmation == kReadOnlyConfirmation;
}

std::int64_t saturatedAddMilliseconds(std::int64_t origin_ns,
                                      std::int64_t milliseconds) {
  if (origin_ns <= 0 || milliseconds <= 0 ||
      milliseconds >
          (std::numeric_limits<std::int64_t>::max() - origin_ns) / 1000000LL) {
    return 0;
  }
  return origin_ns + milliseconds * 1000000LL;
}

const char* transportStatusName(vcu::TransportStatus status) {
  switch (status) {
    case vcu::TransportStatus::kOk:
      return "ok";
    case vcu::TransportStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case vcu::TransportStatus::kDisconnected:
      return "disconnected";
    case vcu::TransportStatus::kWriteDisabled:
      return "write_disabled";
    case vcu::TransportStatus::kInvalidArgument:
      return "invalid_argument";
    case vcu::TransportStatus::kConfigurationError:
      return "configuration_error";
    case vcu::TransportStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

struct Aggregate {
  std::uint64_t frames{0U};
  std::uint64_t allowed{0U};
  std::uint64_t inhibited{0U};
  double linear_sum{0.0};
  double yaw_sum{0.0};
  double voltage_sum{0.0};
  double linear_min{std::numeric_limits<double>::infinity()};
  double linear_max{-std::numeric_limits<double>::infinity()};

  void add(const vcu::FeedbackData& feedback) {
    ++frames;
    allowed += feedback.control_allowed ? 1U : 0U;
    inhibited += feedback.control_inhibited ? 1U : 0U;
    linear_sum += feedback.linear_speed_mps;
    yaw_sum += feedback.yaw_rate_radps;
    voltage_sum += feedback.supply_voltage_v;
    linear_min = std::min(linear_min, feedback.linear_speed_mps);
    linear_max = std::max(linear_max, feedback.linear_speed_mps);
  }
};

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseOptions(argc, argv, &options)) {
    printUsage(std::cerr);
    return 2;
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);

  vcu::PosixSerialTransport transport(options.device,
                                      vcu::SerialAccess::kReadOnly);
  const vcu::SerialOpenResult opened = transport.open();
  if (!opened.ok()) {
    std::cerr << "read-only open failed: status="
              << transportStatusName(opened.status)
              << " os_error=" << opened.os_error << "\n";
    return 3;
  }

  const std::int64_t started_ns = vcu::monotonicNowNs();
  const std::int64_t session_deadline_ns =
      saturatedAddMilliseconds(started_ns, options.timeout_ms);
  if (session_deadline_ns <= 0) {
    transport.close();
    std::cerr << "monotonic clock or timeout overflow\n";
    return 4;
  }

  vcu::FeedbackParser parser;
  Aggregate aggregate;
  std::uint8_t buffer[512U]{};
  int result_code = 0;
  while (aggregate.frames < options.frame_count && g_stop_requested == 0) {
    const std::int64_t now_ns = vcu::monotonicNowNs();
    if (now_ns <= 0 || now_ns >= session_deadline_ns) {
      result_code = 5;
      break;
    }
    const std::int64_t read_deadline_ns = std::min<std::int64_t>(
        session_deadline_ns, now_ns + 100000000LL);
    const vcu::IoResult read =
        transport.readSome(buffer, sizeof(buffer), read_deadline_ns);
    if (read.status == vcu::TransportStatus::kDeadlineExceeded) {
      continue;
    }
    if (read.status != vcu::TransportStatus::kOk) {
      std::cerr << "read failed: status=" << transportStatusName(read.status)
                << " os_error=" << read.os_error << "\n";
      result_code = 6;
      break;
    }
    const auto frames = parser.consume(buffer, read.bytes_transferred);
    for (const auto& frame : frames) {
      aggregate.add(frame.feedback);
      if (aggregate.frames >= options.frame_count) {
        break;
      }
    }
  }
  transport.close();

  if (aggregate.frames > 0U) {
    const double denominator = static_cast<double>(aggregate.frames);
    std::cout << std::fixed << std::setprecision(6)
              << "receipt_frames=" << aggregate.frames
              << " control_allowed=" << aggregate.allowed
              << " control_inhibited=" << aggregate.inhibited
              << " linear_mean_mps=" << aggregate.linear_sum / denominator
              << " linear_range_mps=[" << aggregate.linear_min << ','
              << aggregate.linear_max << ']'
              << " yaw_mean_radps=" << aggregate.yaw_sum / denominator
              << " voltage_mean_v=" << aggregate.voltage_sum / denominator
              << " checksum_failures="
              << parser.statistics().checksum_failures
              << " framing_failures="
              << parser.statistics().framing_failures
              << " discarded_bytes=" << parser.statistics().discarded_bytes
              << " vcu_ack_available=false source_time_available=false\n";
  }
  if (g_stop_requested != 0 && result_code == 0) {
    result_code = 130;
  }
  if (aggregate.frames < options.frame_count && result_code == 0) {
    result_code = 5;
  }
  return result_code;
}
