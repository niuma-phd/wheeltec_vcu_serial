// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/config.hpp"
#include "wheeltec_vcu_serial/feedback_parser.hpp"
#include "wheeltec_vcu_serial/protocol.hpp"
#include "wheeltec_vcu_serial/safety_session.hpp"
#include "wheeltec_vcu_serial/transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace vcu = wheeltec_vcu_serial;

namespace {

constexpr const char* kActuationConfirmation =
    "I_UNDERSTAND_HOST_WRITE_IS_NOT_A_VCU_ACK";
volatile std::sig_atomic_t g_shutdown_requested = 0;

void handleSignal(int) { g_shutdown_requested = 1; }

struct Options {
  std::string config_path;
  bool validate_only{false};
  bool run{false};
  bool acknowledge_unverified_protocol{false};
  bool enable_actuation{false};
  std::string operator_confirmation;
};

void printUsage(std::ostream& output) {
  output
      << "Usage:\n"
      << "  wheeltec_vcu_cli --config FILE --validate-config\n"
      << "  wheeltec_vcu_cli --config FILE --run "
         "--acknowledge-unverified-protocol --enable-actuation \\\n\n"
      << "    --operator-confirmation " << kActuationConfirmation << "\n\n"
      << "Run mode reads line commands from stdin. It can write a physical TTY.\n"
      << "A full host write is never a VCU acknowledgement.\n\n"
      << "Line protocol (CLOCK_MONOTONIC nanoseconds):\n"
      << "  AUTH token\n"
      << "  CMD sequence created_ns deadline_ns linear_mps yaw_radps\n"
      << "  STOP\n"
      << "  ESTOP\n"
      << "  RESET_ESTOP token\n"
      << "  QUIT\n";
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
    if (argument == "--validate-config") {
      options->validate_only = true;
    } else if (argument == "--run") {
      options->run = true;
    } else if (argument == "--acknowledge-unverified-protocol") {
      options->acknowledge_unverified_protocol = true;
    } else if (argument == "--enable-actuation") {
      options->enable_actuation = true;
    } else if (argument == "--config" ||
               argument == "--operator-confirmation") {
      if (index + 1 >= argc) {
        return false;
      }
      const std::string value(argv[++index]);
      if (argument == "--config") {
        options->config_path = value;
      } else {
        options->operator_confirmation = value;
      }
    } else {
      return false;
    }
  }
  if (options->config_path.empty() ||
      options->validate_only == options->run) {
    return false;
  }
  if (options->validate_only) {
    return !options->acknowledge_unverified_protocol &&
           !options->enable_actuation &&
           options->operator_confirmation.empty();
  }
  return options->acknowledge_unverified_protocol &&
         options->enable_actuation &&
         options->operator_confirmation == kActuationConfirmation;
}

std::int64_t millisecondsToNanoseconds(std::int64_t milliseconds) {
  return milliseconds * 1000000LL;
}

vcu::SafetyConfig makeSafetyConfig(const vcu::RuntimeConfig& config) {
  vcu::SafetyConfig safety;
  safety.max_linear_speed_mps = config.max_linear_speed_mps;
  safety.max_abs_yaw_rate_radps = config.max_abs_yaw_rate_radps;
  safety.max_command_age_ns = millisecondsToNanoseconds(
      config.command_timeout_ms);
  safety.feedback_timeout_ns = millisecondsToNanoseconds(
      config.feedback_timeout_ms);
  safety.transmit_period_ns = millisecondsToNanoseconds(
      config.transmit_period_ms);
  safety.write_timeout_ns = millisecondsToNanoseconds(
      config.write_timeout_ms);
  safety.zero_retry_interval_ns = millisecondsToNanoseconds(
      config.zero_retry_interval_ms);
  safety.maximum_zero_attempts = config.zero_retry_attempts;
  safety.fresh_commands_required = config.fresh_commands_required;
  return safety;
}

const char* transportName(vcu::TransportStatus status) {
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

const char* stateName(vcu::SessionState state) {
  switch (state) {
    case vcu::SessionState::kConfigurationInvalid:
      return "configuration_invalid";
    case vcu::SessionState::kDisconnected:
      return "disconnected";
    case vcu::SessionState::kConnectedInhibited:
      return "connected_inhibited";
    case vcu::SessionState::kRecovering:
      return "recovering";
    case vcu::SessionState::kActive:
      return "active";
    case vcu::SessionState::kStopPending:
      return "stop_pending";
    case vcu::SessionState::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case vcu::SessionState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

const char* authorizationName(vcu::AuthorizationStatus status) {
  switch (status) {
    case vcu::AuthorizationStatus::kAuthorized:
      return "authorized";
    case vcu::AuthorizationStatus::kConfigurationInvalid:
      return "configuration_invalid";
    case vcu::AuthorizationStatus::kDisconnected:
      return "disconnected";
    case vcu::AuthorizationStatus::kInitialZeroPending:
      return "initial_zero_pending";
    case vcu::AuthorizationStatus::kFeedbackUnavailable:
      return "feedback_unavailable";
    case vcu::AuthorizationStatus::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case vcu::AuthorizationStatus::kFaulted:
      return "faulted";
    case vcu::AuthorizationStatus::kTokenInvalid:
      return "token_invalid";
    case vcu::AuthorizationStatus::kTimestampInvalid:
      return "timestamp_invalid";
  }
  return "unknown";
}

const char* submissionName(vcu::SubmissionStatus status) {
  switch (status) {
    case vcu::SubmissionStatus::kAccepted:
      return "accepted";
    case vcu::SubmissionStatus::kRecoveryPending:
      return "recovery_pending";
    case vcu::SubmissionStatus::kStopAccepted:
      return "stop_accepted";
    case vcu::SubmissionStatus::kConfigurationInvalid:
      return "configuration_invalid";
    case vcu::SubmissionStatus::kDisconnected:
      return "disconnected";
    case vcu::SubmissionStatus::kNotAuthorized:
      return "not_authorized";
    case vcu::SubmissionStatus::kSequenceInvalid:
      return "sequence_invalid";
    case vcu::SubmissionStatus::kTimestampInvalid:
      return "timestamp_invalid";
    case vcu::SubmissionStatus::kStale:
      return "stale";
    case vcu::SubmissionStatus::kPredatesAuthorization:
      return "predates_authorization";
    case vcu::SubmissionStatus::kFeedbackUnavailable:
      return "feedback_unavailable";
    case vcu::SubmissionStatus::kEncodingRejected:
      return "encoding_rejected";
    case vcu::SubmissionStatus::kEmergencyStopLatched:
      return "emergency_stop_latched";
    case vcu::SubmissionStatus::kFaulted:
      return "faulted";
  }
  return "unknown";
}

bool onlyTrailingWhitespace(std::istringstream* stream) {
  if (stream == nullptr) {
    return false;
  }
  *stream >> std::ws;
  return stream->eof();
}

bool parseUint64Decimal(std::istringstream* stream, std::uint64_t* output) {
  if (stream == nullptr || output == nullptr) {
    return false;
  }
  std::string token;
  if (!(*stream >> token) || token.empty()) {
    return false;
  }
  std::uint64_t value = 0U;
  for (const char character : token) {
    if (character < '0' || character > '9') {
      return false;
    }
    const std::uint64_t digit =
        static_cast<std::uint64_t>(character - '0');
    if (value > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return false;
    }
    value = value * 10U + digit;
  }
  *output = value;
  return true;
}

struct LineResult {
  bool quit{false};
  bool valid{true};
};

LineResult processLine(const std::string& line, vcu::SafetySession* session) {
  LineResult result;
  if (session == nullptr) {
    result.valid = false;
    return result;
  }
  std::istringstream input(line);
  std::string operation;
  input >> operation;
  const std::int64_t now_ns = vcu::monotonicNowNs();
  if (operation.empty() || now_ns <= 0) {
    result.valid = false;
  } else if (operation == "AUTH") {
    std::uint64_t token = 0U;
    if (!parseUint64Decimal(&input, &token) ||
        !onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else {
      const vcu::AuthorizationStatus status = session->authorize(token, now_ns);
      std::cerr << "authorization=" << authorizationName(status)
                << " token=" << token << "\n";
    }
  } else if (operation == "CMD") {
    vcu::TimedMotionCommand command;
    if (!parseUint64Decimal(&input, &command.sequence_id) ||
        !(input >> command.created_monotonic_ns >>
          command.deadline_monotonic_ns >> command.motion.linear_speed_mps >>
          command.motion.yaw_rate_radps) ||
        !onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else {
      command.motion.lateral_speed_mps = 0.0;
      const vcu::SubmissionStatus status = session->submit(command, now_ns);
      if (status != vcu::SubmissionStatus::kAccepted) {
        std::cerr << "submission=" << submissionName(status)
                  << " sequence=" << command.sequence_id << "\n";
      }
    }
  } else if (operation == "STOP") {
    if (!onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else if (!session->requestStop(now_ns)) {
      // STOP remains syntactically valid while disconnected or fault-latched.
      // The session has already cleared local motion intent; false only means
      // that a new zero episode could not be started on this generation.
      std::cerr << "stop_zero_episode_started=false motion_disarmed=true\n";
    }
  } else if (operation == "ESTOP") {
    if (!onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else {
      session->latchEmergencyStop(now_ns);
      std::cerr << "software_estop=latched\n";
    }
  } else if (operation == "RESET_ESTOP") {
    std::uint64_t token = 0U;
    if (!parseUint64Decimal(&input, &token) ||
        !onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else {
      const bool reset = session->resetEmergencyStop(token, now_ns);
      std::cerr << "software_estop_reset=" << (reset ? "accepted" : "rejected")
                << "\n";
    }
  } else if (operation == "QUIT") {
    if (!onlyTrailingWhitespace(&input)) {
      result.valid = false;
    } else {
      (void)session->requestStop(now_ns);
      result.quit = true;
    }
  } else {
    result.valid = false;
  }
  if (!result.valid) {
    (void)session->disarm(now_ns);
    std::cerr << "input_error=invalid_line motion_disarmed=true\n";
  }
  return result;
}

class NonblockingStdin {
 public:
  bool enable() {
    int flags = -1;
    do {
      flags = ::fcntl(STDIN_FILENO, F_GETFL);
    } while (flags < 0 && errno == EINTR);
    if (flags < 0) {
      return false;
    }
    original_flags_ = flags;
    int status = -1;
    do {
      status = ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    } while (status < 0 && errno == EINTR);
    enabled_ = status == 0;
    return enabled_;
  }

  ~NonblockingStdin() {
    if (!enabled_) {
      return;
    }
    int status = -1;
    do {
      status = ::fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    } while (status < 0 && errno == EINTR);
    (void)status;
  }

 private:
  int original_flags_{0};
  bool enabled_{false};
};

bool appendStdin(std::string* pending, bool* eof) {
  if (pending == nullptr || eof == nullptr) {
    return false;
  }
  char buffer[512U]{};
  for (;;) {
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count > 0) {
      pending->append(buffer, static_cast<std::size_t>(count));
      if (pending->size() > 8192U) {
        return false;
      }
      continue;
    }
    if (count == 0) {
      *eof = true;
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;
    }
    return false;
  }
}

std::vector<std::string> takeCompleteLines(std::string* pending) {
  std::vector<std::string> output;
  if (pending == nullptr) {
    return output;
  }
  for (;;) {
    const std::size_t newline = pending->find('\n');
    if (newline == std::string::npos) {
      break;
    }
    std::string line = pending->substr(0U, newline);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    pending->erase(0U, newline + 1U);
    output.push_back(line);
  }
  return output;
}

void boundedSleepNanoseconds(std::int64_t nanoseconds) {
  if (nanoseconds <= 0) {
    return;
  }
  timespec request{};
  request.tv_sec = static_cast<time_t>(nanoseconds / 1000000000LL);
  request.tv_nsec = static_cast<long>(nanoseconds % 1000000000LL);
  while (::nanosleep(&request, &request) != 0 && errno == EINTR &&
         g_shutdown_requested == 0) {
  }
}

bool attemptFinalZero(vcu::PosixSerialTransport* transport,
                      const vcu::RuntimeConfig& config) {
  if (transport == nullptr || !transport->connected()) {
    std::cerr << "final_zero_host_write_complete=false outcome_uncertain=true "
                 "reason=disconnected\n";
    return false;
  }
  const vcu::CommandFrame zero = vcu::makeZeroCommandFrame();
  for (std::uint32_t attempt = 1U; attempt <= config.zero_retry_attempts;
       ++attempt) {
    const std::int64_t now_ns = vcu::monotonicNowNs();
    if (now_ns <= 0) {
      break;
    }
    const vcu::IoResult written = transport->writeAll(
        zero.data(), zero.size(),
        now_ns + millisecondsToNanoseconds(config.write_timeout_ms));
    if (written.status == vcu::TransportStatus::kOk &&
        written.host_write_complete &&
        written.bytes_transferred == zero.size()) {
      std::cerr << "final_zero_host_write_complete=true "
                   "controller_ack_available=false attempt="
                << attempt << "\n";
      return true;
    }
    std::cerr << "final_zero_attempt=" << attempt
              << " status=" << transportName(written.status)
              << " outcome_uncertain="
              << (written.outcome_uncertain ? "true" : "false") << "\n";
    if (!transport->connected()) {
      break;
    }
    boundedSleepNanoseconds(
        millisecondsToNanoseconds(config.zero_retry_interval_ms));
  }
  std::cerr << "final_zero_host_write_complete=false outcome_uncertain=true "
               "controller_ack_available=false\n";
  return false;
}

void reportCycle(const vcu::CycleResult& cycle,
                 vcu::SessionState* previous_state) {
  if (previous_state != nullptr && cycle.state != *previous_state) {
    std::cerr << "session_state=" << stateName(cycle.state) << "\n";
    *previous_state = cycle.state;
  }
  if (cycle.action == vcu::CycleAction::kZeroHostWriteComplete) {
    std::cerr << "zero_host_write_complete=true controller_ack_available=false\n";
  } else if (cycle.action == vcu::CycleAction::kZeroHostWriteFailed ||
             cycle.action == vcu::CycleAction::kZeroRetriesExhausted) {
    std::cerr << "zero_host_write_complete=false status="
              << transportName(cycle.transport_status)
              << " attempts=" << cycle.zero_attempts
              << " outcome_uncertain="
              << (cycle.outcome_uncertain ? "true" : "false") << "\n";
  }
}

int run(const vcu::RuntimeConfig& config) {
  vcu::PosixSerialTransport transport(config.device_path,
                                      vcu::SerialAccess::kReadWrite);
  const vcu::SerialOpenResult opened = transport.open();
  if (!opened.ok()) {
    std::cerr << "serial_open=failed status=" << transportName(opened.status)
              << " os_error=" << opened.os_error << "\n";
    return 4;
  }

  const vcu::SafetyConfig safety_config = makeSafetyConfig(config);
  vcu::SafetySession session(safety_config, &transport);
  if (!session.configurationValid()) {
    transport.close();
    std::cerr << "safety_configuration=invalid\n";
    return 5;
  }
  NonblockingStdin stdin_mode;
  if (!stdin_mode.enable()) {
    (void)attemptFinalZero(&transport, config);
    transport.close();
    std::cerr << "stdin_nonblocking=failed\n";
    return 6;
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::signal(SIGHUP, handleSignal);

  std::cerr << "serial_open=ok generation=" << opened.generation
            << " controller_ack_available=false reauthorization_required=true\n";
  vcu::FeedbackParser parser;
  std::string pending_input;
  bool input_eof = false;
  bool quit = false;
  bool input_failed = false;
  std::int64_t next_reconnect_ns = 0;
  std::int64_t next_feedback_report_ns = 0;
  vcu::SessionState previous_state = session.state();
  std::uint8_t serial_buffer[512U]{};

  while (!quit && g_shutdown_requested == 0) {
    std::int64_t now_ns = vcu::monotonicNowNs();
    if (now_ns <= 0) {
      session.latchEmergencyStop(now_ns);
      input_failed = true;
      break;
    }

    if (!transport.connected()) {
      if (next_reconnect_ns == 0 || now_ns >= next_reconnect_ns) {
        const vcu::SerialOpenResult reopened = transport.reopen();
        next_reconnect_ns = now_ns +
            millisecondsToNanoseconds(config.reconnect_interval_ms);
        if (reopened.ok()) {
          parser.reset();
          std::cerr << "serial_reopen=ok generation=" << reopened.generation
                    << " reauthorization_required=true\n";
        }
      }
    } else {
      const std::int64_t read_deadline_ns = now_ns + 1000000LL;
      const vcu::IoResult read = transport.readSome(
          serial_buffer, sizeof(serial_buffer), read_deadline_ns);
      if (read.status == vcu::TransportStatus::kOk) {
        const auto frames = parser.consume(serial_buffer, read.bytes_transferred);
        const std::int64_t receipt_ns =
            read.completion_monotonic_ns > 0 ? read.completion_monotonic_ns
                                             : vcu::monotonicNowNs();
        for (const auto& frame : frames) {
          const auto observation = session.observeFeedback(frame.frame,
                                                           receipt_ns);
          if (receipt_ns >= next_feedback_report_ns) {
            std::cerr << "feedback_receipt=true control_allowed="
                      << (observation.control_allowed ? "true" : "false")
                      << " linear_mps=" << frame.feedback.linear_speed_mps
                      << " yaw_radps=" << frame.feedback.yaw_rate_radps
                      << " source_time_available=false vcu_ack_available=false\n";
            next_feedback_report_ns = receipt_ns + 1000000000LL;
          }
        }
      } else if (read.status != vcu::TransportStatus::kDeadlineExceeded &&
                 read.status != vcu::TransportStatus::kDisconnected) {
        session.latchEmergencyStop(now_ns);
        std::cerr << "serial_read=failed status=" << transportName(read.status)
                  << "\n";
      }
    }

    if (!appendStdin(&pending_input, &input_eof)) {
      (void)session.disarm(now_ns);
      input_failed = true;
      break;
    }
    const std::vector<std::string> lines = takeCompleteLines(&pending_input);
    for (const std::string& line : lines) {
      if (line.size() > 512U) {
        (void)session.disarm(now_ns);
        input_failed = true;
        break;
      }
      const LineResult line_result = processLine(line, &session);
      quit = quit || line_result.quit;
      input_failed = input_failed || !line_result.valid;
    }
    if (input_failed) {
      break;
    }
    if (input_eof) {
      if (!pending_input.empty()) {
        const LineResult trailing = processLine(pending_input, &session);
        quit = quit || trailing.quit;
        input_failed = input_failed || !trailing.valid;
        pending_input.clear();
      }
      quit = true;
    }

    now_ns = vcu::monotonicNowNs();
    const vcu::CycleResult cycle = session.tick(now_ns);
    reportCycle(cycle, &previous_state);

    pollfd input_descriptor{};
    input_descriptor.fd = STDIN_FILENO;
    input_descriptor.events = POLLIN | POLLHUP;
    int poll_result = -1;
    do {
      poll_result = ::poll(&input_descriptor, 1U, 5);
    } while (poll_result < 0 && errno == EINTR &&
             g_shutdown_requested == 0);
    if (poll_result < 0 && errno != EINTR) {
      (void)session.disarm(now_ns);
      input_failed = true;
      break;
    }
  }

  const std::int64_t stop_ns = vcu::monotonicNowNs();
  if (stop_ns > 0) {
    (void)session.disarm(stop_ns);
    reportCycle(session.tick(stop_ns), &previous_state);
  }
  const bool final_zero_complete = attemptFinalZero(&transport, config);
  transport.close();
  if (g_shutdown_requested != 0) {
    return final_zero_complete ? 130 : 131;
  }
  if (input_failed) {
    return final_zero_complete ? 7 : 8;
  }
  return final_zero_complete ? 0 : 9;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseOptions(argc, argv, &options)) {
    printUsage(std::cerr);
    return 2;
  }

  const vcu::ConfigResult loaded =
      vcu::loadRuntimeConfig(options.config_path, options.run);
  if (!loaded.ok()) {
    std::cerr << "configuration=" << vcu::configErrorName(loaded.error)
              << " message=" << loaded.message << "\n";
    return 3;
  }
  if (options.validate_only) {
    std::cout << "configuration_valid=true max_linear_speed_mps="
              << loaded.config.max_linear_speed_mps
              << " physical_device_selected="
              << (loaded.config.device_path.empty() ? "false" : "true")
              << "\n";
    return 0;
  }
  return run(loaded.config);
}
