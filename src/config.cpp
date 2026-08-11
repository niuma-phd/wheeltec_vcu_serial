// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/config.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace wheeltec_vcu_serial {
namespace {

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1U);
}

ConfigResult failure(ConfigError error, const std::string& message) {
  ConfigResult result;
  result.error = error;
  result.message = message;
  return result;
}

bool parseDoubleStrict(const std::string& text, double* value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || end == nullptr || *end != '\0' || errno == ERANGE) {
    return false;
  }
  *value = parsed;
  return true;
}

bool parseInt64Strict(const std::string& text, std::int64_t* value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const long long parsed = std::strtoll(text.c_str(), &end, 10);
  if (end == text.c_str() || end == nullptr || *end != '\0' || errno == ERANGE) {
    return false;
  }
  *value = static_cast<std::int64_t>(parsed);
  return true;
}

bool assignUint32(const std::map<std::string, std::string>& values,
                  const std::string& key, std::uint32_t* output) {
  std::int64_t parsed = 0;
  const auto found = values.find(key);
  if (found == values.end() || !parseInt64Strict(found->second, &parsed) ||
      parsed < 0 ||
      static_cast<std::uint64_t>(parsed) >
          static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
    return false;
  }
  *output = static_cast<std::uint32_t>(parsed);
  return true;
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool validDevicePath(const std::string& path) {
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

}  // namespace

ConfigResult validateRuntimeConfig(const RuntimeConfig& config,
                                   bool require_device_path) {
  if (require_device_path && !validDevicePath(config.device_path)) {
    return failure(ConfigError::kInvalidValue,
                   "serial.device must be a direct, non-traversing /dev path");
  }
  if (!config.device_path.empty() && !validDevicePath(config.device_path)) {
    return failure(ConfigError::kInvalidValue,
                   "serial.device is not a valid direct /dev path");
  }
  if (!std::isfinite(config.max_linear_speed_mps) ||
      config.max_linear_speed_mps <= 0.0 ||
      config.max_linear_speed_mps >= 6.0) {
    return failure(
        ConfigError::kInvalidValue,
        "limits.max_linear_speed_mps must be finite, greater than 0, and less than 6.0");
  }
  if (!finitePositive(config.max_abs_yaw_rate_radps) ||
      config.max_abs_yaw_rate_radps > 32.767) {
    return failure(ConfigError::kInvalidValue,
                   "limits.max_abs_yaw_rate_radps must fit the signed int16 wire field");
  }
  if (config.reconnect_interval_ms < 10 ||
      config.reconnect_interval_ms > 60000) {
    return failure(ConfigError::kInvalidValue,
                   "serial.reconnect_interval_ms must be in [10, 60000]");
  }
  constexpr std::int64_t kMaximumWatchdogMilliseconds = 60000;
  if (config.command_timeout_ms <= 0 ||
      config.command_timeout_ms > kMaximumWatchdogMilliseconds ||
      config.feedback_timeout_ms <= 0 ||
      config.feedback_timeout_ms > kMaximumWatchdogMilliseconds ||
      config.transmit_period_ms <= 0 ||
      config.transmit_period_ms > kMaximumWatchdogMilliseconds ||
      config.write_timeout_ms <= 0 ||
      config.write_timeout_ms > kMaximumWatchdogMilliseconds ||
      config.zero_retry_interval_ms <= 0 ||
      config.zero_retry_interval_ms > kMaximumWatchdogMilliseconds ||
      config.fresh_commands_required == 0U ||
      config.fresh_commands_required > 100U ||
      config.zero_retry_attempts == 0U || config.zero_retry_attempts > 20U) {
    return failure(ConfigError::kInvalidValue,
                   "watchdog values must be positive and bounded");
  }
  if (config.write_timeout_ms > config.transmit_period_ms ||
      config.transmit_period_ms >= config.command_timeout_ms ||
      config.transmit_period_ms >= config.feedback_timeout_ms) {
    return failure(ConfigError::kInvalidValue,
                   "watchdog timing order must satisfy write <= tx < command and feedback timeouts");
  }
  if (!finitePositive(config.min_selected_speed_mps) ||
      !finitePositive(config.initial_selected_speed_mps) ||
      !finitePositive(config.max_selected_speed_mps) ||
      config.min_selected_speed_mps > config.initial_selected_speed_mps ||
      config.initial_selected_speed_mps > config.max_selected_speed_mps ||
      config.max_selected_speed_mps > config.max_linear_speed_mps) {
    return failure(ConfigError::kInvalidValue,
                   "teleop selected speeds must be ordered and within the immutable speed limit");
  }
  if (!finitePositive(config.speed_step_mps) ||
      config.speed_step_mps > config.max_selected_speed_mps ||
      !finitePositive(config.curvature_inv_m) ||
      !finitePositive(config.command_rate_hz) ||
      config.command_rate_hz > 1000.0 || config.release_timeout_ms <= 0 ||
      config.release_timeout_ms > config.command_timeout_ms) {
    return failure(ConfigError::kInvalidValue,
                   "teleop step, curvature, rate, or release timeout is invalid");
  }
  if (config.max_selected_speed_mps * config.curvature_inv_m >
      config.max_abs_yaw_rate_radps) {
    return failure(ConfigError::kInvalidValue,
                   "teleop turn command would exceed the configured yaw-rate limit");
  }
  if ((1000.0 / config.command_rate_hz) >=
      static_cast<double>(config.command_timeout_ms)) {
    return failure(ConfigError::kInvalidValue,
                   "teleop command period must be shorter than the command watchdog");
  }

  ConfigResult result;
  result.config = config;
  return result;
}

ConfigResult loadRuntimeConfig(const std::string& path,
                               bool require_device_path) {
  std::ifstream stream(path);
  if (!stream) {
    return failure(ConfigError::kFileOpen, "cannot open configuration: " + path);
  }

  const std::set<std::string> known_keys{
      "serial.device",
      "serial.reconnect_interval_ms",
      "limits.max_linear_speed_mps",
      "limits.max_abs_yaw_rate_radps",
      "watchdog.command_timeout_ms",
      "watchdog.feedback_timeout_ms",
      "watchdog.transmit_period_ms",
      "watchdog.fresh_commands_required",
      "watchdog.write_timeout_ms",
      "watchdog.zero_retry_interval_ms",
      "watchdog.zero_retry_attempts",
      "teleop.min_selected_speed_mps",
      "teleop.initial_selected_speed_mps",
      "teleop.max_selected_speed_mps",
      "teleop.speed_step_mps",
      "teleop.curvature_inv_m",
      "teleop.release_timeout_ms",
      "teleop.command_rate_hz"};
  const std::set<std::string> known_sections{
      "serial", "limits", "watchdog", "teleop"};

  std::map<std::string, std::string> values;
  std::set<std::string> seen_sections;
  std::string section;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::string cleaned = trim(line);
    if (cleaned.empty() || cleaned.front() == '#' || cleaned.front() == ';') {
      continue;
    }
    if (cleaned.front() == '[' && cleaned.back() == ']') {
      section = trim(cleaned.substr(1U, cleaned.size() - 2U));
      if (section.empty()) {
        return failure(ConfigError::kSyntax,
                       "empty section at line " + std::to_string(line_number));
      }
      if (known_sections.count(section) == 0U) {
        return failure(ConfigError::kUnknownKey,
                       "unknown configuration section: " + section);
      }
      if (!seen_sections.insert(section).second) {
        return failure(ConfigError::kDuplicateKey,
                       "duplicate configuration section: " + section);
      }
      continue;
    }
    const std::size_t equals = cleaned.find('=');
    if (equals == std::string::npos || section.empty()) {
      return failure(ConfigError::kSyntax,
                     "expected sectioned key=value at line " +
                         std::to_string(line_number));
    }
    const std::string key = section + "." + trim(cleaned.substr(0U, equals));
    const std::string value = trim(cleaned.substr(equals + 1U));
    if (known_keys.count(key) == 0U) {
      return failure(ConfigError::kUnknownKey,
                     "unknown configuration key: " + key);
    }
    if (values.count(key) != 0U) {
      return failure(ConfigError::kDuplicateKey,
                     "duplicate configuration key: " + key);
    }
    values.emplace(key, value);
  }
  if (!stream.eof()) {
    return failure(ConfigError::kFileOpen,
                   "failed while reading configuration: " + path);
  }

  for (const std::string& key : known_keys) {
    if (values.count(key) == 0U) {
      return failure(ConfigError::kMissingKey,
                     "missing required configuration key: " + key);
    }
  }

  RuntimeConfig config;
  config.device_path = values["serial.device"];
  bool parsed =
      parseDoubleStrict(values["limits.max_linear_speed_mps"],
                        &config.max_linear_speed_mps) &&
      parseDoubleStrict(values["limits.max_abs_yaw_rate_radps"],
                        &config.max_abs_yaw_rate_radps) &&
      parseInt64Strict(values["serial.reconnect_interval_ms"],
                       &config.reconnect_interval_ms) &&
      parseInt64Strict(values["watchdog.command_timeout_ms"],
                       &config.command_timeout_ms) &&
      parseInt64Strict(values["watchdog.feedback_timeout_ms"],
                       &config.feedback_timeout_ms) &&
      parseInt64Strict(values["watchdog.transmit_period_ms"],
                       &config.transmit_period_ms) &&
      assignUint32(values, "watchdog.fresh_commands_required",
                   &config.fresh_commands_required) &&
      parseInt64Strict(values["watchdog.write_timeout_ms"],
                       &config.write_timeout_ms) &&
      parseInt64Strict(values["watchdog.zero_retry_interval_ms"],
                       &config.zero_retry_interval_ms) &&
      assignUint32(values, "watchdog.zero_retry_attempts",
                   &config.zero_retry_attempts) &&
      parseDoubleStrict(values["teleop.min_selected_speed_mps"],
                        &config.min_selected_speed_mps) &&
      parseDoubleStrict(values["teleop.initial_selected_speed_mps"],
                        &config.initial_selected_speed_mps) &&
      parseDoubleStrict(values["teleop.max_selected_speed_mps"],
                        &config.max_selected_speed_mps) &&
      parseDoubleStrict(values["teleop.speed_step_mps"],
                        &config.speed_step_mps) &&
      parseDoubleStrict(values["teleop.curvature_inv_m"],
                        &config.curvature_inv_m) &&
      parseInt64Strict(values["teleop.release_timeout_ms"],
                       &config.release_timeout_ms) &&
      parseDoubleStrict(values["teleop.command_rate_hz"],
                        &config.command_rate_hz);
  if (!parsed) {
    return failure(ConfigError::kInvalidValue,
                   "one or more numeric configuration values are malformed");
  }
  return validateRuntimeConfig(config, require_device_path);
}

const char* configErrorName(ConfigError error) noexcept {
  switch (error) {
    case ConfigError::kNone:
      return "none";
    case ConfigError::kFileOpen:
      return "file_open";
    case ConfigError::kSyntax:
      return "syntax";
    case ConfigError::kDuplicateKey:
      return "duplicate_key";
    case ConfigError::kUnknownKey:
      return "unknown_key";
    case ConfigError::kMissingKey:
      return "missing_key";
    case ConfigError::kInvalidValue:
      return "invalid_value";
  }
  return "unknown";
}

}  // namespace wheeltec_vcu_serial
