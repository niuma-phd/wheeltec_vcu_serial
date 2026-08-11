// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <limits>
#include <string>

namespace wheeltec_vcu_serial {

struct RuntimeConfig {
  std::string device_path;
  double max_linear_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double max_abs_yaw_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};

  std::int64_t reconnect_interval_ms{0};
  std::int64_t command_timeout_ms{0};
  std::int64_t feedback_timeout_ms{0};
  std::int64_t transmit_period_ms{0};
  std::uint32_t fresh_commands_required{0U};
  std::int64_t write_timeout_ms{0};
  std::int64_t zero_retry_interval_ms{0};
  std::uint32_t zero_retry_attempts{0U};

  double min_selected_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double initial_selected_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double max_selected_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double speed_step_mps{std::numeric_limits<double>::quiet_NaN()};
  double curvature_inv_m{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t release_timeout_ms{0};
  double command_rate_hz{std::numeric_limits<double>::quiet_NaN()};
};

enum class ConfigError : std::uint8_t {
  kNone = 0,
  kFileOpen,
  kSyntax,
  kDuplicateKey,
  kUnknownKey,
  kMissingKey,
  kInvalidValue,
};

struct ConfigResult {
  ConfigError error{ConfigError::kNone};
  RuntimeConfig config{};
  std::string message;

  bool ok() const noexcept { return error == ConfigError::kNone; }
};

ConfigResult loadRuntimeConfig(const std::string& path,
                               bool require_device_path);
ConfigResult validateRuntimeConfig(const RuntimeConfig& config,
                                   bool require_device_path);
const char* configErrorName(ConfigError error) noexcept;

}  // namespace wheeltec_vcu_serial
