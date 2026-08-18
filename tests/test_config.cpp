// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/config.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
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

vcu::RuntimeConfig validConfig() {
  vcu::RuntimeConfig config;
  config.device_path = "/dev/pts/42";
  config.max_linear_speed_mps = 1.25;
  config.max_abs_yaw_rate_radps = 2.0;
  config.reconnect_interval_ms = 500;
  config.command_timeout_ms = 300;
  config.feedback_timeout_ms = 200;
  config.transmit_period_ms = 20;
  config.fresh_commands_required = 3U;
  config.write_timeout_ms = 10;
  config.zero_retry_interval_ms = 25;
  config.zero_retry_attempts = 4U;
  config.min_selected_speed_mps = 0.05;
  config.initial_selected_speed_mps = 0.20;
  config.max_selected_speed_mps = 0.80;
  config.speed_step_mps = 0.05;
  config.curvature_inv_m = 0.75;
  config.release_timeout_ms = 150;
  config.command_rate_hz = 20.0;
  return config;
}

void testValidationBoundaries() {
  vcu::RuntimeConfig config = validConfig();
  check(vcu::validateRuntimeConfig(config, true).ok(),
        "a complete explicit configuration is valid");

  const double invalid_limits[] = {
      0.0,
      -0.1,
      6.001,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  for (double value : invalid_limits) {
    config = validConfig();
    config.max_linear_speed_mps = value;
    check(!vcu::validateRuntimeConfig(config, true).ok(),
          "invalid maximum linear speed fails closed");
  }

  config = validConfig();
  config.max_linear_speed_mps = 6.0;
  check(vcu::validateRuntimeConfig(config, true).ok(),
        "the six metre-per-second project ceiling is accepted");

  config = validConfig();
  config.max_selected_speed_mps = 1.251;
  check(!vcu::validateRuntimeConfig(config, true).ok(),
        "teleop cannot raise the immutable library limit");

  config = validConfig();
  config.device_path.clear();
  check(vcu::validateRuntimeConfig(config, false).ok(),
        "an empty example device is allowed for offline validation");
  check(!vcu::validateRuntimeConfig(config, true).ok(),
        "an actuation configuration requires a direct device path");
}

std::string makeTemporaryFile(const std::string& contents) {
  char path[] = "/tmp/wheeltec-vcu-config-test-XXXXXX";
  const int descriptor = ::mkstemp(path);
  if (descriptor < 0) {
    return std::string();
  }
  ::close(descriptor);
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << contents;
  output.close();
  return path;
}

void testStrictFileParsing() {
#ifdef WHEELTEC_SOURCE_DIR
  const std::string sample =
      std::string(WHEELTEC_SOURCE_DIR) + "/config/wheeltec_vcu_serial.ini";
  const vcu::ConfigResult loaded = vcu::loadRuntimeConfig(sample, false);
  check(loaded.ok(), "the checked-in example parses in offline mode");
  check(loaded.config.device_path.empty(),
        "the checked-in example selects no physical device");

  std::ifstream sample_stream(sample);
  std::ostringstream sample_contents;
  sample_contents << sample_stream.rdbuf();
  const std::string unknown_empty = makeTemporaryFile(
      sample_contents.str() + "\n[unexpected]\n");
  check(!unknown_empty.empty(),
        "temporary unknown-section fixture is created");
  if (!unknown_empty.empty()) {
    const vcu::ConfigResult result =
        vcu::loadRuntimeConfig(unknown_empty, false);
    check(result.error == vcu::ConfigError::kUnknownKey,
          "an empty unknown section is rejected instead of ignored");
    ::unlink(unknown_empty.c_str());
  }

  const std::string duplicate_section = makeTemporaryFile(
      sample_contents.str() + "\n[limits]\n");
  check(!duplicate_section.empty(),
        "temporary duplicate-section fixture is created");
  if (!duplicate_section.empty()) {
    const vcu::ConfigResult result =
        vcu::loadRuntimeConfig(duplicate_section, false);
    check(result.error == vcu::ConfigError::kDuplicateKey,
          "a repeated section is rejected even without repeated keys");
    ::unlink(duplicate_section.c_str());
  }
#else
  check(false, "WHEELTEC_SOURCE_DIR is defined by the build");
#endif

  const std::string missing = makeTemporaryFile(
      "[limits]\nmax_linear_speed_mps=0.4\n");
  check(!missing.empty(), "temporary missing-key fixture is created");
  if (!missing.empty()) {
    const vcu::ConfigResult result = vcu::loadRuntimeConfig(missing, false);
    check(result.error == vcu::ConfigError::kMissingKey,
          "a missing required key is rejected before use");
    ::unlink(missing.c_str());
  }

  const std::string unknown = makeTemporaryFile(
      "[unknown]\nmagic=1\n");
  check(!unknown.empty(), "temporary unknown-key fixture is created");
  if (!unknown.empty()) {
    const vcu::ConfigResult result = vcu::loadRuntimeConfig(unknown, false);
    check(result.error == vcu::ConfigError::kUnknownKey,
          "an unknown key is rejected instead of silently ignored");
    ::unlink(unknown.c_str());
  }
}

}  // namespace

int main() {
  testValidationBoundaries();
  testStrictFileParsing();
  if (failures != 0) {
    std::fprintf(stderr, "%d configuration checks failed\n", failures);
    return EXIT_FAILURE;
  }
  std::puts("configuration checks passed");
  return EXIT_SUCCESS;
}
