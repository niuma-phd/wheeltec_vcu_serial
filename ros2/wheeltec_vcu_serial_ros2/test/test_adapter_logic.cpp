// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial_ros2/adapter_logic.hpp"

#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

namespace adapter = wheeltec_vcu_serial_ros2;

TEST(ActuationGates, AllDefaultIsOffline) {
  EXPECT_EQ(adapter::evaluateActuationGates(adapter::ActuationGates{}),
            adapter::GateDecision::kOffline);
}

TEST(ActuationGates, ExactThreeGateConfigurationEnablesActuation) {
  adapter::ActuationGates gates;
  gates.acknowledge_unverified_protocol = true;
  gates.enable_actuation = true;
  gates.operator_confirmation = adapter::kActuationConfirmation;
  EXPECT_EQ(adapter::evaluateActuationGates(gates),
            adapter::GateDecision::kEnabled);
}

TEST(ActuationGates, EveryPartialOrMisspelledConfigurationIsInvalid) {
  adapter::ActuationGates gates;
  gates.acknowledge_unverified_protocol = true;
  EXPECT_EQ(adapter::evaluateActuationGates(gates),
            adapter::GateDecision::kInvalidPartialConfiguration);

  gates.enable_actuation = true;
  EXPECT_EQ(adapter::evaluateActuationGates(gates),
            adapter::GateDecision::kInvalidPartialConfiguration);

  gates.operator_confirmation = "I_UNDERSTAND";
  EXPECT_EQ(adapter::evaluateActuationGates(gates),
            adapter::GateDecision::kInvalidPartialConfiguration);

  gates.acknowledge_unverified_protocol = false;
  gates.operator_confirmation = adapter::kActuationConfirmation;
  EXPECT_EQ(adapter::evaluateActuationGates(gates),
            adapter::GateDecision::kInvalidPartialConfiguration);
}

TEST(ReceiptDuration, AcceptsPositiveCanonicalBoundedDuration) {
  std::int64_t result = 0;
  EXPECT_TRUE(adapter::boundedDurationNanoseconds(0, 250000000U,
                                                  250000000LL, &result));
  EXPECT_EQ(result, 250000000LL);
  EXPECT_TRUE(adapter::boundedDurationNanoseconds(1, 1U, 2000000000LL,
                                                  &result));
  EXPECT_EQ(result, 1000000001LL);
}

TEST(ReceiptDuration, RejectsZeroNegativeNoncanonicalAndOverBudget) {
  std::int64_t result = 77;
  EXPECT_FALSE(
      adapter::boundedDurationNanoseconds(0, 0U, 250000000LL, &result));
  EXPECT_FALSE(
      adapter::boundedDurationNanoseconds(-1, 1U, 250000000LL, &result));
  EXPECT_FALSE(adapter::boundedDurationNanoseconds(
      0, 1000000000U, 2000000000LL, &result));
  EXPECT_FALSE(adapter::boundedDurationNanoseconds(
      0, 250000001U, 250000000LL, &result));
  EXPECT_FALSE(
      adapter::boundedDurationNanoseconds(0, 1U, 0, &result));
  EXPECT_FALSE(adapter::boundedDurationNanoseconds(
      0, 1U, 250000000LL, nullptr));
  EXPECT_EQ(result, 77);
}

TEST(ReceiptDeadline, RequiresPositiveInputsAndRejectsOverflow) {
  std::int64_t deadline = 0;
  EXPECT_TRUE(adapter::checkedAddNanoseconds(100, 25, &deadline));
  EXPECT_EQ(deadline, 125);
  EXPECT_FALSE(adapter::checkedAddNanoseconds(0, 25, &deadline));
  EXPECT_FALSE(adapter::checkedAddNanoseconds(100, 0, &deadline));
  EXPECT_FALSE(adapter::checkedAddNanoseconds(
      std::numeric_limits<std::int64_t>::max(), 1, &deadline));
  EXPECT_FALSE(adapter::checkedAddNanoseconds(100, 25, nullptr));
}

TEST(SubmissionStatus, RecoveryAndStopOutcomesAreNotRejections) {
  EXPECT_TRUE(adapter::submissionWasAccepted(
      wheeltec_vcu_serial::SubmissionStatus::kAccepted));
  EXPECT_TRUE(adapter::submissionWasAccepted(
      wheeltec_vcu_serial::SubmissionStatus::kRecoveryPending));
  EXPECT_TRUE(adapter::submissionWasAccepted(
      wheeltec_vcu_serial::SubmissionStatus::kStopAccepted));
  EXPECT_FALSE(adapter::submissionWasAccepted(
      wheeltec_vcu_serial::SubmissionStatus::kNotAuthorized));
  EXPECT_FALSE(adapter::submissionWasAccepted(
      wheeltec_vcu_serial::SubmissionStatus::kEncodingRejected));
}

TEST(FinalZeroBudget, FailureUsesExactlyConfiguredAttemptCount) {
  std::uint32_t attempts = 0U;
  std::uint32_t waits = 0U;
  const adapter::BoundedZeroResult result = adapter::runBoundedZeroAttempts(
      3U,
      [&attempts]() {
        ++attempts;
        return adapter::FinalZeroAttemptDisposition::kRetry;
      },
      [&waits]() { ++waits; });

  EXPECT_FALSE(result.host_write_complete);
  EXPECT_EQ(result.attempts_made, 3U);
  EXPECT_EQ(attempts, 3U);
  EXPECT_EQ(waits, 2U);
}

TEST(FinalZeroBudget, SuccessStopsImmediatelyWithoutExtraRetry) {
  std::uint32_t attempts = 0U;
  std::uint32_t waits = 0U;
  const adapter::BoundedZeroResult result = adapter::runBoundedZeroAttempts(
      5U,
      [&attempts]() {
        ++attempts;
        return attempts == 2U
                   ? adapter::FinalZeroAttemptDisposition::kHostWriteComplete
                   : adapter::FinalZeroAttemptDisposition::kRetry;
      },
      [&waits]() { ++waits; });

  EXPECT_TRUE(result.host_write_complete);
  EXPECT_EQ(result.attempts_made, 2U);
  EXPECT_EQ(attempts, 2U);
  EXPECT_EQ(waits, 1U);
}

TEST(FinalZeroBudget, AbortAndZeroBudgetNeverOverrun) {
  std::uint32_t attempts = 0U;
  std::uint32_t waits = 0U;
  const adapter::BoundedZeroResult aborted = adapter::runBoundedZeroAttempts(
      4U,
      [&attempts]() {
        ++attempts;
        return adapter::FinalZeroAttemptDisposition::kAbort;
      },
      [&waits]() { ++waits; });
  EXPECT_FALSE(aborted.host_write_complete);
  EXPECT_EQ(aborted.attempts_made, 1U);
  EXPECT_EQ(attempts, 1U);
  EXPECT_EQ(waits, 0U);

  const adapter::BoundedZeroResult empty = adapter::runBoundedZeroAttempts(
      0U,
      [&attempts]() {
        ++attempts;
        return adapter::FinalZeroAttemptDisposition::kHostWriteComplete;
      },
      [&waits]() { ++waits; });
  EXPECT_FALSE(empty.host_write_complete);
  EXPECT_EQ(empty.attempts_made, 0U);
  EXPECT_EQ(attempts, 1U);
  EXPECT_EQ(waits, 0U);
}
