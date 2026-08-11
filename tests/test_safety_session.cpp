// SPDX-License-Identifier: Apache-2.0
#include "wheeltec_vcu_serial/safety_session.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <stdexcept>
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

struct WriteOutcome {
  wvs::TransportStatus status{wvs::TransportStatus::kOk};
  bool host_write_complete{true};
  bool outcome_uncertain{false};
  std::size_t bytes_transferred{static_cast<std::size_t>(-1)};
};

class FakeTransport final : public wvs::ITransport {
 public:
  bool connected() const noexcept override { return connected_; }
  std::uint64_t generation() const noexcept override { return generation_; }

  wvs::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                         std::int64_t) override {
    if (throw_next_write_) {
      throw_next_write_ = false;
      throw std::runtime_error("injected write exception");
    }
    writes.emplace_back(data, data + size);
    WriteOutcome outcome;
    if (!outcomes.empty()) {
      outcome = outcomes.front();
      outcomes.pop_front();
    }
    if (outcome.status == wvs::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    wvs::IoResult result;
    result.status = outcome.status;
    result.bytes_transferred =
        outcome.bytes_transferred == static_cast<std::size_t>(-1)
            ? (outcome.host_write_complete ? size : 0U)
            : outcome.bytes_transferred;
    result.os_error = outcome.status == wvs::TransportStatus::kOk ? 0 : EIO;
    result.host_write_complete = outcome.host_write_complete;
    result.outcome_uncertain = outcome.outcome_uncertain;
    result.completion_monotonic_ns = now_ns;
    return result;
  }

  wvs::IoResult readSome(std::uint8_t*, std::size_t,
                         std::int64_t) override {
    wvs::IoResult result;
    result.status = wvs::TransportStatus::kDeadlineExceeded;
    result.os_error = ETIMEDOUT;
    result.completion_monotonic_ns = now_ns;
    return result;
  }

  void disconnect() { connected_ = false; }
  void reconnect() {
    connected_ = true;
    ++generation_;
  }

  bool connected_{true};
  std::uint64_t generation_{1U};
  std::int64_t now_ns{0};
  bool throw_next_write_{false};
  std::deque<WriteOutcome> outcomes;
  std::vector<std::vector<std::uint8_t>> writes;
};

wvs::CycleResult tickAt(wvs::SafetySession* session,
                        FakeTransport* transport, std::int64_t now_ns) {
  transport->now_ns = now_ns;
  return session->tick(now_ns);
}

wvs::SafetyConfig validConfig() {
  wvs::SafetyConfig config;
  config.max_linear_speed_mps = 0.50;
  config.max_abs_yaw_rate_radps = 1.0;
  config.max_command_age_ns = 50;
  config.feedback_timeout_ns = 60;
  config.transmit_period_ns = 10;
  config.write_timeout_ns = 5;
  config.zero_retry_interval_ns = 10;
  config.maximum_zero_attempts = 3U;
  config.fresh_commands_required = 2U;
  return config;
}

wvs::FeedbackFrame feedbackFrame(std::uint8_t stop_flag = 0U) {
  wvs::FeedbackFrame frame{};
  frame[0U] = wvs::kFrameHeader;
  frame[1U] = stop_flag;
  std::uint8_t checksum = 0U;
  for (std::size_t index = 0U; index < 22U; ++index) {
    checksum = static_cast<std::uint8_t>(checksum ^ frame[index]);
  }
  frame[22U] = checksum;
  frame[23U] = wvs::kFrameTail;
  return frame;
}

wvs::TimedMotionCommand command(std::uint64_t sequence,
                                std::int64_t created_ns,
                                double linear_speed_mps = 0.20,
                                double yaw_rate_radps = 0.10) {
  wvs::TimedMotionCommand result;
  result.sequence_id = sequence;
  result.created_monotonic_ns = created_ns;
  result.deadline_monotonic_ns = created_ns + 200;
  result.motion.linear_speed_mps = linear_speed_mps;
  result.motion.yaw_rate_radps = yaw_rate_radps;
  return result;
}

bool isExactZero(const std::vector<std::uint8_t>& bytes) {
  const wvs::CommandFrame expected = wvs::makeZeroCommandFrame();
  return bytes.size() == expected.size() &&
         std::equal(bytes.begin(), bytes.end(), expected.begin());
}

std::int16_t signedBigEndian(std::uint8_t high, std::uint8_t low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low));
  return bits <= 0x7FFFU
             ? static_cast<std::int16_t>(bits)
             : static_cast<std::int16_t>(
                   static_cast<std::int32_t>(bits) - 65536);
}

void completeInitialZero(wvs::SafetySession* session,
                         FakeTransport* transport,
                         std::int64_t at_ns = 100) {
  const wvs::CycleResult result = tickAt(session, transport, at_ns);
  expect(result.action == wvs::CycleAction::kZeroHostWriteComplete &&
             result.zero_write_attempted && result.zero_host_write_complete &&
             session->state() == wvs::SessionState::kConnectedInhibited &&
             !transport->writes.empty() &&
             isExactZero(transport->writes.back()),
         "new connection performs an exact-zero host write before authorization");
}

void prepareActive(wvs::SafetySession* session, FakeTransport* transport,
                   std::int64_t base_ns = 100,
                   double linear_speed_mps = 0.20) {
  completeInitialZero(session, transport, base_ns);
  const wvs::FeedbackObservationResult feedback = session->observeFeedback(
      feedbackFrame(0U), base_ns + 1);
  expect(feedback.status ==
             wvs::FeedbackObservationStatus::kAcceptedControlAllowed,
         "valid FlagStop=0 feedback is accepted");
  expect(session->authorize(1U, base_ns + 2) ==
             wvs::AuthorizationStatus::kAuthorized,
         "fresh explicit authorization binds to the connection generation");
  expect(session->submit(command(1U, base_ns + 3, linear_speed_mps),
                         base_ns + 3) ==
             wvs::SubmissionStatus::kRecoveryPending,
         "first post-authorization command remains recovery inhibited");
  expect(session->submit(command(2U, base_ns + 4, linear_speed_mps),
                         base_ns + 4) == wvs::SubmissionStatus::kAccepted,
         "second distinct fresh command completes recovery");
  expect(session->state() == wvs::SessionState::kActive,
         "fresh recovery run enters active state");
}

void testConfigurationFailsClosed() {
  FakeTransport transport;
  wvs::SafetyConfig missing;
  wvs::SafetySession missing_session(missing, &transport);
  const wvs::CycleResult result = tickAt(&missing_session, &transport, 100);
  expect(!missing_session.configurationValid() &&
             result.action == wvs::CycleAction::kConfigurationInvalid &&
             transport.writes.empty(),
         "missing NaN speed/yaw limits cannot authorize or write motion");

  wvs::SafetyConfig invalid = validConfig();
  invalid.max_linear_speed_mps = 6.0;
  expect(!wvs::safetyConfigIsValid(invalid),
         "6.0 m/s is an exclusive invalid safety-session limit");
  invalid = validConfig();
  invalid.max_linear_speed_mps = 0.0;
  expect(!wvs::safetyConfigIsValid(invalid),
         "a zero linear-speed limit fails closed");
  invalid = validConfig();
  invalid.max_linear_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  expect(!wvs::safetyConfigIsValid(invalid),
         "a NaN linear-speed limit fails closed");
  invalid = validConfig();
  invalid.max_linear_speed_mps =
      std::numeric_limits<double>::infinity();
  expect(!wvs::safetyConfigIsValid(invalid),
         "an infinite linear-speed limit fails closed");
  invalid = validConfig();
  invalid.max_abs_yaw_rate_radps = 32.768;
  expect(!wvs::safetyConfigIsValid(invalid),
         "yaw safety limit cannot exceed the symmetric int16 wire range");
  invalid = validConfig();
  invalid.max_abs_yaw_rate_radps = 0.0;
  expect(!wvs::safetyConfigIsValid(invalid),
         "a zero yaw-rate limit fails closed");
  invalid = validConfig();
  invalid.max_abs_yaw_rate_radps =
      std::numeric_limits<double>::infinity();
  expect(!wvs::safetyConfigIsValid(invalid),
         "an infinite yaw-rate limit fails closed");
  invalid = validConfig();
  invalid.max_abs_yaw_rate_radps = 32.767;
  expect(wvs::safetyConfigIsValid(invalid),
         "the documented yaw-rate wire boundary remains configurable");
  invalid = validConfig();
  invalid.write_timeout_ns = invalid.transmit_period_ns + 1;
  expect(!wvs::safetyConfigIsValid(invalid),
         "direct API rejects a write deadline longer than the transmit period");
  invalid = validConfig();
  invalid.transmit_period_ns = invalid.max_command_age_ns;
  expect(!wvs::safetyConfigIsValid(invalid),
         "direct API requires transmit period below the command watchdog");
  invalid = validConfig();
  invalid.feedback_timeout_ns = 60000000001LL;
  expect(!wvs::safetyConfigIsValid(invalid),
         "direct API bounds feedback deadlines to sixty seconds");
  invalid = validConfig();
  invalid.maximum_zero_attempts = 21U;
  expect(!wvs::safetyConfigIsValid(invalid),
         "direct API bounds each zero retry episode");
  invalid = validConfig();
  invalid.fresh_commands_required = 101U;
  expect(!wvs::safetyConfigIsValid(invalid),
         "direct API bounds the fresh-command recovery count");
}

void testAuthorizationRecoveryRateAndWatchdog() {
  FakeTransport transport;
  wvs::SafetySession session(validConfig(), &transport);
  expect(session.authorize(1U, 90) ==
             wvs::AuthorizationStatus::kInitialZeroPending,
         "authorization is refused until the connection zero completes");
  completeInitialZero(&session, &transport, 100);

  wvs::FeedbackFrame corrupt = feedbackFrame(0U);
  corrupt[22U] ^= 0x01U;
  expect(session.observeFeedback(corrupt, 101).status ==
             wvs::FeedbackObservationStatus::kFrameInvalid &&
             session.authorize(1U, 102) ==
                 wvs::AuthorizationStatus::kFeedbackUnavailable,
         "checksum-invalid feedback cannot satisfy the authorization gate");
  expect(session.observeFeedback(feedbackFrame(0U), 103).status ==
             wvs::FeedbackObservationStatus::kAcceptedControlAllowed &&
             session.authorize(1U, 104) ==
                 wvs::AuthorizationStatus::kAuthorized,
         "only valid FlagStop=0 feedback permits authorization");
  expect(session.submit(command(1U, 105), 105) ==
             wvs::SubmissionStatus::kRecoveryPending &&
             session.submit(command(2U, 106), 106) ==
                 wvs::SubmissionStatus::kAccepted,
         "authorization requires the configured fresh-command run");

  const wvs::CycleResult first_motion = tickAt(&session, &transport, 106);
  expect(first_motion.action ==
                 wvs::CycleAction::kMotionHostWriteComplete &&
             first_motion.motion_host_write_complete &&
             !isExactZero(transport.writes.back()),
         "active session completes a host-side motion write");
  const wvs::CycleResult limited = tickAt(&session, &transport, 111);
  expect(limited.action == wvs::CycleAction::kTransmitRateLimited,
         "transmit period prevents a caller from overspeeding the wire loop");
  const wvs::CycleResult second_motion = tickAt(&session, &transport, 116);
  expect(second_motion.action ==
             wvs::CycleAction::kMotionHostWriteComplete,
         "motion may repeat at the exact configured transmit period");

  const wvs::CycleResult watchdog = tickAt(&session, &transport, 157);
  expect(watchdog.action == wvs::CycleAction::kZeroHostWriteComplete &&
             watchdog.zero_write_attempted &&
             isExactZero(transport.writes.back()) &&
             session.state() == wvs::SessionState::kRecovering,
         "expired command watchdog starts exact zero in the same tick");
}

void testSignedMotionAndSessionLimits() {
  {
    FakeTransport transport;
    wvs::SafetySession session(validConfig(), &transport);
    prepareActive(&session, &transport, 100, -0.20);
    const wvs::CycleResult result = tickAt(&session, &transport, 104);
    expect(result.action == wvs::CycleAction::kMotionHostWriteComplete &&
               signedBigEndian(transport.writes.back()[3U],
                               transport.writes.back()[4U]) == -200,
           "signed reverse command is bounded by the absolute linear limit");
  }
  {
    FakeTransport transport;
    wvs::SafetySession session(validConfig(), &transport);
    prepareActive(&session, &transport);
    wvs::TimedMotionCommand lateral = command(3U, 105);
    lateral.motion.lateral_speed_mps = 0.001;
    expect(session.submit(lateral, 105) ==
               wvs::SubmissionStatus::kEncodingRejected,
           "Ackermann safety-session boundary rejects lateral motion");
    expect(tickAt(&session, &transport, 105).action ==
               wvs::CycleAction::kZeroHostWriteComplete,
           "rejected lateral motion enters an exact-zero episode");
  }
  {
    FakeTransport transport;
    wvs::SafetyConfig config = validConfig();
    config.max_abs_yaw_rate_radps = 0.20;
    wvs::SafetySession session(config, &transport);
    prepareActive(&session, &transport);
    expect(session.submit(command(3U, 105, 0.20, 0.201), 105) ==
               wvs::SubmissionStatus::kEncodingRejected,
           "session enforces its explicit yaw-rate limit before encoding");
  }
}

void testFeedbackWatchdogAndCompositeInhibit() {
  {
    FakeTransport transport;
    wvs::SafetyConfig config = validConfig();
    config.feedback_timeout_ns = 20;
    config.max_command_age_ns = 100;
    wvs::SafetySession session(config, &transport);
    prepareActive(&session, &transport);
    expect(tickAt(&session, &transport, 104).action ==
               wvs::CycleAction::kMotionHostWriteComplete,
           "feedback-watchdog fixture writes motion first");
    wvs::FeedbackFrame corrupt = feedbackFrame(0U);
    corrupt[22U] ^= 0x80U;
    expect(session.observeFeedback(corrupt, 110).status ==
               wvs::FeedbackObservationStatus::kFrameInvalid,
           "invalid frame does not refresh feedback freshness");
    const wvs::CycleResult stale = tickAt(&session, &transport, 122);
    expect(stale.action == wvs::CycleAction::kZeroHostWriteComplete &&
               isExactZero(transport.writes.back()),
           "only valid allowed feedback refreshes the watchdog");
  }
  {
    FakeTransport transport;
    wvs::SafetySession session(validConfig(), &transport);
    prepareActive(&session, &transport);
    expect(session.observeFeedback(feedbackFrame(1U), 105).status ==
               wvs::FeedbackObservationStatus::kAcceptedControlInhibited &&
               !session.authorized(),
           "valid FlagStop=1 immediately revokes motion authorization");
    const wvs::CycleResult stopped = tickAt(&session, &transport, 105);
    expect(stopped.action == wvs::CycleAction::kZeroHostWriteComplete &&
               session.state() == wvs::SessionState::kConnectedInhibited,
           "composite inhibit takes the bounded zero path");
  }
}

void testMotionFailureAttemptsZeroInSameTick() {
  FakeTransport transport;
  wvs::SafetySession session(validConfig(), &transport);
  prepareActive(&session, &transport);
  const std::size_t before = transport.writes.size();
  transport.outcomes.push_back(
      {wvs::TransportStatus::kIoError, false, true, 0U});
  transport.outcomes.push_back(
      {wvs::TransportStatus::kOk, true, false,
       static_cast<std::size_t>(-1)});
  const wvs::CycleResult result = tickAt(&session, &transport, 104);
  expect(result.action == wvs::CycleAction::kZeroHostWriteComplete &&
             result.motion_write_attempted &&
             !result.motion_host_write_complete &&
             result.zero_write_attempted && result.zero_host_write_complete &&
             result.outcome_uncertain &&
             transport.writes.size() == before + 2U &&
             isExactZero(transport.writes.back()),
         "motion write failure invokes bounded exact zero in the same tick");

  FakeTransport throwing_transport;
  wvs::SafetySession throwing_session(validConfig(), &throwing_transport);
  prepareActive(&throwing_session, &throwing_transport);
  throwing_transport.throw_next_write_ = true;
  const wvs::CycleResult exception_result =
      tickAt(&throwing_session, &throwing_transport, 104);
  expect(exception_result.action ==
                 wvs::CycleAction::kZeroHostWriteComplete &&
             exception_result.motion_write_attempted &&
             exception_result.zero_write_attempted &&
             isExactZero(throwing_transport.writes.back()),
         "transport exception is contained and cannot skip the zero path");
}

void testBoundedZeroRetries() {
  FakeTransport transport;
  transport.outcomes.push_back(
      {wvs::TransportStatus::kDeadlineExceeded, false, false, 0U});
  transport.outcomes.push_back(
      {wvs::TransportStatus::kDeadlineExceeded, false, false, 0U});
  transport.outcomes.push_back(
      {wvs::TransportStatus::kDeadlineExceeded, false, false, 0U});
  wvs::SafetySession session(validConfig(), &transport);
  const wvs::CycleResult first = tickAt(&session, &transport, 100);
  const wvs::CycleResult pending = tickAt(&session, &transport, 105);
  const wvs::CycleResult second = tickAt(&session, &transport, 110);
  const wvs::CycleResult exhausted = tickAt(&session, &transport, 120);
  expect(first.action == wvs::CycleAction::kZeroHostWriteFailed &&
             first.zero_attempts == 1U &&
             pending.action == wvs::CycleAction::kZeroRetryPending &&
             second.action == wvs::CycleAction::kZeroHostWriteFailed &&
             second.zero_attempts == 2U &&
             exhausted.action == wvs::CycleAction::kZeroRetriesExhausted &&
             exhausted.zero_attempts == 3U &&
             session.state() == wvs::SessionState::kFaulted &&
             transport.writes.size() == 3U,
         "zero retries obey interval and stop at the configured bound");
  const std::size_t at_bound = transport.writes.size();
  expect(tickAt(&session, &transport, 200).action ==
                 wvs::CycleAction::kNoAction &&
             transport.writes.size() == at_bound,
         "faulted state performs no unbounded extra zero writes");

  expect(!session.requestStop(201) && !session.disarm(202),
         "STOP and disarm cannot silently reset a zero-write fault");
  (void)session.observeFeedback(feedbackFrame(1U), 203);
  session.latchEmergencyStop(204);
  expect(tickAt(&session, &transport, 205).action ==
                 wvs::CycleAction::kNoAction &&
             session.state() == wvs::SessionState::kFaulted &&
             transport.writes.size() == at_bound,
         "feedback and ESTOP cannot restart a faulted generation's retry budget");

  transport.disconnect();
  expect(tickAt(&session, &transport, 206).action ==
                 wvs::CycleAction::kDisconnected &&
             session.requestStop(207),
         "disconnected STOP is accepted as local fail-safe intent");
  transport.reconnect();
  const wvs::CycleResult after_reconnect =
      tickAt(&session, &transport, 210);
  expect(after_reconnect.action ==
                 wvs::CycleAction::kZeroHostWriteComplete &&
             session.state() == wvs::SessionState::kEmergencyStopLatched &&
             transport.writes.size() == at_bound + 1U &&
             isExactZero(transport.writes.back()),
         "new generation resets the write fault only through initial zero and preserves ESTOP");
}

void testReconnectRequiresNewAuthorizationAndCommands() {
  FakeTransport transport;
  wvs::SafetySession session(validConfig(), &transport);
  prepareActive(&session, &transport);
  expect(tickAt(&session, &transport, 104).action ==
             wvs::CycleAction::kMotionHostWriteComplete,
         "reconnect fixture first writes motion");

  transport.disconnect();
  expect(tickAt(&session, &transport, 105).action ==
                 wvs::CycleAction::kDisconnected &&
             !session.authorized(),
         "disconnect clears cached intent and authorization");
  transport.reconnect();
  const wvs::CycleResult initial_zero = tickAt(&session, &transport, 110);
  expect(initial_zero.action == wvs::CycleAction::kZeroHostWriteComplete &&
             session.connectionGeneration() == 2U &&
             isExactZero(transport.writes.back()),
         "reopen generation writes zero and cannot replay cached motion");
  expect(session.observeFeedback(feedbackFrame(0U), 111).control_allowed &&
             session.authorize(1U, 112) ==
                 wvs::AuthorizationStatus::kTokenInvalid,
         "old authorization token cannot cross a reconnect");
  expect(session.submit(command(2U, 113), 113) ==
             wvs::SubmissionStatus::kSequenceInvalid,
         "sequence high-water survives reconnect and rejects replay");
  expect(tickAt(&session, &transport, 113).action ==
             wvs::CycleAction::kZeroHostWriteComplete,
         "replayed sequence remains inhibited through exact zero");

  expect(session.observeFeedback(feedbackFrame(0U), 114).control_allowed &&
             session.authorize(2U, 115) ==
                 wvs::AuthorizationStatus::kAuthorized &&
             session.submit(command(3U, 116), 116) ==
                 wvs::SubmissionStatus::kRecoveryPending &&
             session.submit(command(4U, 117), 117) ==
                 wvs::SubmissionStatus::kAccepted,
         "new generation needs a new token and a new fresh-command run");
  expect(tickAt(&session, &transport, 117).action ==
             wvs::CycleAction::kMotionHostWriteComplete,
         "motion resumes only after explicit post-reconnect recovery");
}

void testClockRollbackAndEmergencyStopLatch() {
  {
    FakeTransport transport;
    wvs::SafetySession session(validConfig(), &transport);
    prepareActive(&session, &transport);
    expect(tickAt(&session, &transport, 104).action ==
               wvs::CycleAction::kMotionHostWriteComplete,
           "clock fixture writes motion before rollback");
    const std::size_t before = transport.writes.size();
    const wvs::CycleResult rollback = tickAt(&session, &transport, 103);
    expect(rollback.action == wvs::CycleAction::kClockInvalid &&
               !session.authorized() && transport.writes.size() == before,
           "monotonic rollback revokes authorization without normal TX");
    expect(tickAt(&session, &transport, 105).action ==
                   wvs::CycleAction::kZeroHostWriteComplete &&
               isExactZero(transport.writes.back()),
           "first valid post-rollback tick performs the pending zero");
  }
  {
    FakeTransport transport;
    wvs::SafetySession session(validConfig(), &transport);
    prepareActive(&session, &transport);
    expect(tickAt(&session, &transport, 104).action ==
               wvs::CycleAction::kMotionHostWriteComplete,
           "E-stop fixture writes motion before assertion");
    session.latchEmergencyStop(105);
    const wvs::CycleResult stopped = tickAt(&session, &transport, 105);
    expect(stopped.action == wvs::CycleAction::kZeroHostWriteComplete &&
               session.emergencyStopLatched() &&
               session.state() == wvs::SessionState::kEmergencyStopLatched,
           "software emergency stop latches and takes the zero path");
    expect(!session.resetEmergencyStop(1U, 106),
           "E-stop reset requires fresh allowed feedback after assertion");
    expect(session.observeFeedback(feedbackFrame(0U), 107).control_allowed &&
               session.resetEmergencyStop(1U, 108) &&
               !session.emergencyStopLatched() && !session.authorized() &&
               session.state() == wvs::SessionState::kConnectedInhibited,
           "authorized reset clears only the latch, never motion authorization");

    session.latchEmergencyStop(109);
    expect(tickAt(&session, &transport, 109).action ==
               wvs::CycleAction::kZeroHostWriteComplete,
           "a later E-stop assertion creates a new zero episode");
    expect(session.observeFeedback(feedbackFrame(0U), 110).control_allowed &&
               !session.resetEmergencyStop(1U, 111) &&
               session.resetEmergencyStop(2U, 112),
           "software E-stop reset tokens are strictly monotonic");
  }
}

}  // namespace

int main() {
  testConfigurationFailsClosed();
  testAuthorizationRecoveryRateAndWatchdog();
  testSignedMotionAndSessionLimits();
  testFeedbackWatchdogAndCompositeInhibit();
  testMotionFailureAttemptsZeroInSameTick();
  testBoundedZeroRetries();
  testReconnectRequiresNewAuthorizationAndCommands();
  testClockRollbackAndEmergencyStopLatch();
  if (failures != 0) {
    std::fprintf(stderr, "%d safety-session test(s) failed\n", failures);
    return 1;
  }
  std::puts("safety-session tests passed");
  return 0;
}
