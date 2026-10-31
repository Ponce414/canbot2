// Unit tests for lib/canbot_core. Run on your laptop:  pio test -e native
//
// Each test pins down one behavior the robot depends on. If a later edit breaks it, the
// test fails here in seconds instead of as a wheel doing something strange on the bench.
#include <unity.h>

#include <cmath>
#include <cstring>

#include "safety.h"
#include "speed_estimator.h"
#include "text_protocol.h"
#include "velocity_pid.h"

using namespace canbot;

static constexpr float kPi = 3.14159265358979f;
static constexpr double kPiD = 3.14159265358979323846;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// VelocityPid
// ---------------------------------------------------------------------------

static PidLimits limits() { return PidLimits{1.0f, 0.3f, 0.05f}; }

void test_pid_zero_target_outputs_zero_and_clears_integral() {
  VelocityPid pid(PidGains{0.1f, 1.0f, 0.05f, 0.04f}, limits());
  pid.update(5.0f, 0.0f, 0.01f);  // build up some integral
  TEST_ASSERT_TRUE(pid.integral() > 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(0.0f, 3.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
}

void test_pid_feedforward_only_when_error_is_zero() {
  // With the wheel already at target, output = kff * target + kstatic.
  VelocityPid pid(PidGains{0.5f, 0.0f, 0.05f, 0.04f}, limits());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f * 10.0f + 0.04f, pid.update(10.0f, 10.0f, 0.01f));
  // Reverse direction: static friction term flips sign with the target.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -(0.05f * 10.0f + 0.04f), pid.update(-10.0f, -10.0f, 0.01f));
}

void test_pid_supply_scale_affects_only_feedforward() {
  VelocityPid pid(PidGains{0.1f, 0.0f, 0.05f, 0.0f}, limits());
  // ff = 1.2 * 0.05 * 4 = 0.24; p = 0.1 * (4 - 3) = 0.1
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.34f, pid.update(4.0f, 3.0f, 0.01f, 1.2f));
}

void test_pid_output_is_clamped() {
  VelocityPid pid(PidGains{1.0f, 0.0f, 0.2f, 0.0f}, limits());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(20.0f, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.update(-20.0f, 0.0f, 0.01f));
}

void test_pid_integral_does_not_wind_up_while_saturated() {
  // Wheel blocked at 0 while the target demands full duty: the integral must not keep
  // growing, or the wheel overshoots for seconds once it's freed.
  VelocityPid pid(PidGains{0.1f, 2.0f, 0.05f, 0.0f}, limits());
  for (int i = 0; i < 500; ++i) pid.update(18.0f, 0.0f, 0.01f);  // 5 s saturated
  const float wound = pid.integral();
  TEST_ASSERT_TRUE(wound < 0.3f + 1e-6f);  // never past i_max
  // ff + p alone = 0.9 + 1.8 > 1, so the loop is saturated from the start and the
  // integral should never have moved at all.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, wound);
}

void test_pid_integral_removes_steady_error() {
  // Toy plant: speed responds to duty with gain 15 (rad/s per duty) but feedforward
  // assumes 20, so feedforward alone undershoots. The integral must close the gap.
  VelocityPid pid(PidGains{0.02f, 0.5f, 0.05f, 0.0f}, limits());
  float speed = 0.0f;
  for (int i = 0; i < 2000; ++i) {
    const float duty = pid.update(10.0f, speed, 0.01f);
    speed += (15.0f * duty - speed) * 0.1f;  // first-order lag toward 15 * duty
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, speed);
}

void test_pid_ignores_bad_dt() {
  VelocityPid pid(PidGains{0.0f, 1.0f, 0.0f, 0.0f}, limits());
  pid.update(5.0f, 0.0f, 5.0f);  // a 5 s gap (task stalled) must not dump 25 into the integral
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
  pid.update(5.0f, 0.0f, -0.01f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
}

// ---------------------------------------------------------------------------
// SpeedEstimator
// ---------------------------------------------------------------------------

void test_speed_first_sample_primes_without_spike() {
  SpeedEstimator est(1320.0f, 4);
  // A non-zero count at boot (e.g. wheel nudged before reset) must not read as a huge speed.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.update(5000, 0.01f));
}

void test_speed_constant_rate() {
  SpeedEstimator est(1320.0f, 4);
  est.reset(0);
  int64_t count = 0;
  float s = 0.0f;
  for (int i = 0; i < 10; ++i) {
    count += 132;  // 132 counts per 10 ms = 0.1 rev per 10 ms = 10 rev/s
    s = est.update(count, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f * 2.0f * kPi, s);
}

void test_speed_window_smooths_quantization() {
  // Alternating 4 and 5 counts per period (true rate 4.5): a 1-sample estimate would jump
  // by 22% every loop; the 4-sample window must read the true average exactly.
  SpeedEstimator est(1320.0f, 4);
  est.reset(0);
  int64_t count = 0;
  float s = 0.0f;
  for (int i = 0; i < 12; ++i) {
    count += (i % 2 == 0) ? 4 : 5;
    s = est.update(count, 0.01f);
  }
  const float expected = 4.5f / 1320.0f * 2.0f * kPi / 0.01f;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, s);
}

void test_speed_negative_and_position() {
  SpeedEstimator est(1000.0f, 2);
  est.reset(100);
  est.update(50, 0.01f);
  const float s = est.update(0, 0.01f);
  TEST_ASSERT_TRUE(s < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-9, -2.0 * kPiD * 0.1, est.positionRad(0));  // -100 counts = -0.1 rev
}

// ---------------------------------------------------------------------------
// CommandWatchdog and StallDetector
// ---------------------------------------------------------------------------

void test_watchdog_expired_before_first_command() {
  CommandWatchdog wd(300);
  TEST_ASSERT_TRUE(wd.expired(0));  // fail safe at boot
}

void test_watchdog_timing() {
  CommandWatchdog wd(300);
  wd.feed(1000);
  TEST_ASSERT_FALSE(wd.expired(1300));
  TEST_ASSERT_TRUE(wd.expired(1301));
}

void test_watchdog_survives_millis_rollover() {
  CommandWatchdog wd(300);
  wd.feed(0xFFFFFF00u);                      // 256 ms before millis() wraps
  TEST_ASSERT_FALSE(wd.expired(0x00000010u));  // 272 ms later, after the wrap
  TEST_ASSERT_TRUE(wd.expired(0x00000030u));   // 304 ms later
}

void test_stall_trips_after_time_and_latches() {
  StallDetector sd(StallParams{0.5f, 0.5f, 100});
  for (int i = 0; i < 9; ++i) TEST_ASSERT_FALSE(sd.update(0.8f, 0.0f, 10));
  TEST_ASSERT_TRUE(sd.update(0.8f, 0.0f, 10));  // 100 ms reached
  TEST_ASSERT_TRUE(sd.update(0.0f, 5.0f, 10));  // latched even once conditions clear
  sd.clear();
  TEST_ASSERT_FALSE(sd.stalled());
}

void test_stall_motion_resets_timer() {
  StallDetector sd(StallParams{0.5f, 0.5f, 100});
  for (int i = 0; i < 9; ++i) sd.update(-0.8f, 0.0f, 10);
  sd.update(-0.8f, -2.0f, 10);  // wheel moved: start over
  for (int i = 0; i < 9; ++i) TEST_ASSERT_FALSE(sd.update(-0.8f, 0.0f, 10));
}

void test_stall_ignores_low_duty() {
  StallDetector sd(StallParams{0.5f, 0.5f, 100});
  for (int i = 0; i < 100; ++i) TEST_ASSERT_FALSE(sd.update(0.3f, 0.0f, 10));
}

// ---------------------------------------------------------------------------
// Text protocol parser
// ---------------------------------------------------------------------------

void test_parse_velocity() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::Ok), static_cast<int>(parseCommand("V,1.5,-2,0,3.25", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Velocity), static_cast<int>(c.type));
  TEST_ASSERT_EQUAL_FLOAT(1.5f, c.values[0]);
  TEST_ASSERT_EQUAL_FLOAT(-2.0f, c.values[1]);
  TEST_ASSERT_EQUAL_FLOAT(3.25f, c.values[3]);
}

void test_parse_tolerates_spaces_and_lowercase() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::Ok), static_cast<int>(parseCommand("  d, 0.5 ,0, 0,-0.25  ", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Duty), static_cast<int>(c.type));
  TEST_ASSERT_EQUAL_FLOAT(-0.25f, c.values[3]);
}

void test_parse_rejects_bad_input() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::Empty), static_cast<int>(parseCommand("   ", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::UnknownCommand), static_cast<int>(parseCommand("X,1", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::UnknownCommand), static_cast<int>(parseCommand("VV,1,2,3,4", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::WrongArgCount), static_cast<int>(parseCommand("V,1,2,3", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::BadNumber), static_cast<int>(parseCommand("V,1,2x,3,4", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::BadNumber), static_cast<int>(parseCommand("V,1,,3,4", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::BadNumber), static_cast<int>(parseCommand("V,1,nan,3,4", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::OutOfRange), static_cast<int>(parseCommand("D,1.5,0,0,0", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::WrongArgCount), static_cast<int>(parseCommand("S,1", c)));
}

void test_parse_gains_and_limits() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::Ok), static_cast<int>(parseCommand("G,-1,0.02,0.2,0.05,0.04", c)));
  TEST_ASSERT_EQUAL(-1, c.wheel);
  TEST_ASSERT_EQUAL_FLOAT(0.04f, c.values[3]);
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::OutOfRange), static_cast<int>(parseCommand("G,4,0,0,0,0", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::OutOfRange), static_cast<int>(parseCommand("G,0,-1,0,0,0", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::OutOfRange), static_cast<int>(parseCommand("W,10000", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::OutOfRange), static_cast<int>(parseCommand("W,10", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::Ok), static_cast<int>(parseCommand("T,50", c)));
  TEST_ASSERT_EQUAL_UINT32(50u, c.integer);
}

void test_parse_rejects_overlong_line() {
  char line[200];
  std::memset(line, '1', sizeof(line) - 1);
  line[0] = 'V';
  line[sizeof(line) - 1] = '\0';
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::TooLong), static_cast<int>(parseCommand(line, c)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pid_zero_target_outputs_zero_and_clears_integral);
  RUN_TEST(test_pid_feedforward_only_when_error_is_zero);
  RUN_TEST(test_pid_supply_scale_affects_only_feedforward);
  RUN_TEST(test_pid_output_is_clamped);
  RUN_TEST(test_pid_integral_does_not_wind_up_while_saturated);
  RUN_TEST(test_pid_integral_removes_steady_error);
  RUN_TEST(test_pid_ignores_bad_dt);
  RUN_TEST(test_speed_first_sample_primes_without_spike);
  RUN_TEST(test_speed_constant_rate);
  RUN_TEST(test_speed_window_smooths_quantization);
  RUN_TEST(test_speed_negative_and_position);
  RUN_TEST(test_watchdog_expired_before_first_command);
  RUN_TEST(test_watchdog_timing);
  RUN_TEST(test_watchdog_survives_millis_rollover);
  RUN_TEST(test_stall_trips_after_time_and_latches);
  RUN_TEST(test_stall_motion_resets_timer);
  RUN_TEST(test_stall_ignores_low_duty);
  RUN_TEST(test_parse_velocity);
  RUN_TEST(test_parse_tolerates_spaces_and_lowercase);
  RUN_TEST(test_parse_rejects_bad_input);
  RUN_TEST(test_parse_gains_and_limits);
  RUN_TEST(test_parse_rejects_overlong_line);
  return UNITY_END();
}
