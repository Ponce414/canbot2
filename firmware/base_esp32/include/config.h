// config.h — every number that describes THIS robot lives here, and nowhere else.
//
// Values marked MEASURE are starting guesses. The bring-up guide (README.md) says how to
// measure each one. Wheel order is always FL, FR, RL, RR, matching the mecanum matrix in
// the system breakdown doc.
#pragma once

#include <stdint.h>

#include "safety.h"
#include "velocity_pid.h"

namespace cfg {

constexpr int kNumWheels = 4;
constexpr const char* kWheelLabel[kNumWheels] = {"FL", "FR", "RL", "RR"};

// ---------------------------------------------------------------------------
// Pins (ESP32-S3-DevKitC-1). Avoided on purpose: 0, 3, 45, 46 (boot strapping),
// 19, 20 (native USB), 26–32 (flash), 33–37 (octal PSRAM on N8R8 modules),
// 38 and 48 (the on-board RGB LED, depending on board revision).
// The S3's GPIO is 3.3 V ONLY: power the Hall encoders from the 3.3 V pin, or
// level-shift their outputs. Put a 10 kΩ pull-down on each PWM pin so the MDD10A
// sees 0% duty while the chip boots and its pins float.
// ---------------------------------------------------------------------------
constexpr uint8_t kPwmPin[kNumWheels] = {4, 5, 6, 7};
constexpr uint8_t kDirPin[kNumWheels] = {15, 16, 17, 18};
constexpr uint8_t kEncAPin[kNumWheels] = {8, 10, 12, 14};
constexpr uint8_t kEncBPin[kNumWheels] = {9, 11, 13, 21};

// E-stop sense: a divider from the SWITCHED motor rail to this pin (e.g. 33 kΩ over
// 10 kΩ puts 12.6 V at ~2.9 V). HIGH = rail live, LOW = E-stop pressed. -1 = not wired.
constexpr int8_t kEstopSensePin = -1;  // set to 40 once wired
// Pack A voltage through a divider into an ADC1 pin (ADC2 is unusable with Wi-Fi on).
// -1 = not wired, which also disables battery compensation of the feedforward.
constexpr int8_t kBatterySensePin = -1;  // set to 1 once wired
constexpr float kBatteryDividerRatio = (33.0f + 10.0f) / 10.0f;  // (Rtop + Rbottom) / Rbottom

// ---------------------------------------------------------------------------
// Direction conventions. A POSITIVE wheel speed means "this wheel pushes the robot
// forward" (REP 103: +x forward), which is what the mecanum kinematics assume.
// Motors on the right side are mounted mirrored, so they usually need inverting.
// MEASURE: bring-up step 3 checks each wheel and you flip these until all agree.
// ---------------------------------------------------------------------------
constexpr bool kMotorInvert[kNumWheels] = {false, true, false, true};
constexpr bool kEncoderInvert[kNumWheels] = {false, true, false, true};

// ---------------------------------------------------------------------------
// Drivetrain. JGB37-520, 1:30 listed ratio. If the encoder has 11 lines per motor rev:
//   counts per wheel rev = 11 lines x 4 (quadrature edges) x 30 = 1,320.
// MEASURE: turn a wheel exactly 10 turns by hand, read the count, divide by 10.
// ---------------------------------------------------------------------------
constexpr float kCountsPerWheelRev = 1320.0f;
// 200 RPM no-load at 12 V = 20.9 rad/s. Targets are clamped a bit below that so the PID
// always has headroom to correct; a loop that's already at 100% duty can't correct anything.
constexpr float kMaxWheelSpeedRadS = 18.0f;

// ---------------------------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------------------------
constexpr uint32_t kControlPeriodMs = 10;    // 100 Hz
constexpr int kSpeedWindow = 4;               // speed averaged over 4 periods = 40 ms
constexpr uint32_t kWatchdogDefaultMs = 300;  // no command for this long → stop
constexpr uint16_t kEncoderGlitchFilter = 1023;  // APB cycles (12.5 ns each) ≈ 12.8 µs

// PWM: 20 kHz is above hearing (no motor whine) and is the MDD10A's maximum. At 20 kHz
// the LEDC timer has 80 MHz / 20 kHz = 4,000 ticks per period, so 11 bits is the ceiling;
// 10 bits (0–1023) leaves margin and is plenty for a velocity loop.
constexpr uint32_t kPwmFreqHz = 20000;
constexpr uint8_t kPwmBits = 10;

// ---------------------------------------------------------------------------
// Default gains, conservative on purpose. MEASURE kff and kstatic with
// tools/fit_feedforward.py, then tune kp/ki on the bench (README step 5).
// kff starting point: duty 1.0 ≈ 20 rad/s no-load  →  ~0.05 duty per rad/s.
// ---------------------------------------------------------------------------
constexpr canbot::PidGains kDefaultGains{
    /*kp=*/0.02f, /*ki=*/0.20f, /*kff=*/0.05f, /*kstatic=*/0.04f};
constexpr canbot::PidLimits kPidLimits{
    /*out_max=*/1.0f, /*i_max=*/0.3f, /*target_epsilon=*/0.05f};

// Battery compensation: the voltage at which kff was measured. Feedforward is scaled by
// kFfReferenceVolts / measured volts, clamped to [0.8, 1.3], only when battery sense is wired.
constexpr float kFfReferenceVolts = 12.0f;

constexpr canbot::StallParams kStall{
    /*duty_threshold=*/0.5f, /*speed_threshold=*/0.5f, /*time_ms=*/1000};

// ---------------------------------------------------------------------------
// micro-ROS interface (used by the `microros` build only). The joint names must match
// the wheel joints in the URDF's <ros2_control> block.
// ---------------------------------------------------------------------------
constexpr const char* kNodeName = "canbot_base";
constexpr const char* kWheelCommandTopic = "/canbot/wheel_commands";
constexpr const char* kWheelStateTopic = "/canbot/wheel_states";
constexpr const char* kFaultTopic = "/canbot/base_faults";
constexpr const char* kWheelJointName[kNumWheels] = {
    "wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};
constexpr uint32_t kStatePublishPeriodMs = 20;  // 50 Hz

}  // namespace cfg
