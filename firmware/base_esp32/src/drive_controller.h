// drive_controller.h — the real-time heart of the base: a 100 Hz FreeRTOS task that
// reads encoders, runs one PID per wheel, and drives the MDD10As.
//
// TWO TASKS, ONE MAILBOX
//   The control task (core 1, high priority) runs every 10 ms no matter what. The
//   transport (text protocol or micro-ROS) runs in Arduino's loop() at low priority and
//   may block for tens of milliseconds waiting on USB. They never call into each other's
//   state directly. Instead they share two small structs:
//     inbox_  — commands flowing DOWN  (written by the transport, read by the control task)
//     state_  — measurements flowing UP (written by the control task, read by the transport)
//   Each side copies a whole struct in or out under a spinlock, then works on its private
//   copy with the lock released. A single 32-bit write is atomic on this chip, but a set of
//   four wheel targets is not: without the lock, the control task could read two old
//   targets and two new ones and the robot would yaw for a cycle.
//
// This is the same separation as the whole robot: the fast loop owns timing, everything
// else just sets its targets.
#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>

#include "config.h"
#include "motor_driver.h"
#include "safety.h"
#include "speed_estimator.h"
#include "velocity_pid.h"

enum class DriveMode : uint8_t {
  Velocity,  // closed loop: targets in rad/s, PID computes duty
  OpenLoop,  // bench testing: duty commanded directly, no PID
};

// Fault bits, reported to the host. Stall bits are latched; the others reflect current state.
namespace fault {
constexpr uint16_t kStallFL = 1u << 0;
constexpr uint16_t kStallFR = 1u << 1;
constexpr uint16_t kStallRL = 1u << 2;
constexpr uint16_t kStallRR = 1u << 3;
constexpr uint16_t kEstop = 1u << 4;
constexpr uint16_t kWatchdog = 1u << 5;   // no recent command (normal when idle)
constexpr uint16_t kOverrun = 1u << 6;    // the control loop missed its 10 ms deadline at least once
constexpr uint16_t kHardware = 1u << 7;   // a PWM channel or encoder failed to initialize
}  // namespace fault

struct DriveState {
  uint32_t stamp_ms = 0;
  int64_t counts[cfg::kNumWheels] = {};
  double position_rad[cfg::kNumWheels] = {};
  float velocity_rad_s[cfg::kNumWheels] = {};
  float target_rad_s[cfg::kNumWheels] = {};
  float duty[cfg::kNumWheels] = {};
  uint16_t faults = 0;
  float battery_v = 0.0f;
  uint32_t loop_us_max = 0;  // worst control-step execution time since boot
  uint32_t overruns = 0;     // missed deadlines since boot
  DriveMode mode = DriveMode::Velocity;
};

class DriveController {
 public:
  // Configure hardware with every motor at 0% and start the control task.
  bool begin();

  // --- Called from the transport (any task) -------------------------------------------
  void commandVelocity(const float rad_s[cfg::kNumWheels]);
  void commandDuty(const float duty[cfg::kNumWheels]);
  void stop();
  void clearFaults();
  void zeroEncoders();
  void setGains(int wheel, const canbot::PidGains& g);  // wheel = -1 → all
  canbot::PidGains gains(int wheel) const;
  void setWatchdogTimeout(uint32_t ms);
  uint32_t watchdogTimeout() const;
  void getState(DriveState& out) const;

 private:
  struct Inbox {
    DriveMode mode = DriveMode::Velocity;
    float target_rad_s[cfg::kNumWheels] = {};
    float duty[cfg::kNumWheels] = {};
    uint32_t last_command_ms = 0;
    bool commanded = false;  // any command since boot?
    canbot::PidGains gains[cfg::kNumWheels];
    uint32_t gains_seq = 0;  // bumped on every gain change, so the task knows to reload
    uint32_t watchdog_ms = cfg::kWatchdogDefaultMs;
    bool clear_faults = false;
    bool zero_encoders = false;
  };

  static void taskEntry(void* self);
  void run();
  void step(float dt_s, uint32_t now_ms);
  bool readEstop();
  float readBattery();

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Inbox inbox_;        // guarded by mux_
  DriveState state_;   // guarded by mux_

  // Everything below is touched ONLY by the control task after begin(), so it needs no lock.
  MotorDriver motors_[cfg::kNumWheels];
  ESP32Encoder encoders_[cfg::kNumWheels];
  canbot::SpeedEstimator estimators_[cfg::kNumWheels];
  canbot::VelocityPid pids_[cfg::kNumWheels];
  canbot::StallDetector stalls_[cfg::kNumWheels];
  canbot::CommandWatchdog watchdog_;
  float applied_duty_[cfg::kNumWheels] = {};
  uint32_t seen_gains_seq_ = 0;
  uint64_t last_step_us_ = 0;
  uint8_t estop_low_count_ = 0;
  uint8_t estop_high_count_ = 0;
  bool estop_sensed_ = false;  // debounced state of the sense pin right now
  bool estop_ = false;         // latched: stays true after release until re-armed
  float battery_v_filtered_ = 0.0f;
  uint32_t loop_us_max_ = 0;
  uint32_t overruns_ = 0;
  bool hardware_ok_ = true;
  TaskHandle_t task_ = nullptr;
};
