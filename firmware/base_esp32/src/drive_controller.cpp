#include "drive_controller.h"

#include <esp_timer.h>

namespace {

// Higher than Arduino's loop() (priority 1), lower than the system's own critical tasks.
constexpr UBaseType_t kTaskPriority = 10;
constexpr uint32_t kTaskStackBytes = 4096;
// Core 1 is where Arduino runs loop(); core 0 runs USB/Wi-Fi/system work. The control task
// shares core 1 with loop() but preempts it, since it has the higher priority.
constexpr BaseType_t kTaskCore = 1;

// E-stop debounce: trip fast (2 samples = 20 ms), release slowly (10 samples = 100 ms).
constexpr uint8_t kEstopTripSamples = 2;
constexpr uint8_t kEstopReleaseSamples = 10;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

// ============================================================================
// Setup
// ============================================================================

bool DriveController::begin() {
  // Motors first: every output is driven to 0% before anything else can happen.
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    hardware_ok_ &= motors_[i].begin(cfg::kPwmPin[i], cfg::kDirPin[i], cfg::kMotorInvert[i], i);
  }

  // PCNT ("pulse counter") units count encoder edges in hardware: no interrupt per edge,
  // so CPU load doesn't grow with wheel speed. The S3 has exactly four units, one per wheel.
  // Full quadrature counts every rising AND falling edge on both channels: 4x the line count.
  // Weak internal pull-ups help with open-collector Hall outputs; most encoder boards also
  // have their own.
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    encoders_[i].attachFullQuad(cfg::kEncAPin[i], cfg::kEncBPin[i]);
    encoders_[i].setFilter(cfg::kEncoderGlitchFilter);  // ignore spikes shorter than ~12.8 µs (PWM noise)
    encoders_[i].clearCount();
    hardware_ok_ &= encoders_[i].isAttached();

    estimators_[i].configure(cfg::kCountsPerWheelRev, cfg::kSpeedWindow);
    estimators_[i].reset(0);
    pids_[i] = canbot::VelocityPid(cfg::kDefaultGains, cfg::kPidLimits);
    stalls_[i] = canbot::StallDetector(cfg::kStall);
    inbox_.gains[i] = cfg::kDefaultGains;
  }

  if (cfg::kEstopSensePin >= 0) {
    // Pull-DOWN on purpose: a broken sense wire reads LOW, which means "E-stop pressed".
    // A wiring fault should fail toward stopped, never toward moving.
    pinMode(cfg::kEstopSensePin, INPUT_PULLDOWN);
  }
  if (cfg::kBatterySensePin >= 0) {
    analogReadResolution(12);
    analogSetPinAttenuation(cfg::kBatterySensePin, ADC_11db);  // ~0–3.1 V input range
  }

  watchdog_.setTimeout(cfg::kWatchdogDefaultMs);
  last_step_us_ = esp_timer_get_time();

  const BaseType_t ok = xTaskCreatePinnedToCore(&DriveController::taskEntry, "drive_ctl", kTaskStackBytes,
                                                this, kTaskPriority, &task_, kTaskCore);
  return ok == pdPASS && hardware_ok_;
}

void DriveController::taskEntry(void* self) { static_cast<DriveController*>(self)->run(); }

// ============================================================================
// The 100 Hz loop
// ============================================================================

void DriveController::run() {
  const TickType_t period = pdMS_TO_TICKS(cfg::kControlPeriodMs);
  TickType_t last_wake = xTaskGetTickCount();
  last_step_us_ = esp_timer_get_time();

  for (;;) {
    // vTaskDelayUntil sleeps until an ABSOLUTE time (last_wake + period), then advances
    // last_wake by exactly one period. delay(10) would sleep 10 ms *after* the step
    // finished, so the true period would be 10 ms + however long the step took, and it
    // would drift whenever the step's length changed.
    vTaskDelayUntil(&last_wake, period);

    const uint64_t now_us = esp_timer_get_time();
    // Use the MEASURED dt, not the nominal 10 ms, in every rate calculation. If the task
    // ever wakes late, speed and the integral stay correct instead of being off by the error.
    const float dt_s = static_cast<float>(now_us - last_step_us_) * 1e-6f;
    last_step_us_ = now_us;
    if (dt_s > 1.5f * cfg::kControlPeriodMs * 1e-3f) ++overruns_;

    step(dt_s, millis());

    const uint32_t exec_us = static_cast<uint32_t>(esp_timer_get_time() - now_us);
    if (exec_us > loop_us_max_) loop_us_max_ = exec_us;
  }
}

void DriveController::step(float dt_s, uint32_t now_ms) {
  // ---- 1. Take a snapshot of the commands, and consume one-shot requests --------------
  Inbox in;
  portENTER_CRITICAL(&mux_);
  in = inbox_;
  inbox_.clear_faults = false;
  inbox_.zero_encoders = false;
  portEXIT_CRITICAL(&mux_);

  if (in.gains_seq != seen_gains_seq_) {
    for (int i = 0; i < cfg::kNumWheels; ++i) pids_[i].setGains(in.gains[i]);
    seen_gains_seq_ = in.gains_seq;
  }
  if (in.zero_encoders) {
    for (int i = 0; i < cfg::kNumWheels; ++i) {
      encoders_[i].clearCount();
      estimators_[i].reset(0);
    }
  }
  watchdog_.setTimeout(in.watchdog_ms);
  if (in.commanded) watchdog_.feed(in.last_command_ms);

  // ---- 2. Measure -------------------------------------------------------------------------
  int64_t counts[cfg::kNumWheels];
  float speed[cfg::kNumWheels];
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    const int64_t raw = encoders_[i].getCount();
    counts[i] = cfg::kEncoderInvert[i] ? -raw : raw;
    speed[i] = estimators_[i].update(counts[i], dt_s);
  }

  const bool estop_now = readEstop();
  const float battery_v = readBattery();
  const bool timed_out = watchdog_.expired(now_ms);

  // Is the host currently asking for zero on every wheel? Used to re-arm after a fault.
  bool all_zero = true;
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    const float cmd = in.mode == DriveMode::Velocity ? in.target_rad_s[i] : in.duty[i];
    if (cmd > cfg::kPidLimits.target_epsilon || cmd < -cfg::kPidLimits.target_epsilon) all_zero = false;
  }

  // E-stop LATCHES: releasing the button never moves the robot by itself. The wheels stay
  // off until the rail is back AND the host has commanded zero (or sent "clear faults").
  // Otherwise a teammate twisting the E-stop back out would let the robot resume whatever
  // it was last told, which might be "drive at the wall".
  if (estop_now) estop_ = true;
  else if (estop_ && (all_zero || in.clear_faults)) estop_ = false;

  float ff_scale = 1.0f;
  if (cfg::kBatterySensePin >= 0 && battery_v > 6.0f) {
    // A drained pack needs more duty for the same speed; scale feedforward to match.
    ff_scale = clampf(cfg::kFfReferenceVolts / battery_v, 0.8f, 1.3f);
  }

  // ---- 3. Stall check: judge the duty we applied LAST period against the speed it produced
  const uint32_t dt_ms = static_cast<uint32_t>(dt_s * 1000.0f + 0.5f);
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    if (in.clear_faults) stalls_[i].clear();
    stalls_[i].update(applied_duty_[i], speed[i], dt_ms);
  }

  // ---- 4. Decide each wheel's output ---------------------------------------------------
  float targets[cfg::kNumWheels] = {};
  uint16_t faults = 0;
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    float duty = 0.0f;
    const bool velocity_mode = (in.mode == DriveMode::Velocity);
    const float target = velocity_mode ? clampf(in.target_rad_s[i], -cfg::kMaxWheelSpeedRadS,
                                                cfg::kMaxWheelSpeedRadS)
                                       : 0.0f;
    const float requested = velocity_mode ? target : in.duty[i];
    const bool wheel_zero = requested < cfg::kPidLimits.target_epsilon &&
                            requested > -cfg::kPidLimits.target_epsilon;

    // A stalled wheel re-arms once the host stops asking it to move.
    if (stalls_[i].stalled() && wheel_zero) stalls_[i].clear();

    if (estop_ || timed_out || stalls_[i].stalled() || !hardware_ok_) {
      pids_[i].reset();  // so stale integral can't kick the wheel when it's re-enabled
    } else if (velocity_mode) {
      duty = pids_[i].update(target, speed[i], dt_s, ff_scale);
    } else {
      pids_[i].reset();
      duty = in.duty[i];
    }

    motors_[i].setDuty(duty);
    applied_duty_[i] = duty;
    targets[i] = target;
    if (stalls_[i].stalled()) faults |= static_cast<uint16_t>(1u << i);
  }
  if (estop_) faults |= fault::kEstop;
  if (timed_out) faults |= fault::kWatchdog;
  if (overruns_ > 0) faults |= fault::kOverrun;
  if (!hardware_ok_) faults |= fault::kHardware;

  // ---- 5. Publish the measurements --------------------------------------------------------
  // Build the whole snapshot first, then hold the lock only for the copy.
  DriveState out;
  out.stamp_ms = now_ms;
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    out.counts[i] = counts[i];
    out.position_rad[i] = estimators_[i].positionRad(counts[i]);
    out.velocity_rad_s[i] = speed[i];
    out.target_rad_s[i] = targets[i];
    out.duty[i] = applied_duty_[i];
  }
  out.faults = faults;
  out.battery_v = battery_v;
  out.loop_us_max = loop_us_max_;
  out.overruns = overruns_;
  out.mode = in.mode;

  portENTER_CRITICAL(&mux_);
  state_ = out;
  portEXIT_CRITICAL(&mux_);
}

bool DriveController::readEstop() {
  if (cfg::kEstopSensePin < 0) return false;
  const bool rail_live = digitalRead(cfg::kEstopSensePin) == HIGH;
  if (!rail_live) {
    estop_high_count_ = 0;
    if (estop_low_count_ < kEstopTripSamples) ++estop_low_count_;
  } else {
    estop_low_count_ = 0;
    if (estop_high_count_ < kEstopReleaseSamples) ++estop_high_count_;
  }
  if (estop_low_count_ >= kEstopTripSamples) estop_sensed_ = true;
  else if (estop_high_count_ >= kEstopReleaseSamples) estop_sensed_ = false;
  // In between the two thresholds the previous verdict stands (hysteresis).
  return estop_sensed_;
}

float DriveController::readBattery() {
  if (cfg::kBatterySensePin < 0) return 0.0f;
  // analogReadMilliVolts applies the chip's factory ADC calibration, which a raw
  // analogRead() count does not.
  const float volts = analogReadMilliVolts(cfg::kBatterySensePin) * 1e-3f * cfg::kBatteryDividerRatio;
  // Low-pass filter (time constant ≈ 0.2 s at 100 Hz) so motor current spikes don't
  // whip the feedforward scale around.
  battery_v_filtered_ = (battery_v_filtered_ <= 0.0f) ? volts : battery_v_filtered_ + 0.05f * (volts - battery_v_filtered_);
  return battery_v_filtered_;
}

// ============================================================================
// Commands from the transport. Each one does the minimum work inside the lock.
// ============================================================================

void DriveController::commandVelocity(const float rad_s[cfg::kNumWheels]) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&mux_);
  inbox_.mode = DriveMode::Velocity;
  for (int i = 0; i < cfg::kNumWheels; ++i) inbox_.target_rad_s[i] = rad_s[i];
  inbox_.last_command_ms = now;
  inbox_.commanded = true;
  portEXIT_CRITICAL(&mux_);
}

void DriveController::commandDuty(const float duty[cfg::kNumWheels]) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&mux_);
  inbox_.mode = DriveMode::OpenLoop;
  for (int i = 0; i < cfg::kNumWheels; ++i) inbox_.duty[i] = duty[i];
  inbox_.last_command_ms = now;
  inbox_.commanded = true;
  portEXIT_CRITICAL(&mux_);
}

void DriveController::stop() {
  const float zero[cfg::kNumWheels] = {0.0f, 0.0f, 0.0f, 0.0f};
  commandVelocity(zero);
}

void DriveController::clearFaults() {
  portENTER_CRITICAL(&mux_);
  inbox_.clear_faults = true;
  portEXIT_CRITICAL(&mux_);
}

void DriveController::zeroEncoders() {
  portENTER_CRITICAL(&mux_);
  inbox_.zero_encoders = true;
  portEXIT_CRITICAL(&mux_);
}

void DriveController::setGains(int wheel, const canbot::PidGains& g) {
  portENTER_CRITICAL(&mux_);
  for (int i = 0; i < cfg::kNumWheels; ++i) {
    if (wheel < 0 || wheel == i) inbox_.gains[i] = g;
  }
  ++inbox_.gains_seq;
  portEXIT_CRITICAL(&mux_);
}

canbot::PidGains DriveController::gains(int wheel) const {
  if (wheel < 0 || wheel >= cfg::kNumWheels) wheel = 0;
  portENTER_CRITICAL(&mux_);
  const canbot::PidGains g = inbox_.gains[wheel];
  portEXIT_CRITICAL(&mux_);
  return g;
}

void DriveController::setWatchdogTimeout(uint32_t ms) {
  portENTER_CRITICAL(&mux_);
  inbox_.watchdog_ms = ms;
  portEXIT_CRITICAL(&mux_);
}

uint32_t DriveController::watchdogTimeout() const {
  portENTER_CRITICAL(&mux_);
  const uint32_t ms = inbox_.watchdog_ms;
  portEXIT_CRITICAL(&mux_);
  return ms;
}

void DriveController::getState(DriveState& out) const {
  portENTER_CRITICAL(&mux_);
  out = state_;
  portEXIT_CRITICAL(&mux_);
}
