// velocity_pid.h — per-wheel velocity controller: feedforward + PI, with anti-windup.
//
// WHY THIS FILE HAS NO Arduino INCLUDES
//   Everything in lib/canbot_core is plain C++ with no hardware calls. That lets the
//   same source compile for the ESP32 *and* for your laptop, where `pio test -e native`
//   runs the unit tests in test/. Logic bugs get caught in milliseconds on the laptop
//   instead of by watching a wheel misbehave on the bench.
//
// THE CONTROL LAW (units: target/measured in rad/s, output in normalized duty -1..+1)
//
//   u = s * (kff * w* + kstatic * sign(w*))   <- feedforward: what duty *should* produce w*
//     + kp * e                                <- proportional: react to the current error
//     + I                                     <- integral: remove the error that's left
//   where e = w* - w, s = supply scale (1.0 unless battery compensation is on),
//   and I accumulates ki * e * dt.
//
//   Feedforward does most of the work: a DC motor's speed is roughly proportional to its
//   average voltage, so duty ≈ kff * speed. The PI terms only correct what feedforward
//   gets wrong (friction, load, a sagging battery). That's why the PI gains stay small
//   and the loop stays calm.
//
//   No D term on purpose: the derivative of a quantized, noisy encoder speed is mostly
//   noise, and velocity loops rarely need it.
#pragma once

namespace canbot {

struct PidGains {
  float kp = 0.0f;       // duty per (rad/s) of error
  float ki = 0.0f;       // duty per (rad/s * s) of accumulated error
  float kff = 0.0f;      // duty per (rad/s) of target — fit this with tools/fit_feedforward.py
  float kstatic = 0.0f;  // duty needed just to break static friction, applied in the target's direction
};

struct PidLimits {
  float out_max = 1.0f;          // output clamp, normalized duty
  float i_max = 0.3f;            // integral clamp, in duty units (so it's comparable to the output)
  float target_epsilon = 0.05f;  // |target| below this (rad/s) is treated as "stop"
};

class VelocityPid {
 public:
  VelocityPid() = default;
  VelocityPid(const PidGains& gains, const PidLimits& limits) : gains_(gains), limits_(limits) {}

  // One control step. dt_s is the measured time since the last step (seconds).
  // supply_scale multiplies only the feedforward term (battery-voltage compensation).
  // Returns the duty to apply, clamped to ±out_max.
  float update(float target, float measured, float dt_s, float supply_scale = 1.0f);

  // Forget the integral. Called whenever the wheel is forced off (stop, watchdog,
  // E-stop, stall) so stale integral can't make the wheel lurch when control resumes.
  void reset() { integral_ = 0.0f; }

  void setGains(const PidGains& g) { gains_ = g; }
  const PidGains& gains() const { return gains_; }
  void setLimits(const PidLimits& l) { limits_ = l; }
  const PidLimits& limits() const { return limits_; }
  float integral() const { return integral_; }

 private:
  PidGains gains_{};
  PidLimits limits_{};
  // Stored in *output* units (sum of ki*e*dt), not as a raw error sum. Consequence:
  // changing ki at runtime doesn't make the output jump, because the accumulated
  // value isn't re-multiplied by the new gain ("bumpless" gain changes).
  float integral_ = 0.0f;
};

}  // namespace canbot
