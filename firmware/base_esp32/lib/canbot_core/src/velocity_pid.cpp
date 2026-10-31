#include "velocity_pid.h"

#include <cmath>

namespace canbot {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

float VelocityPid::update(float target, float measured, float dt_s, float supply_scale) {
  // A zero target means "stop": output exactly 0 and drop the integral. Holding a
  // wheel at zero speed with a PI loop makes it buzz back and forth across the
  // encoder's resolution, wasting current for nothing, so we let it coast instead.
  if (std::fabs(target) < limits_.target_epsilon) {
    reset();
    return 0.0f;
  }
  // Guard against a nonsense time step (first call, or a stalled task) so one bad
  // dt can't dump a huge chunk into the integral.
  if (!(dt_s > 0.0f) || dt_s > 0.1f) dt_s = 0.0f;

  const float error = target - measured;
  const float feedforward =
      supply_scale * (gains_.kff * target + std::copysign(gains_.kstatic, target));
  const float proportional = gains_.kp * error;

  // Anti-windup by conditional integration ("clamping"): compute what the output would
  // be with the integral updated; if that output is saturated AND the error is pushing
  // further into saturation, keep the old integral. Otherwise the integral keeps growing
  // while the motor is already at full duty, and the wheel overshoots badly once it
  // finally catches up.
  float candidate = clampf(integral_ + gains_.ki * error * dt_s, -limits_.i_max, limits_.i_max);
  const float unclamped = feedforward + proportional + candidate;
  const bool pushing_past_top = unclamped > limits_.out_max && error > 0.0f;
  const bool pushing_past_bottom = unclamped < -limits_.out_max && error < 0.0f;
  if (!(pushing_past_top || pushing_past_bottom)) integral_ = candidate;

  return clampf(feedforward + proportional + integral_, -limits_.out_max, limits_.out_max);
}

}  // namespace canbot
