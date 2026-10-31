// safety.h — the two independent ways a wheel gets forced off: a silent host, and a stall.
//
// Both are tiny state machines kept separate from the control loop so each can be unit
// tested on its own, and so the rule "every layer can stop the layer below it without
// help from the layer above" is visible in one place.
#pragma once

#include <cstdint>

namespace canbot {

// ---------------------------------------------------------------------------
// CommandWatchdog: "no command for timeout_ms → stop."
// Catches a crashed Pi, a killed micro-ROS agent, or an unplugged USB cable. At 50 Hz
// commands, a 300 ms timeout tolerates ~15 lost messages before stopping, and the robot
// coasts (speed x 0.3 s) before it halts — shorten it if that distance is too far.
// ---------------------------------------------------------------------------
class CommandWatchdog {
 public:
  explicit CommandWatchdog(uint32_t timeout_ms = 300) : timeout_ms_(timeout_ms) {}

  void feed(uint32_t now_ms) {
    last_feed_ms_ = now_ms;
    ever_fed_ = true;
  }

  // Unsigned subtraction handles the millis() rollover every ~49 days correctly:
  // (now - last) is still the elapsed time even after now wraps past zero.
  bool expired(uint32_t now_ms) const {
    return !ever_fed_ || (now_ms - last_feed_ms_) > timeout_ms_;
  }

  void setTimeout(uint32_t ms) { timeout_ms_ = ms; }
  uint32_t timeout() const { return timeout_ms_; }

 private:
  uint32_t timeout_ms_;
  uint32_t last_feed_ms_ = 0;
  bool ever_fed_ = false;  // never commanded since boot = treat as expired (fail safe)
};

// ---------------------------------------------------------------------------
// StallDetector: "high duty but the wheel isn't turning, for too long → cut it."
// Protects the gearbox and the driver. Stall + reversal is exactly what stripped the old
// plastic TT gearboxes. Once tripped it LATCHES: the wheel stays off until someone
// clears it, so a jammed wheel can't keep retrying on its own.
// ---------------------------------------------------------------------------
struct StallParams {
  float duty_threshold = 0.5f;   // |duty| at or above this counts as "pushing hard"
  float speed_threshold = 0.5f;  // |speed| (rad/s) below this counts as "not moving"
  uint32_t time_ms = 1000;       // must persist this long before tripping
};

class StallDetector {
 public:
  explicit StallDetector(const StallParams& p = StallParams{}) : params_(p) {}

  // duty = what was applied during the period that just ended; speed = measured over it.
  // Returns true while latched.
  bool update(float duty, float speed, uint32_t dt_ms) {
    if (latched_) return true;
    const bool pushing = (duty >= params_.duty_threshold) || (duty <= -params_.duty_threshold);
    const bool stopped = (speed < params_.speed_threshold) && (speed > -params_.speed_threshold);
    if (pushing && stopped) {
      accumulated_ms_ += dt_ms;
      if (accumulated_ms_ >= params_.time_ms) latched_ = true;
    } else {
      accumulated_ms_ = 0;  // any moment of motion restarts the clock
    }
    return latched_;
  }

  bool stalled() const { return latched_; }
  void clear() {
    latched_ = false;
    accumulated_ms_ = 0;
  }
  void setParams(const StallParams& p) { params_ = p; }

 private:
  StallParams params_;
  uint32_t accumulated_ms_ = 0;
  bool latched_ = false;
};

}  // namespace canbot
