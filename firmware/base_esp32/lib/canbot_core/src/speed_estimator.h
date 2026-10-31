// speed_estimator.h — turns raw encoder counts into wheel angle (rad) and speed (rad/s).
//
// THE LOW-SPEED PROBLEM THIS SOLVES
//   Speed from an encoder is (counts this period) / (counts per rev) * 2π / dt. At 10%
//   speed, a JGB37-520 at ~1,320 counts/rev makes only 4–5 counts per 10 ms loop, so a
//   single count of jitter is a 20–25% speed error and the PID would chase noise.
//   Fix used here: sum the last N periods' counts and divide by the last N periods'
//   time (a moving window). With N = 4 the window is 40 ms: ~4x less quantization noise,
//   at the cost of ~20 ms of average lag. That lag is fine for a velocity loop whose
//   outer loops (odometry, Nav2) run at 50 Hz or slower.
//
//   Counts are 64-bit (ESP32Encoder extends the 16-bit hardware counter in software),
//   so position never wraps in any realistic run time.
#pragma once

#include <cstdint>

namespace canbot {

class SpeedEstimator {
 public:
  static constexpr int kMaxWindow = 16;

  SpeedEstimator() = default;
  SpeedEstimator(float counts_per_rev, int window) { configure(counts_per_rev, window); }

  void configure(float counts_per_rev, int window);

  // Start fresh from the current count (call at boot and after zeroing encoders).
  void reset(int64_t count);

  // Feed the latest count and the time since the previous update. Returns speed in rad/s.
  float update(int64_t count, float dt_s);

  float speed() const { return speed_; }
  // Wheel angle in radians, measured from the last reset/zero.
  double positionRad(int64_t count) const;

 private:
  float counts_per_rev_ = 1.0f;
  int window_ = 1;
  int64_t last_count_ = 0;
  int64_t origin_ = 0;
  bool primed_ = false;
  // Ring buffers: per-period count deltas and their durations.
  int32_t deltas_[kMaxWindow] = {};
  float dts_[kMaxWindow] = {};
  int head_ = 0;
  int filled_ = 0;
  float speed_ = 0.0f;
};

}  // namespace canbot
