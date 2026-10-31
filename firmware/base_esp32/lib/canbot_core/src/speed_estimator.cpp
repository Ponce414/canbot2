#include "speed_estimator.h"

namespace canbot {

namespace {
constexpr double kTwoPi = 6.283185307179586;
}

void SpeedEstimator::configure(float counts_per_rev, int window) {
  counts_per_rev_ = counts_per_rev > 0.0f ? counts_per_rev : 1.0f;
  window_ = window < 1 ? 1 : (window > kMaxWindow ? kMaxWindow : window);
  primed_ = false;
}

void SpeedEstimator::reset(int64_t count) {
  last_count_ = count;
  origin_ = count;
  primed_ = true;
  head_ = 0;
  filled_ = 0;
  speed_ = 0.0f;
}

float SpeedEstimator::update(int64_t count, float dt_s) {
  // The first sample after power-up only establishes a reference; there's no
  // previous count to difference against yet.
  if (!primed_) {
    reset(count);
    return 0.0f;
  }
  const int64_t delta = count - last_count_;
  last_count_ = count;
  if (!(dt_s > 0.0f)) return speed_;  // bad timestamp: keep the last estimate

  deltas_[head_] = static_cast<int32_t>(delta);
  dts_[head_] = dt_s;
  head_ = (head_ + 1) % window_;
  if (filled_ < window_) ++filled_;

  // Sum over however many samples we have (fewer than N right after a reset).
  int64_t sum_counts = 0;
  float sum_dt = 0.0f;
  for (int i = 0; i < filled_; ++i) {
    sum_counts += deltas_[i];
    sum_dt += dts_[i];
  }
  speed_ = static_cast<float>(kTwoPi * static_cast<double>(sum_counts) /
                              (static_cast<double>(counts_per_rev_) * sum_dt));
  return speed_;
}

double SpeedEstimator::positionRad(int64_t count) const {
  return kTwoPi * static_cast<double>(count - origin_) / static_cast<double>(counts_per_rev_);
}

}  // namespace canbot
