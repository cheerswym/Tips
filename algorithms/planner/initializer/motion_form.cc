#include "onboard/planner/initializer/motion_form.h"

#include <limits>

#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {
namespace {
constexpr double kZeroAccEpsilon = 0.05;
constexpr double kPositionEpsilon = 0.1;
constexpr double kZeroSpeedEpsilon = 0.1;

template <bool kIsFastSample>
std::vector<MotionState> SampleWithChoice(const GeometryForm& geometry,
                                          double d_t, double duration,
                                          double init_v, double a,
                                          double stop_time, double stop_dist) {
  const int tentative_num_samples = CeilToInt(duration / d_t) + 1;
  double len = geometry.length();
  double t = 0.0;
  double v = init_v;
  double s = 0.0;
  double cur_a = a;
  const double lookforward = 0.9 * d_t;

  struct Sample {
    double s;
    double v;
    double t;
    double a;
  };
  std::vector<Sample> samples;
  samples.reserve(tentative_num_samples);

  while (t + lookforward < duration) {
    // TODO(xiangjun): provide a batch sampling method here. Too time consuming
    // to sample one by one for curvy geometry form.
    samples.push_back({.s = std::min(s, len), .v = v, .t = t, .a = cur_a});

    t += d_t;
    if (t < stop_time) {
      // Vehicle is moving.
      s += (v + 0.5 * a * d_t) * d_t;
      v += d_t * a;
    } else {
      // Vehicle stops.
      s = stop_dist;
      v = 0.0;
      cur_a = 0.0;
    }
  }
  samples.push_back({.s = std::min(s, len), .v = v, .t = duration, .a = cur_a});

  const int num_samples = samples.size();
  std::vector<MotionState> states(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    const auto& geometry_state = kIsFastSample
                                     ? geometry.FastState(samples[i].s)
                                     : geometry.State(samples[i].s);
    states[i] = MotionState{.xy = geometry_state.xy,
                            .h = geometry_state.h,
                            .k = geometry_state.k,
                            .ref_k = geometry_state.ref_k,
                            .t = samples[i].t,
                            .v = samples[i].v,
                            .a = samples[i].a,
                            .accumulated_s = geometry_state.accumulated_s,
                            .s = samples[i].s,
                            .l = geometry_state.l};
  }
  return states;
}

}  // namespace

ConstAccelMotion::ConstAccelMotion(double init_v, double init_a,
                                   const GeometryForm* geometry)
    : init_v_(init_v),
      a_(init_a),
      stop_time_(std::numeric_limits<double>::max()),
      stop_distance_(std::numeric_limits<double>::max()),
      geometry_(geometry) {
  // Avoid the case that init_v = 0.0 and a = 0.0, which will cause the root
  // finding error.
  if (std::fabs(a_) < kZeroAccEpsilon && init_v < kZeroSpeedEpsilon) {
    a_ = -kZeroAccEpsilon;
  }

  // Need to check the case that the motion brakes to stop before reaching the
  // end of geometry.
  if (a_ < 0.0) {
    const double inv_a = 1.0 / a_;
    stop_distance_ = std::fabs(0.5 * init_v_ * init_v_ * inv_a);
    if (stop_distance_ < geometry_->length() + kPositionEpsilon) {
      duration_ = kTrajectoryTimeStep * kTrajectorySteps;
      stop_time_ = std::fabs(init_v_ * inv_a);
      return;
    }
  }

  const auto roots = QuadraticRoot(0.5 * a_, init_v_, -geometry_->length());
  bool found_root = false;
  for (auto root : roots) {
    if (root > 0.0) {
      duration_ = root;
      found_root = true;
      break;
    }
  }
  QCHECK(found_root) << "init v=" << init_v << ", init_a=" << init_a
                     << ", len=" << geometry_->length();
}

ConstAccelMotion::ConstAccelMotion(std::pair<double, double> v_pair,
                                   const GeometryForm* geometry)
    : init_v_(v_pair.first),
      stop_time_(std::numeric_limits<double>::max()),
      stop_distance_(std::numeric_limits<double>::max()),
      geometry_(geometry) {
  QCHECK_GE(v_pair.second, 0.0);
  QCHECK_GE(v_pair.first, 0.0);
  QCHECK(!(v_pair.first == 0 && v_pair.second == 0.0))
      << v_pair.first << ", " << v_pair.second;
  a_ = (Sqr(v_pair.second) - Sqr(init_v_)) * 0.5 / geometry_->length();
  duration_ = geometry_->length() * 2.0 / (init_v_ + v_pair.second);
}

MotionState ConstAccelMotion::GetStartMotionState() const { return State(0.0); }

MotionState ConstAccelMotion::GetEndMotionState() const {
  return State(duration_);
}

std::vector<MotionState> ConstAccelMotion::FastSample(double d_t) const {
  return SampleWithChoice</*kIsFastSample=*/true>(
      *geometry_, d_t, duration_, init_v_, a_, stop_time_, stop_distance_);
}

std::vector<MotionState> ConstAccelMotion::Sample(double d_t) const {
  return SampleWithChoice</*kIsFastSample=*/false>(
      *geometry_, d_t, duration_, init_v_, a_, stop_time_, stop_distance_);
}

MotionState ConstAccelMotion::State(double t) const {
  double s = 0.0;
  // Vehicle is still moving.
  if (t < stop_time_) {
    s = (init_v_ + 0.5 * a_ * t) * t;
    const auto& geo_state = geometry()->State(s);
    return MotionState{.xy = geo_state.xy,
                       .h = geo_state.h,
                       .k = geo_state.k,
                       .ref_k = geo_state.ref_k,
                       .t = t,
                       .v = init_v_ + a_ * t,
                       .a = a_,
                       .accumulated_s = geo_state.accumulated_s,
                       .s = s,
                       .l = geo_state.l};
  }
  // Vehicle already stopped
  s = stop_distance_;
  const auto& geo_state = geometry()->State(s);
  return MotionState{.xy = geo_state.xy,
                     .h = geo_state.h,
                     .k = geo_state.k,
                     .ref_k = geo_state.ref_k,
                     .t = t,
                     .v = 0.0,
                     .a = 0.0,
                     .accumulated_s = geo_state.accumulated_s,
                     .s = s,
                     .l = geo_state.l};
}

std::vector<MotionState> StationaryMotion::Sample(double d_t) const {
  std::vector<MotionState> states;
  states.reserve(CeilToInt(duration() / d_t) + 1);
  GeometryState geo_state = geometry_->State(0.0);
  MotionState state{.xy = geo_state.xy,
                    .h = geo_state.h,
                    .k = geo_state.k,
                    .ref_k = geo_state.ref_k,
                    .t = 0.0,
                    .v = 0.0,
                    .a = 0.0,
                    .accumulated_s = geo_state.accumulated_s,
                    .s = 0.0,
                    .l = geo_state.l};
  double t = 0.0;
  const double lookforward = 0.9 * d_t;
  while (t + lookforward < duration()) {
    state.t = t;
    states.push_back(state);
    t += d_t;
  }
  state.t = duration();
  states.push_back(state);
  return states;
}

std::vector<MotionState> StationaryMotion::FastSample(double d_t) const {
  return Sample(d_t);
}

MotionState StationaryMotion::GetStartMotionState() const { return State(0.0); }

MotionState StationaryMotion::GetEndMotionState() const {
  return State(duration_);
}

MotionState StationaryMotion::State(double t) const {
  GeometryState geom_state = geometry_->State(0.0);
  return MotionState{.xy = geom_state.xy,
                     .h = geom_state.h,
                     .k = geom_state.k,
                     .ref_k = geom_state.ref_k,
                     .t = t,
                     .v = 0.0,
                     .a = 0.0,
                     .accumulated_s = geom_state.accumulated_s,
                     .s = 0.0,
                     .l = geom_state.l};
}

}  // namespace qcraft::planner
