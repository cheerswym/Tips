#include "onboard/prediction/feature_extractor/object_history_sampler.h"

#include <algorithm>
#include <utility>

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {

namespace {
constexpr double kMinDynamicForPolyFit = 0.5;
constexpr int kPolyFitDegree = 3;
constexpr double kDecayFactor = 0.5;
constexpr double kConsistencyFactor = 0.5;
constexpr double kConsistencySpeedTolerance = 4.0;  // m/s.
}  // namespace

std::vector<ObjectProto> ResampleObjectHistorySpan(
    const ObjectHistorySpan &obj_history, double current_ts, double time_step,
    int max_steps) {
  const double last_t = obj_history.timestamp();
  const int lerp_steps =
      std::min(max_steps, static_cast<int>(obj_history.size()));
  const double start_ts = current_ts - time_step * (lerp_steps - 1);
  const auto visit_hist_or = obj_history.GetHistoryFrom(start_ts);
  QCHECK_OK(visit_hist_or.status());
  const auto &visit_hist = visit_hist_or.value();
  std::vector<ObjectProto> vec_objects;
  std::vector<double> vec_ts;
  std::vector<Vec2d> vec_xt;
  std::vector<Vec2d> vec_yt;
  std::vector<double> weights;
  vec_objects.reserve(lerp_steps);
  vec_ts.reserve(lerp_steps);
  vec_xt.reserve(lerp_steps);
  vec_yt.reserve(lerp_steps);
  for (int i = 0; i < visit_hist.size(); ++i) {
    const auto &cur_node = visit_hist[i];
    const auto &cur_obj = cur_node.val.object_proto();
    // NOTE:(zheng): If the object is partially observed, and the pos source
    // type has changed, the pos may be not consistent, we can clear the
    // history.
    if (i > 0) {
      const auto &prev_obj = visit_hist[i - 1].val.object_proto();
      if (prev_obj.has_observation_state() && prev_obj.has_pos_source_type() &&
          cur_obj.has_observation_state() && cur_obj.has_pos_source_type()) {
        if ((cur_obj.observation_state() ==
                 ObservationState::OS_PARTIALLY_OBSERVED ||
             prev_obj.observation_state() ==
                 ObservationState::OS_PARTIALLY_OBSERVED) &&
            cur_obj.pos_source_type() != prev_obj.pos_source_type()) {
          vec_ts.clear();
          vec_xt.clear();
          vec_yt.clear();
          vec_objects.clear();
          weights.clear();
        }
      }
    }
    vec_ts.push_back(cur_node.time);
    const double rel_ts = cur_node.time - start_ts;
    const double final_ts_diff = last_t - cur_node.time;
    vec_xt.push_back(Vec2d(rel_ts, cur_node.val.pos().x()));
    vec_yt.push_back(Vec2d(rel_ts, cur_node.val.pos().y()));
    vec_objects.push_back(cur_node.val.object_proto());
    weights.push_back(std::pow(kDecayFactor, final_ts_diff));
  }

  if (vec_objects.size() == 1) {
    ObjectProto object = obj_history.back().val.object_proto();
    const auto align_status = AlignPerceptionObjectTime(current_ts, &object);
    return {object};
  }
  // 1. Count dynamic state num, only smooth the result if most states are
  // dynamic.
  // 2. If object is static, then force the static states to be the same (avoid
  // position shift).
  int count_dynamic = 0;
  for (int i = vec_objects.size() - 2; i >= 0; --i) {
    auto &cur_obj = vec_objects[i];
    if (cur_obj.moving_state() == ObjectProto::MS_MOVING) {
      count_dynamic++;
    }
    const auto &next_obj = vec_objects[i + 1];
    if (cur_obj.moving_state() == ObjectProto::MS_STATIC &&
        next_obj.moving_state() == ObjectProto::MS_STATIC) {
      double cur_ts = cur_obj.timestamp();
      cur_obj = next_obj;
      cur_obj.set_timestamp(cur_ts);
    }
    vec_xt[i] = Vec2d(vec_xt[i].x(), cur_obj.pos().x());
    vec_yt[i] = Vec2d(vec_xt[i].x(), cur_obj.pos().y());
  }

  class ObjectProtoLerper {
   public:
    ObjectProto operator()(const ObjectProto &a, const ObjectProto &b,
                           double alpha) const {
      return LerpObjectProto(a, b, alpha);
    }
  };
  PolynomialFitter x_fitter, y_fitter;
  // Ensure past traj stability
  bool use_poly_fit = (vec_xt.size() > kPolyFitDegree) &&
                      (obj_history.id() != kAvObjectId) &&
                      (count_dynamic / static_cast<double>(vec_objects.size()) >
                       kMinDynamicForPolyFit);
  if (use_poly_fit) {
    x_fitter.SetDegree(kPolyFitDegree);
    y_fitter.SetDegree(kPolyFitDegree);
    x_fitter.LoadData(vec_xt);
    y_fitter.LoadData(vec_yt);
    x_fitter.FitData();
    y_fitter.FitData();
  }

  PiecewiseLinearFunction<ObjectProto, double, ObjectProtoLerper>
      object_proto_plf(vec_ts, vec_objects);
  std::vector<ObjectProto> raw_resampled_objects;
  raw_resampled_objects.reserve(lerp_steps);
  for (int i = 0; i < lerp_steps; ++i) {
    const double ts = start_ts + i * time_step;
    auto resampled_obj = object_proto_plf.EvaluateWithExtrapolation(ts);
    if (use_poly_fit) {
      Vec2dProto pos;
      pos.set_x(x_fitter.Evaluate(ts - start_ts));
      pos.set_y(y_fitter.Evaluate(ts - start_ts));
      *resampled_obj.mutable_pos() = pos;
    }
    raw_resampled_objects.push_back(std::move(resampled_obj));
  }
  std::vector<ObjectProto> resampled_objects;
  resampled_objects.reserve(raw_resampled_objects.size());
  for (int i = 0; i < raw_resampled_objects.size(); ++i) {
    if (i + 1 < raw_resampled_objects.size()) {
      const auto &next_obj = raw_resampled_objects[i + 1];
      double t_diff =
          next_obj.timestamp() - raw_resampled_objects[i].timestamp();
      if (!CheckHistoryConsistency(raw_resampled_objects[i], next_obj,
                                   t_diff)) {
        resampled_objects.clear();
        continue;
      }
    }
    resampled_objects.push_back(std::move(raw_resampled_objects[i]));
  }
  return resampled_objects;
}

bool CheckHistoryConsistency(const ObjectProto &obj,
                             const ObjectProto &next_obj, double ts) {
  const auto dpos = Vec2dFromProto(next_obj.pos()) - Vec2dFromProto(obj.pos());
  const auto dv_by_pos_diff = dpos / ts;
  const auto vel = Vec2dFromProto(obj.vel());
  const double vx_lb = vel.x() * (1 - kConsistencyFactor);
  const double vx_ub = vel.x() * (1 + kConsistencyFactor);
  const double vy_lb = vel.y() * (1 - kConsistencyFactor);
  const double vy_ub = vel.y() * (1 + kConsistencyFactor);
  const double min_vx =
      std::min(vel.x() - kConsistencySpeedTolerance, std::min(vx_lb, vx_ub));
  const double max_vx =
      std::max(vel.x() + kConsistencySpeedTolerance, std::max(vx_lb, vx_ub));
  const double min_vy =
      std::min(vel.y() - kConsistencySpeedTolerance, std::min(vy_lb, vy_ub));
  const double max_vy =
      std::max(vel.y() + kConsistencySpeedTolerance, std::max(vy_lb, vy_ub));
  if (dv_by_pos_diff.x() < min_vx || dv_by_pos_diff.x() > max_vx) {
    return false;
  }
  if (dv_by_pos_diff.y() < min_vy || dv_by_pos_diff.y() > max_vy) {
    return false;
  }
  return true;
}

}  // namespace prediction
}  // namespace qcraft
