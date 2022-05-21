#include "onboard/prediction/conflict_resolver/svt_feature_cost.h"

#include <string>
#include <vector>

namespace qcraft {
namespace prediction {
namespace {
constexpr double kZeroVelocityEpsilon = 1e-3;
constexpr double kMinTimeDuration = 1e-3;
constexpr int kEvaluateStep = 2;
constexpr double kRelativeSpeedRegularizer = 4.0;  // m/s.
const double kInvRelativeSpeedRegularizer = 1.0 / kRelativeSpeedRegularizer;

// TODO(changqing): Move to config proto.
constexpr double kAssumedMaxAcceleration = 3.0;
const double kInvAssumedMaxAcc = 1.0 / kAssumedMaxAcceleration;
constexpr double kAssumedMaxDeceleration = 2.0;
const double kInvAssumedMaxDec = 1.0 / kAssumedMaxDeceleration;
}  // namespace

// ------------------ Object Collision Feature ------------------
void SvtObjectCollisionFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);  // stationary, dynamic.
  // Stationary collision cost.
  cost[0] = 0.0;
  if (stationary_objects_.size() != 0) {
    if (states.size() == 1) {
      const auto& state = states.front();
      double pass_ratio = (min_s_ - state.s - stationary_follow_distance_) *
                          inv_stationary_follow_distance_;
      pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
      double rel_v_regularized = state.v * kInvRelativeSpeedRegularizer;
      rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
      cost[0] =
          Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) * kMinTimeDuration;
    } else {
      for (int i = 0; i < states.size() - 1; ++i) {
        const auto& state = states[i];
        const auto& next_state = states[i + 1];
        const double dt = next_state.t - state.t;
        double pass_ratio = (min_s_ - state.s - stationary_follow_distance_) *
                            inv_stationary_follow_distance_;
        pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
        double rel_v_regularized = state.v * kInvRelativeSpeedRegularizer;
        rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
        cost[0] += Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) * dt;
      }
    }
  }

  // Dynamic collision cost. Currently deal with on path moving objects. All
  // decision is follow. Do not enter the s range with velocity greater than the
  // object's avg speed at that time.
  cost[1] = 0.0;
  constexpr int kEvalStep = 1;
  if (moving_objects_.size() == 0) {
    return;
  }
  if (states.size() == 1) {
    const auto& state = states[0];
    double cost_accum = 0.0;
    for (const auto& st_bound : moving_objects_) {
      const auto v_ref_or = st_bound->GetStBoundarySpeedAtT(state.t);
      const auto s_range_or = st_bound->GetBoundarySRange(state.t);
      if (!s_range_or.has_value() || !v_ref_or.has_value()) continue;
      const auto& s_min = s_range_or->second;  // lower.
      double rel_v_regularized =
          (-*v_ref_or + state.v) * kInvRelativeSpeedRegularizer;
      rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
      double pass_ratio = (s_min - state.s - dynamic_follow_distance_) *
                          inv_dynamic_follow_distance_;
      pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
      cost_accum += Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) *
                    st_bound->probability();
    }
    cost[1] += cost_accum * kMinTimeDuration;
    return;
  }

  for (int i = 0; i < states.size(); i += kEvalStep) {
    const auto& state = states[i];
    const int next_idx = std::min<int>(i + kEvalStep, states.size() - 1);
    const auto& next_state = states[next_idx];
    double cost_accum = 0.0;
    for (const auto& st_bound : moving_objects_) {
      const auto v_ref_or = st_bound->GetStBoundarySpeedAtT(state.t);
      const auto s_range_or = st_bound->GetBoundarySRange(state.t);
      if (!v_ref_or.has_value() || !s_range_or.has_value()) continue;
      const auto dt =
          std::min<double>(next_state.t, st_bound->max_t()) - state.t;
      const auto& s_min = s_range_or->second;  // lower.
      // Relative speed (with respect to st boundary average speed) regularized
      // by 4.0m/s.
      double rel_v_regularized =
          (-*v_ref_or + state.v) * kInvRelativeSpeedRegularizer;
      rel_v_regularized = rel_v_regularized > 0.0 ? rel_v_regularized : 0.0;
      double pass_ratio = (s_min - state.s - dynamic_follow_distance_) *
                          inv_dynamic_follow_distance_;
      // Passed distance regularzied by dynamic follow distance;
      pass_ratio = pass_ratio > 0.0 ? 0.0 : pass_ratio;
      // Additional penalty if agent v is greater than object reference speed.
      cost_accum += Sqr(pass_ratio) * (1.0 + Sqr(rel_v_regularized)) *
                    st_bound->probability() * dt;
    }
    cost[1] += cost_accum;
  }
}

// ------------------ Stopline Feature ------------------
void SvtStoplineFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);  // only tl red for now.
  cost[0] = 0.0;
  if (stoplines_.size() == 0) return;
  if (states.size() == 1 && std::fabs(states[0].v < kZeroVelocityEpsilon)) {
    // Only one state and it's stationary.
    cost[0] = stoplines_.front() < states[0].s ? kMinTimeDuration : 0.0;
  }
  for (int i = 1; i < states.size(); ++i) {
    const double dt = states[i].t - states[i - 1].t;
    cost[0] += stoplines_.front() < states[i].s ? dt : 0.0;
  }
}

// ------------------ Comfort Feature ------------------
void SvtComfortFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);
  if (states.size() == 1) {
    return;
  }
  const double a =
      (states[1].v - states[0].v) /
      (states[1].t - states[0].t);  // Sampling states all have same acc.
  const double a_ratio =
      a > 0.0 ? a * kInvAssumedMaxAcc : std::fabs(a) * kInvAssumedMaxDec;
  for (int i = 0; i < states.size() - 1; ++i) {
    if (states[i].v < kZeroVelocityEpsilon) break;
    const auto& state = states[i];
    const auto& next_state = states[i + 1];
    cost[0] += Sqr(a_ratio) * (next_state.t - state.t);
  }
  cost[1] = 0.0;
}

// ------------------ Reference Speed Feature ------------------
void SvtReferenceSpeedFeatureCost::ComputeFeatureCost(
    const std::vector<SvtState>& states, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);
  cost[0] = 0.0;
  cost[1] = 0.0;

  if (states.size() == 1) {
    const auto speed_point_or = ref_speed_->EvaluateByS(
        std::min<double>(states.front().s, total_length_));
    const auto v_diff = (*speed_point_or).v() - states.front().v;
    cost[0] += kMinTimeDuration * Sqr(v_diff * inv_average_v_);

    const double s_ratio = (total_length_ - states.front().s) * inv_length_;
    cost[1] += kMinTimeDuration * s_ratio;
    return;
  }

  for (int i = 0; i < states.size(); i += kEvaluateStep) {
    const auto& state = states[i];
    const int next_state_idx =
        std::min<int>(i + kEvaluateStep, states.size() - 1);
    const auto dt = states[next_state_idx].t - state.t;
    const auto speed_point_or =
        ref_speed_->EvaluateByS(std::min<double>(state.s, total_length_));
    const auto v_diff = (*speed_point_or).v() - state.v;
    cost[0] += dt * Sqr(v_diff * inv_average_v_);
    const double s_ratio = (total_length_ - state.s) * inv_length_;
    cost[1] += dt * s_ratio;
  }
}

}  // namespace prediction
}  // namespace qcraft
