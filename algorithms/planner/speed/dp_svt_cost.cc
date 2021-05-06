#include "onboard/planner/speed/dp_svt_cost.h"

#include <algorithm>
#include <utility>

#include "onboard/math/piecewise_linear_function.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace qcraft::planner {
namespace {

using DecisionType = StBoundaryProto::DecisionType;

DecisionType MakeStBoundaryDecision(double s, double s_lower, double s_upper) {
  constexpr double kFollowLeadRatio = 0.7;
  if (s < s_lower + kFollowLeadRatio * (s_upper - s_lower)) {
    return StBoundaryProto::FOLLOW;
  } else {
    return StBoundaryProto::LEAD;
  }
}

double GetStBoundaryCost(
    DecisionType decision, double av_speed, const StBoundary& st_boundary,
    double s, double s_lower, double s_upper, double t,
    double follow_standstill_distance,
    const PiecewiseLinearFunction<double>& follow_distance_rel_speed_plf,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedFinderParamsProto::SamplingDpSpeedParamsProto& params) {
  double cost = 0.0;
  if (decision == StBoundaryProto::FOLLOW) {
    double follow_distance_s = 0.0;
    if (st_boundary.source_type() == StBoundary::SourceType::VIRTUAL ||
        st_boundary.source_type() ==
            StBoundary::SourceType::IMPASSABLE_BOUNDARY) {
      follow_distance_s = follow_standstill_distance;
    } else {
      const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
      QCHECK(object_v.has_value());
      follow_distance_s =
          speed_finder_params.follow_time_headway() * std::max(0.0, *object_v) +
          speed_finder_params.follow_standstill_distance();
      // TODO(renjie): Consider to use speed of st graph point to compute
      // relative speed.
      const double rel_speed_gain =
          follow_distance_rel_speed_plf(*object_v - av_speed);
      follow_distance_s =
          std::max(follow_distance_s * rel_speed_gain,
                   speed_finder_params.follow_standstill_distance());
    }
    if (s + follow_distance_s > s_lower) {
      const double s_diff = follow_distance_s - s_lower + s;
      cost =
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else if (decision == StBoundaryProto::LEAD) {
    const auto object_v = st_boundary.GetStBoundarySpeedAtT(t);
    QCHECK(object_v.has_value());
    const double overtake_distance_s =
        speed_finder_params.lead_time_headway() * std::max(*object_v, 0.0) +
        speed_finder_params.lead_standstill_distance();
    if (s < s_upper + overtake_distance_s) {  // or calculated from velocity
      const double s_diff = overtake_distance_s + s_upper - s;
      cost =
          st_boundary.probability() * params.object_weight() * s_diff * s_diff;
    }
  } else {
    // We must have lead or follow decision here.
    QLOG(FATAL) << "Bad unknown decision for st-boundary " << st_boundary.id()
                << " at s = " << s << " t = " << t;
  }
  return cost;
}

}  // namespace

DpSvtCost::DpSvtCost(
    const SpeedFinderParamsProto* speed_finder_params, double total_t,
    double total_s,
    const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision)
    : speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      params_(&speed_finder_params->sampling_dp_speed_params()),
      st_boundaries_with_decision_(QCHECK_NOTNULL(st_boundaries_with_decision)),
      unit_t_(params_->unit_t()),
      total_s_(total_s) {
  int index = 0;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision_) {
    boundary_map_[st_boundary_with_decision.id()] = index++;
  }
  const auto dimension_t = CeilToInt(total_t / unit_t_) + 1;
  boundary_cost_.resize(st_boundaries_with_decision_->size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
}

std::vector<SvtGraphPoint::StBoundaryDecision>
DpSvtCost::GetStBoundaryDecisionsForInitPoint(
    const SvtGraphPoint& svt_graph_point) {
  std::vector<SvtGraphPoint::StBoundaryDecision> st_boundary_decisions;
  const double s = svt_graph_point.point().s();  // Should be zero.
  const double t = svt_graph_point.point().t();  // Should be zero.

  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision_) {
    const auto& st_boundary = *st_boundary_with_decision.st_boundary();
    if (t < st_boundary.min_t() || t > st_boundary.max_t()) {
      continue;
    }
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    const int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      if (boundary_cost_[boundary_index][svt_graph_point.index_t()].first <
          0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        QCHECK(s_range.has_value());
        boundary_cost_[boundary_index][svt_graph_point.index_t()] = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].first;
        s_lower =
            boundary_cost_[boundary_index][svt_graph_point.index_t()].second;
      }
      // Make decision based on relative position of s, s_lower and s_upper.
      const auto decision = MakeStBoundaryDecision(s, s_lower, s_upper);
      // Record decision on this st_boundary without prior decision.
      st_boundary_decisions.emplace_back(st_boundary.id(), decision);
    }
  }
  return st_boundary_decisions;
}

void DpSvtCost::GetStBoundaryCostAndDecisions(
    const SvtGraphPoint& prev_svt_graph_point,
    const SvtGraphPoint& svt_graph_point, double av_speed,
    double* st_boundary_cost,
    std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions) {
  QCHECK_NOTNULL(st_boundary_cost);
  QCHECK_NOTNULL(st_boundary_decisions);
  QCHECK(st_boundary_decisions->empty());

  const double prev_s = prev_svt_graph_point.point().s();
  const double prev_t = prev_svt_graph_point.point().t();

  double cost = 0.0;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision_) {
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE) {
      // Skip computing cost for st_boundary that has a prior IGNORE decision.
      continue;
    }

    double s = svt_graph_point.point().s();
    double t = svt_graph_point.point().t();
    const auto& st_boundary = *st_boundary_with_decision.st_boundary();
    if (t < st_boundary.min_t() ||
        (prev_t >= st_boundary.min_t() && t > st_boundary.max_t())) {
      continue;
    }

    // If overlap period is too small that located between current and previous
    // point, recalculate decision making s and t.
    if (t >= st_boundary.max_t()) {
      s = Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.max_t()));
      t = st_boundary.max_t();
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    int boundary_index = boundary_map_[st_boundary.id()];
    {
      absl::MutexLock lock(&boundary_cost_mutex_);
      auto& boundary_bounds =
          boundary_cost_[boundary_index][svt_graph_point.index_t()];
      if (boundary_bounds.first < 0.0) {
        const auto s_range = st_boundary.GetBoundarySRange(t);
        QCHECK(s_range.has_value());
        boundary_bounds = *s_range;
        s_upper = s_range->first;
        s_lower = s_range->second;
      } else {
        s_upper = boundary_bounds.first;
        s_lower = boundary_bounds.second;
      }
    }

    DecisionType decision = StBoundaryProto::UNKNOWN;
    if (st_boundary_with_decision.decision_type() != StBoundaryProto::UNKNOWN) {
      // If st_boundary has a prior decision, use it.
      decision = st_boundary_with_decision.decision_type();
    } else {
      // st_boundary doesn't have a prior decision.
      const auto prev_point_decision =
          prev_svt_graph_point.GetStBoundaryDecision(st_boundary.id());
      if (prev_point_decision.ok()) {
        QCHECK_NE(*prev_point_decision, StBoundaryProto::UNKNOWN);
        // If prev point has decision on the st_boundary, keep it.
        decision = *prev_point_decision;
      } else {
        // Compute decision decisive s, we can make decision via decisive st
        // point (decisive_s, min_t) no matter whether speed profile
        // cross st boundary between lower points or first lower point and upper
        // point.
        const double decisive_s =
            Lerp(prev_s, s, LerpFactor(prev_t, t, st_boundary.min_t()));
        const double first_s_upper = st_boundary.upper_points().front().s();
        const double first_s_lower = st_boundary.lower_points().front().s();
        // Make decision based on relative position of s, s_lower and s_upper.
        decision =
            MakeStBoundaryDecision(decisive_s, first_s_lower, first_s_upper);
      }
      // Record decision on this st_boundary without prior decision.
      st_boundary_decisions->emplace_back(st_boundary.id(), decision);
    }
    cost += GetStBoundaryCost(
        decision, av_speed, st_boundary, s, s_lower, s_upper, t,
        st_boundary_with_decision.follow_standstill_distance(),
        follow_distance_rel_speed_plf_, *speed_finder_params_, *params_);
  }
  *st_boundary_cost = cost * unit_t_;
}

double DpSvtCost::GetSpatialPotentialCost(double s) const {
  return (total_s_ - s) * params_->spatial_potential_weight();
}

double DpSvtCost::GetSpeedLimitCost(double speed, double speed_limit) const {
  double cost = 0.0;
  const double det_speed = speed - speed_limit;
  if (det_speed > 0) {
    cost += params_->exceed_speed_penalty() * params_->speed_weight() *
            (det_speed * det_speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += params_->low_speed_penalty() * params_->speed_weight() *
            (det_speed * det_speed) * unit_t_;
  }
  return cost;
}

double DpSvtCost::GetReferenceSpeedCost(double speed,
                                        double cruise_speed) const {
  double cost = 0.0;

  if (FLAGS_enable_sampling_dp_reference_speed) {
    const double diff_speed = speed - cruise_speed;
    cost += params_->reference_speed_penalty() * params_->speed_weight() *
            diff_speed * diff_speed * unit_t_;
  }

  return cost;
}

double DpSvtCost::GetAccelCost(double accel) const {
  double cost = 0.0;

  const double accel_sq = accel * accel;
  const double max_acc = params_->max_acceleration();
  const double max_dec = params_->max_deceleration();
  const double accel_penalty = params_->accel_penalty();
  const double decel_penalty = params_->decel_penalty();

  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }
  cost += accel_sq * decel_penalty * decel_penalty /
              (1 + std::exp(1.0 * (accel - max_dec))) +
          accel_sq * accel_penalty * accel_penalty /
              (1 + std::exp(-1.0 * (accel - max_acc)));

  return cost * unit_t_;
}

}  // namespace qcraft::planner
