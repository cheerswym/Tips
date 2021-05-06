#ifndef ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_
#define ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/svt_graph_point.h"

namespace qcraft::planner {
class DpSvtCost {
 public:
  DpSvtCost(
      const SpeedFinderParamsProto* speed_finder_params, double total_t,
      double total_s,
      const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

  std::vector<SvtGraphPoint::StBoundaryDecision>
  GetStBoundaryDecisionsForInitPoint(const SvtGraphPoint& svt_graph_point);
  void GetStBoundaryCostAndDecisions(
      const SvtGraphPoint& prev_svt_graph_point,
      const SvtGraphPoint& svt_graph_point, double av_pseed,
      double* st_boundary_cost,
      std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions);

  double GetSpatialPotentialCost(double s) const;

  double GetSpeedLimitCost(double speed, double speed_limit) const;

  double GetReferenceSpeedCost(double speed, double cruise_speed) const;

  double GetAccelCost(double accel) const;

  void SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunction<double> plf) {
    follow_distance_rel_speed_plf_ = std::move(plf);
  }

 private:
  using SamplingDpSpeedParamsProto =
      SpeedFinderParamsProto::SamplingDpSpeedParamsProto;

  const SpeedFinderParamsProto* speed_finder_params_;
  const SamplingDpSpeedParamsProto* params_;

  PiecewiseLinearFunction<double> follow_distance_rel_speed_plf_;

  const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision_;
  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  absl::flat_hash_map<std::string, int> boundary_map_;
  // Note: boundary_cost_[boundary_index][t] = (s_upper,s_lower).
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  absl::Mutex boundary_cost_mutex_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_
