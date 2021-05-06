#ifndef ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_
#define ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/selector/candidate_stats.h"
#include "onboard/planner/selector/cost_feature_base.h"

namespace qcraft {
namespace planner {

// Value range (0, 1], `base` for shape and `reg` for scaling along x axis.
inline double ExpDecayCoeffAtStep(double base, double reg, int i) {
  return std::pow(base, -i / (reg * kTrajectorySteps));
}

using PlannerTrajectory = std::vector<ApolloTrajectoryPointProto>;

class TrajProgressCost : public CostFeatureBase {
 public:
  TrajProgressCost(const SemanticMapManager *smm,
                   const RouteSectionsInfo *sections_info, ProgressStats stats)
      : CostFeatureBase("progress", {"progress", "follow_slow"},
                        /*is_common=*/true),
        smm_(smm),
        sections_info_(sections_info),
        ego_v_(stats.ego_v),
        lane_speed_map_(std::move(stats.lane_speed_map)),
        max_lane_speed_(stats.max_lane_speed) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  const SemanticMapManager *smm_;
  const RouteSectionsInfo *sections_info_;
  double ego_v_;
  absl::flat_hash_map<mapping::ElementId, LaneSpeedInfo> lane_speed_map_;
  double max_lane_speed_;
};

class TrajMaxJerkCost : public CostFeatureBase {
 public:
  explicit TrajMaxJerkCost(
      const MotionConstraintParamsProto &motion_constraints)
      : CostFeatureBase("max_jerk", {"max_lon_jerk", "max_lat_jerk"},
                        /*is_common=*/true),
        accel_jerk_constraint_(motion_constraints.max_accel_jerk()),
        decel_jerk_constraint_(motion_constraints.max_decel_jerk()),
        lat_jerk_constraint_(motion_constraints.max_lateral_jerk()) {
    for (int i = 0; i < kTrajectorySteps; ++i) {
      coeffs_[i] = ExpDecayCoeffAtStep(10, 0.6, i);
    }
  }

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  double accel_jerk_constraint_, decel_jerk_constraint_;
  double lat_jerk_constraint_;
  double coeffs_[kTrajectorySteps];  // Decaying factor w.r.t. time step.
};

class TrajLaneChangeCost : public CostFeatureBase {
 public:
  TrajLaneChangeCost(const SemanticMapManager *smm,
                     const mapping::LanePath *prev_lp_from_current,
                     const PlannerTrajectory *prev_traj,
                     const ApolloTrajectoryPointProto &plan_start_point,
                     const VehicleGeometryParamsProto &vehicle_geom)
      : CostFeatureBase("lane_change",
                        {"lane_path_diff", "pose_to_target", "prev_lat_diff",
                         "lc_in_intersection"},
                        /*is_common=*/true),
        smm_(smm),
        prev_lp_from_current_(prev_lp_from_current),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
        ego_ra_to_front_(vehicle_geom.front_edge_to_center()),
        ego_v_(plan_start_point.v()) {
    std::vector<Vec2d> prev_pts;
    if (prev_traj != nullptr && prev_traj->size() > 1) {
      prev_pts.reserve(prev_traj->size());
      std::transform(prev_traj->begin(), prev_traj->end(),
                     std::back_inserter(prev_pts), [](const auto &traj_pt) {
                       return Vec2dFromApolloTrajectoryPointProto(traj_pt);
                     });
    }
    prev_traj_ff_or_ = BuildBruteForceFrenetFrame(prev_pts);
  }

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  const SemanticMapManager *smm_;
  const mapping::LanePath *prev_lp_from_current_;
  Vec2d ego_pos_;
  double ego_ra_to_front_, ego_v_;
  absl::StatusOr<BruteForceFrenetFrame> prev_traj_ff_or_;
};

class TrajCrossSolidBoundaryCost : public CostFeatureBase {
 public:
  explicit TrajCrossSolidBoundaryCost(
      const VehicleGeometryParamsProto &vehicle_geom)
      : CostFeatureBase(
            "cross_solid_boundary",
            {"solid_white", "solid_yellow", "solid_double_yellow", "curb"},
            /*is_common=*/true),
        ego_length_(vehicle_geom.length()),
        ego_width_(vehicle_geom.width()) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  double ego_length_, ego_width_;
};

class TrajMinDistToObjectsCost : public CostFeatureBase {
 public:
  explicit TrajMinDistToObjectsCost(
      const VehicleGeometryParamsProto &vehicle_geom)
      : CostFeatureBase("dist_to_objects", {"dist_to_objects"},
                        /*is_common=*/true),
        ego_length_(vehicle_geom.length()),
        ego_width_(vehicle_geom.width()),
        ego_center_to_ra_(vehicle_geom.front_edge_to_center() -
                          0.5 * vehicle_geom.length()) {
    for (int i = 0; i < kTrajectorySteps; ++i) {
      coeffs_[i] = ExpDecayCoeffAtStep(1e3, 0.8, i);
    }
  }

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  double ego_length_, ego_width_, ego_center_to_ra_;
  double coeffs_[kTrajectorySteps];  // Decaying factor w.r.t. time step.
};

class TrajRouteLookAheadCost : public CostFeatureBase {
 public:
  explicit TrajRouteLookAheadCost(RouteLookAheadStats stats)
      : CostFeatureBase("route_look_ahead",
                        {"length_along_route", "reach_destination",
                         "preview_beyond_horizon"},
                        /*is_common=*/true),
        route_len_(stats.route_len),
        local_horizon_(stats.local_horizon),
        driving_dist_map_(std::move(stats.driving_dist_map)),
        len_along_route_map_(std::move(stats.len_along_route_map)),
        lc_num_to_targets_map_(std::move(stats.lc_num_to_targets_map)),
        max_len_along_route_(stats.max_length_along_route),
        min_lc_num_(stats.min_lc_num) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  double route_len_, local_horizon_;
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map_,
      len_along_route_map_;
  absl::flat_hash_map<mapping::ElementId, int> lc_num_to_targets_map_;
  double max_len_along_route_;
  int min_lc_num_;
};

// To make selector favor trajectories from non-expanded path boundary.
//
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || e   | ||       || e   . ||       || e ||       || .   e   . ||
//     low               high            low               high
class TrajBoundaryExpansionCost : public CostFeatureBase {
 public:
  explicit TrajBoundaryExpansionCost(
      const ApolloTrajectoryPointProto &plan_start_point)
      : CostFeatureBase("boundary_expansion", {"boundary_expansion"},
                        /*is_common=*/false),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  Vec2d ego_pos_;
};

class TrajProhibitedRegionCost : public CostFeatureBase {
 public:
  TrajProhibitedRegionCost(
      const RouteSectionsInfo *sections_info,
      const ApolloTrajectoryPointProto &plan_start_point,
      const absl::flat_hash_set<std::string> *stalled_objects,
      const VehicleGeometryParamsProto &vehicle_geom)
      : CostFeatureBase("prohibited_region", {"behind_stalled_object"},
                        /*is_common=*/true),
        sections_info_(sections_info),
        ego_v_(plan_start_point.v()),
        ego_half_width_(0.5 * vehicle_geom.width()),
        stalled_objects_(stalled_objects) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;

 private:
  const RouteSectionsInfo *sections_info_;
  double ego_v_, ego_half_width_;
  const absl::flat_hash_set<std::string> *stalled_objects_;
};

class TrajIsFallbackCost : public CostFeatureBase {
 public:
  TrajIsFallbackCost()
      : CostFeatureBase("is_fallback", {"is_fallback"},
                        /*is_common=*/false) {}

  CostFeatureBase::CostVec ComputeCost(
      const SchedulerOutput &scheduler_output,
      const EstPlannerOutput &planner_output,
      std::vector<std::string> *extra_info) const override;
};

// TODO(boqian): find more suitable cost features.

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_
