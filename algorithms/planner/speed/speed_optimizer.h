#ifndef ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_H_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/math/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_limit_generator.h"
#include "onboard/planner/speed/speed_optimizer_constraint_manager.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

class SpeedOptimizer {
 public:
  SpeedOptimizer(const MotionConstraintParamsProto *motion_constraint_params,
                 const SpeedFinderParamsProto *speed_finder_params,
                 double path_length, double default_speed_limit,
                 int traj_steps);

  absl::Status Optimize(
      const ApolloTrajectoryPointProto &init_point,
      absl::Span<const StBoundaryWithDecision> decision_boundaries,
      const absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType,
                                SpeedLimit> &speed_limit_map,
      const SpeedVector &reference_speed, SpeedVector *optimized_speed,
      SpeedFinderDebugProto *speed_finder_debug_proto);

  absl::Status OptimizeWithMaxSpeed(
      const ApolloTrajectoryPointProto &init_point,
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      SpeedVector *optimized_speed,
      SpeedFinderDebugProto *speed_finder_debug_proto);

  struct SpeedBoundWithInfo {
    double bound = 0.0;
    std::string info;
  };

 private:
  bool AddConstarints(const ApolloTrajectoryPointProto &init_point,
                      const SpeedOptimizerConstraintManager &constraint_mgr);

  bool AddKernel(const SpeedOptimizerConstraintManager &constraint_mgr,
                 const std::vector<SpeedBoundWithInfo> &floor_speed_limit);

  absl::Status Solve();

  void MakeSConstraint(
      int knot_idx, double av_speed,
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      SpeedOptimizerConstraintManager *constraint_mgr,
      SpeedFinderDebugProto *speed_finder_debug_proto);

  void MakeFollowConstraintForStationaryObject(
      int knot_idx, double s_upper, double min_s,
      double follow_standstill_distance, const std::string &st_boundary_id,
      SpeedOptimizerConstraintManager *constraint_mgr,
      SpeedFinderDebugProto *speed_finder_debug_proto) const;

  void MakeFollowConstraintForMovingObject(
      int knot_idx, double s_upper, double obj_speed, double prob,
      double av_speed, double follow_standstill_distance,
      const std::string &st_boundary_id, const std::string &integration_id,
      SpeedOptimizerConstraintManager *constraint_mgr,
      SpeedFinderDebugProto *speed_finder_debug_proto) const;

  void MakeLeadConstraint(
      int knot_idx, double s_lower, double obj_speed, double prob,
      double av_speed, double lead_standstill_distance,
      const std::string &integration_id,
      SpeedOptimizerConstraintManager *constraint_mgr,
      SpeedFinderDebugProto *speed_finder_debug_proto) const;

  void MakeSpeedConstraint(
      int knot_idx,
      const absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType,
                                std::vector<SpeedBoundWithInfo>>
          &speed_limit_map,
      const absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType, double>
          &speed_weight_map,
      SpeedOptimizerConstraintManager *constraint_mgr,
      SpeedFinderDebugProto *speed_finder_debug_proto) const;

  void MakeAccelConstraint(
      int knot_idx, double reference_speed,
      const std::pair<double, double> &accel_bound,
      SpeedOptimizerConstraintManager *constraint_mgr) const;

  absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType,
                      std::vector<SpeedBoundWithInfo>>
  EstimateSpeedBound(
      const absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType,
                                SpeedLimit> &speed_limit_map,
      const SpeedVector &reference_speed) const;

 private:
  const MotionConstraintParamsProto *motion_constraint_params_;
  const SpeedFinderParamsProto *speed_finder_params_;
  const SpeedFinderParamsProto::SpeedOptimizerParamsProto
      *speed_optimizer_params_;

  double total_time_ = 0.0;
  double delta_t_ = 0.0;
  int knot_num_ = 0;
  double allowed_max_speed_ = 0.0;
  double max_path_length_ = 0.0;
  ApolloTrajectoryPointProto init_point_;
  PiecewiseLinearFunction<double> follow_weight_min_s_plf_;
  PiecewiseLinearFunction<double> follow_distance_rel_speed_plf_;
  PiecewiseLinearFunction<double> probability_gain_plf_;
  PiecewiseLinearFunction<double> accel_weight_gain_plf_;
  std::vector<double> piecewise_time_range_;
  PiecewiseLinearFunction<double> moving_obj_time_gain_plf_;
  PiecewiseLinearFunction<double> stationary_obj_time_gain_plf_;
  PiecewiseLinearFunction<double> object_speed_follow_time_headway_plf_;
  PiecewiseLinearFunction<double> accel_lower_bound_plf_;

  std::unique_ptr<PiecewiseJerkQpSolver> solver_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_H_
