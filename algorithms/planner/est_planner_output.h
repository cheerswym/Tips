#ifndef ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_

#include <memory>
#include <vector>

#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/trajectory_end_info.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

struct EstPlannerDebug {
  SpacetimePlannerTrajectories st_planner_trajectories;
  FilteredTrajectories filtered_prediction_trajectories;
  ConstraintProto decision_constraints;
  InitializerDebugProto initializer_debug_proto;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  SpeedFinderDebugProto speed_finder_debug;
  TrajectoryValidationResultProto traj_validation_result;
};

struct EstPlannerOutput {
  // Final trajectory.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  // Return planner state that needs to persist here.
  DeciderStateProto decider_state;
  // Return planner state that needs to persist here.
  InitializerStateProto initializer_state;
  // Return a list of st planner trajectories.
  std::unique_ptr<SpacetimeTrajectoryManager> filtered_traj_mgr;
  SpacetimePlannerTrajectories st_planner_trajectories;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfo> trajectory_end_info;

  AccumulatedDiscountedCostsProto feature_costs;
  AccumulatedDiscountedCostsProto expert_costs;

  EstPlannerDebug debug_info;
  vis::vantage::ChartDataBundleProto chart_data;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_
