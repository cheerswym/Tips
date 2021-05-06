#include "onboard/planner/plan/aeb_planner.h"

#include <utility>

#include "onboard/planner/emergency_stop.h"

namespace qcraft::planner {

absl::StatusOr<AebPlannerOutput> RunAebPlanner(
    const PlannerInput &input,
    const ApolloTrajectoryPointProto &plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto> &prev_traj_points,
    const PlannerParams &planner_params, Polygon2d *output_risk_area) {
  SCOPED_QTRACE("AebPlan");

  auto emergency_stop_or =
      aeb::CheckEmergencyStop(planner_params, input, output_risk_area);

  if (!emergency_stop_or.ok()) {
    return emergency_stop_or.status();
  }

  std::vector<ApolloTrajectoryPointProto> aeb_traj_points =
      aeb::PlanEmergencyStopTrajectory(
          plan_start_point, path_s_inc_from_prev, reset, prev_traj_points,
          planner_params.planner_params().emergency_stop_params(),
          planner_params.planner_params().motion_constraint_params(),
          *input.semantic_map_manager);

  return AebPlannerOutput{.trajectory_points = std::move(aeb_traj_points),
                          .aeb_object_id = emergency_stop_or->id,
                          .aeb_object_type = emergency_stop_or->type};
}

}  // namespace qcraft::planner
