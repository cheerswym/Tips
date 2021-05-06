#ifndef ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/planner_input.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct AebPlannerOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  std::string aeb_object_id;
  ObjectType aeb_object_type;
};

absl::StatusOr<AebPlannerOutput> RunAebPlanner(
    const PlannerInput &input,
    const ApolloTrajectoryPointProto &plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto> &prev_traj_points,
    const PlannerParams &planner_params, Polygon2d *output_risk_area);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_AEB_PLANNER_H_
