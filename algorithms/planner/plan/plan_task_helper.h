#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_

#include <deque>
#include <vector>

#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

std::deque<PlanTask> CreatePlanTasksQueueFromRoutingResult(
    const RouteManagerOutput &route_output, const SemanticMapManager &smm);

bool PlanTaskCompeleted(const PlanTask &task, const CoordinateConverter &cc,
                        Vec2d front_bumper_pos, double ego_heading,
                        double ego_v, const PlannerSemanticMapManager &psmm);

absl::StatusOr<std::vector<PlanTask>> SplitCruiseByUturnTask(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePath &prev_target_lane_path,
    const mapping::LanePoint &route_destination);

absl::StatusOr<std::vector<PlanTask>> CreateUturnTask(
    const PlannerSemanticMapManager &psmm, const PoseProto &ego_pose,
    const mapping::LanePath &prev_target_lane_path,
    const mapping::LanePoint &route_destination);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_
