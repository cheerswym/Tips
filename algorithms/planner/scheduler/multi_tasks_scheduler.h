#ifndef ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
#define ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/scheduler/scheduler_input.h"
#include "onboard/planner/scheduler/scheduler_output.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput &input,
    const std::vector<LanePathInfo> &target_lp_infos, ThreadPool *thread_pool);

bool ShouldSmoothRefLane(const TrafficLightInfoMap &tl_info_map,
                         const DrivePassage &dp, bool prev_smooth_state);

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &route_sections_info,
    const std::vector<LanePathInfo> &lp_infos, DrivePassage drive_passage,
    const LanePathInfo &lp_info, const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerParamsProto &planner_params,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &prev_lane_path_from_current,
    const SmoothedReferenceLineResultMap &smooth_result_map, bool borrow,
    bool should_smooth, ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
