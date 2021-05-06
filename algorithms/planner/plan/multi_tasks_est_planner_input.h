#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_INPUT_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_INPUT_H_

#include <string>
#include <vector>

#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {

struct MultiTasksEstPlannerInput {
  const PlannerInput *planner_input = nullptr;
  const PlannerParams *planner_params = nullptr;
  const Route *route = nullptr;
  const RouteSections *route_sections_from_start = nullptr;
  const PlanStartPointInfo *start_point_info = nullptr;

  const SpacetimeTrajectoryManager *st_traj_mgr = nullptr;
  const PlannerObjectManager *object_manager = nullptr;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const SceneOutputProto *scene_reasoning = nullptr;
  const TeleopState *teleop_state = nullptr;

  // TODO(weijun): min dependency
  const PlannerState *planner_state = nullptr;
  // Prev states:
  const std::vector<ApolloTrajectoryPointProto> *time_aligned_prev_traj =
      nullptr;
  const mapping::LanePath *prev_target_lane_path = nullptr;
  const RouteSections *prev_route_sections = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  const mapping::LanePoint *station_anchor = nullptr;
  const SmoothedReferenceLineResultMap *smooth_result_map = nullptr;
  bool prev_smooth_state = false;
  const SensorFovsProto *sensor_fovs = nullptr;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_INPUT_H_
