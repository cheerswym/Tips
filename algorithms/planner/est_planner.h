#ifndef ONBOARD_PLANNER_EST_PLANNER_H_
#define ONBOARD_PLANNER_EST_PLANNER_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft {
namespace planner {

struct EstPlannerInput {
  const SemanticMapManager *semantic_map_manager = nullptr;
  const PlannerSemanticMapManager *planner_semantic_map_manager = nullptr;
  int plan_id = 1 /*for plot and debugging*/;
  const PoseProto *pose = nullptr;
  const PlannerState *planner_state = nullptr;
  const VehicleParamApi *vehicle_params = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  const PlannerObjectManager *obj_mgr = nullptr;
  const PlanStartPointInfo *start_point_info = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const SceneOutputProto *scene_reasoning = nullptr;
  const SchedulerOutput *scheduler_output = nullptr;
  const RouteSections *route_sections_from_start = nullptr;
  const std::vector<ApolloTrajectoryPointProto> *time_aligned_prev_traj =
      nullptr;
  const TeleopState *teleop_state = nullptr;
  const SensorFovsProto *sensor_fovs = nullptr;
};

PlannerStatus RunEstPlanner(const EstPlannerInput &input,
                            EstPlannerOutput *est_output,
                            ThreadPool *thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EST_PLANNER_H_
