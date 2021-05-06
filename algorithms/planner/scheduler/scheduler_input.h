#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_

#include <string>
#include <vector>

#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct MultiTasksSchedulerInput {
  const PlannerSemanticMapManager *psmm;
  const VehicleGeometryParamsProto *vehicle_geom;
  const PlannerParamsProto *planner_params;
  const SpacetimeTrajectoryManager *st_traj_mgr;
  const absl::flat_hash_set<std::string> *stalled_objects;
  const std::vector<LanePathInfo> *lane_path_infos;
  const RouteSectionsInfo *sections_info_from_current;
  const TrafficLightInfoMap *tl_info_map;
  bool prev_smooth_state = false;
  const ApolloTrajectoryPointProto *plan_start_point;
  const mapping::LanePoint *station_anchor;
  double start_route_s;
  const mapping::LanePath *prev_lane_path_from_current;
  const SmoothedReferenceLineResultMap *smooth_result_map;
  ThreadPool *thread_pool;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_
