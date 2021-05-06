#ifndef ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_
#define ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_

#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_module.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct CruiseTaskOutput {
  TrajectoryProto trajectory_info;
  // TODO(weijun): min dependency.
  PlannerDebugProto debug_info;
  vis::vantage::ChartsDataProto chart_data;
  HmiContentProto hmi_content;
};

struct CruiseTaskInput {
  bool rerouted;
  const CoordinateConverter *coordinate_converter;
  const PlannerInput *planner_input;
  const PlannerParams *planner_params;
  const TeleopState *teleop_state;
  const RouteManagerOutput *route_output;

  const PlanStartPointInfo *plan_start_point_info;
  absl::Time plan_time;

  const SpacetimeTrajectoryManager *st_traj_mgr;
  const PlannerObjectManager *object_manager;

  const std::vector<ApolloTrajectoryPointProto> *time_aligned_prev_traj_points;

  const GeometryGraphProto::EndInfo *prev_end_info;
};

PlannerStatus RunCruiseTask(const CruiseTaskInput &input,
                            CruiseTaskOutput *result,
                            PlannerState *planner_state,
                            ThreadPool *thread_pool, LiteModule *lite_module);

}  // namespace qcraft::planner

#endif  // #ifndef ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_
