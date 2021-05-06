#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_module.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_output.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

absl::Status RunPlanTaskDispatcher(
    const CoordinateConverter &coordinate_converter,
    // TODO(lidong): Remove dependency on the planner params class.
    const PlannerParams &planner_params, const PlannerInput &input,
    const RouteManagerOutput &route_output, const ObjectsProto *objects_proto,
    const TeleopState &teleop_state,
    const GeometryGraphProto::EndInfo &prev_end_info, absl::Time current_time,
    PlannerState *planner_state, PlannerOutput *output, ThreadPool *thread_pool,
    LiteModule *lite_module);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_
