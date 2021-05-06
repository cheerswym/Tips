#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_ASYNC_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_ASYNC_PLANNER_H_

#include <memory>
#include <vector>

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/multi_tasks_est_planner_input.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/plan/st_path_planner.h"
#include "onboard/planner/scheduler/scheduler_output.h"

namespace qcraft {
namespace planner {

struct AsyncPathOutput {
  struct ScheduledPathInfo {
    StPathPlannerOutput path_output;
    PlannerStatus path_status;
  };
  std::vector<ScheduledPathInfo> path_infos;
  std::vector<SchedulerOutput> scheduler_output;

  // Store a copy of path related auxiliary data.
  RouteSections route_sections_from_start;
  std::vector<LanePathInfo> lp_infos;
  mapping::LanePath prev_target_lane_path_from_start;
  absl::flat_hash_set<mapping::ElementId> avoid_lanes;
  PlannerStatus async_path_status;
};

// The state lives across iteration for async path planner.
struct AsyncPathState {
  Future<std::shared_ptr<AsyncPathOutput>> path_future;
  std::shared_ptr<AsyncPathOutput> current_path;
};

// The function reuses prev_path if provided. Otherwise, it recomputes the path.
PlannerStatus RunMultiTasksAsyncPlanner(const MultiTasksEstPlannerInput &input,
                                        const AsyncPathOutput *prev_path,
                                        PathBoundedEstPlannerOutput *output,
                                        AsyncPathState *async_path_state,
                                        ThreadPool *thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_ASYNC_PLANNER_H_
