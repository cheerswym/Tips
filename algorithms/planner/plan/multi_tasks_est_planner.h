#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_H_

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/multi_tasks_est_planner_input.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"

namespace qcraft::planner {

PlannerStatus RunMultiTasksEstPlanner(const MultiTasksEstPlannerInput &input,
                                      PathBoundedEstPlannerOutput *output,
                                      ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_EST_PLANNER_H_
