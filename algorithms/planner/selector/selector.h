#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/selector_input.h"

namespace qcraft {
namespace planner {

absl::StatusOr<int> SelectTrajectory(
    const SelectorInput &input,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &results,
    SelectorDebugProto *selector_debug);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
