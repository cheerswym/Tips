#ifndef ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_

#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft {
namespace planner {

// Set pass time and yield time for st-boundary according to its type, right of
// way, etc.
void DecideTimeBuffersForStBoundary(StBoundaryWithDecision* st_boundary_wd);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_TIME_BUFFER_DECIDER_H_
