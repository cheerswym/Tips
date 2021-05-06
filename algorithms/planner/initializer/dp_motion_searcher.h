#ifndef ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_graph_cache.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/multi_traj_selector.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft::planner {
// The function searches space-time trajectory with dynamic programming. In
// order to accelerate computing, time and speed are discretized to several
// intervals, similar to the spirit of Hybrid A-Star. This function is only
// tested for curvy geometry graph. Do not use it for now for straight
// geometry graph.
absl::StatusOr<MotionSearchOutput> DpSearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool,
    InitializerDebugProto* debug_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
