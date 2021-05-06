#ifndef ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
#define ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft::planner {

absl::StatusOr<MotionSearchOutput> SearchMotion(
    const MotionSearchInput& motion_input, ThreadPool* thread_pool,
    InitializerDebugProto* debug_proto);

// TODO(lidong): Refactor this function to just create initializer input.
absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput& initializer_input,
    InitializerDebugProto* debug_proto,
    vis::vantage::ChartDataBundleProto* charts, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
