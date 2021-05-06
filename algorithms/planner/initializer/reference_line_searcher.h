#ifndef ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_SEARCHER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

absl::StatusOr<ReferenceLineSearcherOutput> SearchReferenceLine(
    const ReferenceLineSearcherInput& input, InitializerDebugProto* debug_proto,
    ThreadPool* thread_pool);

absl::Status ActivateGeometryGraph(
    const ReferenceLineSearcherOutput& searcher_result,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geom_graph);

void ParseReferenceLineResultToProto(const ReferenceLineSearcherOutput& result,
                                     GeometryGraphProto* debug_proto);
}  // namespace qcraft::planner
#endif
