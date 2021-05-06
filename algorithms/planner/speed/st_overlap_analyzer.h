#ifndef ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_

#include <vector>

#include "absl/types/span.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary);

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double init_v,
    std::vector<StBoundaryRef>* st_boundaries);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
