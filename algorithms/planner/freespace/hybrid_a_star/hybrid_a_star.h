#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H

#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/hybrid_a_star/node_3d.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<DirectionalPath>> FindPath(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    FreespaceTaskProto::TaskType task_type, const FreespaceMap &freespace_map,
    absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
    const PathPoint &start, const PathPoint &end,
    HybridAStartDebugProto *debug_info);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_HYBRID_A_STAR_H
