#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft {
namespace planner {

absl::StatusOr<FreespaceMap> ConstructFreespaceMap(
    FreespaceTaskProto::TaskType task_type,
    const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerSemanticMapManager &psmm, const PoseProto &ego_pose,
    const mapping::ParkingSpotInfo *parking_spot_info, const PathPoint &goal);

void AddUTurnBoundary(const PlannerSemanticMapManager &psmm,
                      const mapping::LanePath *lane_path,
                      const VehicleGeometryParamsProto &vehicle_geom,
                      FreespaceMap *freespace_map);

absl::StatusOr<ConstraintManager> BuildFreespacePlannerConstraint(
    const VehicleGeometryParamsProto &veh_geo_params,
    const DirectionalPath &path);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_
