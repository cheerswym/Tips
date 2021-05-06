#ifndef ONBOARD_PLANNER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PLANNER_UTIL_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

////////////////////////////////////////////////////////////////////////////////
// Vehicle kinematics related.
double ComputeLongitudinalJerk(const TrajectoryPoint &traj_point);
double ComputeLateralAcceleration(const TrajectoryPoint &traj_point);
double ComputeLateralJerk(const TrajectoryPoint &traj_point);

////////////////////////////////////////////////////////////////////////////////
// Perception related.
bool IsVulnerableRoadUserType(ObjectType type);
bool IsStaticObjectType(ObjectType type);

////////////////////////////////////////////////////////////////////////////////
// Semantic map and lane path related.

// Find a lane path that is continuously neighboring and going in the same
// direction as the given lane path.
// @param cut_extra: cut extra lane path beyond max_length
mapping::LanePath FindContinuousCodirectionalLaneNeighborPath(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePath &lane_path, bool left, double max_length,
    bool broken_boundary_must = false, bool cut_extra = false);
mapping::LanePath FindCodirectionalAlignedLaneNeighborPath(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePath &lane_path, bool left, double max_length,
    bool broken_boundary_must = false, bool cut_extra = false);

// Searches for a lane path between two given lane points and returns its
// length, going through at most one lane end/beginning junction. In other
// words, this function succeeds only if the two lane points are on the same
// lane or on two connected lanes. For long range path searching, see
// route_searcher.h.
double SignedDistanceBetweenNearbyLanePoints(
    const SemanticMapManager &semantic_map_manager, mapping::LanePoint from,
    mapping::LanePoint to, mapping::LanePath *lane_path = nullptr);

double DistanceBetweenNearbyLanePoints(
    const SemanticMapManager &semantic_map_manager, mapping::LanePoint point0,
    mapping::LanePoint point1, mapping::LanePath *lane_path = nullptr);

// Preprocess semantic map modifier proto. Convert it to
// PlannerSemanticMapModification data structure. Then it can be used to
// initialize planner semantic map manager directly.
PlannerSemanticMapModification CreateSemanticMapModification(
    const SemanticMapManager &semantic_map_manager,
    const mapping::SemanticMapModifierProto &modifier);

mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
    const PlannerSemanticMapModification &modifier);

// Copied from planner module.
std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const ApolloTrajectoryPointProto &plan_start_point,
    const TrajectoryProto &prev_traj, bool reset);

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto> &prev_reset_planned_point,
    const PoseProto &pose, const Chassis &chassis,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &vehicle_geom_params,
    const VehicleDriveParamsProto &vehicle_drive_params);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_UTIL_H_
