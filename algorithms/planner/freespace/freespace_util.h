#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_

#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

absl::StatusOr<Vec2d> MaybeAdjustGoalForTightParkingSpot(
    const VehicleGeometryParamsProto &vehicle_geom,
    const mapping::ParkingSpotInfo &parking_spot_info, const Vec2d &goal_pos);

absl::StatusOr<PathPoint> GenerateGoalFromParkingSpot(
    const VehicleGeometryParamsProto &vehicle_geom,
    const mapping::ParkingSpotInfo &parking_spot_info,
    const PlannerObjectManager &object_mgr);

PathPoint RecoverGoalFromState(
    const CoordinateConverter &coordinate_converter,
    const PathManagerStateProto::GlobalGoal &global_goal);

std::pair<PathPoint, PathManagerStateProto::GlobalGoal>
ConvertGlobalPoseToPathPoint(const GlobalPose &pose,
                             const CoordinateConverter &cc);

TrajectoryProto CreateFreespaceTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto> &planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto> &past_points,
    const Chassis::GearPosition &gear_position,
    const DrivingStateProto &driving_state, bool low_speed_freespace);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_
