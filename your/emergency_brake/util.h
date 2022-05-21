#ifndef ONBOARD_EMERGENCY_BRAKE_UTIL_H_
#define ONBOARD_EMERGENCY_BRAKE_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/safe_unit.h"
#include "onboard/math/util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace emergency_brake {

// This function returns the polygon swept by the front bumper of AV box when
// the AV rotates around a center.
//
// When `signed_drive_distance` is positive, it means AV is driving forward, and
// when `signed_drive_distance` is negative, it means AV is driving backward.
//
// When `signed_circular_radius` is positive, it means av is turning left in
// AV's moving direction, and when `signed_circular_radius` is negative, it
// means av is turning right in AV's moving direction.
//
// It is implemented by simulating rotating the AV box, and recording the
// positions of the left corner and right corners of AV in the moving direction
// while doing the rotation. The polygon is created by recording the traces of
// the box corners.
absl::StatusOr<Polygon2d> ComputeCircularDetectionPolygon(
    const Box2d &av_box, double rac_to_center, double signed_drive_distance,
    double signed_circular_radius);

// Compute a circular constant jerk motion of the AV. `start` is the current
// state of the vehicle. `dt` is the time step. `max_duration` is the maximum
// time duration of the trajectory. The trajectory ends when the AV is
// stopped, even when the trajectory's duration is less than `max_duration`.
//
// This function works regardless of AV is moving forward or backward.
std::vector<ApolloTrajectoryPointProto> ComputeConstJerkCircularMotion(
    const ApolloTrajectoryPointProto &start, Duration dt, Duration max_duration,
    Jerk jerk);

// Compute a circular constant acceleration motion of the AV. `start` is the
// current state of the vehicle. `dt` is the time step. `max_duration` is the
// maximum time duration of the trajectory. The trajectory ends when the AV is
// stopped, even when the trajectory's duration is less than `max_duration`.
//
// This function works regardless of AV is moving forward or backward.
std::vector<ApolloTrajectoryPointProto> ComputeConstAccelerationCircularMotion(
    const ApolloTrajectoryPointProto &start, Duration dt, Duration max_duration,
    Acceleration accel);

// Returns the first FenDetectionProto that has collision with the AV's
// trajectory.
// We can assume the fen detection movings with const speed.
std::optional<FenDetectionProto> ComputeFenCollisions(
    const std::vector<ApolloTrajectoryPointProto> &traj, double traj_start_time,
    const VehicleGeometryParamsProto &vehicle_geom,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const FenDetectionsProto &fen_detections);

}  // namespace emergency_brake
}  // namespace qcraft

#endif  // ONBOARD_EMERGENCY_BRAKE_UTIL_H_
