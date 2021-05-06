#ifndef ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_

#include <array>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

using DriveState = PathManagerStateProto::DriveState;

struct DirectionalPath {
  // Heading of a path point is the moving direction.
  DiscretizedPath path;
  // How should AV follow the path. False for reverse driving.
  bool forward;

  void FromProto(const DirectionalPathProto &proto) {
    std::vector<PathPoint> path_points(proto.path().begin(),
                                       proto.path().end());
    path = DiscretizedPath(std::move(path_points));
    forward = proto.forward();
  }
  void ToProto(DirectionalPathProto *proto) const {
    *proto->mutable_path() = {path.begin(), path.end()};
    proto->set_forward(forward);
  }
};

std::vector<Box2d> PathSweptVolume(
    const VehicleGeometryParamsProto &vehicle_geom,
    absl::Span<const DirectionalPath *const> paths, double length_buffer,
    double width_buffer);

// This path safety check just check global paths after current driving
// segment.
absl::Status PathSafetyCheck(
    const VehicleGeometryParamsProto &vehicle_geom,
    absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
    absl::Span<const DirectionalPath> paths, int current_index);

absl::StatusOr<PathPoint> GetPathPointFromGlobalIndex(
    absl::Span<const DirectionalPath *const> paths, int global_index);

void UpdatePathManagerState(const VehicleGeometryParamsProto &vehicle_geom,
                            const VehicleDriveParamsProto &vehicle_drive,
                            const FreespaceTaskProto::TaskType &task_type,
                            const PoseProto &av_pose, const Chassis &chassis,
                            absl::Span<const DirectionalPath> paths,
                            DriveState *drive_state, int *next_path_idx,
                            bool *switched_to_new_path);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_
