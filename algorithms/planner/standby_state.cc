#include "onboard/planner/standby_state.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/halfplane.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {
// Note: This stop line just used for visualization.
void AddConstraint(const VehicleGeometryParamsProto &vehicle_geom,
                   const PoseProto &pose, ConstraintProto *constrant) {
  QCHECK_NOTNULL(constrant);
  const auto tangent = Vec2d::FastUnitFromAngle(pose.yaw());
  const Vec2d cur_point(pose.pos_smooth().x(), pose.pos_smooth().y());
  const Vec2d front_edge_center_point =
      cur_point + tangent * vehicle_geom.front_edge_to_center();
  constexpr double kHalfplaneHalfWidth = 5.0;  // m.
  const Vec2d start =
      front_edge_center_point - tangent.Perp() * kHalfplaneHalfWidth;
  const Vec2d end =
      front_edge_center_point + tangent.Perp() * kHalfplaneHalfWidth;
  const HalfPlane halfplane(start, end);
  auto *stop_line = constrant->add_stop_line();
  halfplane.ToProto(stop_line->mutable_half_plane());
  stop_line->set_id("standby");
  stop_line->mutable_source()->mutable_standby()->set_reason(
      "No routing received.");
}

void FillTrajectoryProto(
    absl::Time plan_time,
    absl::Span<const ApolloTrajectoryPointProto> planned_trajectory,
    absl::Span<const ApolloTrajectoryPointProto> past_trajectory,
    TrajectoryProto *trajectory) {
  QCHECK_NOTNULL(trajectory);
  trajectory->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory->add_trajectory_point() = planned_trajectory[i];
  }
  for (int i = 0; i < past_trajectory.size(); ++i) {
    *trajectory->add_past_points() = past_trajectory[i];
  }

  trajectory->mutable_driving_state()->set_type(DrivingStateProto::STANDBY);
  trajectory->set_gear(Chassis::GEAR_DRIVE);
}

absl::Status ValidateStandbyPrecondition(const PoseProto &pose,
                                         PlannerStatusProto *planner_status) {
  QCHECK_NOTNULL(planner_status);
  const Vec2d vel(pose.vel_smooth().x(), pose.vel_smooth().y());
  constexpr double kEnterStandbySpeedThreshold = 0.1;  // m/s.
  if (std::abs(vel.norm()) > kEnterStandbySpeedThreshold) {
    const std::string message =
        "AV not stop, refuse to enter standby state, route msg is necessary.";
    planner_status->set_status(PlannerStatusProto::ROUTE_MSG_UNAVAILABLE);
    planner_status->set_message(message);
    return absl::NotFoundError(message);
  }

  return absl::OkStatus();
}

}  // namespace

absl::Status EnterStandbyState(const PoseProto &pose,
                               const VehicleGeometryParamsProto &vehicle_geom,
                               TrajectoryProto *trajectory,
                               PlannerDebugProto *planner_debug) {
  QCHECK_NOTNULL(trajectory);
  QCHECK_NOTNULL(planner_debug);

  RETURN_IF_ERROR(ValidateStandbyPrecondition(
      pose, planner_debug->mutable_planner_status()));

  PathPoint plan_start_path_point;
  plan_start_path_point.set_x(pose.pos_smooth().x());
  plan_start_path_point.set_y(pose.pos_smooth().y());
  plan_start_path_point.set_z(pose.pos_smooth().z());
  plan_start_path_point.set_theta(pose.yaw());
  // Generate standby trajectory.
  std::vector<ApolloTrajectoryPointProto> standby_trajectory;
  standby_trajectory.reserve(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = plan_start_path_point;
    traj_point.set_relative_time(i * kTrajectoryTimeStep);
    standby_trajectory.push_back(std::move(traj_point));
  }
  std::vector<ApolloTrajectoryPointProto> standby_past_trajectory;
  standby_past_trajectory.reserve(kMaxPastPointNum);
  for (int i = 1, n = kMaxPastPointNum + 1; i < n; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = plan_start_path_point;
    traj_point.set_relative_time(-i * kTrajectoryTimeStep);
    standby_past_trajectory.push_back(std::move(traj_point));
  }
  std::reverse(standby_past_trajectory.begin(), standby_past_trajectory.end());

  FillTrajectoryProto(FromUnixDoubleSeconds(pose.timestamp()),
                      standby_trajectory, standby_past_trajectory, trajectory);
  // For visualization only.
  AddConstraint(vehicle_geom, pose,
                planner_debug->add_est_planner_debugs()->mutable_constraint());
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
