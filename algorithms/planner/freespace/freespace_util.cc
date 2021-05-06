#include "onboard/planner/freespace/freespace_util.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/utils/status_macros.h"

// TODO(all): This flag should be positive. But for the convenience of testing,
// set it to a negative value.
DEFINE_double(front_edge_to_parking_spot_distance_buffer, -1.0,
              "Front edge to parking spot distance buffer.");

namespace qcraft::planner {

namespace {

absl::Status IsGoalLegal(const VehicleGeometryParamsProto &vehicle_geom,
                         const PlannerObjectManager &object_mgr,
                         const PathPoint &goal) {
  constexpr double kLonBuffer = 0.15;  // m.
  constexpr double kLatBuffer = 0.15;  // m.
  const auto pos = Vec2d(goal.x(), goal.y());
  const auto tangent = Vec2d::FastUnitFromAngle(goal.theta());
  const auto start =
      pos + tangent * (vehicle_geom.front_edge_to_center() + kLonBuffer);
  const auto end =
      pos - tangent * (vehicle_geom.back_edge_to_center() + kLonBuffer);
  const auto parked_av_box =
      Box2d(Segment2d(start, end), vehicle_geom.width() + kLatBuffer * 2.0);

  for (const auto stat_obj : object_mgr.stationary_objects()) {
    if (stat_obj->contour().HasOverlap(parked_av_box)) {
      return absl::InternalError(absl::StrFormat(
          "Stationary object: %s has overlap with AV destination, "
          "generate goal failed.",
          stat_obj->id()));
    }
  }

  return absl::OkStatus();
}

}  // namespace

// Adjust goal if parked vehicle back edge too close to parking spot. There is a
// potential collision with curb risk. Move goal away from risky side to
// mitigate this problem.
// For intuitive description, refer to:
// https://drive.google.com/file/d/1uHOxCNgydhmj7THY4NYO0iCV2_nMkXzx/view?usp=sharing
absl::StatusOr<Vec2d> MaybeAdjustGoalForTightParkingSpot(
    const VehicleGeometryParamsProto &vehicle_geom,
    const mapping::ParkingSpotInfo &parking_spot_info, const Vec2d &goal_pos) {
  // Only consider vertical parking spots whose entering side is on the front.
  // TODO(yumeng): Adjust goal for parallel parking spot.
  // TODO(yumeng): Only adjust if curb exist.
  Vec2d adjusted_goal_pos(goal_pos);
  if (parking_spot_info.entering_sides_size() == 1 &&
      parking_spot_info.IsSideEnterable(mapping::ParkingSpotInfo::FRONT)) {
    const auto tangent = parking_spot_info.unit_direction();

    const double av_back_to_parking_spot_rear =
        parking_spot_info.rear_edge().DistanceTo(goal_pos) -
        vehicle_geom.back_edge_to_center();

    constexpr double kBackEdgeToParkingSpotDistanceBuffer = 0.2;  // m.
    const double allow_rac_move_forward_dist =
        (parking_spot_info.length() - vehicle_geom.length()) * 0.5;
    const double expected_rac_move_forward_dist = std::max(
        kBackEdgeToParkingSpotDistanceBuffer - av_back_to_parking_spot_rear,
        0.0);

    // TODO(all): Adjust this value, should be greater equal than 0.0.
    const double kFrontEdgeToParkingSpotDistanceBuffer =
        FLAGS_front_edge_to_parking_spot_distance_buffer;

    if (expected_rac_move_forward_dist <
        allow_rac_move_forward_dist - kFrontEdgeToParkingSpotDistanceBuffer) {
      adjusted_goal_pos = goal_pos + tangent * expected_rac_move_forward_dist;
    } else {
      return absl::InternalError(absl::StrCat(
          "Adjust goal faild, current av back edge to parking spot distance: ",
          av_back_to_parking_spot_rear));
    }
  }
  return adjusted_goal_pos;
}

absl::StatusOr<PathPoint> GenerateGoalFromParkingSpot(
    const VehicleGeometryParamsProto &vehicle_geom,
    const mapping::ParkingSpotInfo &parking_spot_info,
    const PlannerObjectManager &object_mgr) {
  const auto tangent = parking_spot_info.unit_direction();
  const auto parking_spot_centroid = parking_spot_info.polygon().centroid();
  const double rac_to_center =
      vehicle_geom.front_edge_to_center() - vehicle_geom.length() * 0.5;

  const Vec2d center_pos = parking_spot_centroid - tangent * rac_to_center;
  ASSIGN_OR_RETURN(auto goal_pos,
                   MaybeAdjustGoalForTightParkingSpot(
                       vehicle_geom, parking_spot_info, center_pos));
  // TOOD(yumeng): Wrap as a function.
  if (parking_spot_info.stopper().has_value()) {
    auto stopper = *parking_spot_info.stopper();
    // Make sure stopper direction points to the left of car heading direction.
    if (tangent.CrossProd(stopper.unit_direction()) < 0.0) {
      stopper.Reverse();
    }
    constexpr double kRearAxleToStopperMinDistance = 0.3;  // m.
    const double signed_dist_to_goal = stopper.SignedDistanceTo(goal_pos);
    // If rear axle exceed stopper boundary, move goal_pos ahead.
    if (signed_dist_to_goal < kRearAxleToStopperMinDistance) {
      goal_pos +=
          tangent * (kRearAxleToStopperMinDistance - signed_dist_to_goal);
    }
  }
  PathPoint goal;
  goal.set_x(goal_pos.x());
  goal.set_y(goal_pos.y());
  goal.set_theta(tangent.Angle());

  RETURN_IF_ERROR(IsGoalLegal(vehicle_geom, object_mgr, goal));

  return goal;
}

PathPoint RecoverGoalFromState(
    const CoordinateConverter &coordinate_converter,
    const PathManagerStateProto::GlobalGoal &global_goal) {
  PathPoint goal;
  const auto pos =
      coordinate_converter.GlobalToSmooth(Vec2d(global_goal.pos()));
  goal.set_x(pos.x());
  goal.set_y(pos.y());
  const double smooth_theta =
      coordinate_converter.GlobalYawToSmoothNoNormalize(global_goal.theta());
  goal.set_theta(smooth_theta);
  return goal;
}

std::pair<PathPoint, PathManagerStateProto::GlobalGoal>
ConvertGlobalPoseToPathPoint(const GlobalPose &pose,
                             const CoordinateConverter &cc) {
  PathManagerStateProto::GlobalGoal global_goal;
  global_goal.mutable_pos()->set_x(pose.pos.x());
  global_goal.mutable_pos()->set_y(pose.pos.y());
  global_goal.set_theta(pose.heading);

  PathPoint smooth_path_point;
  const Vec2d smooth_pos = cc.GlobalToSmooth(Vec2d(pose.pos.x(), pose.pos.y()));
  smooth_path_point.set_x(smooth_pos.x());
  smooth_path_point.set_y(smooth_pos.y());
  smooth_path_point.set_theta(cc.GlobalYawToSmoothNoNormalize(pose.heading));

  return std::make_pair(std::move(smooth_path_point), std::move(global_goal));
}

TrajectoryProto CreateFreespaceTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto> &planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto> &past_points,
    const Chassis::GearPosition &gear_position,
    const DrivingStateProto &driving_state, bool low_speed_freespace) {
  TrajectoryProto trajectory;
  trajectory.set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory.add_trajectory_point() = planned_trajectory[i];
  }

  // NOTE: past_points are designed for controller.
  for (const auto &past_point : past_points) {
    *trajectory.add_past_points() = past_point;
  }

  // TODO(renjie): redesign it after onboard freespace planner.
  trajectory.set_gear(gear_position);

  trajectory.mutable_driving_state()->CopyFrom(driving_state);

  trajectory.set_low_speed_freespace(low_speed_freespace);

  return trajectory;
}

}  // namespace qcraft::planner
