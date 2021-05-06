#include "onboard/planner/test_util/util.h"

#include "onboard/utils/file_util.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {

PoseProto CreatePose(double timestamp, Vec2d pos, double heading,
                     Vec2d vel_body) {
  PoseProto pose;
  pose.set_timestamp(timestamp);

  pose.mutable_pos_smooth()->set_x(pos.x());
  pose.mutable_pos_smooth()->set_y(pos.y());
  pose.mutable_pos_smooth()->set_z(0.0);
  pose.set_yaw(heading);
  pose.set_speed(vel_body.norm());

  pose.mutable_vel_body()->set_x(vel_body.x());
  pose.mutable_vel_body()->set_y(vel_body.y());
  pose.mutable_vel_body()->set_z(0.0);

  const double sin_heading = std::sin(heading), cos_heading = std::cos(heading);
  pose.mutable_vel_smooth()->set_x(vel_body.x() * cos_heading -
                                   vel_body.y() * sin_heading);
  pose.mutable_vel_smooth()->set_y(vel_body.x() * sin_heading +
                                   vel_body.y() * cos_heading);
  pose.mutable_vel_smooth()->set_z(0.0);

  return pose;
}

ApolloTrajectoryPointProto ConvertToTrajPointProto(const PoseProto &pose) {
  ApolloTrajectoryPointProto traj_point;
  traj_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  traj_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  traj_point.mutable_path_point()->set_z(pose.pos_smooth().z());
  traj_point.mutable_path_point()->set_theta(pose.yaw());
  traj_point.mutable_path_point()->set_kappa(pose.curvature());
  traj_point.mutable_path_point()->set_lambda(0.0);
  traj_point.mutable_path_point()->set_s(0.0);
  traj_point.set_v(pose.vel_body().x());
  traj_point.set_a(
      std::hypot(pose.accel_smooth().x(), pose.accel_smooth().y()));
  traj_point.set_j(0.0);
  traj_point.set_relative_time(0.0);

  return traj_point;
}

VehicleGeometryParamsProto DefaultVehicleGeometry() {
  VehicleGeometryParamsProto geom;
  geom.set_front_edge_to_center(4.0);
  geom.set_back_edge_to_center(1.0);
  geom.set_left_edge_to_center(1.0);
  geom.set_right_edge_to_center(1.0);
  geom.set_length(5.0);
  geom.set_width(2.0);
  geom.set_height(2.2);
  geom.set_min_turn_radius(6.0);
  geom.set_wheel_base(3.0);
  geom.set_wheel_rolling_radius(0.3);
  return geom;
}

VehicleDriveParamsProto DefaultVehicleDriveParams() {
  VehicleDriveParamsProto drive;
  drive.set_max_steer_angle(8.0);
  drive.set_max_steer_angle_rate(7.0);
  drive.set_min_steer_angle_rate(0.0);
  drive.set_steer_ratio(16.0);
  drive.set_brake_deadzone(10.0);
  drive.set_throttle_deadzone(15.0);
  drive.set_wheel_drive_mode(FRONT_WHEEL_DRIVE);

  return drive;
}

PlannerParamsProto DefaultPlannerParams() {
  PlannerParamsProto default_planner_params;
  file_util::FileToProto("onboard/planner/params/planner_default_params.pb.txt",
                         &default_planner_params);
  // Fill default speed finder params into default planner params.
  SpeedFinderParamsProto default_speed_finder_params;
  file_util::FileToProto(
      "onboard/planner/params/speed_finder_default_params.pb.txt",
      &default_speed_finder_params);

  // Fill default hybrid_a_star params into default planner params.
  HybridAStarParamsProto default_hybrid_a_star_params;
  file_util::FileToProto(
      "onboard/planner/params/hybrid_a_star_default_params.pb.txt",
      &default_hybrid_a_star_params);

  // Fill default local_smoother params into default planner params.
  FreespaceLocalSmootherParamsProto default_local_smoother_params;
  file_util::FileToProto(
      "onboard/planner/params/local_smoother_default_params.pb.txt",
      &default_local_smoother_params);

  // Fill est planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_params());

  // Fill freespace planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_hybrid_a_star_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_hybrid_a_star_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_motion_constraint_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_hybrid_a_star_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_hybrid_a_star_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_motion_constraint_params());

  // Fill fallback planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_fallback_planner_params()
          ->mutable_speed_finder_params());
  return default_planner_params;
}

}  // namespace planner
}  // namespace qcraft
