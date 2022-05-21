#include "onboard/emergency_brake/util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace emergency_brake {

namespace {
constexpr double kEpsilon = 1e-2;

TEST(ComputeCircularDetectionPolygon, ForwardLeftTurn) {
  const Vec2d av_center(1.0, 100.0);
  const Box2d av_box(av_center, /*heading=*/0.0, /*length=*/4.0, /*width=*/2.0);
  const double rac_to_center = 1.0;
  const double drive_distance = 0.1;
  const double signed_circular_radius = 99.0;

  ASSIGN_OR_DIE(
      const Polygon2d polygon1,
      ComputeCircularDetectionPolygon(av_box, rac_to_center, drive_distance,
                                      signed_circular_radius));
  EXPECT_EQ(polygon1.num_points(), 4);

  for (const auto& pt : polygon1.points()) {
    ASSERT_GE(pt.x(), 3.0 - kEpsilon);
    ASSERT_LE(pt.x(), 3.1 + kEpsilon);
    ASSERT_GE(pt.y(), 99.0 - kEpsilon);
    ASSERT_LE(pt.y(), 101.1 + kEpsilon);
  }
}

TEST(ComputeCircularDetectionPolygon, ForwardRightTurn) {
  const Vec2d av_center(1.0, 100.0);
  const Box2d av_box(av_center, /*heading=*/0.0, /*length=*/4.0,
                     /*width=*/2.0);
  const double rac_to_center = 1.0;
  const double drive_distance = 0.1;
  const double signed_circular_radius = -99.0;

  ASSIGN_OR_DIE(
      const Polygon2d polygon1,
      ComputeCircularDetectionPolygon(av_box, rac_to_center, drive_distance,
                                      signed_circular_radius));
  EXPECT_EQ(polygon1.num_points(), 4);

  for (const auto& pt : polygon1.points()) {
    ASSERT_GE(pt.x(), 3.0 - kEpsilon);
    ASSERT_LE(pt.x(), 3.1 + kEpsilon);
    ASSERT_LE(pt.y(), 101.0 + kEpsilon);
    ASSERT_GE(pt.y(), 98.9 - kEpsilon);
  }
}

TEST(ComputeCircularDetectionPolygon, BackwardLeftTurn) {
  const Vec2d av_center(1.0, 100.0);
  const Box2d av_box(av_center, /*heading=*/0.0, /*length=*/4.0, /*width=*/2.0);
  const double rac_to_center = 1.0;
  const double drive_distance = -0.1;
  const double signed_circular_radius = 99.0;

  ASSIGN_OR_DIE(
      const Polygon2d polygon1,
      ComputeCircularDetectionPolygon(av_box, rac_to_center, drive_distance,
                                      signed_circular_radius));
  EXPECT_EQ(polygon1.num_points(), 4);

  for (const auto& pt : polygon1.points()) {
    ASSERT_LE(pt.x(), -1.0 + kEpsilon);
    ASSERT_GE(pt.x(), -1.1 - kEpsilon);
    ASSERT_LE(pt.y(), 101.1 + kEpsilon);
    ASSERT_GE(pt.y(), 98.9 - kEpsilon);
  }
}

TEST(ComputeCircularDetectionPolygon, BackwardRightTurn) {
  const Vec2d av_center(1.0, 100.0);
  const Box2d av_box(av_center, /*heading=*/0.0, /*length=*/4.0,
                     /*width=*/2.0);
  const double rac_to_center = 1.0;
  const double drive_distance = -0.1;
  const double signed_circular_radius = -99.0;

  ASSIGN_OR_DIE(
      const Polygon2d polygon1,
      ComputeCircularDetectionPolygon(av_box, rac_to_center, drive_distance,
                                      signed_circular_radius));
  EXPECT_EQ(polygon1.num_points(), 4);

  for (const auto& pt : polygon1.points()) {
    ASSERT_LE(pt.x(), -1.0 + kEpsilon);
    ASSERT_GE(pt.x(), -1.1 - kEpsilon);
    ASSERT_LE(pt.y(), 101.1 + kEpsilon);
    ASSERT_GE(pt.y(), 99.0 - kEpsilon);
  }
}

TEST(ComputeConstJerkCircularMotion, StraightLine) {
  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(2.0);
  start.set_j(0.0);
  start.set_relative_time(1.0);
  start.mutable_path_point()->set_x(100.0);
  start.mutable_path_point()->set_y(100.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(1.0);
  start.mutable_path_point()->set_lambda(0.0);

  const auto traj =
      ComputeConstJerkCircularMotion(start, /*dt=*/Seconds(0.1),
                                     /*duration=*/Seconds(2.0),
                                     /*jerk=*/MetersPerCubicSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  EXPECT_THAT(traj[10], ProtoPartialApproximatelyMatchesText(
                            R"(path_point {
                                x: 110.8333
                                y: 100.0
                                z: 0.0
                                kappa: 0.0
                                theta: 0.0
                                s: 11.8333
                              }
                              v : 11.5
                              a : 1.0
                              j : -1.0
                              relative_time: 2.0)",
                            1e-4));
}

TEST(ComputeConstJerkCircularMotion, Turning) {
  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(2.0);
  start.set_j(0.0);
  start.set_relative_time(1.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(100.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(M_PI_2);
  start.mutable_path_point()->set_s(1.0);
  start.mutable_path_point()->set_lambda(0.0);

  // Left turning trajectory.
  start.mutable_path_point()->set_kappa(0.02);
  const auto left_traj = ComputeConstJerkCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0), /*jerk=*/MetersPerCubicSecond(-1.0));

  ASSERT_GE(left_traj.size(), 20);
  EXPECT_EQ(left_traj[0], start);
  EXPECT_THAT(left_traj[10], ProtoPartialApproximatelyMatchesText(
                                 R"(path_point {
                                   x: -1.169
                                   y: 110.748
                                   theta: 1.7874
                                   kappa: 0.02
                                   s: 11.8333
                                 }
                                 v : 11.5
                                 a : 1.0
                                 j : -1.0
                                 relative_time: 2.0)",
                                 1e-3));

  // Right turning trajectory.
  start.mutable_path_point()->set_kappa(-0.02);
  const auto right_traj = ComputeConstJerkCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0), /*jerk=*/MetersPerCubicSecond(-1.0));
  ASSERT_GE(right_traj.size(), 20);
  EXPECT_EQ(right_traj[0], start);
  EXPECT_THAT(right_traj[10], ProtoPartialApproximatelyMatchesText(
                                  R"(path_point {
                                    x: 1.169
                                    y: 110.748
                                    theta: 1.354
                                    kappa: -0.02
                                    s: 11.8333
                                  }
                                  v : 11.5
                                  a : 1.0
                                  j : -1.0
                                  relative_time: 2.0)",
                                  1e-3));
}

TEST(ComputeConstAccelerationCircularMotion, StraightLine) {
  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(2.0);
  start.set_j(0.0);
  start.set_relative_time(1.0);
  start.mutable_path_point()->set_x(100.0);
  start.mutable_path_point()->set_y(100.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(1.0);
  start.mutable_path_point()->set_lambda(0.0);

  const auto traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0),
      /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  EXPECT_THAT(traj[10], ProtoPartialApproximatelyMatchesText(
                            R"(path_point {
                                x: 109.5
                                y: 100.0
                                z: 0.0
                                kappa: 0.0
                                theta: 0.0
                                s: 10.5
                              }
                              v : 9.0
                              a : -1.0
                              j : 0.0
                              relative_time: 2.0)",
                            1e-4));
}

TEST(ComputeConstAccelerationCircularMotion, Turning) {
  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(2.0);
  start.set_j(0.0);
  start.set_relative_time(1.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(100.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(M_PI_2);
  start.mutable_path_point()->set_s(1.0);
  start.mutable_path_point()->set_lambda(0.0);

  // Left turning trajectory.
  start.mutable_path_point()->set_kappa(0.02);
  const auto left_traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0), /*accel=*/MetersPerSquaredSecond(-1.0));

  ASSERT_GE(left_traj.size(), 20);
  EXPECT_EQ(left_traj[0], start);
  EXPECT_THAT(left_traj[10], ProtoPartialApproximatelyMatchesText(
                                 R"(path_point {
                                   x: -0.899
                                   y: 109.443
                                   theta: 1.761
                                   kappa: 0.02
                                   s: 10.5
                                 }
                                 v : 9.0
                                 a : -1.0
                                 j : 0.0
                                 relative_time: 2.0)",
                                 1e-3));

  // Right turning trajectory.
  start.mutable_path_point()->set_kappa(-0.02);
  const auto right_traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0), /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(right_traj.size(), 20);
  EXPECT_EQ(right_traj[0], start);
  EXPECT_THAT(right_traj[10], ProtoPartialApproximatelyMatchesText(
                                  R"(path_point {
                                    x: 0.899
                                    y: 109.443
                                    theta: 1.381
                                    kappa: -0.02
                                    s: 10.5
                                  }
                                  v : 9.0
                                  a : -1.0
                                  j : 0.0
                                  relative_time: 2.0)",
                                  1e-3));
}

VehicleGeometryParamsProto vehicle_geom = planner::DefaultVehicleGeometry();
VehicleDriveParamsProto vehicle_drive_params =
    planner::DefaultVehicleDriveParams();

TEST(ComputeFenCollisions, WithCollision) {
  vehicle_drive_params.set_max_deceleration(-4.0);
  ApolloTrajectoryPointProto start;
  start.set_v(std::abs(vehicle_drive_params.max_deceleration()) * 2.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);
  const auto traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0),
      /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  double traj_start_time = 0.0;

  FenDetectionsProto fen_detections;
  fen_detections.set_timestamp(0.0);
  Box2dProto obs0_box;
  obs0_box.set_x(5);
  obs0_box.set_y(2.5);
  obs0_box.set_heading(0.0);
  obs0_box.set_width(1.0);
  obs0_box.set_length(1.0);
  double obs0_vx = 2;
  double obs0_vy = 0;
  auto* obs0 = fen_detections.add_detections();
  obs0->mutable_bounding_box()->CopyFrom(obs0_box);
  obs0->set_velocity_x(obs0_vx);
  obs0->set_velocity_y(obs0_vy);
  Box2dProto obs1_box;
  obs1_box.set_x(10);
  obs1_box.set_y(-7.5);
  obs1_box.set_heading(M_PI_2);
  obs1_box.set_width(1.0);
  obs1_box.set_length(1.0);
  double obs1_vx = 0;
  double obs1_vy = 10;
  auto* obs1 = fen_detections.add_detections();
  obs1->mutable_bounding_box()->CopyFrom(obs1_box);
  obs1->set_velocity_x(obs1_vx);
  obs1->set_velocity_y(obs1_vy);

  auto res = ComputeFenCollisions(traj, traj_start_time, vehicle_geom,
                                  vehicle_drive_params, fen_detections);

  EXPECT_EQ(res, *obs1);
}

TEST(ComputeFenCollisions, WithoutCollision) {
  vehicle_drive_params.set_max_deceleration(-4.0);
  ApolloTrajectoryPointProto start;
  start.set_v(std::abs(vehicle_drive_params.max_deceleration()) * 2.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);
  const auto traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0),
      /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  double traj_start_time = 0.0;

  FenDetectionsProto fen_detections;
  fen_detections.set_timestamp(0.0);
  Box2dProto obs0_box;
  obs0_box.set_x(5);
  obs0_box.set_y(2.5);
  obs0_box.set_heading(0.0);
  obs0_box.set_width(1.0);
  obs0_box.set_length(1.0);
  double obs0_vx = 2;
  double obs0_vy = 0;
  auto* obs0 = fen_detections.add_detections();
  obs0->mutable_bounding_box()->CopyFrom(obs0_box);
  obs0->set_velocity_x(obs0_vx);
  obs0->set_velocity_y(obs0_vy);
  Box2dProto obs1_box;
  obs1_box.set_x(10);
  obs1_box.set_y(-5);
  obs1_box.set_heading(M_PI_2);
  obs1_box.set_width(1.0);
  obs1_box.set_length(1.0);
  double obs1_vx = 0;
  double obs1_vy = 2;
  auto* obs1 = fen_detections.add_detections();
  obs1->mutable_bounding_box()->CopyFrom(obs1_box);
  obs1->set_velocity_x(obs1_vx);
  obs1->set_velocity_y(obs1_vy);

  auto res = ComputeFenCollisions(traj, traj_start_time, vehicle_geom,
                                  vehicle_drive_params, fen_detections);

  EXPECT_EQ(res, std::nullopt);
}

TEST(ComputeFenCollisions, PoseMsgDelayed) {
  vehicle_drive_params.set_max_deceleration(-4.0);
  ApolloTrajectoryPointProto start;
  start.set_v(std::abs(vehicle_drive_params.max_deceleration()) * 2.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);
  const auto traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0),
      /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  double traj_start_time = 0.0;

  FenDetectionsProto fen_detections;
  fen_detections.set_timestamp(3.0);
  Box2dProto obs0_box;
  obs0_box.set_x(5);
  obs0_box.set_y(2.5);
  obs0_box.set_heading(0.0);
  obs0_box.set_width(1.0);
  obs0_box.set_length(1.0);
  double obs0_vx = 2;
  double obs0_vy = 0;
  auto* obs0 = fen_detections.add_detections();
  obs0->mutable_bounding_box()->CopyFrom(obs0_box);
  obs0->set_velocity_x(obs0_vx);
  obs0->set_velocity_y(obs0_vy);
  Box2dProto obs1_box;
  obs1_box.set_x(10);
  obs1_box.set_y(-7.5);
  obs1_box.set_heading(M_PI_2);
  obs1_box.set_width(1.0);
  obs1_box.set_length(1.0);
  double obs1_vx = 0;
  double obs1_vy = 10;
  auto* obs1 = fen_detections.add_detections();
  obs1->mutable_bounding_box()->CopyFrom(obs1_box);
  obs1->set_velocity_x(obs1_vx);
  obs1->set_velocity_y(obs1_vy);

  auto res = ComputeFenCollisions(traj, traj_start_time, vehicle_geom,
                                  vehicle_drive_params, fen_detections);

  EXPECT_EQ(res, std::nullopt);
}

TEST(ComputeFenCollisions, FenMsgDelayed) {
  vehicle_drive_params.set_max_deceleration(-4.0);
  ApolloTrajectoryPointProto start;
  start.set_v(std::abs(vehicle_drive_params.max_deceleration()) * 2.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);
  const auto traj = ComputeConstAccelerationCircularMotion(
      start, /*dt=*/Seconds(0.1),
      /*duration=*/Seconds(2.0),
      /*accel=*/MetersPerSquaredSecond(-1.0));
  ASSERT_GE(traj.size(), 20);
  EXPECT_EQ(traj[0], start);

  double traj_start_time = 1.0;

  FenDetectionsProto fen_detections;
  fen_detections.set_timestamp(0.0);
  Box2dProto obs0_box;
  obs0_box.set_x(5);
  obs0_box.set_y(2.5);
  obs0_box.set_heading(0.0);
  obs0_box.set_width(1.0);
  obs0_box.set_length(1.0);
  double obs0_vx = 2;
  double obs0_vy = 0;
  auto* obs0 = fen_detections.add_detections();
  obs0->mutable_bounding_box()->CopyFrom(obs0_box);
  obs0->set_velocity_x(obs0_vx);
  obs0->set_velocity_y(obs0_vy);
  Box2dProto obs1_box;
  obs1_box.set_x(10);
  obs1_box.set_y(-7.5);
  obs1_box.set_heading(M_PI_2);
  obs1_box.set_width(1.0);
  obs1_box.set_length(1.0);
  double obs1_vx = 0;
  double obs1_vy = 10;
  auto* obs1 = fen_detections.add_detections();
  obs1->mutable_bounding_box()->CopyFrom(obs1_box);
  obs1->set_velocity_x(obs1_vx);
  obs1->set_velocity_y(obs1_vy);

  auto res = ComputeFenCollisions(traj, traj_start_time, vehicle_geom,
                                  vehicle_drive_params, fen_detections);

  EXPECT_EQ(res, std::nullopt);
}

}  // namespace
}  // namespace emergency_brake
}  // namespace qcraft
