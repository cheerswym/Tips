#include "onboard/control/trajectory_interface.h"

#include "gtest/gtest.h"
#include "onboard/control/control_defs.h"
#include "onboard/math/vec.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kScenarioStartTime = 100;  // s.
constexpr bool kOnTrasitionTraj = true;
constexpr double kPlannerInterval = 0.2;  // s.

TrajectoryProto MakeTrajectoryProto(
    double trajectory_start_timestamp,
    const std::vector<ApolloTrajectoryPointProto> &traj_point,
    const std::vector<ApolloTrajectoryPointProto> &past_point) {
  TrajectoryProto trajectory_proto;
  trajectory_proto.set_trajectory_start_timestamp(trajectory_start_timestamp);

  for (int i = 0; i < traj_point.size(); ++i) {
    *trajectory_proto.add_trajectory_point() = traj_point[i];
  }

  for (int i = 0; i < past_point.size(); ++i) {
    *trajectory_proto.add_past_points() = past_point[i];
  }

  trajectory_proto.set_gear(Chassis::GEAR_DRIVE);

  return trajectory_proto;
}

AutonomyStateProto MakeAutonomyStateProto(
    AutonomyStateProto::State autonomy_state) {
  AutonomyStateProto autonomy_state_proto;
  autonomy_state_proto.set_autonomy_state(autonomy_state);
  return autonomy_state_proto;
}

ApolloTrajectoryPointProto GenerateTrajectoryPoint(Vec2d xy, double s,
                                                   double theta, double v,
                                                   double relative_time) {
  ApolloTrajectoryPointProto apollo_traj_point;
  apollo_traj_point.mutable_path_point()->set_x(xy.x());
  apollo_traj_point.mutable_path_point()->set_y(xy.y());
  apollo_traj_point.mutable_path_point()->set_s(s);
  apollo_traj_point.mutable_path_point()->set_theta(theta);
  apollo_traj_point.set_v(v);
  apollo_traj_point.set_relative_time(relative_time);

  return apollo_traj_point;
}

TrajectoryProto GenerateTrajectory(double trajectory_start_timestamp,
                                   const Vec2d &start_point, double theta,
                                   double v) {
  TrajectoryProto trajectory_proto;
  trajectory_proto.set_trajectory_start_timestamp(trajectory_start_timestamp);

  constexpr int kNumOfTrajPoint = 30;
  constexpr int kNumOfPastPoint = 15;

  *trajectory_proto.add_trajectory_point() = GenerateTrajectoryPoint(
      start_point, /* s = */ 0.0, theta, v, /* relative_time = */ 0.0);

  for (int i = 1; i < kNumOfTrajPoint; ++i) {
    const double s = v * kTrajPointInterval * i;
    Vec2d traj_point_xy = start_point + Vec2d::UnitFromAngle(theta) * s;
    const double relative_time = i * kTrajPointInterval;

    *trajectory_proto.add_trajectory_point() =
        GenerateTrajectoryPoint(traj_point_xy, s, theta, v, relative_time);
  }

  for (int i = -kNumOfPastPoint; i < 0; ++i) {
    const double s_past = v * i * kTrajPointInterval;
    const Vec2d past_point_xy =
        start_point - Vec2d::UnitFromAngle(theta) * s_past;
    const double relative_time = i * kTrajPointInterval;

    *trajectory_proto.add_past_points() =
        GenerateTrajectoryPoint(past_point_xy, s_past, theta, v, relative_time);
  }

  trajectory_proto.set_gear(Chassis::GEAR_DRIVE);

  return trajectory_proto;
}

TEST(TrajectoryInterfaceTest, TransitionTrajStartTime) {
  ApolloTrajectoryPointProto empty_point =
      GenerateTrajectoryPoint({.0, .0}, .0, .0, .0, .0);
  constexpr int kTrajPointNum = 10;
  const std::vector<ApolloTrajectoryPointProto> empty_traj_point{kTrajPointNum,
                                                                 empty_point};
  ControllerDebugProto debug_proto;

  TrajectoryInterface trajectory_interface;
  TrajectoryProto init_trajectory_proto = MakeTrajectoryProto(
      kScenarioStartTime, empty_traj_point, empty_traj_point);
  AutonomyStateProto autonomy_state_proto =
      MakeAutonomyStateProto(AutonomyStateProto::AUTO_DRIVE);

  auto status = trajectory_interface.Update(
      autonomy_state_proto, init_trajectory_proto, &debug_proto);
  EXPECT_TRUE(status.ok());

  for (int i = 0; i < 5; ++i) {
    const double planner_start_time = kScenarioStartTime + kPlannerInterval * i;
    TrajectoryProto trajectory_proto = MakeTrajectoryProto(
        planner_start_time, empty_traj_point, empty_traj_point);

    for (int j = 0; j < 20; ++j) {
      status = trajectory_interface.Update(autonomy_state_proto,
                                           trajectory_proto, &debug_proto);
      EXPECT_TRUE(status.ok());
      if (i > 1) {
        const double expected_transition_start_time =
            kScenarioStartTime + (i - 1) * kPlannerInterval +
            std::min(TrajectoryInterface::kTimeIncrement * (j + 1),
                     kPlannerInterval);
        const double transition_trajectory_start_time =
            trajectory_interface.GetPlannerStartTime(kOnTrasitionTraj);

        EXPECT_NEAR(transition_trajectory_start_time,
                    expected_transition_start_time, kEpsilon)
            << "i = " << i << ", j = " << j
            << ", t = " << transition_trajectory_start_time
            << ", t_e = " << expected_transition_start_time;
      }
    }
  }
}

TEST(TrajectoryInterfaceTest, ReceiveStationaryTrajectory) {
  TrajectoryInterface trajectory_interface;
  ControllerDebugProto debug_proto;
  AutonomyStateProto autonomy_state_proto =
      MakeAutonomyStateProto(AutonomyStateProto::AUTO_DRIVE);

  const Vec2d kStartPoint{100, 200};
  const double kTheta = M_PI;
  const double kSpeed = 0;

  for (int i = 0; i < 10; ++i) {
    const TrajectoryProto trajectory_proto = GenerateTrajectory(
        kScenarioStartTime + i * kPlannerInterval, kStartPoint, kTheta, kSpeed);
    for (int j = 0; j < 20; ++j) {
      const auto status = trajectory_interface.Update(
          autonomy_state_proto, trajectory_proto, &debug_proto);
      EXPECT_TRUE(status.ok());
      for (const auto &p :
           trajectory_interface.GetAllTrajPoints(kOnTrasitionTraj)) {
        const Vec2d xy{p.path_point().x(), p.path_point().y()};
        EXPECT_EQ(xy.DistanceTo(kStartPoint), 0.0);
      }
    }
  }
}

}  // namespace
}  // namespace qcraft::control
