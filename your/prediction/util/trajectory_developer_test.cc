#include "onboard/prediction/util/trajectory_developer.h"

#include "gtest/gtest.h"

namespace qcraft::prediction {
namespace {
constexpr double kEpsilon = 1e-3;
TEST(TrajectoryDeveloperTest, StaticTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopStaticTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_EQ(last_pt.pos(), Vec2d(0.0, 0.0));
  EXPECT_EQ(last_pt.s(), 0.0);
  EXPECT_EQ(last_pt.v(), 0.0);
}

TEST(TrajectoryDeveloperTest, CYCVTrajectoryAxisAlignedTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/false);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().y(), 0.0, kEpsilon);

  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, CYCVTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/false);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, CYCVReverseTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), -2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, DummyCTRATrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCTRATrajectory(state, /*dt=*/0.1, /*stop_time=*/3.0,
                                         /*horizon=*/3.0, /*use_acc=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, AccCTRATrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 0.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 1.0};
  const auto pts = DevelopCTRATrajectory(state, /*dt=*/0.1, /*stop_time=*/1.0,
                                         /*horizon=*/3.0, /*use_acc=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.4, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.4, 1e-1);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, AccCTRATrajectoryTest2) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 0.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 1.0};
  const auto pts = DevelopCTRATrajectory(state, /*dt=*/0.1, /*stop_time=*/3.0,
                                         /*horizon=*/3.0, /*use_acc=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.5 * 2.9 * 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 0.5 * 2.9 * 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, YawRateCTRATrajectoryTest2) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = M_PI / 2.9,
                            .acc = 0.0};
  const auto pts = DevelopCTRATrajectory(state, /*dt=*/0.1, /*stop_time=*/3.0,
                                         /*horizon=*/3.0, /*use_acc=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.0, kEpsilon);
  EXPECT_NEAR(last_pt.pos().y(), 2.0 * 2.9 / M_PI, 1e-2);
  EXPECT_NEAR(last_pt.s(), 2.9, 1e-2);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI, kEpsilon);
}

}  // namespace
}  // namespace qcraft::prediction
