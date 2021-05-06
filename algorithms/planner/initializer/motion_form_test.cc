#include "onboard/planner/initializer/motion_form.h"

#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"

namespace qcraft::planner {
namespace {

constexpr double kEpsilon = 1e-1;

TEST(MotionForm, ConstAccelMotion) {
  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const ConstAccelMotion motion(/*init_v=*/1.0, /*init_a=*/1.0, &line);
  EXPECT_NEAR(motion.duration(), 2.0, kEpsilon);

  const std::vector<MotionState> states = motion.FastSample(0.2);
  EXPECT_EQ(states.size(), 11);

  EXPECT_THAT(states.front().xy, Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(states.front().v, 1.0);
  EXPECT_EQ(states.front().a, 1.0);
  EXPECT_EQ(states.front().t, 0.0);

  EXPECT_THAT(states[5].xy, Vec2dNearXY(1.5, 0.0, kEpsilon));
  EXPECT_THAT(states[5].t, 1.0);
  EXPECT_THAT(states[5].a, 1.0);
  EXPECT_NEAR(states[5].v, 2.0, kEpsilon);

  EXPECT_THAT(states.back().xy, Vec2dEqXY(4.0, 0.0));
  EXPECT_NEAR(states.back().v, 3.0, kEpsilon);
  EXPECT_EQ(states.back().a, 1.0);
  EXPECT_EQ(states.back().t, 2.0);
}

TEST(MotionForm, ConstAccelStopMotion) {
  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const ConstAccelMotion motion(/*init_v=*/1.0, /*init_a=*/-1.0, &line);
  EXPECT_NEAR(motion.duration(), 10.0, kEpsilon);

  const std::vector<MotionState> states = motion.FastSample(0.2);
  EXPECT_EQ(states.size(), 51);

  EXPECT_THAT(states.front().xy, Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(states.front().v, 1.0);
  EXPECT_EQ(states.front().a, -1.0);
  EXPECT_EQ(states.front().t, 0.0);

  EXPECT_THAT(states[1].xy, Vec2dNearXY(0.18, 0.0, kEpsilon));
  EXPECT_THAT(states[1].t, 0.2);
  EXPECT_THAT(states[1].a, -1.0);
  EXPECT_NEAR(states[1].v, 0.8, kEpsilon);

  EXPECT_THAT(states[50].xy, Vec2dNearXY(0.5, 0.0, kEpsilon));
  EXPECT_THAT(states[50].t, 10.0);
  EXPECT_THAT(states[50].a, 0.0);
  EXPECT_NEAR(states[50].v, 0.0, kEpsilon);

  auto stop_state = motion.State(/*t=*/8.0);
  EXPECT_THAT(stop_state.xy, Vec2dNearXY(0.5, 0.0, kEpsilon));
  EXPECT_THAT(stop_state.t, 8.0);
  EXPECT_THAT(stop_state.a, 0.0);
  EXPECT_NEAR(stop_state.v, 0.0, kEpsilon);

  stop_state = motion.State(/*t=*/0.2);
  EXPECT_THAT(stop_state.xy, Vec2dNearXY(0.18, 0.0, kEpsilon));
  EXPECT_THAT(stop_state.t, 0.2);
  EXPECT_THAT(stop_state.a, -1.0);
  EXPECT_NEAR(stop_state.v, 0.8, kEpsilon);
}

TEST(MotionForm, StationaryMotion) {
  const GeometryState state({.xy = Vec2d(1.0, 2.0), .h = 3.0, .k = 4.0});
  const StationaryGeometry geometry(state);
  const StationaryMotion motion(4.0, &geometry);
  EXPECT_EQ(motion.duration(), 4.0);

  const std::vector<MotionState> states = motion.FastSample(0.2);
  EXPECT_EQ(states.size(), 21);

  EXPECT_THAT(states.front().xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(states.front().v, 0.0);
  EXPECT_EQ(states.front().a, 0.0);
  EXPECT_EQ(states.front().t, 0.0);

  EXPECT_THAT(states.back().xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_NEAR(states.back().v, 0.0, kEpsilon);
  EXPECT_EQ(states.back().a, 0.0);
  EXPECT_EQ(states.back().t, 4.0);
}

}  // namespace
}  // namespace qcraft::planner
