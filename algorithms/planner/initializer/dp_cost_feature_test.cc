#include "onboard/planner/initializer/dp_cost_feature.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(DpAccelerationFeatureCost, Cost) {
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const DpAccelerationFeatureCost acc_cost(&planner_params);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const double duration = 2.0;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();
  const MotionState dummy_state = MotionState{.t = 0.0, .a = 1.0};
  edge_info.states.push_back(dummy_state);
  std::vector<double> cost(1);
  acc_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(
      cost[0],
      std::pow(dummy_state.a /
                   planner_params.motion_constraint_params().max_acceleration(),
               2) *
          duration,
      1e-4);

  const MotionState dummy_state2 = MotionState{.t = 0.5, .a = 2.0};
  edge_info.states.push_back(dummy_state2);
  acc_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(
      cost[0],
      std::pow(dummy_state.a /
                   planner_params.motion_constraint_params().max_acceleration(),
               2) *
          (dummy_state2.t - dummy_state.t),
      1e-4);
}

TEST(DpLaneBoundaryFeatureCost, Cost) {
  int n = 10;
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);

  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(s_vec, right_l, left_l, target_right_l,
                            target_left_l, right_xy, left_xy, target_right_xy,
                            target_left_xy);
  const auto vehicle_geom = DefaultVehicleGeometry();
  const double sdc_half_width = vehicle_geom.width() * 0.5;
  const DpLaneBoundaryFeatureCost lb_cost(&path_bound, sdc_half_width);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const double duration = 2.0;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();
  const MotionState dummy_state = MotionState{.t = 0.0, .l = 1.0};
  edge_info.states.push_back(dummy_state);
  std::vector<double> cost(2);
  lb_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0],
              std::pow((0.75 - (2.0 - dummy_state.l - sdc_half_width)) /
                           kDefaultHalfLaneWidth,
                       2) *
                  2.0 * duration,
              1e-4);
  ASSERT_NEAR(
      cost[1],
      std::pow(std::clamp(dummy_state.l / kDefaultHalfLaneWidth, -1.0, 1.0),
               2) *
          duration,
      1e-4);

  const MotionState dummy_state2 = MotionState{.t = 0.5, .s = 0.0, .l = 2.0};
  edge_info.states.clear();
  edge_info.states.push_back(dummy_state2);
  lb_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0],
              std::pow((0.75 - (2.0 - dummy_state2.l - sdc_half_width)) /
                           kDefaultHalfLaneWidth,
                       2) *
                  2.0 * duration,
              1e-4);
  ASSERT_NEAR(
      cost[1],
      std::pow(std::clamp(dummy_state2.l / kDefaultHalfLaneWidth, -1.0, 1.0),
               2) *
          duration,
      1e-4);

  // Narrower target boundary case.
  target_left_l.clear();
  target_right_l.clear();
  target_left_xy.clear();
  target_right_xy.clear();

  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    target_left_l.emplace_back(1.5);
    target_right_l.emplace_back(-1.5);
    target_left_xy.emplace_back(s, 1.5);
    target_right_xy.emplace_back(s, -1.5);
  }

  PathSlBoundary path_bound2(std::move(s_vec), std::move(right_l),
                             std::move(left_l), std::move(target_right_l),
                             std::move(target_left_l), std::move(right_xy),
                             std::move(left_xy), std::move(target_right_xy),
                             std::move(target_left_xy));
  const DpLaneBoundaryFeatureCost lb_cost2(&path_bound2, sdc_half_width);

  const MotionState dummy_state3 = MotionState{.t = 0.5, .s = 0.0, .l = 1.75};
  edge_info.states.clear();
  edge_info.states.push_back(dummy_state3);
  lb_cost2.ComputeCost(edge_info, absl::MakeSpan(cost));

  ASSERT_NEAR(cost[0],
              std::pow((0.75 - (2.0 - dummy_state3.l - sdc_half_width)) /
                           kDefaultHalfLaneWidth,
                       2) *
                      duration +
                  1.0 * duration,
              1e-4);

  ASSERT_NEAR(
      cost[1],
      std::pow(std::clamp(dummy_state3.l / kDefaultHalfLaneWidth, -1.0, 1.0),
               2) *
          duration,
      1e-4);
}

TEST(DpLateralAccelerationFeatureCost, Cost) {
  const DpLateralAccelerationFeatureCost lat_cost(false);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const MotionState dummy_state = MotionState{.k = 0.2, .t = 0.0, .v = 1.0};
  edge_info.states.push_back(dummy_state);
  std::vector<double> cost(3);
  const double duration = 2.0;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();

  lat_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 0.0, 1e-4);
  ASSERT_NEAR(cost[1], 0.0, 1e-4);

  const MotionState dummy_state2 = MotionState{.k = 0.2, .t = 0.5, .v = 1.0};
  edge_info.states.push_back(dummy_state2);
  lat_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[1], 0, 1e-4);
}

TEST(DpStopConstraintFeatureCost, Cost) {
  const std::vector<double> stop_s = {10.0};
  const DpStopConstraintFeatureCost stop_cost(nullptr, stop_s);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  const double duration = 2.0;
  GeometryState fake_state({.xy = Vec2d(0.0, 0.0)});
  const auto motion_form =
      std::make_unique<StationaryMotion>(duration, fake_state);
  edge_info.motion_form = motion_form.get();

  const MotionState dummy_state = MotionState{.accumulated_s = 9};
  edge_info.states.push_back(dummy_state);
  std::vector<double> cost(1);
  stop_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 0, 1e-4);

  const MotionState dummy_state2 = MotionState{.accumulated_s = 11.0};
  edge_info.states.push_back(dummy_state2);
  stop_cost.ComputeCost(edge_info, absl::MakeSpan(cost));
  ASSERT_NEAR(cost[0], 1.0, 1e-4);
}

TEST(DpFinalLongitudinalProgressCost, Cost) {
  // Create fake path boundary.
  const int n = 100;
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);

  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(s_vec, right_l, left_l, target_right_l,
                            target_left_l, right_xy, left_xy, target_right_xy,
                            target_left_xy);

  const double max_accumulated_s = 50.0;
  const DpFinalProgressFeatureCost progress_cost(&path_bound,
                                                 max_accumulated_s);
  // Create a straight geometry form.
  const int state_size = 11;
  std::vector<GeometryState> states;
  states.reserve(state_size);
  const double step_length = 1.0;
  const double s_offset = 2.0;
  for (int i = 0; i < state_size; ++i) {
    states.push_back(GeometryState{
        .xy = Vec2d(0.0, s_offset + step_length * i),
        .accumulated_s = s_offset + step_length * i,
        .l = 0.0,
    });
  }
  std::unique_ptr<GeometryForm> ptr_geometry_form =
      std::make_unique<PiecewiseLinearGeometry>(states);
  ASSERT_NEAR(ptr_geometry_form->length(), 10.0, 1e-4);

  // Build motion forms. Brake to stationary state.
  std::unique_ptr<MotionForm> ptr_decelerate_to_stationary =
      std::make_unique<ConstAccelMotion>(2.0, -1.0, ptr_geometry_form.get());
  ASSERT_NEAR(ptr_decelerate_to_stationary->duration(), 10.0, 1e-4);
  const auto motion_states = ptr_decelerate_to_stationary->Sample(0.1);
  MotionEdgeInfo edge_info;
  edge_info.start_t = 0.0;
  edge_info.states = std::move(motion_states);
  edge_info.motion_form = ptr_decelerate_to_stationary.get();

  std::vector<double> costs(2);
  progress_cost.ComputeCost(edge_info, absl::MakeSpan(costs));
  ASSERT_EQ(costs[0], 46.0);

  // Const velocity.
  std::unique_ptr<MotionForm> ptr_const_v =
      std::make_unique<ConstAccelMotion>(2.0, 0.0, ptr_geometry_form.get());
  ASSERT_NEAR(ptr_const_v->duration(), 5.0, 1e-4);
  const auto motion_states_2 = ptr_const_v->Sample(0.2);
  MotionEdgeInfo edge_info_2;
  edge_info_2.start_t = 1.0;
  edge_info_2.states = std::move(motion_states_2);
  edge_info_2.motion_form = ptr_const_v.get();

  progress_cost.ComputeCost(edge_info_2, absl::MakeSpan(costs));
  ASSERT_EQ(costs[0], 0.0);

  edge_info_2.start_t = 6.0;
  progress_cost.ComputeCost(edge_info_2, absl::MakeSpan(costs));
  ASSERT_LT(costs[0], 41.0);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
