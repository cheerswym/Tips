#include <iostream>
#include <vector>

#include "gtest/gtest.h"

#define HAVE_STDDEF_H
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#undef HAVE_STDDEF_H

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/ipopt/ipopt_adapter.h"
#include "onboard/planner/optimization/ipopt/ipopt_optimizer_debug_hook.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_rate_cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

namespace qcraft {
namespace planner {

namespace {

using Mfob = MixedFourthOrderBicycle<kDdpTrajectorySteps>;

std::vector<TrajectoryPoint> GetInitTraj(
    const TrajectoryPoint plan_start_point) {
  std::vector<TrajectoryPoint> trajectory;
  const Vec2d ref_tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta());

  // trajectory.reserve(kDdpTrajectorySteps);
  for (int k = 0; k < kDdpTrajectorySteps; ++k) {
    trajectory.emplace_back();
    auto traj_point = trajectory.back();
    trajectory.back().set_pos(plan_start_point.pos() +
                              k * kDdpTrajectoryTimeStep *
                                  plan_start_point.v() * ref_tangent);
    trajectory.back().set_theta(plan_start_point.theta());
    trajectory.back().set_kappa(0.0);
    trajectory.back().set_s(plan_start_point.v() * k * kDdpTrajectoryTimeStep);
    trajectory.back().set_v(plan_start_point.v());
    trajectory.back().set_a(0.0);
    trajectory.back().set_t(k * kDdpTrajectoryTimeStep);
  }

  return trajectory;
}

void AddCosts(const std::vector<TrajectoryPoint> &init_traj,
              const MotionConstraintParamsProto &motion_constraint_params,
              const VehicleGeometryParamsProto &veh_geo_params,
              const VehicleDriveParamsProto &veh_drive_params, Mfob *problem) {
  constexpr double kAccelerationBufferRatio = 0.75;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kCurvatureBufferRatio = 0.75;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  problem->AddCost(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio));
  problem->AddCost(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio));
  problem->AddCost(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio));
  problem->AddCost(std::make_unique<CurvatureCost<Mfob>>(
      GetRelaxedCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      Mfob::kHorizon));
  problem->AddCost(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
      motion_constraint_params.max_chi()));

  std::vector<Vec2d> ref_points;
  ref_points.reserve(init_traj.size());
  for (int k = 0; k < init_traj.size(); ++k) {
    ref_points.push_back(init_traj[k].pos() + Vec2d(0.0, 0.0));
  }

  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  problem->AddCost(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      kPathGain, kEndStateGain, ref_points, /*center_line_helper=*/nullptr));

  std::vector<double> speed_limit(ref_points.size() - 1);
  for (int i = 0; i < speed_limit.size(); ++i) {
    speed_limit[i] = 11.0;
  }
  const auto under_speed_gain_compensation_plf =
      PiecewiseLinearFunction<double>({0.0, 10.0, 20.0}, {12.0, 1.0, 0.5});

  problem->AddCost(std::make_unique<SegmentedSpeedLimitCost<Mfob>>(
      ref_points, speed_limit, ref_points, speed_limit, kDdpTrajectorySteps - 1,
      under_speed_gain_compensation_plf));

  // Regularization
  const Mfob::StateType x0 = Mfob::FitInitialState(init_traj);
  Mfob::ControlsType init_us = Mfob::FitControl(init_traj, x0);
  Mfob::StatesType init_xs = Mfob::FitState(init_traj);
  init_xs = init_xs + Mfob::StatesType::Constant(1e-6);
  init_us = init_us + Mfob::ControlsType::Constant(1e-6);
  std::vector<double> state_regularization_weights(init_xs.size(), 1.0);
  std::vector<double> control_regularization_weights(init_us.size(), 1.0);
  problem->AddCost(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      init_xs, std::move(state_regularization_weights), "StateRegularization",
      1e-6));
  problem->AddCost(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      init_us, std::move(control_regularization_weights),
      "ControlRegularization", 1e-6));
}

TEST(IpoptAdapterTest, IpoptAdapterTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  qcraft::planner::TrajectoryPoint plan_start_point;
  plan_start_point.set_pos(Vec2d(20.0, 0.0));
  plan_start_point.set_theta(0.1);
  plan_start_point.set_v(10.0);

  std::vector<TrajectoryPoint> init_traj = GetInitTraj(plan_start_point);

  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
  VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  Mfob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
               kDdpTrajectoryTimeStep, false);

  AddCosts(init_traj, motion_constraint_params, veh_geo_params,
           veh_drive_params, &problem);

  IpoptAdapter<Mfob> solver(&problem, "ipopt_tester");
  solver.SetInitialPoints(init_traj);
  IpoptOptimizerDebugHook<Mfob> debug_hook;
  solver.AddHook(&debug_hook);

  std::string result_info;
  const auto output = solver.Solve(&result_info);
  EXPECT_TRUE(output.ok());
  std::vector<TrajectoryPoint> result_points = std::move(*output);

  for (int k = 0; k < Mfob::kHorizon; ++k) {
    EXPECT_EQ(result_points[k].pos(), result_points[k].pos());
  }

  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&init_traj](int index) { return init_traj[index].pos(); },
          init_traj.size(), /*z_inc=*/0.0, /*z0=*/0.0),
      vis::Color(0.1, 0.6, 0.8),
      /*render_indices=*/true, "ipopt_tester/init");

  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&result_points](int index) { return result_points[index].pos(); },
          result_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
      vis::Color(0.1, 0.1, 0.8),
      /*render_indices=*/true, "ipopt_tester/result");

  vantage_client_man::FlushAll();
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
