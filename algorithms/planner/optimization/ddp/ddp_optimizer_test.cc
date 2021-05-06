#include "onboard/planner/optimization/ddp/ddp_optimizer.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/trace.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"
#include "onboard/planner/optimization/problem/static_boundary_cost.h"
#include "onboard/planner/optimization/problem/tob_curvature_rate_cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

namespace qcraft {
namespace planner {

namespace {

constexpr int kSteps = 100;
constexpr double kDt = 0.1;
using Tob = ThirdOrderBicycle<kSteps>;

MotionConstraintParamsProto motion_constraint_params =
    DefaultPlannerParams().motion_constraint_params();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

template <typename PROBLEM>
class IterationVisualizerHook : public DdpOptimizerHook<PROBLEM> {
 public:
  using OptimizerInspector =
      typename DdpOptimizerHook<PROBLEM>::OptimizerInspector;

  explicit IterationVisualizerHook(std::string name) : name_(std::move(name)) {}

  void OnSolveStart(const typename PROBLEM::StatesType &xs,
                    const typename PROBLEM::ControlsType &us,
                    const OptimizerInspector &oi) override {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&xs](int index) {
              return PROBLEM::StateGetPos(PROBLEM::GetStateAtStep(xs, index));
            },
            PROBLEM::kHorizon),
        vis::Color(0.8, 0.4, 0.4),
        /*render_indices=*/true,
        absl::StrFormat("dopt_test/%s/iters/init", name_));
  }

  void OnIterationEnd(int iteration, const typename PROBLEM::StatesType &xs,
                      const typename PROBLEM::ControlsType &us,
                      const OptimizerInspector &oi) override {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&xs](int index) {
              return PROBLEM::StateGetPos(PROBLEM::GetStateAtStep(xs, index));
            },
            PROBLEM::kHorizon),
        vis::Color(0.4, 0.8, 0.8),
        /*render_indices=*/true,
        absl::StrFormat("dopt_test/%s/iters/iter_%03d", name_, iteration));
  }

 private:
  std::string name_;
};

void TestTobDdpOptimizer(
    const std::string &name, const std::vector<Vec2d> &ref_points,
    const std::vector<TrajectoryPoint> &init_traj,
    const std::function<void(Tob *)> &problem_mutator = [](Tob *) {},
    const std::function<void(DdpOptimizer<Tob> *)> &dopt_mutator =
        [](DdpOptimizer<Tob> *) {}) {
  Tob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
              kDt);
  constexpr double kAccelerationBufferRatio = 0.75;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kCurvatureBufferRatio = 0.75;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  problem.AddCost(std::make_unique<IntrinsicJerkCost<Tob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio));
  problem.AddCost(std::make_unique<TobCurvatureRateCost<Tob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio));
  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Tob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio));
  problem.AddCost(std::make_unique<CurvatureCost<Tob>>(
      GetRelaxedCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      Tob::kHorizon));

  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Tob>>(
      kPathGain, kEndStateGain, ref_points, /*center_line_helper=*/nullptr));

  std::vector<double> speed_limit(ref_points.size() - 1);
  for (int i = 0; i < speed_limit.size(); ++i) {
    speed_limit[i] = 10.0;
  }
  const auto under_speed_gain_compensation_plf =
      PiecewiseLinearFunction<double>({0.0, 10.0, 20.0}, {12.0, 1.0, 0.5});

  // Use normal speed limit info instead of free speed limit info.
  problem.AddCost(std::make_unique<SegmentedSpeedLimitCost<Tob>>(
      ref_points, speed_limit, ref_points, speed_limit, kSteps,
      under_speed_gain_compensation_plf));

  problem_mutator(&problem);

  DdpOptimizer<Tob> dopt(&problem);
  dopt.SetInitialPoints(init_traj);
  dopt_mutator(&dopt);

  IterationVisualizerHook<Tob> hook(name);
  dopt.AddHook(&hook);

  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&ref_points](int index) { return ref_points[index]; },
          ref_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
      vis::Color(0.8, 0.4, 0.8),
      /*render_indices=*/true, "dopt_test/" + name + "/ref_line");

  std::vector<TrajectoryPoint> trajectory;
  const auto output = dopt.Solve(/*forward=*/true);
  if (output.ok()) {
    trajectory = std::move(*output);
  } else {
    trajectory = std::move(init_traj);
  }

  for (int k = 0; k < Tob::kHorizon; ++k) {
    EXPECT_EQ(trajectory[k].pos(), trajectory[k].pos());
  }

  vantage_client_man::FlushAll();
}

TEST(DdpOptimizerTest, TobConstSpeedTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobConstSpeedTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobParallelRefLineTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 1.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobParallelRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobFarParallelRefLineTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 10.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobFarParallelRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobDivergingRefLineTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, i * 2.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobDivergingRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobLeftTurnTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 25; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }
  for (int i = 25; i < 50; ++i) {
    ref_points[i] = {50, i * 2.0 - 48};
  }

  // Initial guess trajectory: going north-east.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 1.0 * k});
    init_traj[k].set_v(14.1421356);
    init_traj[k].set_theta(0.78539825);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobLeftTurnTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobLeftTurnTestWithConflictingBoundary) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 25; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }
  for (int i = 25; i < 50; ++i) {
    ref_points[i] = {50, i * 2.0 - 48};
  }

  std::vector<Vec2d> boundary_path_points(10);
  std::vector<double> dists_to_left_boundary(10, 3.0);
  std::vector<double> dists_to_right_boundary(10, 3.0);
  for (int i = 0; i < 10; ++i) {
    boundary_path_points[i] = {i * 10.0 - 20.0, 0.0};
  }
  const std::vector<double> cascade_buffers = {0.0};
  const std::vector<double> cascade_gains = {1.0};
  const std::vector<double> mid_edges = {2.0};
  const std::vector<std::string> sub_names = {""};

  // Initial guess trajectory: going 5 deg north of east.
  std::vector<TrajectoryPoint> init_traj(Tob::kHorizon);
  const double theta = 5.0 / 180.0 * M_PI;
  for (int k = 0; k < Tob::kHorizon; ++k) {
    init_traj[k].set_pos({1.0 * k, 1.0 * k * std::tan(theta)});
    init_traj[k].set_v(10.0 / std::cos(theta));
    init_traj[k].set_theta(theta);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer(
      "TobLeftTurnTestWithConflictingBoundary", ref_points, init_traj,
      [&vehicle_geometry_params, &boundary_path_points, &dists_to_left_boundary,
       &dists_to_right_boundary, &cascade_buffers, &cascade_gains, &mid_edges,
       &sub_names](Tob *ddp) {
        ddp->AddCost(std::make_unique<StaticBoundaryCost<Tob>>(
            vehicle_geometry_params, boundary_path_points,
            dists_to_left_boundary, true, mid_edges, sub_names, cascade_buffers,
            cascade_gains));
        ddp->AddCost(std::make_unique<StaticBoundaryCost<Tob>>(
            vehicle_geometry_params, boundary_path_points,
            dists_to_right_boundary, false, mid_edges, sub_names,
            cascade_buffers, cascade_gains));
      });
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
