#include "onboard/planner/optimization/problem/step_static_boundary_cost.h"

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/optimization/problem/cost_comparison_test_util.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle<kSteps>;
using Tob = ThirdOrderBicycle<kSteps>;
using CMfobTest = CostConvergenceTest<Mfob>;

const std::vector<Vec2d> path_points = {
    {-20.0, 10.0}, {0.0, 15.0}, {20.0, 25.0}};
const std::vector<double> path_boundary_dists = {3.0, 4.0, 5.0};
const std::vector<double> cascade_buffers = {0.0};
const std::vector<double> cascade_gains = {1.0};
const std::vector<double> mid_edges = {2.0};
const char name[] = "StepStaticBoundaryCost";
const std::vector<std::string> sub_names = {""};

const std::vector<Mfob::StateType> right_states = {
    Mfob::MakeState(-30.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 5.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 10.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 18.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::StateType> left_states = {
    Mfob::MakeState(-30.0, 15.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 18.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 20.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 35.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> controls = {Mfob::MakeControl(0.1, 0.1)};

class StepStaticBoundaryCostTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
};
}  // namespace

using CMfobTest = CostConvergenceTest<Mfob>;

TEST_F(StepStaticBoundaryCostTest, CostComparisonTest) {
  StepStaticBoundaryCost<Tob> tob_cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  StepStaticBoundaryCost<Mfob> mfob_cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobSumGTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobEvaluateWithDebugInfoGTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobUpdateTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  Mfob::StatesType xs;
  Mfob::ControlsType us;
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, /*value_only=*/false);
}

TEST_F(StepStaticBoundaryCostTest, MfobRightDGDxTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      right_states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobLeftDGDxTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, true,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      left_states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobRightDDGDxDxTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, false,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      right_states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StepStaticBoundaryCostTest, MfobLeftDDGDxDxTest) {
  StepStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points, path_boundary_dists, true,
      mid_edges, sub_names, cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      left_states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
