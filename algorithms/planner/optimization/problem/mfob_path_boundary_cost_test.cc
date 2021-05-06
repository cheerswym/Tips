#include "onboard/planner/optimization/problem/mfob_path_boundary_cost.h"

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle<kSteps>;
using CMfobTest = CostConvergenceTest<Mfob>;

const std::vector<Vec2d> path_points = {
    {-20.0, 10.0}, {0.0, 15.0}, {20.0, 25.0}};
const std::vector<std::vector<double>> path_boundary_dists = {
    {40.0, 70.0, 65.0}};
const std::vector<double> l_offsets = {5.0, -5.0, 5.0};
const std::vector<std::vector<double>> ref_gains = {{1.0, 1.0, 1.0}};
const std::vector<double> mid_edges = {2.0};

const std::vector<double> buffers_min = {-1000.0};
const std::vector<double> rear_buffers_max = {1000.0};
const std::vector<double> front_buffers_max = {1000.0};
const std::vector<double> mid_buffers_max = {1000.0};
const std::vector<double> cascade_gains = {0.5};
const std::vector<double> rear_gain = {0.5};
const std::vector<double> front_gain = {0.5};

const std::vector<std::string> sub_names = {""};

const std::vector<Mfob::StateType> mfob_right_states = {
    Mfob::MakeState(-30.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 5.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 11.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 18.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::StateType> mfob_left_states = {
    Mfob::MakeState(-30.0, 15.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 18.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 50.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> mfob_controls = {
    Mfob::MakeControl(0.1, 0.1)};

class MfobPathBoundaryCostTest : public ::testing::Test {
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

// Mfob tests.
TEST_F(MfobPathBoundaryCostTest, SumGTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, false,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

// Mfob tests.
TEST_F(MfobPathBoundaryCostTest, EvaluateWithDebugInfoGTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, false,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(MfobPathBoundaryCostTest, UpdateTest) {
  using Mfob = Mfob;
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, false,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  Mfob::StatesType xs;
  Mfob::ControlsType us;
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, /*value_only=*/false);
}

TEST_F(MfobPathBoundaryCostTest, RightDGDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, false,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_right_states, mfob_controls,
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MfobPathBoundaryCostTest, LeftDGDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, true,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_left_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      &cost);
}

TEST_F(MfobPathBoundaryCostTest, RightDDGDxDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, false,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_right_states, mfob_controls,
      CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MfobPathBoundaryCostTest, LeftDDGDxDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, path_points,
      /*center_line_helper=*/nullptr, l_offsets, path_boundary_dists, true,
      mid_edges, ref_gains, sub_names, /*use_qtfm=*/false, buffers_min,
      rear_buffers_max, front_buffers_max, mid_buffers_max, cascade_gains,
      rear_gain, front_gain);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_left_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
