#include "onboard/planner/optimization/problem/tob_lateral_jerk_cost.h"

#include "gtest/gtest.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Tob = ThirdOrderBicycle<kSteps>;
using TobLateralJerkCost = TobLateralJerkCost<Tob>;

using CTest = CostConvergenceTest<Tob>;

const std::vector<Tob::StateType> states = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> controls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};

TEST(TobLateralJerkCostTest, TobSumGTest) {
  TobLateralJerkCost cost;
  CostEvaluationTest<Tob>::SumForAllStepsTest(&cost);
}

TEST(TobLateralJerkCostTest, DGDxTest) {
  TobLateralJerkCost cost;
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DGDuTest) {
  TobLateralJerkCost cost;
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDxDxTest) {
  TobLateralJerkCost cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDuDxTest) {
  TobLateralJerkCost cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDuDuTest) {
  TobLateralJerkCost cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
