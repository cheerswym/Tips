#include "onboard/planner/optimization/problem/longitudinal_jerk_cost.h"

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

const std::vector<Tob::StateType> states = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> controls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};

using CTest = CostConvergenceTest<Tob>;

TEST(LongitudinalJerkCostTest, CostComparisonTest) {
  LongitudinalJerkCost<Tob> tob_cost;
  LongitudinalJerkCost<Mfob> mfob_cost;
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(LongitudinalJerkCostTest, TobSumGTest) {
  LongitudinalJerkCost<Tob> cost;
  CostEvaluationTest<Tob>::SumForAllStepsTest(&cost);
}

TEST(LongitudinalJerkCostTest, TobDGDxTest) {
  LongitudinalJerkCost<Tob> cost;
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalJerkCostTest, TobDGDuTest) {
  LongitudinalJerkCost<Tob> cost;
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(LongitudinalJerkCostTest, TobDDGDxDxTest) {
  LongitudinalJerkCost<Tob> cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalJerkCostTest, TobDDGDuDxTest) {
  LongitudinalJerkCost<Tob> cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalJerkCostTest, TobDDGDuDuTest) {
  LongitudinalJerkCost<Tob> cost;
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
