#include "onboard/planner/optimization/problem/curvature_cost.h"

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
using CTest = CostConvergenceTest<Mfob>;

constexpr double kCurvatureBuffer = 2.0;
const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 4.0, 0.1, 0.05, 0.4)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(CurvatureCostTest, CostComparisionTest) {
  CurvatureCost<Mfob> mfob_cost(kCurvatureBuffer, kSteps);
  CurvatureCost<Tob> tob_cost(kCurvatureBuffer, kSteps);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(CurvatureCostTest, MfobSumGTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(CurvatureCostTest, MfobEvaluateWithDebugInfoGTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(&cost);
}

TEST(CurvatureCostTest, MfobDGDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDGDuTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDxDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDuDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDuDuTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
