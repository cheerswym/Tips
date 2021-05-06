#include "onboard/planner/optimization/problem/end_heading_cost.h"

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

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(2.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(6.0, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

const std::vector<Vec2d> ref_points = {{0.0, 0.1}, {5.0, 0.0}, {10.0, 0.1}};

TEST(EndHeadingCostTest, CostComparisonTest) {
  EndHeadingCost<Mfob> mfob_cost(ref_points, /*center_line_helper=*/nullptr);
  EndHeadingCost<Tob> tob_cost(ref_points, /*center_line_helper=*/nullptr);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(EndHeadingCostTest, MfobSumGTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(EndHeadingCostTest, MfobDGDxTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDGDuTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDxDxTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDuDxTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDuDuTest) {
  EndHeadingCost<Mfob> cost(ref_points, /*center_line_helper=*/nullptr);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
