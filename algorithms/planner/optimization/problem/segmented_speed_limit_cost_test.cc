#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"

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

const std::vector<Vec2d> ref_points = {{0.0, 2.0}, {5.0, 0.0}, {10.0, 0.1}};
const std::vector<double> speed_limits = {10.0, 20.0};
const std::vector<Vec2d> free_ref_points = {{0.0, 0.2}, {6.0, 0.0}, {9.0, 0.2}};
const std::vector<double> free_speed_limits = {8.0, 30.0};
constexpr int kFreeIndex = kSteps / 2;
const auto under_speed_gain_compensation_plf =
    PiecewiseLinearFunction<double>({0.0, 20.0}, {5.0, 1.0});

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(SegmentedSpeedLimitCostTest, CostComparisonTest) {
  SegmentedSpeedLimitCost<Mfob> mfob_cost(
      ref_points, speed_limits, free_ref_points, free_speed_limits, kFreeIndex,
      under_speed_gain_compensation_plf);
  SegmentedSpeedLimitCost<Tob> tob_cost(
      ref_points, speed_limits, free_ref_points, free_speed_limits, kFreeIndex,
      under_speed_gain_compensation_plf);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobSumGTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDGDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDGDuTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDxDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDuDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDuDuTest) {
  SegmentedSpeedLimitCost<Mfob> cost(ref_points, speed_limits, free_ref_points,
                                     free_speed_limits, kFreeIndex,
                                     under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
