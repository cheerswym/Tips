#include "onboard/planner/optimization/problem/speed_limit_cost.h"

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
const auto under_speed_gain_compensation_plf =
    PiecewiseLinearFunction<double>({0.0, 20.0}, {5.0, 1.0});

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(SpeedLimitCostTest, CostComparisonTest) {
  SpeedLimitCost<Mfob> mfob_cost(ref_points, speed_limits,
                                 under_speed_gain_compensation_plf);
  SpeedLimitCost<Tob> tob_cost(ref_points, speed_limits,
                               under_speed_gain_compensation_plf);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(SpeedLimitCostTest, MfobSumGTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(SpeedLimitCostTest, MfobDGDxTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDGDuTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDxDxTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDuDxTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDuDuTest) {
  SpeedLimitCost<Mfob> cost(ref_points, speed_limits,
                            under_speed_gain_compensation_plf);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
