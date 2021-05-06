#include "onboard/planner/optimization/problem/aggregate_static_object_cost.h"

#include <memory>

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/optimization/problem/cost_comparison_test_util.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle<kSteps>;
using Tob = ThirdOrderBicycle<kSteps>;
using CMfobTest = CostConvergenceTest<Mfob>;

const std::vector<Mfob::StateType> mfob_states = {
    Mfob::MakeState(15.5, 0.5, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(12.0, 7.5, M_PI / 4.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(7.5, 6.0, -0.1, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(1.5, 3.5, 0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.0, 0.5, -0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(3.0, 6.0, 0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(-1.0, 5.0, -0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.5, 0.5, -M_PI / 2.0, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> mfob_controls = {
    Mfob::MakeControl(0.1, 0.1)};

const std::vector<Segment2d> Segments = {
    Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
    Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};

std::vector<double> buffers = {15.0, 15.0};
std::vector<double> gains = {1.0, 0.5};
std::vector<std::string> sub_names = {"a", "b"};

constexpr double kL = 3.0;
const auto mfob_cost = std::make_unique<AggregateStaticObjectCost<Mfob>>(
    Segments, kL, buffers, gains, sub_names, /*num_objects=*/50);

TEST(AggregateStaticObjectCostTest, CostComparisonTest) {
  const auto tob_cost = std::make_unique<AggregateStaticObjectCost<Tob>>(
      Segments, kL, buffers, gains, sub_names, /*num_objects=*/50);
  CostComparisonTest::CompareCostTest(tob_cost.get(), mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobSumGTest) {
  CostEvaluationTest<Mfob>::SumForAllStepsTest(mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDGDxTest) {
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDGDuTest) {
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDxDxTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDuDxTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), mfob_cost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDuDuTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), mfob_cost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
