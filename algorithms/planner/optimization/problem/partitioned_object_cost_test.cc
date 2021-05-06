#include "onboard/planner/optimization/problem/partitioned_object_cost.h"

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

std::vector<PartitionedObjectCost<Mfob>::Object> GetMfobObjects() {
  std::vector<PartitionedObjectCost<Mfob>::Object> mfob_objects;
  mfob_objects.reserve(Mfob::kHorizon / 2);
  const std::vector<Segment2d> lines = {
      Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
      Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};
  for (int k = 0; k < Mfob::kHorizon / 2; ++k) {
    mfob_objects.push_back(PartitionedObjectCost<Mfob>::Object{
        .lines = std::move(lines),
        .buffers = {15.0, 15.0},
        .gains = {1.0, 0.5},
        .ref_x = {7.5, 2.5},
        .offset = 100.0,
        .ref_tangent = {1.0, 0.0},
        .enable = k % 2 == 0 ? true : false});
  }
  return mfob_objects;
}

std::vector<PartitionedObjectCost<Tob>::Object> GetTobObjects() {
  std::vector<PartitionedObjectCost<Tob>::Object> tob_objects;
  tob_objects.reserve(Tob::kHorizon / 2);
  const std::vector<Segment2d> lines = {
      Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
      Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};
  for (int k = 0; k < Tob::kHorizon / 2; ++k) {
    tob_objects.push_back(PartitionedObjectCost<Tob>::Object{
        .lines = std::move(lines),
        .buffers = {15.0, 15.0},
        .gains = {1.0, 0.5},
        .ref_x = {7.5, 2.5},
        .offset = 100.0,
        .ref_tangent = {1.0, 0.0},
        .enable = k % 2 == 0 ? true : false});
  }
  return tob_objects;
}

const auto mfob_objects = GetMfobObjects();
const auto tob_objects = GetTobObjects();

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

constexpr double dist_to_rac = 3.0;
constexpr double angle_to_axis = 0.1;
const std::vector<double> cascade_buffers = {1.0, 0.5};
const std::vector<std::string> sub_names = {"a", "b"};
const auto mfob_cost = std::make_unique<PartitionedObjectCost<Mfob>>(
    mfob_objects, dist_to_rac, angle_to_axis, cascade_buffers,
    /*av_model_helper=*/nullptr, sub_names);

TEST(PartitionedObjectCostTest, CostComparisonTest) {
  const auto tob_cost = std::make_unique<PartitionedObjectCost<Tob>>(
      tob_objects, dist_to_rac, angle_to_axis, cascade_buffers,
      /*av_model_helper=*/nullptr, sub_names);
  CostComparisonTest::CompareCostTest(tob_cost.get(), mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobSumGTest) {
  CostEvaluationTest<Mfob>::SumForAllStepsTest(mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobDGDxTest) {
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobDGDuTest) {
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDxDxTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDuDxTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), mfob_cost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDuDuTest) {
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), mfob_cost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
