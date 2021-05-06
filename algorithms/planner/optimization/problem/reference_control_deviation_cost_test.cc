#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"

#include "gtest/gtest.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle<kSteps>;
using Tob = ThirdOrderBicycle<kSteps>;

using CTobTest = CostConvergenceTest<Tob>;
const std::vector<Tob::StateType> tob_states = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> tob_controls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
const Tob::ControlsType tob_ref_us = Tob::ControlsType::Zero();

TEST(ReferenceControlDeviationCostTest, TobSumGTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CostEvaluationTest<Tob>::SumForAllStepsTest(&cost);
}

TEST(ReferenceControlDeviationCostTest, TobDGDxTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDGDuTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDxDxTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDuDxTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDuDuTest) {
  ReferenceControlDeviationCost<Tob> cost(tob_ref_us);
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      CTobTest::GenerateControlVariationBasis(), &cost);
}

using CMfobTest = CostConvergenceTest<Mfob>;
const std::vector<Mfob::StateType> mfob_states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
const std::vector<Mfob::ControlType> mfob_controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
const Mfob::ControlsType mfob_ref_us = Mfob::ControlsType::Zero();

TEST(ReferenceControlDeviationCostTest, MfobDGDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(mfob_ref_us);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDGDuTest) {
  ReferenceControlDeviationCost<Mfob> cost(mfob_ref_us);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(mfob_ref_us);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(mfob_ref_us);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceControlDeviationCost<Mfob> cost(mfob_ref_us);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
