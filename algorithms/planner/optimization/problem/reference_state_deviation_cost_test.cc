#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"

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
const Tob::StatesType tob_ref_xs = Tob::StatesType::Zero();

TEST(ReferenceStateDeviationCostTest, TobSumGTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
  CostEvaluationTest<Tob>::SumForAllStepsTest(&cost);
}

TEST(ReferenceStateDeviationCostTest, TobDGDxTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDGDuTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDxDxTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDuDxTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDuDuTest) {
  ReferenceStateDeviationCost<Tob> cost(tob_ref_xs);
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
const Mfob::StatesType mfob_ref_xs = Mfob::StatesType::Zero();

TEST(ReferenceStateDeviationCostTest, MfobDGDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(mfob_ref_xs);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDGDuTest) {
  ReferenceStateDeviationCost<Mfob> cost(mfob_ref_xs);
  CMfobTest::ExpectCostGradientResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(mfob_ref_xs);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(mfob_ref_xs);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceStateDeviationCost<Mfob> cost(mfob_ref_xs);
  CMfobTest::ExpectCostHessianResidualOrder(
      mfob_states, mfob_controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
