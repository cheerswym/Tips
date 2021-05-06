#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"

#include "gtest/gtest.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle<kSteps>;
using CTest = CostConvergenceTest<Mfob>;

constexpr double kCurvatureRateBuffer = 2.0;
const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.2, 0.1, 0.05, 0.05)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(MfobCurvatureRateCostTest, SumGTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(MfobCurvatureRateCostTest, DGDxTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateCostTest, DGDuTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(MfobCurvatureRateCostTest, DDGDxDxTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateCostTest, DDGDuDxTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateCostTest, DDGDuDuTest) {
  MfobCurvatureRateCost<Mfob> cost(kCurvatureRateBuffer);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
