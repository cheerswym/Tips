#include "onboard/planner/optimization/problem/curvature_deviation_cost.h"

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

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 30.0)};
const std::vector<Mfob::ControlType> controls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

const std::vector<double> ref_path_s = {0.0, 3.0, 8.0, 12.0, 50};
const std::vector<double> ref_kappa = {-0.05, 0.0, 0.05, -0.1, 0.15};

TEST(CurvatureDeviationCostTest, CostComparisionTest) {
  CurvatureDeviationCost<Mfob> mfob_cost(ref_path_s, ref_kappa,
                                         /*weight=*/500.0,
                                         /*weight_decay_rate=*/-0.05);
  CurvatureDeviationCost<Tob> tob_cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(CurvatureDeviationCostTest, MfobSumGTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST(CurvatureDeviationCostTest, MfobDGDxTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDGDuTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CTest::ExpectCostGradientResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDxDxTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDuDxTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDuDuTest) {
  CurvatureDeviationCost<Mfob> cost(ref_path_s, ref_kappa, 500.0, -0.05);
  CTest::ExpectCostHessianResidualOrder(
      states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
