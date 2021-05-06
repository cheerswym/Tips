#include "onboard/planner/optimization/problem/tob_object_cost.h"

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Tob = ThirdOrderBicycle<kSteps>;
using TobObjectCost = TobObjectCost<Tob>;

class TobObjectCostTest
    : public ::testing::TestWithParam<typename TobObjectCost::Object::Type> {
 public:
  void Initialize(typename TobObjectCost::Object::Type type) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);

    std::array<typename TobObjectCost::Object, Tob::kHorizon> objects;
    for (int k = 0; k < Tob::kHorizon; ++k) {
      objects[k] = TobObjectCost::Object{.dir = -Vec2d::UnitY(),
                                         .ref = Vec2d(0.0, 10.0),
                                         .lateral_extent = 10.0,
                                         .gain = 1.0,
                                         .depth = 100.0,
                                         .type = type,
                                         .enable = true};
    }

    auto vehicle_geo = run_params.vehicle_params().vehicle_geometry_params();
    cost_ = std::make_unique<TobObjectCost>(&vehicle_geo, std::move(objects));
  }

 protected:
  std::unique_ptr<TobObjectCost> cost_;
};

INSTANTIATE_TEST_SUITE_P(
    TobObjectCostTests, TobObjectCostTest,
    ::testing::Values(TobObjectCost::Object::Type::kRac,
                      TobObjectCost::Object::Type::kFrontLeft,
                      TobObjectCost::Object::Type::kFrontRight,
                      TobObjectCost::Object::Type::kFrontCenter));

using CTobTest = CostConvergenceTest<Tob>;
const std::vector<Tob::StateType> tob_states = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> tob_controls = {Tob::MakeControl(0.1, 0.1)};

TEST_P(TobObjectCostTest, TobSumGTest) {
  Initialize(GetParam());
  CostEvaluationTest<Tob>::SumForAllStepsTest(cost_.get());
}

TEST_P(TobObjectCostTest, DGDxTest) {
  Initialize(GetParam());
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(),
      cost_.get());
}

TEST_P(TobObjectCostTest, DGDuTest) {
  Initialize(GetParam());
  CTobTest::ExpectCostGradientResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      cost_.get());
}

TEST_P(TobObjectCostTest, DDGDxDxTest) {
  Initialize(GetParam());
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateStateVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobObjectCostTest, DDGDuDxTest) {
  Initialize(GetParam());
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      CTobTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobObjectCostTest, DDGDuDuTest) {
  Initialize(GetParam());
  CTobTest::ExpectCostHessianResidualOrder(
      tob_states, tob_controls, CTobTest::GenerateControlVariationBasis(),
      CTobTest::GenerateControlVariationBasis(), cost_.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
