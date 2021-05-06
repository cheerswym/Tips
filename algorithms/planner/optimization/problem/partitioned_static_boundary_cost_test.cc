#include "onboard/planner/optimization/problem/partitioned_static_boundary_cost.h"

#include <memory>

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
using CTest = CostConvergenceTest<Tob>;

const std::vector<Tob::StateType> rac_states = {
    Tob::MakeState(-5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::StateType> fac_states = {
    Tob::MakeState(-5.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::StateType> flc_states = {
    Tob::MakeState(-5.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::ControlType> controls = {Tob::MakeControl(0.0, 0.0)};

const VehicleGeometryParamsProto vehicle_geo_params = DefaultVehicleGeometry();
const Segment2d boundary_segment(Vec2d(0.0, 0.0), Vec2d(10.0, 0.0));
const std::vector<double> cascade_buffers = {10.0, 9.0};
const std::vector<double> cascade_gains = {1.0, 0.9};
const std::vector<std::string> sub_names = {"a", "b"};

const double front_corner_dist_to_rac =
    Hypot(vehicle_geo_params.width() * 0.5,
          vehicle_geo_params.front_edge_to_center());
const double front_corner_angle_to_axis =
    std::atan2(vehicle_geo_params.width() * 0.5,
               vehicle_geo_params.front_edge_to_center());

TEST(PartitionedStaticBoundaryCostTest, CostComparisonTest) {
  PartitionedStaticBoundaryCost<Tob> tob_cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  PartitionedStaticBoundaryCost<Mfob> mfob_cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Mfob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobSumGTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CostEvaluationTest<Tob>::SumForAllStepsTest(&cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobEvaluateWithDebugInfoGTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CostEvaluationTest<Tob>::EvaluateWithDebugInfoTest(&cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      rac_states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      rac_states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      rac_states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      rac_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      fac_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      vehicle_geo_params.wheel_base(), /*angle_to_axis=*/0.0, sub_names,
      cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      fac_states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      vehicle_geo_params.wheel_base(), /*angle_to_axis=*/0.0, sub_names,
      cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      fac_states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      vehicle_geo_params.wheel_base(), /*angle_to_axis=*/0.0, sub_names,
      cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      fac_states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      vehicle_geo_params.wheel_base(), /*angle_to_axis=*/0.0, sub_names,
      cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      fac_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      vehicle_geo_params.wheel_base(), /*angle_to_axis=*/0.0, sub_names,
      cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      fac_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, front_corner_dist_to_rac,
      front_corner_angle_to_axis, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      flc_states, controls, CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, front_corner_dist_to_rac,
      front_corner_angle_to_axis, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostGradientResidualOrder(
      flc_states, controls, CTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, front_corner_dist_to_rac,
      front_corner_angle_to_axis, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      flc_states, controls, CTest::GenerateStateVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, front_corner_dist_to_rac,
      front_corner_angle_to_axis, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      flc_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      &vehicle_geo_params, boundary_segment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, front_corner_dist_to_rac,
      front_corner_angle_to_axis, sub_names, cascade_buffers, cascade_gains);
  CTest::ExpectCostHessianResidualOrder(
      flc_states, controls, CTest::GenerateControlVariationBasis(),
      CTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
