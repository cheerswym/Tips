#include "onboard/planner/optimization/problem/msd_static_boundary_cost.h"

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

// Test case visualized here:
// https://qcraft.feishu.cn/docs/doccnFSXXJ2uJm3fOM399i5NN1g
const std::vector<std::pair<std::string, Segment2d>> named_curb_segments = {
    {"curb_0", Segment2d{Vec2d{0.206653, 7.273863}, Vec2d{1.006765, 6.214802}}},
    {"curb_1", Segment2d{Vec2d{1.006765, 6.214802}, Vec2d{1.985958, 6.750146}}},
    {"curb_2", Segment2d{Vec2d{1.985958, 6.750146}, Vec2d{2.869103, 7.267413}}},
    {"curb_3", Segment2d{Vec2d{2.869103, 7.267413}, Vec2d{4.245848, 7.948356}}},
    {"curb_4", Segment2d{Vec2d{4.245848, 7.948356}, Vec2d{4.886089, 7.189495}}},
    {"curb_5", Segment2d{Vec2d{4.886089, 7.189495}, Vec2d{5.983286, 5.302104}}},
    {"curb_6", Segment2d{Vec2d{5.983286, 5.302104}, Vec2d{7.247807, 7.899819}}},
    {"curb_7", Segment2d{Vec2d{7.247807, 7.899819}, Vec2d{7.856295, 7.415083}}},
    {"curb_8", Segment2d{Vec2d{7.856295, 7.415083}, Vec2d{9.131823, 6.196471}}},
    {"curb_9", Segment2d{Vec2d{9.131823, 6.196471}, Vec2d{9.700686, 6.480734}}},
    {"curb_10",
     Segment2d{Vec2d{9.700686, 6.480734}, Vec2d{10.895123, 7.611414}}},
    {"curb_11",
     Segment2d{Vec2d{10.895123, 7.611414}, Vec2d{11.843170, 7.902621}}},
    {"curb_12",
     Segment2d{Vec2d{11.843170, 7.902621}, Vec2d{12.748267, 5.960164}}},
    {"curb_13",
     Segment2d{Vec2d{12.748267, 5.960164}, Vec2d{13.765435, 6.653802}}},
    {"curb_14",
     Segment2d{Vec2d{13.765435, 6.653802}, Vec2d{15.188680, 6.620851}}},
    {"curb_15",
     Segment2d{Vec2d{15.188680, 6.620851}, Vec2d{16.052570, 6.334967}}},
    {"curb_16",
     Segment2d{Vec2d{16.052570, 6.334967}, Vec2d{17.045391, 5.870989}}},
    {"curb_17",
     Segment2d{Vec2d{17.045391, 5.870989}, Vec2d{18.067664, 6.969978}}},
    {"curb_18",
     Segment2d{Vec2d{18.067664, 6.969978}, Vec2d{19.154562, 7.630311}}},
    {"curb_19",
     Segment2d{Vec2d{-0.047657, -5.776750}, Vec2d{1.170279, -5.909938}}},
    {"curb_20",
     Segment2d{Vec2d{1.170279, -5.909938}, Vec2d{2.244868, -6.514061}}},
    {"curb_21",
     Segment2d{Vec2d{2.244868, -6.514061}, Vec2d{3.071021, -5.751519}}},
    {"curb_22",
     Segment2d{Vec2d{3.071021, -5.751519}, Vec2d{4.186130, -7.706498}}},
    {"curb_23",
     Segment2d{Vec2d{4.186130, -7.706498}, Vec2d{5.239303, -7.051952}}},
    {"curb_24",
     Segment2d{Vec2d{5.239303, -7.051952}, Vec2d{5.960503, -6.832661}}},
    {"curb_25",
     Segment2d{Vec2d{5.960503, -6.832661}, Vec2d{6.986206, -7.595930}}},
    {"curb_26",
     Segment2d{Vec2d{6.986206, -7.595930}, Vec2d{8.029220, -5.042125}}},
    {"curb_27",
     Segment2d{Vec2d{8.029220, -5.042125}, Vec2d{9.194907, -7.004460}}},
    {"curb_28",
     Segment2d{Vec2d{9.194907, -7.004460}, Vec2d{10.220562, -5.731733}}},
    {"curb_29",
     Segment2d{Vec2d{10.220562, -5.731733}, Vec2d{10.814640, -6.702532}}},
    {"curb_30",
     Segment2d{Vec2d{10.814640, -6.702532}, Vec2d{12.181908, -6.343909}}},
    {"curb_31",
     Segment2d{Vec2d{12.181908, -6.343909}, Vec2d{13.004764, -7.798501}}},
    {"curb_32",
     Segment2d{Vec2d{13.004764, -7.798501}, Vec2d{14.123937, -6.642323}}},
    {"curb_33",
     Segment2d{Vec2d{14.123937, -6.642323}, Vec2d{15.278303, -6.809557}}},
    {"curb_34",
     Segment2d{Vec2d{15.278303, -6.809557}, Vec2d{16.057772, -6.154703}}},
    {"curb_35",
     Segment2d{Vec2d{16.057772, -6.154703}, Vec2d{16.813635, -5.560189}}},
    {"curb_36",
     Segment2d{Vec2d{16.813635, -5.560189}, Vec2d{17.985919, -5.269473}}},
    {"curb_37",
     Segment2d{Vec2d{17.985919, -5.269473}, Vec2d{19.254029, -7.527381}}},
};
const std::vector<double> cascade_buffers = {1.0};
const std::vector<double> cascade_gains = {1.0};
const std::vector<std::string> sub_names = {""};
const char name[] = "MsdStaticBoundaryCostTest";

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(2.262738, 3.647666, 0.181121, 0.029193, -0.055400, 0.048041,
                    0.0, 0.0),
    Mfob::MakeState(1.440344, 4.628956, 0.509273, 0.441304, -0.084985,
                    -0.096750, 0.0, 0.0),
    Mfob::MakeState(4.888592, 4.598231, -0.311408, 0.936393, -0.085806,
                    -0.001919, 0.0, 0.0),
    Mfob::MakeState(10.520543, 4.296994, 0.548456, 0.515375, -0.026579,
                    -0.050016, 0.0, 0.0),
    Mfob::MakeState(7.707249, -4.520323, 0.596523, 0.335939, -0.012294,
                    -0.011686, 0.0, 0.0)};
const std::vector<Mfob::ControlType> controls = {Mfob::MakeControl(0.1, 0.1)};

class MsdStaticBoundaryCostTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();
    curb_msd_.emplace(named_curb_segments, /*use_qtfm=*/false,
                      /*cutoff_distance=*/50.0);

    const double wheel_base = vehicle_geometry_params_.wheel_base();
    const double vehicle_half_width = vehicle_geometry_params_.width();
    circle_center_offsets_ = std::vector<Vec2d>{
        Vec2d{0.0, 0.0}, Vec2d{0.5 * wheel_base, 0.0}, Vec2d{wheel_base, 0.0}};
    radiuses_ = std::vector<double>{vehicle_half_width, vehicle_half_width,
                                    vehicle_half_width};
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::optional<MinSegmentDistanceProblem> curb_msd_;
  std::vector<Vec2d> circle_center_offsets_;
  std::vector<double> radiuses_;
};
}  // namespace

TEST_F(MsdStaticBoundaryCostTest, CostComparisonTest) {
  MsdStaticBoundaryCost<Tob> tob_cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  MsdStaticBoundaryCost<Mfob> mfob_cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobSumGTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobEvaluateWithDebugInfoGTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CostEvaluationTest<Mfob>::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobUpdateTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  Mfob::StatesType xs;
  Mfob::ControlsType us;
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, /*value_only=*/false);
}

TEST_F(MsdStaticBoundaryCostTest, MfobDGDxTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobDDGDxDxTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobDGDuTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(), &cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobDDGDuDxTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdStaticBoundaryCostTest, MfobDDGDuDuTest) {
  MsdStaticBoundaryCost<Mfob> cost(
      vehicle_geometry_params_, *curb_msd_, sub_names, cascade_buffers,
      cascade_gains, circle_center_offsets_, radiuses_, kSteps, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
