#include "onboard/planner/optimization/problem/msd_keep_close_to_center_line_cost.h"

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
const std::vector<std::pair<std::string, Segment2d>>
    named_center_line_segments = {
        {"center_0",
         Segment2d{Vec2d{0.206653, -0.257954}, Vec2d{0.952343, 0.241083}}},
        {"center_1",
         Segment2d{Vec2d{0.952343, 0.241083}, Vec2d{2.006765, 0.095066}}},
        {"center_2",
         Segment2d{Vec2d{2.006765, 0.095066}, Vec2d{3.170279, 0.196687}}},
        {"center_3",
         Segment2d{Vec2d{3.170279, 0.196687}, Vec2d{3.985958, -0.083382}}},
        {"center_4",
         Segment2d{Vec2d{3.985958, -0.083382}, Vec2d{5.244868, -0.004687}}},
        {"center_5",
         Segment2d{Vec2d{5.244868, -0.004687}, Vec2d{5.869103, -0.255804}}},
        {"center_6",
         Segment2d{Vec2d{5.869103, -0.255804}, Vec2d{7.071021, 0.249494}}},
        {"center_7",
         Segment2d{Vec2d{7.071021, 0.249494}, Vec2d{8.245848, -0.482785}}},
        {"center_8",
         Segment2d{Vec2d{8.245848, -0.482785}, Vec2d{9.186130, -0.402166}}},
        {"center_9",
         Segment2d{Vec2d{9.186130, -0.402166}, Vec2d{9.886089, -0.229832}}},
        {"center_10",
         Segment2d{Vec2d{9.886089, -0.229832}, Vec2d{11.239303, -0.183984}}},
        {"center_11",
         Segment2d{Vec2d{11.239303, -0.183984}, Vec2d{11.983286, 0.399299}}},
        {"center_12",
         Segment2d{Vec2d{11.983286, 0.399299}, Vec2d{12.960503, -0.110887}}},
        {"center_13",
         Segment2d{Vec2d{12.960503, -0.110887}, Vec2d{14.247807, -0.466606}}},
        {"center_14",
         Segment2d{Vec2d{14.247807, -0.466606}, Vec2d{14.986206, -0.365310}}},
        {"center_15",
         Segment2d{Vec2d{14.986206, -0.365310}, Vec2d{15.856295, -0.305028}}},
        {"center_16",
         Segment2d{Vec2d{15.856295, -0.305028}, Vec2d{17.029220, 0.485958}}},
        {"center_17",
         Segment2d{Vec2d{17.029220, 0.485958}, Vec2d{18.131823, 0.101176}}},
        {"center_18",
         Segment2d{Vec2d{18.131823, 0.101176}, Vec2d{19.194907, -0.168153}}}};
const std::vector<double> cascade_buffers = {0.0};
const std::vector<double> cascade_gains = {1.0};
const double max_distance_to_center = 8.0;
const char name[] = "MsdKeepCloseToCenterLineCostTest";

const std::vector<Mfob::StateType> states = {
    Mfob::MakeState(11.369316, 9.533266, -0.095314, 0.603920, 0.025144,
                    -0.086894, 0.0, 0.0),
    Mfob::MakeState(12.711506, 7.403093, 0.316530, 0.013168, 0.067494,
                    -0.048129, 0.0, 0.0),
    Mfob::MakeState(14.217412, 9.868103, -0.532138, 0.234331, 0.099129,
                    -0.005947, 0.0, 0.0),
    Mfob::MakeState(8.163438, 7.713894, -0.156054, 0.836461, -0.004729,
                    0.027814, 0.0, 0.0),
    Mfob::MakeState(11.369316, -9.533266, -0.095314, 0.150616, 0.026972,
                    0.073609, 0.0, 0.0)};
const std::vector<Mfob::ControlType> controls = {Mfob::MakeControl(0.1, 0.1)};

class MsdKeepCloseToCenterLineCostTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();
    center_line_msd_.emplace(named_center_line_segments, /*use_qtfm=*/false,
                             /*cutoff_distance=*/50.0);
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::optional<MinSegmentDistanceProblem> center_line_msd_;
};
}  // namespace

TEST_F(MsdKeepCloseToCenterLineCostTest, CostComparisonTest) {
  MsdKeepCloseToCenterLineCost<Tob> tob_cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  MsdKeepCloseToCenterLineCost<Mfob> mfob_cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CostComparisonTest::CompareCostTest(&tob_cost, &mfob_cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobSumGTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CostEvaluationTest<Mfob>::SumForAllStepsTest(&cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobUpdateTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  Mfob::StatesType xs;
  Mfob::ControlsType us;
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, /*value_only=*/false);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobDGDxTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobDDGDxDxTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateStateVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobDGDuTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostGradientResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(), &cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobDDGDuDxTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MsdKeepCloseToCenterLineCostTest, MfobDDGDuDuTest) {
  MsdKeepCloseToCenterLineCost<Mfob> cost(
      vehicle_geometry_params_, *center_line_msd_, max_distance_to_center,
      cascade_buffers, cascade_gains, name);
  CMfobTest::ExpectCostHessianResidualOrder(
      states, controls, CMfobTest::GenerateControlVariationBasis(),
      CMfobTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
