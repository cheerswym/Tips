#include "onboard/planner/router/drive_passage_builder.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_test_util.h"

namespace qcraft::planner {

namespace {

TEST(DrivePassageBuilder, BuildStraightDrivePassage) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const auto route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);

  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        kDrivePassageKeepBehindLength);

  EXPECT_TRUE(dp_or.ok());

  SendDrivePassageToCanvas(dp_or.value(), "test/straight_drive_passage");
}

TEST(DrivePassageBuilder, BuildUturnDrivePassage) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const auto route_result = CreateAUturnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);

  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        kDrivePassageKeepBehindLength);

  EXPECT_TRUE(dp_or.ok());

  SendDrivePassageToCanvas(dp_or.value(), "test/uturn_drive_passage");
}

TEST(DrivePassageBuilder, BuildLeftTurnDrivePassage) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const auto route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);

  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        kDrivePassageKeepBehindLength);

  EXPECT_TRUE(dp_or.ok());

  SendDrivePassageToCanvas(dp_or.value(), "test/left_turn_drive_passage");
}

TEST(DrivePassageBuilder, BuildLongDrivePassage) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const auto route_result = CreateAForkLaneRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);

  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        kDrivePassageKeepBehindLength);

  EXPECT_TRUE(dp_or.ok());

  SendDrivePassageToCanvas(dp_or.value(), "test/long_drive_passage");
}

}  // namespace

}  // namespace  qcraft::planner
