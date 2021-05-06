#include "onboard/planner/decision/end_of_current_lane_path.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from 'dojo.planner.end_of_drive_passage.pb.txt'.
TEST(BuildEndOfCurrentLanePathConstraintsTest,
     BuildEndOfCurrentLanePathConstraints) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(124.0);
  pose.mutable_pos_smooth()->set_y(63.0);
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, /*name_spot=*/"end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  SendRouteLanePathToCanvas(
      semantic_map_manager, route_path,
      "test/route_build_end_of_current_lane_path_constraints");

  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_behind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  SendDrivePassageToCanvas(
      drive_passage.value(),
      "test/drive_passage_build_end_of_current_lane_path_constraints");

  const auto& passage = drive_passage.value();
  const auto end_of_current_lane_path =
      BuildEndOfCurrentLanePathConstraint(passage);
  EXPECT_TRUE(end_of_current_lane_path.ok());
  EXPECT_NEAR(end_of_current_lane_path.value().s(), 61.78, 0.01);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
