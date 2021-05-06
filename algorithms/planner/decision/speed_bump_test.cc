#include "onboard/planner/decision/speed_bump.h"

#include "gtest/gtest.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from `dojo.planner.speed_bump.pb.txt`.
TEST(BuildSpeedBumpConstraintsTest, BuildSpeedBumpConstraints) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(-10.5);
  pose.mutable_pos_smooth()->set_y(21.4);
  pose.set_yaw(1.55);
  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"a7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto speed_bumps =
      BuildSpeedBumpConstraints(planner_semantic_map_manager, *drive_passage);
  EXPECT_EQ(speed_bumps.size(), 2);
  EXPECT_GT(speed_bumps[0].end_s() - speed_bumps[0].start_s(), 0.1);
  EXPECT_GT(speed_bumps[1].end_s() - speed_bumps[1].start_s(), 0.1);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
