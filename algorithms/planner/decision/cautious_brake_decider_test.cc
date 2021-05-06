#include "onboard/planner/decision/cautious_brake_decider.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
namespace {
TEST(BuildCautiousBrakeDeciderConstraintsTest,
     BuildCautiousBrakeDeciderConstraints) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);
  std::vector<PlannerObject> planner_objects;
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(53.7);
  pose.mutable_pos_smooth()->set_y(66.3);
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, /*name_spot=*/"a9_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();

  SendDrivePassageToCanvas(passage, "test/drive_passage");

  const auto speed_regions = BuildCautiousBrakeConstraints(
      planner_semantic_map_manager, passage, st_traj_mgr);
  EXPECT_GT(speed_regions.size(), 1);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
