#include "onboard/planner/decision/end_of_path_boundary.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from 'dojo.planner.end_of_drive_passage.pb.txt'.
TEST(BuildEndOfPathBoundaryConstraintsTest, BuildEndOfPathBoundaryConstraints) {
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
      "test/route_build_end_of_path_boundary_constraints");

  ASSIGN_OR_DIE(
      const auto drive_passage,
      BuildDrivePassage(
          planner_semantic_map_manager, route_path.lane_paths().front(),
          /*anchor_point=*/mapping::LanePoint(),
          route_sections.planning_horizon(planner_semantic_map_manager),
          /*keep_behind_len=*/10.0),
      "Building drive passage failed!");
  SendDrivePassageToCanvas(
      drive_passage,
      "test/drive_passage_build_end_of_path_boundary_constraints");

  const auto vehicle_geom = DefaultVehicleGeometry();
  const SpacetimeTrajectoryManager st_traj_mgr({});
  LaneChangeStateProto lc_state;
  SmoothedReferenceLineResultMap smooth_result_map;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);

  ASSIGN_OR_DIE(const auto path_bound_lk,
                BuildPathBoundaryFromPose(
                    planner_semantic_map_manager, drive_passage,
                    ConvertToTrajPointProto(pose), vehicle_geom, st_traj_mgr,
                    lc_state, smooth_result_map, /*borrow_lane_boundary=*/false,
                    /*should_smooth=*/false));

  const auto end_of_path_boundary =
      BuildEndOfPathBoundaryConstraint(drive_passage, path_bound_lk);
  EXPECT_TRUE(end_of_path_boundary.ok());
  EXPECT_NEAR(end_of_path_boundary.value().s(),
              drive_passage.lane_path().length(), 0.01);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
