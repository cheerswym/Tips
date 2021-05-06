#include "onboard/planner/decision/parking_brake_release.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/clock.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
namespace {
TEST(BuildParkingBrakeReleaseConstraintTest,
     BuildParkingBrakeReleaseConstraint) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(124.0);
  pose.mutable_pos_smooth()->set_y(63.0);
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, /*name_spot=*/"end");
  SendRouteLanePathToCanvas(
      semantic_map_manager, route_path,
      "test/route_build_parking_brake_release_constraints");

  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_behind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  SendDrivePassageToCanvas(
      *drive_passage,
      "test/drive_passage_build_parking_brake_release_constraints");

  const absl::Time clock_now = Clock::Now();
  const auto& passage = *drive_passage;
  const absl::Time parking_brake_release_time_1 = Clock::Now();
  const auto parking_brake_release_1 = BuildParkingBrakeReleaseConstraint(
      vehicle_geometry_params, passage, parking_brake_release_time_1,
      clock_now);
  EXPECT_TRUE(parking_brake_release_1.ok());

  const absl::Time parking_brake_release_time_2 =
      clock_now - absl::Seconds(1.1);
  const auto parking_brake_release_2 = BuildParkingBrakeReleaseConstraint(
      vehicle_geometry_params, passage, parking_brake_release_time_2,
      clock_now);
  EXPECT_FALSE(parking_brake_release_2.ok());
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
