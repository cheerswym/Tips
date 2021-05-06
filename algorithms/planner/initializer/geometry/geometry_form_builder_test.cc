#include "onboard/planner/initializer/geometry/geometry_form_builder.h"

#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft::planner {
namespace {
TEST(GeometryFormBuilder, GeometryFormBuilderTest) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_behind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  const auto& station0 = drive_passage.value().station(StationIndex(0));
  const auto& station5 = drive_passage.value().station(StationIndex(5));
  const DrivePassageSamplePoint state0 = {
      .xy = station0.xy(),
      .l = 0,
      .accumulated_s = station0.accumulated_s(),
      .station_index = 0};
  const DrivePassageSamplePoint state5 = {
      .xy = station5.xy(),
      .l = 0,
      .accumulated_s = station5.accumulated_s(),
      .station_index = 5};
  GeometryFormBuilder builder(&drive_passage.value(), drive_passage->end_s(),
                              0.0);
  const auto geom = builder.BuildLateralQuinticPolyGeometry(state0, state5);
  ASSERT_TRUE(geom.ok());
  EXPECT_THAT(state0.xy, Vec2dNear(geom.value().State(0.0).xy, 1e-3));
  EXPECT_THAT(state5.xy, Vec2dNear(geom.value().State(5.0).xy, 1e-3));
}

TEST(GeometryFormBuilder, QuinticLateralPolynomialTest) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_behind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  const auto& station0 = drive_passage.value().station(StationIndex(0));
  const auto& station4 = drive_passage.value().station(StationIndex(4));
  const DrivePassageSamplePoint state0 = {
      .xy = station0.xy(),
      .l = 0,
      .accumulated_s = station0.accumulated_s(),
      .station_index = 0};
  const DrivePassageSamplePoint state4 = {
      .xy = station4.lat_point(1.0),
      .l = 1.0,
      .accumulated_s = station4.accumulated_s(),
      .station_index = 4};
  const auto& passage = drive_passage.value();
  GeometryFormBuilder builder(&drive_passage.value(), drive_passage->end_s(),
                              0.0);
  const auto geom = builder.BuildLateralQuinticPolyGeometry(state0, state4);
  ASSERT_TRUE(geom.ok());
  const auto state2 = geom.value().State(geom.value().length() / 2.0);
  const auto lon_waypoint = passage.QueryFrenetLonOffsetAt(state2.xy);
  const auto lat_offset = passage.QueryFrenetLatOffsetAt(state2.xy);
  const auto lat_offset_smoothed =
      builder.smoothed_frenet_frame_->XYToSL(state2.xy).l;
  ASSERT_TRUE(lon_waypoint.ok());
  ASSERT_TRUE(lat_offset.ok());
  EXPECT_EQ(lon_waypoint.value().station_index.value(), 2);
  EXPECT_LT(fabs(lon_waypoint.value().lon_offset), 1e-2);
  EXPECT_LT(fabs(lat_offset_smoothed - 0.5), 1e-2);
}

TEST(GeometryFormBuilder, SpiralTest) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_behind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  const auto& station0 = drive_passage.value().station(StationIndex(0));
  const auto& station4 = drive_passage.value().station(StationIndex(4));
  const DrivePassageSamplePoint state0 = {
      .xy = station0.xy(),
      .l = 0,
      .accumulated_s = station0.accumulated_s(),
      .station_index = 0};
  const DrivePassageSamplePoint state4 = {
      .xy = station4.xy(),
      .l = 1.0,
      .accumulated_s = station4.accumulated_s(),
      .station_index = 4};
  GeometryFormBuilder builder(&drive_passage.value(), drive_passage->end_s(),
                              0.0);
  const auto geom = builder.BuildCubicSpiralGeometry(state0, state4);
  ASSERT_TRUE(geom.ok());
  const auto final_state = geom.value().State(geom.value().length());
  const auto init_state = geom.value().State(0.0);

  EXPECT_EQ(final_state.xy, state4.xy);
  EXPECT_EQ(init_state.xy, state0.xy);
}

}  // namespace
}  // namespace qcraft::planner
