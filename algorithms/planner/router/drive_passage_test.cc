#include "onboard/planner/router/drive_passage.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TEST(DrivePassage, QueryTestStraight) {
  FLAGS_planner_increase_lane_speed_limit_fraction = 0.0;

  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7c_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(
      const auto passage,
      BuildDrivePassage(
          planner_semantic_map_manager, route_path.lane_paths().front(),
          /*anchor_point=*/mapping::LanePoint(),
          route_sections.planning_horizon(planner_semantic_map_manager),
          /*keep_bebind_len=*/0.0),
      "Building drive passage failed!");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();
  // straight lane point
  const Vec2d query_point(42.2, 4.0);

  ASSIGN_OR_DIE(const auto point_xy, passage.QueryPointXYAtS(42.2));
  EXPECT_LT(point_xy.DistanceTo(Vec2d(42.2, 0.06)), 1e-3);

  ASSIGN_OR_DIE(const auto speed_limit, passage.QuerySpeedLimitAt(query_point));
  EXPECT_NEAR(speed_limit, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto curb_offset, passage.QueryCurbOffsetAt(query_point));
  EXPECT_NEAR(curb_offset.first, -9.2907, 1e-4);
  EXPECT_NEAR(curb_offset.second, 5.69, 1e-4);

  ASSIGN_OR_DIE(const auto curb_offset1, passage.QueryCurbOffsetAtS(42.2));
  EXPECT_NEAR(curb_offset1.first, -5.3503, 1e-4);
  EXPECT_NEAR(curb_offset1.second, 9.6304, 1e-4);

  ASSIGN_OR_DIE(const auto lane_boundaries,
                passage.QueryEnclosingLaneBoundariesAt(query_point));
  EXPECT_EQ(lane_boundaries.first->type, StationBoundaryType::BROKEN_WHITE);
  EXPECT_NEAR(lane_boundaries.first->lat_offset, -2.2427, 1e-4);
  EXPECT_EQ(lane_boundaries.second->type,
            StationBoundaryType::SOLID_DOUBLE_YELLOW);
  EXPECT_NEAR(lane_boundaries.second->lat_offset, 1.18, 1e-4);

  ASSIGN_OR_DIE(const auto tangent, passage.QueryTangentAt(query_point));
  EXPECT_NEAR(tangent.Angle(), 0.0, 2e-3);

  ASSIGN_OR_DIE(const auto tangent1, passage.QueryTangentAtS(42.2));
  EXPECT_NEAR(tangent1.Angle(), 0.0, 2e-3);

  ASSIGN_OR_DIE(const auto frenet_lon_offset,
                passage.QueryFrenetLonOffsetAt(query_point));
  EXPECT_EQ(frenet_lon_offset.station_index.value(), 42);
  EXPECT_NEAR(frenet_lon_offset.lon_offset, 0.204, 1e-3);

  ASSIGN_OR_DIE(const auto frenet_lat_offset,
                passage.QueryFrenetLatOffsetAt(query_point));
  EXPECT_NEAR(frenet_lat_offset, 3.94, 1e-3);

  FrenetCoordinate frenet_coord;
  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryFrenetCoordinateAt(Vec2d(10.0, -2.0)));
  EXPECT_NEAR(frenet_coord.s, 10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -2.0, 0.1);
  EXPECT_FALSE(passage.QueryFrenetCoordinateAt(Vec2d(10.0, -10.0)).ok());
  EXPECT_FALSE(passage.QueryFrenetCoordinateAt(Vec2d(-10.0, -10.0)).ok());

  ASSIGN_OR_DIE(frenet_coord, passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                                  Vec2d(10.0, -10.0)));
  EXPECT_NEAR(frenet_coord.s, 10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -10.0, 0.1);
  EXPECT_FALSE(
      passage.QueryLaterallyUnboundedFrenetCoordinateAt(Vec2d(-10.0, -10.0))
          .ok());

  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryUnboundedFrenetCoordinateAt(Vec2d(-10.0, -10.0)));
  EXPECT_NEAR(frenet_coord.s, -10.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -10.0, 0.1);

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in query operations";

  SendDrivePassageToCanvas(passage, "drive_passage2");
  EXPECT_EQ(true, true);
}

TEST(DrivePassage, QueryTestBox) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(-3.5);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7c_s3");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(
      const auto passage,
      BuildDrivePassage(
          planner_semantic_map_manager, route_path.lane_paths().front(),
          /*anchor_point=*/mapping::LanePoint(),
          route_sections.planning_horizon(planner_semantic_map_manager),
          /*keep_bebind_len=*/0.0),
      "Building drive passage failed!");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();

  const Box2d small_box = Box2d::CreateAABox({6.0, 0.0}, {10.0, -3.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(small_box));

  const Box2d wide_box = Box2d::CreateAABox({6.0, 10.0}, {10.0, -10.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(wide_box));

  const Box2d long_box = Box2d::CreateAABox({-5.0, 10.0}, {5.0, 0.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(long_box));

  const Box2d super_long_box = Box2d::CreateAABox({-5.0, 10.0}, {100.0, 0.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(super_long_box));

  const Box2d super_large_box =
      Box2d::CreateAABox({-10.0, 10.0}, {100.0, -60.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(super_large_box));

  const Box2d corner_box = Box2d::CreateAABox({70.0, -7.0}, {80.0, -14.0});
  EXPECT_OK(passage.QueryFrenetBoxAt(corner_box));

  const Box2d back_box = Box2d::CreateAABox({-10.0, 3.0}, {-3.0, -3.0});
  EXPECT_FALSE(passage.QueryFrenetBoxAt(back_box).ok());

  const Box2d side_box = Box2d::CreateAABox({6.0, -7.0}, {10.0, -10.0});
  EXPECT_FALSE(passage.QueryFrenetBoxAt(side_box).ok());

  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in query operations";

  SendDrivePassageToCanvas(passage, "drive_passage_query_box");
}

TEST(DrivePassage, BatchQuerySL) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7c_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in routing";

  start_time = absl::Now();
  ASSIGN_OR_DIE(
      const auto passage,
      BuildDrivePassage(
          planner_semantic_map_manager, route_path.lane_paths().front(),
          /*anchor_point=*/mapping::LanePoint(),
          route_sections.planning_horizon(planner_semantic_map_manager),
          /*keep_bebind_len=*/0.0),
      "Building drive passage failed!");
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";

  ASSERT_FALSE(passage.empty()) << "Empty drive passage!";

  start_time = absl::Now();
  // straight lane point
  const Vec2d query_point1(42.2, 4.0);
  ASSIGN_OR_DIE(const auto lon_pt,
                passage.QueryFrenetLonOffsetAt(query_point1));
  ASSIGN_OR_DIE(const auto lat, passage.QueryFrenetLatOffsetAt(query_point1));
  auto lon_idx = lon_pt.station_index;
  ++lon_idx;
  ++lon_idx;
  const auto& next_station = passage.station(lon_idx);
  const Vec2d query_point2 = next_station.lat_point(lat);
  std::vector<Vec2d> queries;
  queries.push_back(query_point1);
  queries.push_back(query_point2);
  ASSIGN_OR_DIE(const auto frenet_coords,
                passage.BatchQueryFrenetCoordinates(queries));
  EXPECT_NEAR(
      passage.station(lon_pt.station_index).accumulated_s() + lon_pt.lon_offset,
      frenet_coords[0].s, 0.01);
  EXPECT_NEAR(lat, frenet_coords[0].l, 0.01);
  EXPECT_NEAR(next_station.accumulated_s(), frenet_coords[1].s, 0.01);
  EXPECT_NEAR(lat, frenet_coords[1].l, 0.01);
}

TEST(DrivePassage, QueryTestCrossing) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
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
      RoutingToNameSpot(semantic_map_manager, pose, "b7_e2_start");
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
  SendDrivePassageToCanvas(passage, "test_drive_passage_query_crossing");
  // point at crossing
  const Vec2d query_point(83.0, 0.0);
  constexpr double kDistanceError = 0.05;  // m.
  ASSIGN_OR_DIE(const auto point_xy, passage.QueryPointXYAtS(82.9));
  EXPECT_LT(point_xy.DistanceTo(Vec2d(82.9, 0.03)), kDistanceError)
      << point_xy.DebugString();

  ASSIGN_OR_DIE(const auto speed_limit, passage.QuerySpeedLimitAt(query_point));
  EXPECT_NEAR(speed_limit, 11.111, 1e-3);

  ASSIGN_OR_DIE(const auto curb_offset, passage.QueryCurbOffsetAt(query_point));
  EXPECT_NEAR(curb_offset.first, -10.0, kDistanceError);
  EXPECT_NEAR(curb_offset.second, 10.0, kDistanceError);

  ASSIGN_OR_DIE(const auto curb_offset1, passage.QueryCurbOffsetAtS(82.9));
  EXPECT_NEAR(curb_offset1.first, -10.0, kDistanceError);
  EXPECT_NEAR(curb_offset1.second, 10.0, kDistanceError);

  ASSIGN_OR_DIE(const auto lane_boundaries,
                passage.QueryEnclosingLaneBoundariesAt(query_point));
  EXPECT_EQ(lane_boundaries.first->type, StationBoundaryType::VIRTUAL_CURB);
  EXPECT_EQ(lane_boundaries.second->type, StationBoundaryType::VIRTUAL_CURB);

  ASSIGN_OR_DIE(const auto tangent, passage.QueryTangentAt(query_point));
  EXPECT_NEAR(tangent.Angle(), 0.0, 1e-3);

  ASSIGN_OR_DIE(const auto frenet_lon_offset,
                passage.QueryFrenetLonOffsetAt(query_point));
  EXPECT_EQ(frenet_lon_offset.station_index.value(), 83);
  EXPECT_NEAR(frenet_lon_offset.lon_offset, 0.0, kDistanceError);

  ASSIGN_OR_DIE(const auto frenet_lat_offset,
                passage.QueryFrenetLatOffsetAt(query_point));
  EXPECT_NEAR(frenet_lat_offset, 0.0, kDistanceError);

  FrenetCoordinate frenet_coord;
  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryFrenetCoordinateAt(Vec2d(85.0, -5.0)));
  EXPECT_NEAR(frenet_coord.s, 85.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -5.0, 0.1);

  ASSIGN_OR_DIE(frenet_coord, passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                                  Vec2d(85.0, -13.0)));
  EXPECT_NEAR(frenet_coord.s, 85.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -13.0, 0.1);

  ASSIGN_OR_DIE(frenet_coord,
                passage.QueryUnboundedFrenetCoordinateAt(Vec2d(130.0, -3.0)));
  EXPECT_NEAR(frenet_coord.s, 130.0, 0.1);
  EXPECT_NEAR(frenet_coord.l, -3.0, 0.1);
}

}  // namespace
}  // namespace qcraft::planner
