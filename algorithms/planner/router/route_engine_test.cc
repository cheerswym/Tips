#include "onboard/planner/router/route_engine.h"

#include "gtest/gtest.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/test_util/route_builder.h"
namespace qcraft {
namespace planner {
namespace {

TEST(SearchForRouteLanepathFromLanePoint, Test1) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::LanePoint origin(3, 0.5);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const auto sections_lanes_or = SearchForRouteLanePathFromLanePoint(
      semantic_map_manager, origin, routing_request, blacklist,
      /*use_time=*/true);

  mapping::LanePath lane_path0(&semantic_map_manager,
                               {3, 950, 35, 2472, 54, 168, 129}, 0.5, 0.5);

  CompositeLanePath expected_clp({lane_path0});
  bool expected = false;
  if (sections_lanes_or.ok()) {
    expected = expected_clp.IsEqual(sections_lanes_or->second);
  }

  EXPECT_TRUE(expected);
}

TEST(SearchForRouteLanepathFromPose, Test2) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PoseProto car_pose;
  car_pose.mutable_pos_smooth()->set_x(27.8);
  car_pose.mutable_pos_smooth()->set_y(1.161);
  car_pose.set_yaw(-0.0226);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const auto res = SearchForRouteLanePathFromPose(
      semantic_map_manager, CreateDefaultRouteParam(), car_pose,
      routing_request, blacklist,
      /*use_time=*/true);

  mapping::LanePath lane_path0(&semantic_map_manager,
                               {2448, 1, 34, 2471, 53, 169, 130},
                               0.49726629007832024, 0.5);
  mapping::LanePath lane_path1(&semantic_map_manager, {129}, 0.5, 0.5);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0,
      .lc_left = false,
      .lc_section_id = 3047,
  };
  CompositeLanePath expected_clp({lane_path0, lane_path1}, {trans1});

  bool expected = false;
  if (res.ok()) {
    expected = expected_clp.IsEqual(res.value().second);
  }
  EXPECT_TRUE(expected);
}

TEST(SearchForRouteLanePathFromDestination, Test3) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RoutingDestinationProto destination_proto;
  destination_proto.mutable_lane_point()->set_lane_id(3);
  destination_proto.mutable_lane_point()->set_fraction(0.5);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(129);
  dest1.mutable_lane_point()->set_fraction(0.5);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const auto res = SearchForRouteLanePathFromDestination(
      semantic_map_manager, destination_proto, routing_request, blacklist,
      /*use_time=*/true);

  mapping::LanePath lane_path0(&semantic_map_manager,
                               {3, 950, 35, 2472, 54, 168, 129}, 0.5, 0.5);

  CompositeLanePath expected_clp({lane_path0});
  bool expected = false;
  if (res.ok()) {
    expected = expected_clp.IsEqual(res.value().second);
  }

  EXPECT_TRUE(expected);
}

TEST(SearchForRouteLanePathAlongLanePoints, Test4) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::LanePoint origin(3, 0.5);

  mapping::LanePoint destination1(2471, 0.5);
  std::vector<mapping::LanePoint> destinations;
  destinations.push_back(destination1);

  mapping::LanePoint destination2(113, 0.5);
  destinations.push_back(destination2);

  absl::flat_hash_set<mapping::ElementId> blacklist = {950, 53};

  const auto res = SearchForRouteLanePathAlongLanePoints(
      semantic_map_manager, origin, absl::MakeSpan(destinations), blacklist,
      /*use_time=*/true);

  mapping::LanePath lane_path0(&semantic_map_manager, {3}, 0.5, 1.0);
  mapping::LanePath lane_path1(&semantic_map_manager, {2448, 1, 34, 2471}, 1.0,
                               1.0);
  mapping::LanePath lane_path2(&semantic_map_manager, {2470, 51, 186, 113}, 1.0,
                               0.5);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3034};
  CompositeLanePath::TransitionInfo trans2 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3045};

  CompositeLanePath expected_clp({lane_path0, lane_path1, lane_path2},
                                 {trans1, trans2});

  bool expected = false;
  if (res.ok()) {
    expected = expected_clp.IsEqual(res.value().second);
  }
  EXPECT_TRUE(expected);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
