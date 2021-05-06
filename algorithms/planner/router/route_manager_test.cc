#include "onboard/planner/router/route_manager.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/planner/router/route_ego_tracker.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteManager, Proto) {
  // SetMap("dojo");
  // SemanticMapManager semantic_map_manager;
  // semantic_map_manager.LoadWholeMap().Build();

  // RouteManagerStateProto proto;

  // RouteProto route;
  // route.add_lane_ids(2476);
  // route.add_lane_ids(2491);

  // *proto.mutable_route() = route;
  // *proto.mutable_route_from_current() = route;
  // proto.set_reached_dest_cnt(1);

  // RoutingRequestProto request;
  // request.add_destinations()->set_named_spot("dest");

  // MultipleStops multi_stops(request);
  // MultipleStopsRequestProto multi_stops_proto;
  // multi_stops.ToProto(&multi_stops_proto);

  // *proto.add_pending_routing_requests() = multi_stops_proto;
  // *proto.add_pending_routing_requests() = multi_stops_proto;

  // PlannerRoutingRequestProto planner_request;
  // planner_request.set_request_id(1234);
  // *proto.mutable_active_routing_request() = planner_request;

  // proto.set_last_request_frame(5);

  // proto.set_request_id(10);

  // RoutingResultProto routing_result;
  // routing_result.set_cost(10.0);
  // *proto.add_routing_response_queue() = routing_result;

  // *proto.mutable_multi_stops() = multi_stops_proto;

  // proto.set_next_destination_index(6);

  // proto.set_distance_to_next_stop(10.0);

  // proto.set_goto_next_stop(false);

  // proto.set_start_next_route_time(10.0);

  // RouteManager route_manager(&semantic_map_manager);
  // route_manager.FromProto(proto);

  // RouteManagerStateProto serialized;
  // route_manager.ToProto(&serialized);

  // RouteManager from_proto(&semantic_map_manager);
  // from_proto.FromProto(serialized);

  // RouteManagerStateProto to;
  // from_proto.ToProto(&to);

  // EXPECT_THAT(to, ProtoEq(serialized));
}
TEST(RouteManager, RouteSTest) {
  CompositeLanePath composite_lane_path;
  mapping::LanePath lane_path1;
  lane_path1.lane_ids_ = {1, 2, 3};
  mapping::LanePath lane_path2;
  lane_path2.lane_ids_ = {4, 5};
  mapping::LanePath lane_path3;
  lane_path3.lane_ids_ = {6};
  composite_lane_path.lane_paths_ = {lane_path1, lane_path2, lane_path3};
  composite_lane_path.transitions_ = {};
  composite_lane_path.lane_path_end_s_ = {0, 10, 20, 30};

  CompositeLanePath::CompositeIndex composite_index = {0, 1};  // lane_id=2
  std::vector<mapping::ElementId> section_lane_ids = {3, 30, 300};
  mapping::ElementId to_find_lane_id = 3;
  absl::StatusOr<PointToCompositeLanePath> foundIndex_or =
      FindLaneFromLastCompositeIndex(composite_lane_path, composite_index,
                                     section_lane_ids, to_find_lane_id);
  CHECK(foundIndex_or.ok());
  CHECK(foundIndex_or->point_on_route_lane);
  CompositeLanePath::CompositeIndex expected_composite_index(0, 2);
  EXPECT_EQ(foundIndex_or->composite_index,
            expected_composite_index);  // perfect match

  to_find_lane_id = 30;
  foundIndex_or = FindLaneFromLastCompositeIndex(
      composite_lane_path, composite_index, section_lane_ids, to_find_lane_id);

  CHECK(foundIndex_or.ok());
  EXPECT_FALSE(foundIndex_or->point_on_route_lane);
  EXPECT_EQ(foundIndex_or->composite_index, expected_composite_index);

  foundIndex_or = FindLaneFromLastCompositeIndex(
      composite_lane_path, composite_index, {5, 50, 500}, 50);
  CHECK(foundIndex_or.ok());
  EXPECT_EQ(foundIndex_or->composite_index,
            CompositeLanePath::CompositeIndex(1, 1));
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
