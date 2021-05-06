#include "onboard/planner/router/route_manager_util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteManagerUtilTest, FindNextDestinationIndexTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  std::vector<std::vector<mapping::LanePoint>> lane_point_destinations = {
      {mapping::LanePoint(2448, 0.1), mapping::LanePoint(2448, 0.5),
       mapping::LanePoint(1, 0.5)},
      {mapping::LanePoint(2471, 0.3), mapping::LanePoint(2471, 0.9)},
      {mapping::LanePoint(53, 0.5)}};

  RoutingRequestProto routing_request;

  RoutingDestinationProto des1;
  des1.mutable_lane_point()->set_lane_id(2448);
  des1.mutable_lane_point()->set_fraction(0.1);

  RoutingDestinationProto des2;
  des2.mutable_lane_point()->set_lane_id(2448);
  des2.mutable_lane_point()->set_fraction(0.5);

  RoutingDestinationProto des3;
  des3.mutable_lane_point()->set_lane_id(1);
  des3.mutable_lane_point()->set_fraction(0.5);

  RoutingDestinationProto des4;
  des4.mutable_lane_point()->set_lane_id(2471);
  des4.mutable_lane_point()->set_fraction(0.3);

  RoutingDestinationProto des5;
  des5.mutable_lane_point()->set_lane_id(2471);
  des5.mutable_lane_point()->set_fraction(0.9);

  RoutingDestinationProto des6;
  des6.mutable_lane_point()->set_lane_id(53);
  des6.mutable_lane_point()->set_fraction(0.5);

  MultipleStopsRequestProto multi_stops_proto;

  auto *stop1 = multi_stops_proto.add_stops();
  *stop1->add_via_points() = des1;
  *stop1->add_via_points() = des2;
  *stop1->mutable_stop_point() = des3;
  stop1->set_stop_name("1");

  auto *stop2 = multi_stops_proto.add_stops();
  *stop2->add_via_points() = des4;
  *stop2->mutable_stop_point() = des5;
  stop2->set_stop_name("2");

  auto *stop3 = multi_stops_proto.add_stops();
  *stop3->mutable_stop_point() = des6;
  stop3->set_stop_name("3");

  multi_stops_proto.set_infinite_loop(true);

  *routing_request.mutable_multi_stops() = multi_stops_proto;

  MultipleStopsRequest multi_stops_request =
      BuildMultipleStopsRequest(semantic_map_manager, routing_request).value();

  PoseProto car_pose;
  car_pose.mutable_pos_smooth()->set_x(79.650);
  car_pose.mutable_pos_smooth()->set_y(-0.011);
  car_pose.set_yaw(0.0);
  const auto next_idx_or = FindNextDestinationIndexViaLanePoints(
      semantic_map_manager, multi_stops_request,
      [&car_pose,
       &semantic_map_manager](const RouteSectionSequence &section_seq) {
        return section_seq.IsPointOnSections(
            semantic_map_manager,
            Vec2d(car_pose.pos_smooth().x(), car_pose.pos_smooth().y()),
            car_pose.yaw(), kDefaultLaneWidth, 10.0);
      });
  if (!next_idx_or.ok()) {
    EXPECT_TRUE(false);
  }
  EXPECT_EQ(next_idx_or.value(), 3);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
