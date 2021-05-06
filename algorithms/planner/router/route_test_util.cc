#include "onboard/planner/router/route_test_util.h"

#include <string>

#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_engine.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {

namespace {
TestRouteResult CreateTestRouteResult(
    const PoseProto &pose, const RoutingRequestProto &routing_request,
    const std::string &route_name) {
  SetMap("dojo");
  TestRouteResult result;
  result.smm = std::make_unique<mapping::SemanticMapManager>();
  result.smm->LoadWholeMap().Build();
  result.pose = pose;
  const auto route_lane_path_or = SearchForRouteLanePathFromPose(
      *result.smm, CreateDefaultRouteParam(), result.pose, routing_request,
      /*blacklist=*/{}, /*use_time=*/true);
  if (route_lane_path_or.ok()) {
    result.route_lane_path = std::move(route_lane_path_or)->second;
    result.route_sections =
        RouteSectionsFromCompositeLanePath(*result.smm, result.route_lane_path);
  }

  SendRouteLanePathToCanvas(*result.smm, result.route_lane_path, route_name);

  return result;
}
}  // namespace

TestRouteResult CreateAStraightForwardRouteInUrbanDojo() {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(117.471, 0.0), 0.0, Vec2d(0.0, 0.0));

  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot("b7_e2_end");

  return CreateTestRouteResult(pose, routing_request,
                               "test/straight_route_in_urban_dojo");
}

TestRouteResult CreateAStraightForwardRouteWithSolidInUrbanDojo() {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(234.605, 0.0), 0.0, Vec2d(0.0, 0.0));

  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot("b8_e2_start");

  return CreateTestRouteResult(pose, routing_request,
                               "test/straight_route_with_solid_in_urban_dojo");
}

TestRouteResult CreateAUturnRouteInDojo() {
  const auto pose = CreatePose(/*timestamp=*/10.0, Vec2d(83.75, -19.5), -1.584,
                               Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot("b7_e3_start");

  return CreateTestRouteResult(pose, routing_request,
                               "test/uturn_route_in_dojo");
}

TestRouteResult CreateALeftTurnRouteInDojo() {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(238.0, 70.0), 0.0, Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot("8a_n2");

  return CreateTestRouteResult(pose, routing_request,
                               "test/left_turn_route_in_dojo");
}

TestRouteResult CreateALeftTurnWithDirectionInfoRouteInDojo() {
  const auto pose = CreatePose(/*timestamp=*/10.0, Vec2d(848.186, -594.204),
                               0.0, Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  auto *dest = routing_request.add_destinations();
  dest->mutable_global_point()->set_longitude(0.00014181231986573297);
  dest->mutable_global_point()->set_latitude(-8.3332314787164984e-05);
  dest->mutable_global_point()->set_altitude(0.0);

  return CreateTestRouteResult(
      pose, routing_request,
      "test/left_turn_with_direction_info_route_in_dojo");
}

TestRouteResult CreateARightTurnWithDirectionInfoRouteInDojo() {
  const auto pose = CreatePose(/*timestamp=*/10.0, Vec2d(848.186, -594.204),
                               0.0, Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  auto *dest = routing_request.add_destinations();
  dest->mutable_global_point()->set_longitude(0.0001412980847158321);
  dest->mutable_global_point()->set_latitude(-0.00012006638556027881);
  dest->mutable_global_point()->set_altitude(0.0);

  return CreateTestRouteResult(
      pose, routing_request,
      "test/right_turn_with_direction_info_route_in_dojo");
}

TestRouteResult CreateAForkLaneRouteInDojo() {
  const auto pose = CreatePose(/*timestamp=*/10.0, Vec2d(382.786, -166.23),
                               -3.13, Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  auto *dest = routing_request.add_destinations();
  dest->mutable_global_point()->set_longitude(0.0000089847599078);
  dest->mutable_global_point()->set_latitude(-0.0000247182312831);
  dest->mutable_global_point()->set_altitude(0.0);

  return CreateTestRouteResult(pose, routing_request,
                               "test/fork_lane_route_in_dojo");
}

TestRouteResult CreateASingleLaneRouteInDojo() {
  const auto pose = CreatePose(/*timestamp=*/10.0, Vec2d(-222.14, 3.54), 0.0,
                               Vec2d(0.0, 0.0));
  RoutingRequestProto routing_request;
  auto *dest = routing_request.add_destinations();
  dest->mutable_global_point()->set_longitude(-0.0000099403226573);
  dest->mutable_global_point()->set_latitude(0.0000005833861421);
  dest->mutable_global_point()->set_altitude(0.0);

  return CreateTestRouteResult(pose, routing_request,
                               "test/single_lane_route_in_dojo");
}

TestRouteResult CreateAContinuousLaneChangeRouteInDojo() {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(144.5, -217), 0.0, Vec2d(0.0, 0.0));

  mapping::LanePointProto *destination = new mapping::LanePointProto();
  destination->set_lane_id(1492);
  destination->set_fraction(0.2);

  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_allocated_lane_point(destination);

  return CreateTestRouteResult(pose, routing_request,
                               "test/continuous_lc_in_dojo");
}

TestRouteResult CreateAContinuousLaneChangeRouteWithSolidInDojo() {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(120, 3.5), 0.0, Vec2d(0.0, 0.0));

  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot("b7_e2_end");

  return CreateTestRouteResult(pose, routing_request,
                               "test/continuous_lc_with_solid_in_dojo");
}

}  // namespace qcraft::planner
