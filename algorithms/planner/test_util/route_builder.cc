#include "onboard/planner/test_util/route_builder.h"

#include <string>

#include "onboard/planner/router/route_engine.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/file_util.h"
namespace qcraft {
namespace planner {

RouteParamProto CreateDefaultRouteParam() {
  RouteParamProto route_param_proto;
  file_util::TextFileToProto(FLAGS_route_default_params_file,
                             &route_param_proto);
  return route_param_proto;
}

CompositeLanePath RoutingToNameSpot(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    std::string name_spot) {
  RoutingRequestProto routing_request;
  routing_request.add_destinations()->set_named_spot(name_spot);

  const auto route_lane_path = SearchForRouteLanePathFromPose(
      semantic_map_manager, CreateDefaultRouteParam(), pose, routing_request,
      /*blacklist=*/{},
      /*use_time=*/true);
  if (!route_lane_path.ok()) {
    return CompositeLanePath();
  }
  return route_lane_path.value().second;
}

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const CompositeLanePath::LanePoint &lane_point) {
  RoutingRequestProto routing_request;
  auto *lane_point_proto =
      routing_request.add_destinations()->mutable_lane_point();
  lane_point_proto->set_lane_id(lane_point.lane_id());
  lane_point_proto->set_fraction(lane_point.fraction());

  const auto route_lane_path = SearchForRouteLanePathFromPose(
      semantic_map_manager, CreateDefaultRouteParam(), pose, routing_request,
      /*blacklist=*/{},
      /*use_time=*/true);
  if (!route_lane_path.ok()) {
    return route_lane_path.status();
  }
  return route_lane_path.value().second;
}

absl::StatusOr<CompositeLanePath> RoutingToGlobalPoint(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const Vec2d &global_point) {
  RoutingRequestProto routing_request;
  auto *global_point_proto =
      routing_request.add_destinations()->mutable_global_point();
  global_point_proto->set_longitude(global_point.x());
  global_point_proto->set_latitude(global_point.y());
  global_point_proto->set_altitude(0.0);

  const auto route_lane_path = SearchForRouteLanePathFromPose(
      semantic_map_manager, CreateDefaultRouteParam(), pose, routing_request,
      /*blacklist=*/{},
      /*use_time=*/true);
  if (!route_lane_path.ok()) {
    return route_lane_path.status();
  }
  return route_lane_path.value().second;
}
}  // namespace planner
}  // namespace qcraft
