#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_REROUTE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_REROUTE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

struct RerouteInput {
  const SemanticMapManager *semantic_map_manager = nullptr;
  const PoseProto *pose = nullptr;
  const RoutingRequestProto *origin_routing_request = nullptr;
  const MultipleStopsRequest *multi_stops = nullptr;
  const RouteParamProto *route_param_proto = nullptr;
  std::int64_t cur_time_micros = 0;
  double route_s = 0.0;
  std::int64_t next_request_id = 0;
  bool is_bus = false;
};

struct RerouteOutput {
  RoutingResultProto routing_result_proto;
  std::unique_ptr<Route> route;
  std::unique_ptr<Route> route_from_current;
};

absl::StatusOr<RerouteOutput> RerouteFromPose(const RerouteInput &input);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_REROUTE_H_
