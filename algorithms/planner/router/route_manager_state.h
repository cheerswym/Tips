#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_

#include <limits>
#include <memory>
#include <vector>

#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

struct RouteManagerState {
  bool has_route() const { return route != nullptr; }
  std::shared_ptr<Route> route;
  std::shared_ptr<Route> route_from_current;
  int reached_dest_cnt = 0;

  // Queued manual requests.
  std::vector<MultipleStopsRequest> pending_routing_requests;

  // Routing request to be sent to routing_module.
  std::unique_ptr<PlannerRoutingRequestProto> active_routing_request = nullptr;
  int pending_next_destination_index = 0;
  MultipleStopsRequest pending_multi_stops;
  // Request id count.
  int64_t request_id = 0;
  MultipleStopsRequest multi_stops;
  int next_destination_index = -1;
  int reached_destination_index = -1;
  double distance_to_next_stop = std::numeric_limits<double>::infinity();
  double eta_secs_to_next_stop = std::numeric_limits<double>::infinity();

  Vec2d start_next_route_pos = {0.0, 0.0};  // smooth
  bool is_new_route = false;
  bool is_internal_reroute = false;
  bool goto_next_stop = false;
  CompositeLanePath::CompositeIndex composite_index = {0, 0};
  RoutingResultProto routing_result_proto;
  mapping::LanePoint last_lane_point = {mapping::kInvalidElementId, 0.0};
  int64 last_update_time = -1;
  double route_s = 0.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_STATE_H_
