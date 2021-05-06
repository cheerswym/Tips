#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_

#include <memory>

#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

struct RouteManagerOutput {
  void ToProto(RouteManagerOutputProto *proto) const;
  void FromProto(const SemanticMapManager *map,
                 const RouteManagerOutputProto &proto);

  // not owned, managed by route_manager
  // TODO(xiang & weijun): Replace Route by route sections.
  std::shared_ptr<Route> route;
  std::shared_ptr<Route> route_from_current;

  // TODO(xiang & weijun): add route avoid lanes.

  RouteContentProto route_content_proto;

  TurnSignal signal = TURN_SIGNAL_NONE;
  // TODO(weijun): add signal reason.
  bool rerouted = false;

  MultipleStopsRequestProto::StopProto destination_stop;
  int64_t update_id;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_OUTPUT_H_
