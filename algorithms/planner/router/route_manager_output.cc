#include "onboard/planner/router/route_manager_output.h"

#include "onboard/global/trace.h"

namespace qcraft {
namespace planner {

void RouteManagerOutput::ToProto(RouteManagerOutputProto *proto) const {
  if (route != nullptr) {
    route->ToProto(proto->mutable_route());
  }
  if (route_from_current != nullptr) {
    route_from_current->ToProto(proto->mutable_route_from_current());
  }
  *proto->mutable_route_content() = route_content_proto;

  proto->set_signal(signal);
  proto->set_rerouted(rerouted);

  *proto->mutable_destination_stop() = destination_stop;
  proto->set_update_id(update_id);
}

void RouteManagerOutput::FromProto(const SemanticMapManager *map,
                                   const RouteManagerOutputProto &proto) {
  SCOPED_QTRACE("RouteManagerOutput::FromProto");

  if (proto.has_route()) {
    route = std::make_shared<Route>(map, proto.route());
  }
  if (proto.has_route_from_current()) {
    route_from_current =
        std::make_shared<Route>(map, proto.route_from_current());
  }
  route_content_proto = proto.route_content();
  signal = proto.signal();
  rerouted = proto.rerouted();

  destination_stop = proto.destination_stop();
  update_id = proto.update_id();
}

}  // namespace planner
}  // namespace qcraft
