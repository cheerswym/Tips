#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_RECORDER_MODULE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_RECORDER_MODULE_H_

#include <memory>

#include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_module.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

class RouteRecorderModule : public LiteModule {
 public:
  explicit RouteRecorderModule(LiteClientBase *client) : LiteModule(client) {}

  void OnSubscribeChannels() override {
    Subscribe(&RouteRecorderModule::ForwardRouteProto, this);
  }

  void OnInit() override {}
  void OnSetUpTimers() override {}

 protected:
  void ForwardRouteProto(std::shared_ptr<const TrajectoryProto> trajectory) {
    RecordedRouteProto msg;
    *msg.mutable_header() = trajectory->header();
    *msg.mutable_lane_path() = trajectory->route_proto().lane_path();
    *msg.mutable_routing_request() =
        trajectory->route_proto().routing_request();
    QLOG_IF_NOT_OK(WARNING, Publish(msg));
  }
};

REGISTER_LITE_MODULE(RouteRecorderModule);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_RECORDER_MODULE_H_
