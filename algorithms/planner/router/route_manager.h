#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_error.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

// TODO(xiang): merge RoutingInput RouteManagerInput into one struct.
struct RoutingInput {
  //////////////////////////////////////////////////////////////////////////////
  // Teleop states. but only the fields route related  will be used.
  mutable TeleopState teleop_state;
  bool res_button_pressed;  //
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car;
  std::shared_ptr<const ReroutingRequestProto> rerouting_request;
  std::shared_ptr<const RecordedRouteProto> recorded_route;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const RoutingResultProto> routing_result;
  std::shared_ptr<const DriverAction> driver_action;
  int64_t injected_teleop_micro_secs = 0;  // TODO(xiang): set value
};
struct RouteManagerInput {
  explicit RouteManagerInput(const RoutingInput &routing_input) {
    res_button_pressed = false;
    // TODO(xiang): remove this consecutive actions to a single frame action.
    auto &pending_driver_actions =
        routing_input.teleop_state.pending_driver_actions;
    while (!pending_driver_actions.empty()) {
      const DriverAction driver_action =
          std::move(pending_driver_actions.front());
      pending_driver_actions.pop();
      if (driver_action.has_press_res_button() &&
          driver_action.press_res_button()) {
        res_button_pressed = true;
      }
    }
    autonomy_state = routing_input.autonomy_state;
    pose = routing_input.pose;
    recorded_route = routing_input.recorded_route;
    localization_transform = routing_input.localization_transform;
  }
  bool res_button_pressed = false;

  // from input, constant
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const RecordedRouteProto> recorded_route;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  bool reroute = false;
};

class RouteManager {
 public:
  explicit RouteManager(const SemanticMapManager *semantic_map_manager,
                        const RouteParamProto *route_param_proto)
      : semantic_map_manager_(semantic_map_manager),
        route_param_proto_(route_param_proto) {}

  bool has_route() const { return data_.has_route(); }
  bool has_offroad_route() const;

  RouteContentProto route_content_proto() const;

  // initialization of the route manager at the start of run
  void InitRequest(
      const std::shared_ptr<const RecordedRouteProto> recorded_route,
      int64_t injected_teleop_micro_secs);

  // Return the routing result as a RouteManagerOutput struct.
  absl::StatusOr<RouteManagerOutput> Update(bool is_bus,
                                            const RouteManagerInput &input);

  absl::Status AddManualReroutingRequest(const ReroutingRequestProto &request);

  RoutingResultProto *MutableRoutingResultProto() {
    return &data_.routing_result_proto;
  }

  const RoutingResultProto &GetRoutingResultProto() const {
    return data_.routing_result_proto;
  }

  const route::RouteErrorCode &GetLastError() const { return last_error_; }
  void ClearLastError() { last_error_ = route::OkRouteStatus(); }

 private:
  static void UpdateTurnSignalOnRouteStartOrEnd(double start_next_route_time,
                                                double distance_to_next_stop,
                                                const double now_in_seconds,
                                                TurnSignal *signal);

  bool ReceivedSuccessfulRoutingResult(const RoutingResultProto &response,
                                       bool is_bus);

  absl::Status UpdateRouteFromCurrent(
      bool is_bus, const CompositeLanePath &route_path_from_current);
  void ResetRoutePathInfo();  // call when track vehicle failed.

  absl::StatusOr<RoutingResultProto> InitiateRoutingRequest(
      const RouteManagerInput &input);

  static constexpr double kReachRouteDestinationRadius = 7;  // m.

  RouteManagerState data_;

  // Not owned.
  const SemanticMapManager *semantic_map_manager_;
  const RouteParamProto *route_param_proto_;
  route::RouteErrorCode last_error_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_
