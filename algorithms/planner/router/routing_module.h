#ifndef ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_

#include <memory>
#include <utility>

#include "onboard/lite/lite_module.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_manager.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/q_run_event_states.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {
class RouteManager;
struct RoutingInput;

class RoutingModule : public LiteModule {
 public:
  explicit RoutingModule(LiteClientBase *client);
  virtual ~RoutingModule() = default;

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 protected:
  void HandlePose(std::shared_ptr<const PoseProto> pose) {
    input_.pose = std::move(pose);
  }

  void HandleLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform) {
    input_.localization_transform = std::move(localization_transform);
  }

  void HandleRemoteAssistToCar(
      std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car) {
    input_.remote_assist_to_car = std::move(remote_assist_to_car);
  }

  void HandleReroutingRequest(
      std::shared_ptr<const ReroutingRequestProto> rerouting_request);

  void HandleRunEventStates(std::shared_ptr<const QRunEventStatesProto> states);

  void HandleRecordedRoute(
      std::shared_ptr<const RecordedRouteProto> recorded_route) {
    input_.recorded_route = std::move(recorded_route);
  }

  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy) {
    input_.autonomy_state = std::move(autonomy);
  }

  void HandleDriverAction(std::shared_ptr<const DriverAction> driver_action) {
    input_.teleop_state.AddDriveAction(*driver_action);
  }

  // -----------------teleop related with route.
  void ProcessTeleopProto(const RemoteAssistToCarProto &teleop_proto,
                          RouteManager *route_manager);

  void MaybeInjectTeleopProto(RouteManager *route_manager,
                              int64_t injected_teleop_micro_secs);

  void PublishRoutingResult();

  // Publish current state
  void MainLoop();

 private:
  void CleanBeforeNextIteration();

 private:
  std::shared_ptr<SemanticMapManager> semantic_map_manager_;
  RoutingStateProto::RouteState route_state_;
  RoutingInput input_;
  std::unique_ptr<RouteManager> route_mgr_;
  int64_t last_publish_microsecs_ = 0;
  RouteParamProto route_param_proto_;
};

REGISTER_LITE_MODULE(RoutingModule);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_
