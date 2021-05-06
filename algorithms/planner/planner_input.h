#ifndef ONBOARD_PLANNER_PLANNER_INPUT_H_
#define ONBOARD_PLANNER_PLANNER_INPUT_H_

#include <memory>
#include <string>
#include <utility>

#include "onboard/async/async_util.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/semantic_map_modification.pb.h"
#include "onboard/utils/objects_view.h"

namespace qcraft::planner {

// The input data of planner module. The struct should be read-only after
// construction.
struct PlannerInput {
  std::unique_ptr<SemanticMapManager> semantic_map_manager;
  std::unique_ptr<PlannerSemanticMapManager> planner_semantic_map_manager;

  // The state of previous planner iteration.
  std::shared_ptr<const PlannerStateProto> planner_state_proto;

  PlannerDebugProto prev_planner_debug;

  VehicleParamApi vehicle_params;

  std::shared_ptr<const PoseProto> pose;
  // Three ObjectsProto:scope different objects
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  std::shared_ptr<const ObjectsProto> av_objects;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const TrafficLightStatesProto> traffic_light_states;
  std::shared_ptr<const DriverAction> driver_action;
  std::shared_ptr<const ReroutingRequestProto> rerouting_request;
  std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car;
  std::shared_ptr<const Chassis> chassis;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const RoutingResultProto> routing_result;
  std::shared_ptr<const RouteManagerOutputProto> route_mgr_output;
  std::shared_ptr<const SemanticMapModificationProto> semantic_map_modification;
  std::shared_ptr<const SensorFovsProto> sensor_fovs;

  // Optional in onboard as planner computes prediction inside planner module.
  // Required in offboard case as planner may not able to reproduce the same
  // prediction result.
  std::shared_ptr<const ObjectsPredictionProto> prediction;
  PredictionDebugProto prediction_debug;

  // Optional in onboard mode;
  // Optional in simulation. Controlled by Flags, will help to recover route
  // from log.
  // Required input in snapshot.
  std::shared_ptr<const RecordedRouteProto> recorded_route;

  // Simulation only: use oracle prediction from log
  std::shared_ptr<const ObjectsPredictionProto> log_prediction;
  std::shared_ptr<const TrajectoryProto> log_av_trajectory;

  // Previous planner frame trajectory.
  std::shared_ptr<const TrajectoryProto> previous_trajectory_unconverted;

  // Process this struct before use it for next planner iteration.
  void BeforeNextIteration(ThreadPool* thread_pool) {
    // Despose of all objects asynchronously to avoid blocking main thread.
    ScheduleFuture(
        thread_pool,
        [_1 = std::move(prediction), _2 = std::move(remote_assist_to_car),
         _3 = std::move(routing_result), _4 = std::move(rerouting_request)] {});
  }

  std::string DebugString() const {
    if (planner_state_proto == nullptr) {
      return "empty planner state";
    } else {
      return planner_state_proto->DebugString();
    }
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_INPUT_H_
