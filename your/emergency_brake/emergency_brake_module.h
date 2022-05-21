#ifndef ONBOARD_EMERGENCY_BRAKE_EMERGENCY_BRAKE_MODULE_H_
#define ONBOARD_EMERGENCY_BRAKE_EMERGENCY_BRAKE_MODULE_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "onboard/lite/lite_module.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_params.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/emergency_brake.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::emergency_brake {

// This file defines the emergency brake module. It detects unsafe conditions
// and produce a brake signal for risk mitigation.

// The input of the emergency brake module.
struct EmergencyBrakeInput {
  std::shared_ptr<const AutonomyStateProto> autonomy;
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const FenDetectionsProto> fen_detections;
  std::shared_ptr<const Chassis> chassis;
  VehicleParamApi vehicle_params;
  PlannerParamsProto planner_params;
};

// The module is triggered by the fen_detections_proto message.
class EmergencyBrakeModule : public LiteModule {
 public:
  explicit EmergencyBrakeModule(LiteClientBase* client) : LiteModule(client) {}

  void OnInit() override;

  void OnSubscribeChannels() override {
    Subscribe(&EmergencyBrakeModule::HandlePose, this);
    Subscribe(&EmergencyBrakeModule::HandleAutonomyState, this);
    Subscribe(&EmergencyBrakeModule::HandleFenDetections, this);
    Subscribe(&EmergencyBrakeModule::HandleChassis, this);
  }

  void OnSetUpTimers() override {}

  ~EmergencyBrakeModule() {}

 private:
  absl::StatusOr<EmergencyBrakeProto> MainLoop(
      const EmergencyBrakeInput& input) const;

  void HandlePose(std::shared_ptr<const PoseProto> pose) {
    input_.pose = std::move(pose);
  }

  // Use this feature instead of objects_proto for faster access of obstacles.
  void HandleFenDetections(
      std::shared_ptr<const FenDetectionsProto> fen_detections);

  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy) {
    input_.autonomy = std::move(autonomy);
  }

  void HandleChassis(std::shared_ptr<const Chassis> chassis) {
    input_.chassis = std::move(chassis);
  }

  EmergencyBrakeInput input_;
};

REGISTER_LITE_MODULE(EmergencyBrakeModule);

}  // namespace qcraft::emergency_brake

#endif  // ONBOARD_EMERGENCY_BRAKE_EMERGENCY_BRAKE_MODULE_H_
