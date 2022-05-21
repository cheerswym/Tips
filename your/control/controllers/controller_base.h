#ifndef ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_H_

#include <cmath>
#include <limits>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/steering_protection.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

struct ControlConstraints {
  SteeringProtectionResult steering_protection_result;
  // TODO(zhichao): build a longitudinal control constraint.
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual absl::Status Init(
      const ControllerConf *control_conf,
      const VehicleGeometryParamsProto &vehicle_geometry_params,
      const VehicleDriveParamsProto &vehicle_drive_params) = 0;

  virtual absl::Status ComputeControlCommand(
      const VehicleStateProto &vehicle_state,
      const TrajectoryInterface &trajectory_interface,
      const ControlConstraints &control_constraint,
      const ControlHistoryStateManager &control_history_state_mgr,
      ControlCommand *cmd, ControllerDebugProto *controller_debug_proto) = 0;

  virtual void Reset(const Chassis &chassis,
                     const VehicleStateProto &vehicle_state) = 0;

  virtual std::string_view Name() const = 0;

  virtual void Stop() = 0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_CONTROLLER_H_
