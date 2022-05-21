#ifndef ONBOARD_CONTROL_CONTROL_VALIDATION_H_
#define ONBOARD_CONTROL_CONTROL_VALIDATION_H_

#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

bool ValidateControlOutput(
    const TrajectoryInterface& trajectory_interface,
    const VehicleDriveParamsProto& vehicle_drive_param,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ControllerConf& controller_conf, const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_VALIDATION_H_
