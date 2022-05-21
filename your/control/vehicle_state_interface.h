#ifndef ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_
#define ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_

#include "absl/status/statusor.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

// Construct the current vehicle state from autonomy state, pose, and chassis.
absl::StatusOr<VehicleStateProto> ConstructVehicleState(
    const AutonomyStateProto& autonomy_state, const PoseProto& pose,
    const Chassis& chassis,
    const VehicleDriveParamsProto& vehicle_drive_params);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_
