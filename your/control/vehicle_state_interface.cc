#include "onboard/control/vehicle_state_interface.h"

#include "absl/status/status.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/lite/check.h"

namespace qcraft::control {
namespace {

absl::Status ConstructVehicleStateFromChassis(
    const qcraft::Chassis& chassis,
    const VehicleDriveParamsProto& vehicle_drive_params,
    VehicleStateProto* vehicle_state) {
  QCHECK_NOTNULL(vehicle_state);

  if (chassis.has_gear_location()) {
    vehicle_state->set_gear(chassis.gear_location());
  } else {
    return absl::FailedPreconditionError("Chassis has no gear location.");
  }

  if (chassis.has_driving_mode()) {
    vehicle_state->set_driving_mode(chassis.driving_mode());
  } else {
    return absl::FailedPreconditionError("Chassis has no driving mode.");
  }

  if (chassis.has_steering_percentage()) {
    vehicle_state->set_chassis_steering_percentage(
        chassis.steering_percentage());

    const double front_wheel_angle = SteeringPct2FrontWheelAngle(
        chassis.steering_percentage(), vehicle_drive_params.steer_ratio(),
        vehicle_drive_params.max_steer_angle());
    vehicle_state->set_front_wheel_steering_angle(front_wheel_angle);
  } else {
    return absl::FailedPreconditionError("Chassis has no steering percentage.");
  }

  return absl::OkStatus();
}

absl::Status ConstructVehicleStateFromPose(const qcraft::PoseProto& pose,
                                           VehicleStateProto* vehicle_state) {
  QCHECK_NOTNULL(vehicle_state);

  vehicle_state->mutable_pose()->CopyFrom(pose);

  if (pose.has_pos_smooth()) {
    vehicle_state->set_x(pose.pos_smooth().x());
    vehicle_state->set_y(pose.pos_smooth().y());
    vehicle_state->set_z(pose.pos_smooth().z());
  } else {
    return absl::FailedPreconditionError("Pose has no pos smooth.");
  }

  vehicle_state->set_yaw(pose.yaw());
  vehicle_state->set_pitch(pose.pitch());
  vehicle_state->set_roll(pose.roll());

  if (pose.has_ar_body()) {
    vehicle_state->set_angular_velocity(pose.ar_body().z());
  } else {
    return absl::FailedPreconditionError("Pose has no angular velocity.");
  }

  if (pose.has_accel_body()) {
    vehicle_state->set_linear_acceleration(pose.accel_body().x());
  } else {
    return absl::FailedPreconditionError("Pose has no linear acceleration.");
  }

  if (pose.has_vel_body()) {
    vehicle_state->set_linear_velocity(pose.vel_body().x());
  } else {
    return absl::FailedPreconditionError("Pose has no linear velocity.");
  }

  constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state->linear_velocity()) < kEpsilon) {
    vehicle_state->set_kappa(0.0);
  } else {
    vehicle_state->set_kappa(vehicle_state->angular_velocity() /
                             vehicle_state->linear_velocity());
  }

  vehicle_state->set_timestamp(pose.timestamp());

  return absl::OkStatus();
}

absl::Status ConstructVehicleStateFromAutonomyState(
    const AutonomyStateProto& autonomy_state,
    VehicleStateProto* vehicle_state) {
  QCHECK_NOTNULL(vehicle_state);

  // TODO(shijun/zhichao): need to confim is it reasonable to use
  // IS_AUTO_DRIVE();
  vehicle_state->set_is_auto_mode(
      IS_AUTO_DRIVE(autonomy_state.autonomy_state()));

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<VehicleStateProto> ConstructVehicleState(
    const AutonomyStateProto& autonomy_state, const PoseProto& pose,
    const Chassis& chassis,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  VehicleStateProto vehicle_state;
  if (const auto status = ConstructVehicleStateFromChassis(
          chassis, vehicle_drive_params, &vehicle_state);
      !status.ok()) {
    return status;
  }

  if (const auto status = ConstructVehicleStateFromPose(pose, &vehicle_state);
      !status.ok()) {
    return status;
  }

  if (const auto status = ConstructVehicleStateFromAutonomyState(
          autonomy_state, &vehicle_state);
      !status.ok()) {
    return status;
  }

  return vehicle_state;
}

}  // namespace qcraft::control
