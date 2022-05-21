#include "onboard/control/control_validation.h"

#include <algorithm>
#include <string>

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

DEFINE_bool(enable_mpc_validation, false, "whether validate mpc result.");

namespace qcraft {
namespace control {

namespace {

constexpr double kMPCPredictedDeviationKickoutThreshold = 5.0;  // m.
constexpr double kMPCPredictedDeviationQEventThreshold = 1.0;   // m.
constexpr double kEpsilon = 1e-6;

bool ValidateLongitudinalControlOutput(
    const ControllerConf& controller_conf, const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto) {
  auto* debug = controller_debug_proto->mutable_validation_result_proto();

  const double accel_mpc = control_cmd.acceleration();
  const double accel_offset = control_cmd.acceleration_offset();
  const double accel_calibration = control_cmd.acceleration_calibration();
  const double throttle = control_cmd.throttle();
  const double brake = control_cmd.brake();
  const double e_brake = control_cmd.e_brake();

  const double vehicle_max_accel = controller_conf.has_max_acceleration_cmd()
                                       ? controller_conf.max_acceleration_cmd()
                                       : kMaxAccelerationCmd;
  const double vehicle_max_decel = controller_conf.has_max_deceleration_cmd()
                                       ? controller_conf.max_deceleration_cmd()
                                       : kMaxDecelerationCmd;
  const double accel_offset_limit = kSinSlopeLimit * kGravitationalAcceleration;

  const bool accel_mpc_valid = InRange(accel_mpc, vehicle_max_decel - kEpsilon,
                                       vehicle_max_accel + kEpsilon);
  const bool accel_offset_valid =
      InRange(accel_offset, -accel_offset_limit - kEpsilon,
              accel_offset_limit + kEpsilon);
  const bool accel_calibration_valid = InRange(
      accel_calibration, vehicle_max_decel - accel_offset_limit - kEpsilon,
      vehicle_max_accel + accel_offset_limit + kEpsilon);

  if (!accel_mpc_valid || !accel_offset_valid || !accel_calibration_valid) {
    debug->set_error_code(
        ControlValidationResultProto::ACCELERATION_OVER_LIMIT);
    const std::string error_message = absl::StrCat(
        "accel_mpc, ", accel_mpc, ", accel_offset, ", accel_offset,
        ", or accel_calibration, ", accel_calibration, ", is out of range.");
    debug->set_error_message(error_message);
    return false;
  }

  constexpr double kCmdUpperLimit = 100.0;
  constexpr double kCmdLowerLimit = 0.0;

  const bool throttle_brake_cmd_valid =
      std::min(throttle, brake) == 0.0 &&
      InRange(std::max(throttle, brake), kCmdLowerLimit - kEpsilon,
              kCmdUpperLimit + kEpsilon) &&
      InRange(e_brake, kCmdLowerLimit - kEpsilon, kCmdUpperLimit + kEpsilon);
  if (!throttle_brake_cmd_valid) {
    debug->set_error_code(
        ControlValidationResultProto::CALIBRATION_RESULT_OVER_LIMIT);
    const std::string error_message = absl::StrCat(
        "throttle, ", throttle, " or brake, ", brake, ", is out of range.");
    debug->set_error_message(error_message);
    return false;
  }

  return true;
}

bool ValidateLateralControlOutput(
    const VehicleDriveParamsProto& vehicle_drive_param,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto) {
  auto* debug = controller_debug_proto->mutable_validation_result_proto();

  const double curvature_cmd = control_cmd.curvature();
  const double steering_cmd = control_cmd.steering_target();

  const double front_wheel_angle = SteeringPct2FrontWheelAngle(
      steering_cmd, vehicle_drive_param.steer_ratio(),
      vehicle_drive_param.max_steer_angle());
  const double curvature_from_steering_cmd = FrontWheelAngle2Kappa(
      front_wheel_angle, vehicle_geometry_params.wheel_base());

  const auto& steering_protection_result =
      control_cmd.debug().simple_mpc_debug().steering_protection_result();
  const double curvature_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               steering_protection_result.kappa_limit_wrt_speed());

  const bool curvature_cmd_valid = InRange(
      curvature_cmd, -curvature_limit - kEpsilon, curvature_limit + kEpsilon);
  const bool steering_cmd_valid =
      InRange(curvature_from_steering_cmd, -curvature_limit - kEpsilon,
              curvature_limit + kEpsilon);

  if (!curvature_cmd_valid) {
    debug->set_error_code(ControlValidationResultProto::CURVATURE_OVER_LIMIT);
    const std::string error_message =
        absl::StrCat("curvature_cmd, ", curvature_cmd, ", is out of range, ",
                     curvature_limit, ".");
    debug->set_error_message(error_message);
    return false;
  }

  if (!steering_cmd_valid) {
    debug->set_error_code(
        ControlValidationResultProto::STEERING_COMMAND_OVER_LIMIT);
    const std::string error_message =
        absl::StrCat("steering_cmd, ", steering_cmd,
                     ", is out of range, while the curvature_limit is ",
                     curvature_limit, ".");
    debug->set_error_message(error_message);
    return false;
  }
  return true;
}

bool ValidateMPCPrediction(double speed_ref,
                           const TrajectoryInterface& trajectory_interface,
                           ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("ValidateMPCPrediction");
  if (!controller_debug_proto->has_mpc_debug_proto()) return true;

  auto* debug = controller_debug_proto->mutable_validation_result_proto();
  const auto& mpc_debug = controller_debug_proto->mpc_debug_proto();
  constexpr int kMpcValidationIndex = 5;
  const int mpc_validation_index =
      std::min(kMpcValidationIndex, mpc_debug.mpc_predicted_traj_point_size());

  for (int i = 0; i < mpc_validation_index; ++i) {
    const Vec2d predicted_point(mpc_debug.mpc_predicted_traj_point(i));
    const ApolloTrajectoryPointProto closest_point =
        trajectory_interface.QueryNearestTrajectoryPointByPosition(
            /*on_transition_traj*/ false, predicted_point.x(),
            predicted_point.y());

    const Vec2d closest_xy{closest_point.path_point().x(),
                           closest_point.path_point().y()};
    const double distance = predicted_point.DistanceTo(closest_xy);

    if (distance > kMPCPredictedDeviationQEventThreshold) {
      QEVENT_EVERY_N_SECONDS("zhichao", "mpc_predicted_deviation",
                             /*every_n_seconds=*/5.0, [&](QEvent* qevent) {
                               qevent->AddField("distance", distance)
                                   .AddField("speed_ref", speed_ref);
                             });
      QLOG_EVERY_N_SEC(ERROR, 1.0) << "MPC result validation failed.";
    }

    if (distance > kMPCPredictedDeviationKickoutThreshold) {
      debug->set_error_code(
          ControlValidationResultProto::MPC_PREDICTION_ABNORMAL);
      const std::string error_message = absl::StrCat(
          "MPC prediction is abnormal: i: ", i, ", distance: ", distance);
      debug->set_error_message(error_message);
      return false;
    }
  }

  return true;
}

}  // namespace

bool ValidateControlOutput(
    const TrajectoryInterface& trajectory_interface,
    const VehicleDriveParamsProto& vehicle_drive_param,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ControllerConf& controller_conf, const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("ValidateControlOutput");

  if (!ValidateLongitudinalControlOutput(controller_conf, control_cmd,
                                         controller_debug_proto) ||
      !ValidateLateralControlOutput(vehicle_drive_param,
                                    vehicle_geometry_params, control_cmd,
                                    controller_debug_proto)) {
    return false;
  }

  const double speed_ref =
      control_cmd.debug().simple_mpc_debug().speed_reference();
  if (!ValidateMPCPrediction(speed_ref, trajectory_interface,
                             controller_debug_proto) &&
      FLAGS_enable_mpc_validation) {
    return false;
  }

  return true;
}

}  // namespace control
}  // namespace qcraft
