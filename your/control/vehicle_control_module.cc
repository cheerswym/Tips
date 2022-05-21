#include "onboard/control/vehicle_control_module.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/control_validation.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/control/steering_protection.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/positioning/positioning_flags.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

DEFINE_string(manual_control_cmd_path, "",
              "The manual control command protobuf text file path.");
DEFINE_bool(enable_steering_rate_kickout, true,
            "enable set canbus steering rate. ");
DEFINE_bool(apply_weak_steering_kickout_condition, true,
            "apply weak steering rate kickout condition.");

namespace qcraft::control {

namespace {

absl::Status IsAllowToEngage(double av_speed, double max_steer_angle,
                             double steering_pct_cmd, double steering_pct_fb) {
  const double steering_cmd_rad = 0.01 * max_steer_angle * steering_pct_cmd;
  const double steering_canbus_rad = 0.01 * max_steer_angle * steering_pct_fb;

  if (std::fabs(steering_cmd_rad - steering_canbus_rad) >
          FLAGS_max_steering_angle_diff_threshold &&
      std::fabs(av_speed) > FLAGS_engage_protection_min_speed) {
    return absl::OutOfRangeError(absl::StrCat(
        "The steering angle diff between control command and "
        "canbus status is out of range: \n control command is:\t",
        r2d(steering_cmd_rad), " degree, while canbus steering is at \t",
        r2d(steering_canbus_rad), " degree, at speed:\t", av_speed, " m/s"));
  }

  return absl::OkStatus();
}
}  // namespace

using apollo::common::ErrorCode;

VehicleControlModule::VehicleControlModule(LiteClientBase *lite_client)
    : LiteModule(lite_client),
      control_tracking_statistics_(/*monitor_window_size=*/absl::Seconds(0.2)) {
}

VehicleControlModule::~VehicleControlModule() {
  if (IsOnboardMode()) {
    const auto status = SaveDynamicParamProto(dynamic_param_);
    if (!status.ok()) {
      QEVENT("shijun", "Fail_save_dynamic_param", [&](QEvent *qevent) {
        qevent->AddField("error message", status.ToString());
      });
    }
  }
}

void VehicleControlModule::OnInit() {
  QLOG(INFO) << "Control init, starting ...";

  RunParamsProtoV2 run_params = GetRunParams();

  if (IsOnboardMode()) {
    QCHECK(run_params.vehicle_params().has_vehicle_geometry_params() &&
           run_params.vehicle_params().has_vehicle_drive_params() &&
           run_params.vehicle_params().has_controller_conf());
  } else {
    // Legacy.
    if (!run_params.vehicle_params().has_vehicle_geometry_params() ||
        !run_params.vehicle_params().has_vehicle_drive_params() ||
        !run_params.vehicle_params().has_controller_conf()) {
      // Note(mike): this logic is needed to provide backward compatibility to
      // old runs which don't have the new params in the log. Here, we use a new
      // param manager to avoid interfering with other places using the global
      // param manager.
      CreateParamManagerFromCarId("Q0001")->GetRunParams(&run_params);
    }
  }

  QLOG(INFO) << "Load vehicle-based controller configuration from: "
             << run_params.vehicle_params().car_id();

  vehicle_geometry_params_ =
      run_params.vehicle_params().vehicle_geometry_params();
  vehicle_drive_params_ = run_params.vehicle_params().vehicle_drive_params();
  control_conf_ = run_params.vehicle_params().controller_conf();
  if (control_conf_.active_controllers_size() == 0) {
    control_conf_.add_active_controllers(ControllerConf::TS_PKMPC_CONTROLLER);
  }
  car_id_ = run_params.vehicle_params().car_id();

  if (IsOnboardMode()) {
    const auto status = LoadDynamicParamProto(&dynamic_param_);
    // Default steering angle bias in dynamic param is 0.0 which is invalid.
    // When dynamic parameter identification has not finished, set steer angle
    // bias from configure files to forbid the car driving not straight.
    steer_angle_bias_ = dynamic_param_.steering_angle_bias() == 0.0
                            ? vehicle_drive_params_.steering_angle_bias()
                            : dynamic_param_.steering_angle_bias();
    if (!status.ok()) {
      QLOG(ERROR) << "Fail load dynamic param: " << status.ToString();
      QEVENT("shijun", "Fail_load_dynamic_param", [&](QEvent *qevent) {
        qevent->AddField("error message", status.ToString());
      });
      steer_angle_bias_ = vehicle_drive_params_.steering_angle_bias();
    }
  }

  parameter_identificator_.emplace(vehicle_drive_params_,
                                   vehicle_geometry_params_, control_conf_);

  // set controller
  CHECK(
      controller_agent_
          .Init(vehicle_geometry_params_, vehicle_drive_params_, &control_conf_)
          .ok());

  // Test control and two safety redundancy
  if (FLAGS_enable_openloop_control && !FLAGS_openloop_control_path.empty()) {
    openloop_control_ = std::make_unique<control::OpenloopControl>();
    CHECK_OK(openloop_control_->Init(FLAGS_openloop_control_path));
    enable_openloop_control_ = true;
  }

  wire_control_checker_ = std::make_unique<WireControlChecker>(
      control_conf_.control_period(), control_conf_.steer_delay_time());
  steer_calibration_ = std::make_unique<SteerCalibration>(
      control_conf_, vehicle_geometry_params_.wheel_base());
}

void VehicleControlModule::OnSubscribeChannels() {
  Subscribe(&VehicleControlModule::OnAutonomyState, this);
  Subscribe(&VehicleControlModule::OnChassis, this);
  Subscribe(&VehicleControlModule::OnTrajectory, this);
  Subscribe(&VehicleControlModule::OnPoseProto, this);
  Subscribe(&VehicleControlModule::OnRemoteAssistToCarProto, this);
}

void VehicleControlModule::OnSetUpTimers() {
  if (OnTestBench()) {
    return;
  }
  QLOG(ERROR) << "Control resetting vehicle state, sleeping for 1000 ms ...";

  AddTimerOrDie("vehicle_control_main_loop", &VehicleControlModule::Proc, this,
                absl::Milliseconds(1000), absl::Milliseconds(10),
                /*one_shot=*/false);

  // should init_vehicle first, let car enter work status, then use status
  // msg trigger control
  QLOG(INFO) << "Control default driving action is "
             << PadMessage::DrivingAction_Name(control_conf_.action());
}

void VehicleControlModule::OnAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy_state) {
  local_view_.autonomy_state = std::move(autonomy_state);
}

void VehicleControlModule::OnChassis(std::shared_ptr<const Chassis> chassis) {
  local_view_.chassis = std::move(chassis);
}

void VehicleControlModule::OnTrajectory(
    std::shared_ptr<const TrajectoryProto> trajectory) {
  local_view_.trajectory = std::move(trajectory);
}

void VehicleControlModule::OnPoseProto(std::shared_ptr<const PoseProto> pose) {
  local_view_.pose = std::move(pose);
}

absl::Status VehicleControlModule::ProduceControlCommand(
    ControlCommand *control_command,
    ControllerDebugProto *controller_debug_proto) {
  RETURN_IF_ERROR(UpdateInput(local_view_, controller_debug_proto));
  RETURN_IF_ERROR(CheckTimestamp(local_view_, is_input_ready_));
  is_input_ready_ = true;

  if (local_view_.trajectory->trajectory_point().empty()) {
    if (planner_trajectory_ready_) {
      QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_EMPTY,
              "Check control msg: EMPTY(trajectory proto)");
    }
    return absl::InvalidArgumentError(
        "Check control msg: trajectory is not ready.");
  } else {
    planner_trajectory_ready_ = true;
  }

  const auto &autonomy_state = local_view_.autonomy_state->autonomy_state();
  const bool controller_reset =
      !IS_AUTO_DRIVE(autonomy_state) &&
      !(autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP);
  if (controller_reset) {
    controller_agent_.Reset(*local_view_.chassis, vehicle_state_);
    QLOG_EVERY_N_SEC(INFO, 3.0) << "Reset Controllers in Manual Mode";
  }
  control_command->mutable_debug()
      ->mutable_bias_estimation_debug()
      ->set_steer_delay_online(steer_delay_time_);
  const double previous_kappa_cmd =
      control_history_state_mgr_.GetControlStateCache().back().kappa_cmd;

  const SteeringProtection steering_protection(
      &vehicle_geometry_params_, &vehicle_drive_params_, &control_conf_);
  SteeringProtectionResult steering_protection_result =
      steering_protection.CalcKappaAndKappaRateLimit(
          *local_view_.autonomy_state, vehicle_state_.linear_velocity(),
          vehicle_state_.front_wheel_steering_angle(), previous_kappa_cmd,
          *control_command);

  ControlConstraints control_constraints = {steering_protection_result};

  absl::Status status_compute = controller_agent_.ComputeControlCommand(
      vehicle_state_, trajectory_interface_, control_constraints,
      control_history_state_mgr_, control_command, controller_debug_proto);

  if (steering_protection.IsProtectiveKickout(
          vehicle_state_.linear_velocity(),
          control_history_state_mgr_.GetControlStateCache(),
          *controller_debug_proto, &steering_protection_result)) {
    std::string steering_debug_msg;
    steering_protection.FillDebugMessage(
        vehicle_state_, steering_protection_result, &steering_debug_msg);
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_CONTROL_STEER_ANGLE_SPEED_TOO_FAST,
                      "Steering too fast", steering_debug_msg);
  }
  control_command->mutable_debug()
      ->mutable_simple_mpc_debug()
      ->mutable_steering_protection_result()
      ->CopyFrom(steering_protection_result);

  if (control_conf_.bias_estimation_conf().enable_online_bias_estimation()) {
    ParameterIdentificationInput input = {
        .steer_cmd =
            Kappa2FrontWheelAngle(control_command->curvature(),
                                  vehicle_geometry_params_.wheel_base()),
        .steer_pose =
            Kappa2FrontWheelAngle(local_view_.pose->curvature(),
                                  vehicle_geometry_params_.wheel_base()),
        .steer_feedback = SteeringPct2FrontWheelAngle(
            local_view_.chassis->steering_percentage(),
            vehicle_drive_params_.steer_ratio(),
            vehicle_drive_params_.max_steer_angle()),
        .speed_measurement = vehicle_state_.linear_velocity(),
        .lat_error =
            control_command->debug().simple_mpc_debug().lateral_error(),
        .heading_err =
            control_command->debug().simple_mpc_debug().heading_error(),
        .is_auto = IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state()),
        .control_history_state_mgr = &control_history_state_mgr_};
    *control_command->mutable_debug()->mutable_bias_estimation_debug() =
        parameter_identificator_->EstimateSteerBias(input);
  }

  parameter_identificator_->UpdateSteerDelay(*local_view_.chassis,
                                             *local_view_.autonomy_state);
  steer_delay_time_ = parameter_identificator_->GetSteerDelay();
  control_command->mutable_debug()
      ->mutable_bias_estimation_debug()
      ->set_steer_delay_online(steer_delay_time_);

  double steer_target_calibration =
      std::clamp(Kappa2SteerPercentage(control_command->curvature(),
                                       vehicle_geometry_params_.wheel_base(),
                                       vehicle_drive_params_.steer_ratio(),
                                       vehicle_drive_params_.max_steer_angle()),
                 -100.0, 100.0);
  if (IsOnboardMode()) {
    steer_target_calibration = steer_calibration_->SteerCalibrationMain(
        control_command->curvature(),
        control_history_state_mgr_
            .GetControlStateCache()[kCacheSize -
                                    static_cast<int>(0.1 / kControlInterval)]
            .kappa_cmd,
        vehicle_state_.linear_velocity(), vehicle_geometry_params_.wheel_base(),
        vehicle_drive_params_.steer_ratio(),
        vehicle_drive_params_.max_steer_angle(),
        controller_debug_proto->mutable_steer_calibration_debug_proto());
  }

  control_command->set_steering_target(steer_target_calibration);
  control_command->mutable_debug()
      ->mutable_simple_mpc_debug()
      ->set_steer_percentage_feedforward(Kappa2SteerPercentage(
          control_command->debug().simple_mpc_debug().kappa_feedforward(),
          vehicle_geometry_params_.wheel_base(),
          vehicle_drive_params_.steer_ratio(),
          vehicle_drive_params_.max_steer_angle()));

  if (IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state()) &&
      !status_compute.ok()) {
    const auto args_message = std::string(status_compute.message());
    // TODO(zhichao): separate controller error types.
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_CONTROL_COMPUTE_COMMAND_FAILED,
                      "Check control main function: FAILED", args_message);
    return absl::InternalError(args_message);
  }

  const auto &controller_info = control_command->debug().simple_mpc_debug();
  ReportControlError(*local_view_.autonomy_state, controller_info,
                     control_command->debug().control_error());
  const auto now = absl::Now();
  control_tracking_statistics_.Process(now, controller_info);
  if (control_tracking_statistics_.size() > 0) {
    *controller_debug_proto->mutable_lateral_error_stat() =
        control_tracking_statistics_.ComputeLateralErrorStats();
    *controller_debug_proto->mutable_station_error_stat() =
        control_tracking_statistics_.ComputeStationErrorStats();
  }

  // QEvent: hard brake qevent.
  constexpr double kHardBrakeThreshold = -2.0;  // m/s^2.
  if (IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state()) &&
      control_command->acceleration() < kHardBrakeThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "control_hard_brake", /*every_n_seconds=*/3.0,
        [&](QEvent *qevent) {
          qevent->AddField("control_accel", control_command->acceleration())
              .AddField("planner_accel",
                        controller_info.acceleration_reference());
        });
  }

  // QEvent: large tracking error qevent.
  QEventTrackingError(controller_info);

  LightControl(*local_view_.trajectory, control_command);
  DoorControl(*local_view_.trajectory, control_command);
  const auto status = GearControl(local_view_, control_command);
  DrivingScenariosControl(*local_view_.trajectory, control_command);
  RETURN_IF_ERROR(status);

  control_history_state_mgr_.UpdateHistoryData(
      IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state()),
      control_command->curvature(), local_view_.chassis->steering_percentage(),
      control_command->debug().simple_mpc_debug().acceleration_cmd_closeloop());
  return absl::OkStatus();
}

// TODO(lidong): Change this function to return absl::Status.
void VehicleControlModule::Proc() {
  SCOPED_QTRACE("VehicleControlModule::Proc");

  if (use_manual_cmd_) {
    if (FLAGS_manual_control_cmd_path.empty()) {
      QLOG(ERROR) << FLAGS_manual_control_cmd_path << " is not provide.";
      return;
    }
    ControlCommand control_command;
    if (!file_util::TextFileToProto(FLAGS_manual_control_cmd_path,
                                    &control_command)) {
      QLOG(ERROR) << "Failed to parse file " << FLAGS_manual_control_cmd_path;
      return;
    }
    QLOG_IF_NOT_OK(WARNING, Publish(control_command));
    return;
  }
  if (local_view_.pose == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Pose does not exist.";
    return;
  }
  if (local_view_.chassis == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Chassis does not exist.";
    return;
  }

  ControlCommand control_command;
  ControllerDebugProto controller_debug_proto;
  vis::vantage::ChartsDataProto chart_data;

  // Openloop control
  if (enable_openloop_control_) {
    CHECK_OK(openloop_control_->Process(
        local_view_.pose->vel_body().x(), vehicle_drive_params_.steer_ratio(),
        local_view_.chassis->steering_percentage(),
        vehicle_geometry_params_.wheel_base(),
        vehicle_drive_params_.max_steer_angle(), vehicle_state_.kappa(),
        &control_command, &controller_debug_proto));
    QLOG_EVERY_N_SEC(INFO, 0.1)
        << "[OpenLoop Control]: "
        << "speed: " << local_view_.pose->vel_body().x() << "  "
        << "gear: " << local_view_.chassis->gear_location() << "  "
        << "gear_cmd: " << control_command.gear_location() << "  "
        << "accel_cmd: " << control_command.acceleration() << "  "
        << "throttle_cmd: " << control_command.throttle() << "  "
        << "brake_cmd: " << control_command.brake() << "  "
        << "steer_cmd: " << control_command.steering_target();

    QLOG_IF_NOT_OK(WARNING, Publish(control_command));
    QLOG_IF_NOT_OK(WARNING, Publish(controller_debug_proto));
    return;
  }

  if (local_view_.autonomy_state == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "AutonomyState does not exist.";
    return;
  }
  parameter_identificator_->Process(*local_view_.pose, *local_view_.chassis,
                                    *local_view_.autonomy_state,
                                    &controller_debug_proto);
  // When steering bias estimation is not ready, the control module publishes
  // steering angle bias from vehicle param, when it is ready, gradually switch
  // to the estimation value.
  constexpr int kInterationThreshold = 8000;
  if (controller_debug_proto.has_parameter_identification_stat() &&
      controller_debug_proto.parameter_identification_stat().stat_index() >
          kInterationThreshold) {
    steer_angle_bias_ = controller_debug_proto.parameter_identification_stat()
                            .steer_angle_bias();
  }
  dynamic_param_.set_steering_angle_bias(steer_angle_bias_);
  control_command.set_steer_angle_bias(steer_angle_bias_);
  control_command.set_steering_rate(
      vehicle_drive_params_.max_steer_angle_rate());
  QCOUNTER("steer_angle_bias_rad*100", RoundToInt(steer_angle_bias_ * 100.0));

  const auto status =
      ProduceControlCommand(&control_command, &controller_debug_proto);
  // TODO(shijun): Tease return logic on a macro level.
  if (!status.ok()) {
    QLOG_EVERY_N(ERROR, 200)
        << "Failed to produce control command:" << status.message();
    return;
  }

  // Wire control checker
  const auto wire_control_fail = wire_control_checker_->CheckProc(
      local_view_.autonomy_state->autonomy_state(),
      control_command.acceleration_calibration(),
      local_view_.pose->accel_body().x(), control_command.curvature(),
      local_view_.pose->curvature(), local_view_.pose->vel_body().x(),
      local_view_.chassis->gear_location());
  QCOUNTER("wire_control_fail", wire_control_fail);
  *controller_debug_proto.mutable_wire_control_check_proto() =
      wire_control_checker_->GetCheckDebug();
  if (wire_control_fail) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Check wire control is failed.";
  }

  // Engage protection, check steering cmd and steering fb status when system is
  // in READY_TO_AUTO_DRIVE state to forbid collision accident.
  if (local_view_.autonomy_state->autonomy_state() ==
      AutonomyStateProto::READY_TO_AUTO_DRIVE) {
    const auto allow_engage_status =
        IsAllowToEngage(local_view_.pose->vel_body().x(),
                        vehicle_drive_params_.max_steer_angle(),
                        control_command.steering_target(),
                        local_view_.chassis->steering_percentage());
    if (!allow_engage_status.ok()) {
      const auto args_message = std::string(allow_engage_status.message());
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR,
                        QIssueType::QIT_PREVENT_ENGAGE,
                        QIssueSubType::QIST_CONTROL_STEER_FEEDBACK_BIG_DIFF,
                        "Steering cmd and feedback from canbus have a big diff",
                        args_message);
      return;
    }
  }

  if (local_view_.autonomy_state->autonomy_state() ==
          AutonomyStateProto::READY_TO_AUTO_DRIVE ||
      local_view_.autonomy_state->autonomy_state() ==
          AutonomyStateProto::AUTO_DRIVE ||
      local_view_.autonomy_state->autonomy_state() ==
          AutonomyStateProto::EMERGENCY_TO_STOP) {
    const bool output_valid = ValidateControlOutput(
        trajectory_interface_, vehicle_drive_params_, vehicle_geometry_params_,
        control_conf_, control_command, &controller_debug_proto);
    if (!output_valid) {
      const auto args_message = std::string(
          controller_debug_proto.validation_result_proto().DebugString());
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_CONTROL_VALIDATE_OUTPUT_FAILED,
                        "Control output validation fails: ", args_message);
    }
  }

  const double prev_kappa_cmd =
      control_history_state_mgr_.GetControlStateCache().back().kappa_cmd;

  WrapSteerConstraintChartData(prev_kappa_cmd, control_conf_, control_command,
                               controller_debug_proto, &chart_data);

  QLOG_IF_NOT_OK(WARNING, Publish(control_command));
  QLOG_IF_NOT_OK(WARNING, Publish(controller_debug_proto));
  QLOG_IF_NOT_OK(WARNING, Publish(chart_data));
}

absl::Status VehicleControlModule::UpdateInput(
    const LocalView &local_view, ControllerDebugProto *controller_debug_proto) {
  if (local_view.pose == nullptr) {
    return absl::FailedPreconditionError("Pose does not exist.");
  }
  if (local_view.chassis == nullptr) {
    return absl::FailedPreconditionError("Chassis does not exist.");
  }
  if (local_view.autonomy_state == nullptr) {
    return absl::FailedPreconditionError("AutonomyState does not exist.");
  }
  if (local_view.trajectory == nullptr) {
    return absl::FailedPreconditionError("Trajectory does not exist.");
  }

  const auto vehicle_state =
      ConstructVehicleState(*local_view.autonomy_state, *local_view.pose,
                            *local_view.chassis, vehicle_drive_params_);
  if (!vehicle_state.ok()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Construct vehicle state fails: ", vehicle_state.status().ToString()));
  } else {
    vehicle_state_ = *vehicle_state;
  }

  const auto av_speed_kph =
      RoundToInt(Mps2Kph(vehicle_state_.linear_velocity()));
  QCOUNTER("av_speed_kph", av_speed_kph);

  RETURN_IF_ERROR(trajectory_interface_.Update(*local_view.autonomy_state,
                                               *local_view.trajectory,
                                               controller_debug_proto))
      << "Trajectory has empty message.";

  return absl::OkStatus();
}

absl::Status VehicleControlModule::CheckTimestamp(const LocalView &local_view,
                                                  bool is_input_ready) {
  const double current_timestamp = ToUnixDoubleSeconds(Clock::Now());

  // Positioning signal time delay (QEvent or kickout).
  const double pose_time_diff =
      current_timestamp - local_view.pose->timestamp();
  const double max_time_diff =
      control_conf_.max_pose_miss_num() * control_conf_.positioning_period();
  constexpr double kQEventPoseDelayThreshold = 0.15;  // s.
  if (pose_time_diff > kQEventPoseDelayThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "pose_delay_too_much",
                           /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
                             qevent->AddField("pose_time_diff", pose_time_diff)
                                 .AddField("car_id", car_id_);
                           });
  }
  if (pose_time_diff > max_time_diff) {
    const auto args_message = absl::StrFormat(
        "Pose msg timeout, pose_time: %f, now: %f, diff: %f, threshold: %f",
        local_view.pose->timestamp(), current_timestamp, pose_time_diff,
        max_time_diff);
    if (is_input_ready) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_POSITION_PROTO_POSE_TIMEOUT,
                        "Check timestamp: FAILED", args_message);
    }
    return absl::DataLossError(args_message);
  }

  // Canbus signal time delay (QEvent or kickout).
  const double chassis_time_diff =
      current_timestamp - local_view.chassis->header().timestamp() * 1e-6;
  const double max_chassis_time_diff =
      control_conf_.max_chassis_miss_num() * control_conf_.chassis_period();
  constexpr double kQEventChassisDelayThreshold = 0.15;  // s.
  if (chassis_time_diff > kQEventChassisDelayThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "canbus_delay_too_much",
        /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
          qevent->AddField("chassis_time_diff", chassis_time_diff)
              .AddField("car_id", car_id_);
        });
  }
  if (chassis_time_diff > max_chassis_time_diff) {
    const auto args_message = absl::StrFormat(
        "Chassis msg timeout, time_diff: %f, max_chassis_time_diff: %f",
        chassis_time_diff, max_chassis_time_diff);
    if (is_input_ready) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_CHASSIS_PROTO_CHASSIS_TIMEOUT,
                        "Check timestamp: FAILED", args_message);
    }
    return absl::DataLossError(args_message);
  }

  // Planner signal time delay (QEvent).
  const double planner_time_delay =
      current_timestamp - local_view.trajectory->header().timestamp() * 1e-6;
  constexpr double kQEventPlannerDelayThreshold = 0.5;  // s.
  if (planner_time_delay > kQEventPlannerDelayThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "planner_delay_too_much",
        /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
          qevent->AddField("planner_time_diff", planner_time_delay)
              .AddField("car_id", car_id_);
        });
  }
  // Kickout if planner delays too much.
  constexpr double kQIssuePlannerDelayThreshold = 1.0;  // s.
  if (planner_time_delay > kQIssuePlannerDelayThreshold) {
    const auto args_message = absl::StrFormat(
        "Trajectory msg timeout, time_diff: %f, max_planner_time_diff: %f",
        planner_time_delay, kQIssuePlannerDelayThreshold);
    if (is_input_ready) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_TIMEOUT,
                        "Control check planner's timestamp failed.",
                        args_message);
    }
    return absl::DataLossError(args_message);
  }

  return absl::OkStatus();
}

void VehicleControlModule::QEventTrackingError(
    const SimpleMPCDebug &controller_info) {
  if (!IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state())) {
    tracking_error_queue_.clear();
    return;
  }

  const double lat_error = controller_info.lateral_error();
  const double lon_error = controller_info.station_error();

  constexpr double kLatErrorThreshold = 0.3;  // m.
  constexpr double kLonErrorThreshold = 1.0;  // m.
  // Large lateral error.
  if (std::fabs(lat_error) > kLatErrorThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "large_lat_error",
        /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
          qevent->AddField("lat_error", lat_error)
              .AddField("car_id", car_id_)
              .AddField("steer_ref_pct",
                        controller_info.steer_percentage_feedforward());
        });
  }

  // Large longitudinal error.
  if (std::fabs(lon_error) > kLonErrorThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "large_lon_error",
                           /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
                             qevent->AddField("lon_error", lon_error)
                                 .AddField("car_id", car_id_)
                                 .AddField("speed_ref",
                                           controller_info.speed_reference());
                           });
  }

  if (!tracking_error_queue_.empty()) {
    constexpr double kLatErrorChangeThreshold = 0.05;  // m.
    constexpr double kLonErrorChangeThreshold = 0.1;   // m.

    const double lat_error_change =
        lat_error - tracking_error_queue_.back().lat_error;
    const double lon_error_change =
        lon_error - tracking_error_queue_.back().lon_error;

    // Lateral error sudden change over threshold.
    if (std::fabs(lat_error_change) > kLatErrorChangeThreshold) {
      QEVENT("zhichao", "lateral_error_jump", [&](QEvent *qevent) {
        qevent->AddField("lat_error_change", lat_error_change)
            .AddField("car_id", car_id_);
      });
    }
    // Longitudinal error sudden change over threshold.
    if (std::fabs(lon_error_change) > kLonErrorChangeThreshold) {
      QEVENT("zhichao", "longitudinal_error_jump", [&](QEvent *qevent) {
        qevent->AddField("lon_error_change", lon_error_change)
            .AddField("car_id", car_id_);
      });
    }
  }

  constexpr int kTrackingErrorCacheSize = 500;
  constexpr double kLatErrorCacheThreshold = 0.15;  // m.
  constexpr double kLonErrorCacheThreshold = 0.5;   // m.
  if (tracking_error_queue_.size() < kTrackingErrorCacheSize) {
    tracking_error_queue_.push_back({lat_error, lon_error});
    return;
  }
  tracking_error_queue_.pop_front();
  tracking_error_queue_.push_back({lat_error, lon_error});

  // Continuous large lateral tracking error.
  const auto min_fabs_lat_error_cache = std::min_element(
      tracking_error_queue_.begin(), tracking_error_queue_.end(),
      [](TrackingError e1, TrackingError e2) {
        return (std::fabs(e1.lat_error) < std::fabs(e2.lat_error));
      });

  if (std::fabs(min_fabs_lat_error_cache->lat_error) >
      kLatErrorCacheThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "continuous_large_lat_error",
        /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
          qevent->AddField("min_lat_error", min_fabs_lat_error_cache->lat_error)
              .AddField("car_id", car_id_);
        });
  }

  // Continuous large longitudinal tracking error.
  const auto min_fabs_lon_error_cache = std::min_element(
      tracking_error_queue_.begin(), tracking_error_queue_.end(),
      [](TrackingError e1, TrackingError e2) {
        return (std::fabs(e1.lon_error) < std::fabs(e2.lon_error));
      });

  if (std::fabs(min_fabs_lon_error_cache->lon_error) >
      kLonErrorCacheThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "continuous_large_lon_error",
        /*every_n_seconds=*/5.0, [&](QEvent *qevent) {
          qevent->AddField("min_lon_error", min_fabs_lon_error_cache->lon_error)
              .AddField("car_id", car_id_);
        });
  }
}

void VehicleControlModule::LightControl(const TrajectoryProto &trajectory,
                                        ControlCommand *control_command) const {
  switch (trajectory.turn_signal()) {
    case TURN_SIGNAL_LEFT:
      control_command->mutable_signal()->set_emergency_light(false);
      control_command->mutable_signal()->set_turn_signal(
          apollo::common::VehicleSignal::TURN_LEFT);
      break;
    case TURN_SIGNAL_RIGHT:
      control_command->mutable_signal()->set_emergency_light(false);
      control_command->mutable_signal()->set_turn_signal(
          apollo::common::VehicleSignal::TURN_RIGHT);
      break;
    case TURN_SIGNAL_EMERGENCY:
      control_command->mutable_signal()->set_emergency_light(true);
      control_command->mutable_signal()->set_turn_signal(
          apollo::common::VehicleSignal::TURN_NONE);
      break;
    case TURN_SIGNAL_NONE:
      control_command->mutable_signal()->set_emergency_light(false);
      control_command->mutable_signal()->set_turn_signal(
          apollo::common::VehicleSignal::TURN_NONE);
      break;
  }
}
void VehicleControlModule::DoorControl(const TrajectoryProto &trajectory,
                                       ControlCommand *control_command) const {
  control_command->set_door_open(trajectory.has_door_decision() &&
                                 trajectory.door_decision().door_state() ==
                                     DoorDecision::DOOR_OPEN);
}

void VehicleControlModule::DrivingScenariosControl(
    const TrajectoryProto &trajectory, ControlCommand *control_command) const {
  control_command->mutable_custom_command()->set_low_speed_freespace(
      trajectory.low_speed_freespace());
}

absl::Status VehicleControlModule::GearControl(
    const LocalView &local_view, ControlCommand *control_command) const {
  constexpr double kStopStandStillSpeedThreshold = 0.2;  // m/s;
  // No need to shift gear.
  if (local_view.chassis->gear_location() == local_view.trajectory->gear()) {
    control_command->set_gear_location(local_view.trajectory->gear());
    return absl::OkStatus();
  }
  // Need to shift gear below.
  const auto is_stop_standstill = std::fabs(local_view.pose->vel_body().x()) <
                                  kStopStandStillSpeedThreshold;
  if (!is_stop_standstill) {
    // Allow to set gear from N to D or R when moving.
    // TODO(shijun): It is to be confirmed.
    if (local_view.chassis->gear_location() == Chassis::GEAR_NEUTRAL) {
      control_command->set_gear_location(local_view.trajectory->gear());
      return absl::OkStatus();
    }
    // Unreasonable gear shifting from R to D, or D to R.
    control_command->set_gear_location(local_view.chassis->gear_location());
    return absl::InternalError(absl::StrCat(
        "Gear shifting is not reasonable \n, speed: ",
        local_view_.pose->vel_body().x(),
        " | stop standstill speed threshold: ", kStopStandStillSpeedThreshold,
        " | chassis_gear: ",
        Chassis::GearPosition_Name(local_view.chassis->gear_location()),
        " | trajectory_gear: ",
        Chassis::GearPosition_Name(local_view.trajectory->gear())));
  }

  // It is ok to set P directly from other gear location while D -> R or R -> D
  // need to go through N first.
  if (local_view.chassis->gear_location() == Chassis::GEAR_NEUTRAL ||
      local_view.trajectory->gear() == Chassis::GEAR_PARKING) {
    control_command->set_gear_location(local_view.trajectory->gear());
  } else {
    control_command->set_gear_location(Chassis::GEAR_NEUTRAL);
  }
  return absl::OkStatus();
}

void VehicleControlModule::ReportControlError(
    const AutonomyStateProto &autonomy_state,
    const SimpleMPCDebug &simple_mpc_debug, const ControlError &control_error) {
  // When planner reset, lateral error and station error becomes 0.0, ignore
  // these data.
  if (IS_AUTO_DRIVE(autonomy_state.autonomy_state())) {
    // ~1s, at 10ms per cycle.
    constexpr int kAutoCruiseCountThreshold = 100;
    auto_cruise_count_ =
        std::min(++auto_cruise_count_, kAutoCruiseCountThreshold + 1);
    // Ignore just engaged data in which error data may be invalid.
    if (auto_cruise_count_ > kAutoCruiseCountThreshold) {
      const auto control_lateral_error_cm =
          RoundToInt(control_error.lateral_error() * 100.0);
      if (control_lateral_error_cm != 0) {
        QCOUNTER("control_lateral_error_cm", control_lateral_error_cm);
      }

      const auto control_station_error_cm =
          RoundToInt(control_error.station_error() * 100.0);
      if (control_station_error_cm != 0) {
        QCOUNTER("control_station_error_cm", control_station_error_cm);
      }

      const auto control_speed_error_cm =
          RoundToInt(control_error.speed_error() * 100.0);
      if (control_speed_error_cm != 0) {
        QCOUNTER("control_speed_error_cm/s", control_speed_error_cm);
      }

      if (simple_mpc_debug.is_full_stop()) {
        const auto full_stop_lateral_error_cm =
            RoundToInt(control_error.lateral_error() * 100.0);
        if (full_stop_lateral_error_cm != 0) {
          QCOUNTER("full_stop_lateral_error_cm", full_stop_lateral_error_cm);
        }

        const auto full_stop_station_error_cm =
            RoundToInt(control_error.station_error() * 100.0);
        if (full_stop_station_error_cm != 0) {
          QCOUNTER("full_stop_station_error_cm", full_stop_station_error_cm);
        }
      }
    }
  } else {
    auto_cruise_count_ = 0;
  }
}

}  // namespace qcraft::control
