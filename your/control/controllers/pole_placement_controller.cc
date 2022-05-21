#include "onboard/control/controllers/pole_placement_controller.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/control/math/mpc_solver.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/utils/time_util.h"

namespace qcraft::control {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

namespace {
constexpr double kPiecewiseVel[] = {0.0, 0.1, 0.5, 1.8,  4.5,  5.5,
                                    6.6, 7.5, 8.5, 10.5, 11.5, 100.0};
constexpr double kPiecewisePoles[] = {0.01, 0.01, 0.2, 0.3, 0.4, 0.5,
                                      0.6,  0.7,  0.8, 1.0, 1.2, 1.2};
bool IsSteeringCmdBackToCenter(
    const std::vector<Eigen::VectorXd> &kappa_cmd_prediction) {
  // Consider next kKappaCmdSize kappa command prediction.
  constexpr int kKappaCmdSize = 5;
  QCHECK_GT(kappa_cmd_prediction.size(), kKappaCmdSize);

  std::vector<double> kappa_cmd;
  kappa_cmd.reserve(kKappaCmdSize);
  for (int i = 0; i < kKappaCmdSize; ++i) {
    kappa_cmd.push_back(kappa_cmd_prediction[i](0));
  }

  return control::IsCmdBackToZero(kappa_cmd);
}

// Return curvature_cmd(unit: rad).
double CalcStationarySteeringCmd(double previous_control_kappa, double av_kappa,
                                 double ref_kappa, double wheel_base,
                                 double steer_ratio) {
  const double kappa_error = ref_kappa - av_kappa;
  if (std::fabs(kappa_error) < FLAGS_stationary_steering_precision) {
    QLOG_EVERY_N_SEC(INFO, 3)
        << "Stationary steering finished"
        << "with kappa error: " << kappa_error << " | av_kappa: " << av_kappa
        << "| ref_kappa: " << ref_kappa;
    return ref_kappa;
  }

  constexpr double kSteeringSpeed = d2r(300.0);  // rad/s.
  const double kappa_rate = SteeringSpeed2KappaRateWithKappa(
      kSteeringSpeed, av_kappa, wheel_base, steer_ratio);
  const double control_curvature =
      previous_control_kappa +
      std::copysign(kappa_rate * kControlInterval, kappa_error);
  VLOG(1) << "**********************" << '\n'
          << "previous_control_kappa: " << previous_control_kappa << '\n'
          << "av_kappa: " << av_kappa << '\n'
          << "ref_kappa: " << ref_kappa << '\n'
          << "control curvature: " << control_curvature << '\n'
          << "kappa error: " << kappa_error << '\n'
          << "kappa_rate: " << kappa_rate << '\n'
          << "**********************";
  return control_curvature;
}

PiecewiseLinearFunctionDoubleProto GetDefaultPolesConf() {
  PiecewiseLinearFunctionDoubleProto proto;

  for (double x : kPiecewiseVel) {
    proto.add_x(x);
  }
  for (double y : kPiecewisePoles) {
    proto.add_y(y);
  }
  CHECK_EQ(proto.x_size(), proto.y_size());
  return proto;
}

}  // namespace

absl::Status PolePlacementController::LoadControlConf(
    const ControllerConf *control_conf,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  control_conf_ = control_conf;

  ts_ = control_conf_->ts_pkmpc_controller_conf().ts();
  QCHECK_GT(ts_, 0.0) << "[PolePlacementController] Invalid control time step "
                      << ts_;
  step_ratio_ = ts_ / kControlInterval;
  wheel_base_ = vehicle_geometry_params.wheel_base();
  steer_ratio_ = vehicle_drive_params.steer_ratio();
  steering_wheel_max_angle_ = vehicle_drive_params.max_steer_angle();
  max_curvature_ =
      SteerAngle2Kappa(steering_wheel_max_angle_, wheel_base_, steer_ratio_);
  min_curvature_ = -max_curvature_;
  max_acceleration_ = control_conf_->max_acceleration_cmd();
  max_deceleration_ = control_conf_->max_deceleration_cmd();

  throttle_lowerbound_ = vehicle_drive_params.throttle_deadzone();
  brake_lowerbound_ = vehicle_drive_params.brake_deadzone();
  full_stop_condition_ = control_conf_->full_stop_condition();
  ebrake_enable_ = control_conf_->ebrake_enable();

  calibration_manager_ = std::make_unique<CalibrationManager>();
  CHECK_OK(calibration_manager_->Init(vehicle_drive_params, ebrake_enable_));
  if (control_conf_->has_v_pole_plf()) {
    v_pole_plf_ = PiecewiseLinearFunctionFromProto(control_conf_->v_pole_plf());
  } else {
    v_pole_plf_ = PiecewiseLinearFunctionFromProto(GetDefaultPolesConf());
  }

  wheel_drive_mode_ = vehicle_drive_params.wheel_drive_mode();
  max_steer_angle_rate_ = vehicle_drive_params.max_steer_angle_rate();
  VLOG(1) << "PolePlacementController conf loaded";
  return absl::OkStatus();
}

void PolePlacementController::LogInitParameters() {
  VLOG(1) << "TS_PKMPC begin.";
}

void PolePlacementController::InitializeFilters() {
  // Mean filter for sin(slope).
  const int kSinSlopeFilterWindowSize = 120;
  sin_slope_mean_filter_ =
      apollo::common::MeanFilter(kSinSlopeFilterWindowSize);
}

absl::Status PolePlacementController::Init(
    const ControllerConf *control_conf,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  if (!LoadControlConf(control_conf, vehicle_geometry_params,
                       vehicle_drive_params)
           .ok()) {
    const auto error_msg = absl::StrCat("Failed to load control conf");
    QLOG(ERROR) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }
  // t-control matrix initialization
  t_initial_state_ = VecXd::Zero(kTControlStateNum);

  t_matrix_a_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_ad_ = t_matrix_a_;
  t_matrix_ad_t_.assign(kTControlHorizon, t_matrix_ad_);
  t_matrix_b_ = Matrix::Zero(kTControlStateNum, 1);
  t_matrix_bd_ = t_matrix_b_;
  t_matrix_c_ = Matrix::Zero(kTControlStateNum, 1);
  t_matrix_cd_ = t_matrix_c_;

  t_matrix_q_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_q_updated_ = t_matrix_q_;
  t_matrix_r_ = Matrix::Zero(1, 1);
  t_matrix_r_updated_ = t_matrix_r_;
  t_matrix_n_ = Matrix::Zero(kTControlStateNum, kTControlStateNum);
  t_matrix_n_updated_ = t_matrix_n_;

  t_control_state_reference_.assign(kTControlHorizon,
                                    VecXd::Zero(kTControlStateNum));
  t_control_input_reference_.assign(kTControlHorizon,
                                    VecXd::Zero(kTControlInputNum));
  t_control_input_lower_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlInputNum));
  t_control_input_upper_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlInputNum));
  t_control_state_lower_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlStateNum));
  t_control_state_upper_.assign(kTControlHorizon,
                                VecXd::Zero(kTControlStateNum));
  t_control_output_.assign(kTControlHorizon, VecXd::Zero(kTControlInputNum));
  t_control_output_unconstrained_.assign(kTControlHorizon,
                                         VecXd::Zero(kTControlInputNum));

  t_control_input_constraint_enable_ =
      Matrix::Zero(kTControlInputNum, kTControlInputNum);
  t_control_state_constraint_enable_ =
      Matrix::Zero(kTControlStateNum, kTControlStateNum);

  // Load configuration parameter
  const int t_matrix_q_size =
      control_conf->ts_pkmpc_controller_conf().t_matrix_q_size();
  const int t_matrix_r_size =
      control_conf->ts_pkmpc_controller_conf().t_matrix_r_size();
  if (control_conf->ts_pkmpc_controller_conf().has_t_matrix_q_integral()) {
    t_matrix_q_(0, 0) =
        control_conf->ts_pkmpc_controller_conf().t_matrix_q_integral();
  }
  if (control_conf->ts_pkmpc_controller_conf().has_t_matrix_n_integral()) {
    t_matrix_n_(0, 0) =
        control_conf->ts_pkmpc_controller_conf().t_matrix_n_integral();
  }
  for (int i = 1; i < kTControlStateNum; ++i) {
    t_matrix_q_(i, i) =
        control_conf->ts_pkmpc_controller_conf().t_matrix_q(i - 1);
    t_matrix_n_(i, i) =
        control_conf->ts_pkmpc_controller_conf().t_matrix_n(i - 1);
  }
  for (int i = 0; i < t_matrix_r_size; ++i) {
    t_matrix_r_(i, i) = control_conf->ts_pkmpc_controller_conf().t_matrix_r(i);
  }

  QCHECK(kTControlStateNum >= t_matrix_q_size)
      << "ST-PKMPC controller error: t_matrix_q_size in parameter file is "
         "larger "
         "than control state size";

  InitializeFilters();

  LoadGainScheduler(
      control_conf->ts_pkmpc_controller_conf().t_control_gain_scheduler(),
      &t_control_gain_scheduler_plf_);
  VLOG(1) << "T control gain scheduler loaded.";

  LogInitParameters();
  if (control_conf->enable_speed_mode_manager()) {
    closed_loop_acc_.InitClosedLoopAcc(*control_conf, throttle_lowerbound_,
                                       brake_lowerbound_);
  }

  if (control_conf->ts_pkmpc_controller_conf().has_lateral_error_integrator()) {
    lateral_error_integrator_.reset(new AntiWindupIntegrator);
    lateral_error_integrator_->Init(
        control_conf->ts_pkmpc_controller_conf().lateral_error_integrator(),
        control_conf->ts_pkmpc_controller_conf().ts());
  }
  if (control_conf->ts_pkmpc_controller_conf()
          .has_longitudinal_error_integrator()) {
    longitudinal_error_integrator_.reset(new AntiWindupIntegrator);
    longitudinal_error_integrator_->Init(
        control_conf->ts_pkmpc_controller_conf()
            .longitudinal_error_integrator(),
        control_conf->ts_pkmpc_controller_conf().ts());
  }
  // Mrac Init
  if (control_conf->mrac_conf().enable_mrac()) {
    MracConfig mrac_config;
    mrac_config.mrac_conf = control_conf->mrac_conf();
    mrac_config.steer_delay = control_conf->steer_delay_time();
    mrac_control_ = std::make_unique<MracControl>(mrac_config);
    CHECK_OK(mrac_control_->Init());
  }

  VLOG(1) << "[PolePlacementController] init done!";
  return absl::OkStatus();
}

void PolePlacementController::Stop() {}

void PolePlacementController::LoadGainScheduler(
    const GainScheduler &gain_scheduler,
    std::optional<PiecewiseLinearFunction<double>> *gain_scheduler_plf) {
  // TODO(shijun): replace it with PiecewiseLinearFunctionFromProto.
  std::vector<double> gain_scheduler_speed_vec;
  std::vector<double> gain_scheduler_ratio_vec;
  gain_scheduler_speed_vec.reserve(gain_scheduler.scheduler().size());
  gain_scheduler_ratio_vec.reserve(gain_scheduler.scheduler().size());
  for (const auto &scheduler : gain_scheduler.scheduler()) {
    gain_scheduler_speed_vec.push_back(scheduler.speed());
    gain_scheduler_ratio_vec.push_back(scheduler.ratio());
  }
  gain_scheduler_plf->emplace(gain_scheduler_speed_vec,
                              gain_scheduler_ratio_vec);
}

absl::Status PolePlacementController::ComputeControlCommand(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface,
    const ControlConstraints &control_constraint,
    const ControlHistoryStateManager &control_history_state_mgr,
    ControlCommand *cmd, ControllerDebugProto *controller_debug_proto) {
  QCHECK(cmd != nullptr);
  QCHECK(controller_debug_proto != nullptr);

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  ControlError *control_error_debug =
      cmd->mutable_debug()->mutable_control_error();
  const auto &steering_protection_result =
      control_constraint.steering_protection_result;
  double st_pole_placement_start_timestamp = ToUnixDoubleSeconds(Clock::Now());

  switch (wheel_drive_mode_) {
    case FRONT_WHEEL_DRIVE:
      ratio_of_RAC_speed_over_linear_speed_ =
          std::cos(vehicle_state.front_wheel_steering_angle());
    case REAR_WHEEL_DRIVE:
      ratio_of_RAC_speed_over_linear_speed_ = 1.0;
    case FOUR_WHEEL_DRIVE:
      // TODO(Zhichao): need to figure out what four or all wheel drive's
      // chassis speed stands for.
      ratio_of_RAC_speed_over_linear_speed_ = 1.0;
  }

  speed_feedback_ = vehicle_state.linear_velocity();

  // Apply state estimator to replace state measurement from positioning to
  // achieve a preset braking or starting behavior;
  // enable only v: half preset braking and starting mode;
  // TODO(zhichao, shijun): enable both v and s: full preset braking and
  // starting mode.
  if (control_conf_->state_estimator_setting().v_enable()) {
    const double v_trust_threshold =
        control_conf_->state_estimator_setting().v_threshold();
    if (speed_feedback_ < v_trust_threshold) {
      speed_feedback_ = state_estimator_.v;
    } else {
      state_estimator_.v = speed_feedback_;
    }
  }
  CalculateCurrError(vehicle_state, trajectory_interface, control_error_debug);

  // Time-control part for computing acceleration cmd
  FindTControlReference(vehicle_state, trajectory_interface, debug);

  TControlComputeLongitudinalErrors(vehicle_state, debug);

  TControlUpdateInitialStateAndMatrix(debug);

  if (control_conf_->enable_gain_scheduler() &&
      t_control_gain_scheduler_plf_.has_value()) {
    const double scheduler_gain =
        t_control_gain_scheduler_plf_->Evaluate(std::fabs(speed_feedback_));
    QCHECK_GT(scheduler_gain, 0.0)
        << "Longitudinal control loads negative gain "
           "schedular at the speed of "
        << speed_feedback_;
    t_matrix_q_updated_ = t_matrix_q_ * scheduler_gain;
    t_matrix_r_updated_ = t_matrix_r_;
    t_matrix_n_updated_ = t_matrix_n_ * scheduler_gain;
  } else {
    t_matrix_q_updated_ = t_matrix_q_;
    t_matrix_r_updated_ = t_matrix_r_;
    t_matrix_n_updated_ = t_matrix_n_;
  }

  TControlConstraintsSetup();

  double acceleration_cmd;
  {
    SCOPED_QTRACE("VehicleControlModule::POLEPLACEMENT_AccelerationController");
    const auto status = math::SolveLinearMPC(
        t_matrix_ad_t_, t_matrix_bd_, t_matrix_cd_, t_matrix_q_updated_,
        t_matrix_r_updated_, t_matrix_n_updated_,
        t_control_input_constraint_enable_, t_control_input_lower_,
        t_control_input_upper_, t_control_state_constraint_enable_,
        t_control_state_lower_, t_control_state_upper_, t_initial_state_,
        t_control_state_reference_, t_control_input_reference_,
        &t_control_output_, &t_control_output_unconstrained_);
    if (!status.ok()) {
      QEVENT("zhichao", "mpc_solver_fails", [&](QEvent *qevent) {
        qevent->AddField("controller_name", "ts_pkmpc_controller")
            .AddField("controller_type", "longitudinal_control");
      });
      const std::string error_msg = absl::StrCat(
          "MPC solver failed, controller_name: ts_pkmpc_controller ",
          " controller_type: longitudinal_control, detailed reason: ",
          status.ToString());
      return absl::InternalError(error_msg);
    } else {
      acceleration_cmd = t_control_output_[0](0);
    }
  }

  debug->set_acceleration_cmd_closeloop(t_control_output_[0](0));
  acceleration_cmd /= ratio_of_RAC_speed_over_linear_speed_;

  TControlVLOG();

  // Predict pose after steer delay
  VehState veh_state = {
      .steer_delay_time =
          cmd->debug().bias_estimation_debug().steer_delay_online(),
      .init_pose = {.x = vehicle_state.x(),
                    .y = vehicle_state.y(),
                    .v = speed_feedback_,
                    .heading = vehicle_state.yaw()},
      .control_history_state_mgr = &control_history_state_mgr};
  VehPose veh_predicted_pose =
      PredictedPoseByConstAccKinematicModel(veh_state, wheel_base_);
  veh_predicted_pose.ToProto(
      controller_debug_proto->mutable_predicted_veh_pose_proto());

  FindPoleControlReference(veh_predicted_pose.x, veh_predicted_pose.y,
                           veh_predicted_pose.heading, trajectory_interface,
                           debug, controller_debug_proto);

  double steer_feedback;
  const bool is_auto_mode = vehicle_state.is_auto_mode();
  constexpr double WeightUpdateHeadingErr = 0.1;
  if (!is_auto_mode) {
    filted_heading_error_ = 0.0;
  } else {
    filted_heading_error_ =
        filted_heading_error_ * (1.0 - WeightUpdateHeadingErr) +
        debug->heading_error() * WeightUpdateHeadingErr;
  }

  steer_feedback = PolePlacementFeedbackSteer(
      debug->lateral_error(), filted_heading_error_, speed_feedback_,
      wheel_base_, kControlInterval,
      cmd->mutable_debug()->mutable_pole_placement_debug());

  double steer_cmd_kappa_controller =
      debug->kappa_feedforward() + steer_feedback;
  cmd->mutable_debug()->mutable_pole_placement_debug()->set_steer_feedback(
      steer_feedback);
  cmd->mutable_debug()->mutable_pole_placement_debug()->set_steer_forward(
      debug->kappa_feedforward());
  cmd->mutable_debug()->mutable_pole_placement_debug()->set_steer_output(
      steer_cmd_kappa_controller);

  double st_pole_placement_end_timestamp = ToUnixDoubleSeconds(Clock::Now());

  VLOG(1) << "MPC core algorithm: calculation time is: "
          << (st_pole_placement_end_timestamp -
              st_pole_placement_start_timestamp) *
                 1000
          << " ms.";

  // TControl cmd post-processing.
  // Add deceleration filter to avoid hard brake.
  constexpr double kHardBrakeFilterThreshold = -1.0;  // m/s^2.
  constexpr double kHardBrakeCmdIntegralRatio = 1.07;
  if (acceleration_cmd < kHardBrakeFilterThreshold) {
    hard_brake_cmd_integral_ *= kHardBrakeCmdIntegralRatio;
    acceleration_cmd = std::clamp(
        acceleration_cmd, previous_acceleration_cmd_ - hard_brake_cmd_integral_,
        kHardBrakeFilterThreshold);
  } else {
    constexpr double kInitAccelRate =
        1.0;  // m/s^3, initial acceleration changing rate.
    hard_brake_cmd_integral_ = kInitAccelRate * kControlInterval;
  }

  double acc_calibration = acceleration_cmd;

  // Stop standstill TControl post-process logic.
  const bool is_full_stop = FullStopState(
      debug->acceleration_reference(), debug->speed_reference(),
      speed_feedback_, full_stop_condition_, vehicle_state.gear());
  debug->set_is_full_stop(is_full_stop);
  if (is_full_stop) {
    full_stop_brake_cmd_integral_ *=
        std::max(1.0, full_stop_condition_.brake_integral_ratio());
    double standstill_acceleration =
        full_stop_condition_.standstill_acceleration();
    if (std::fabs(speed_feedback_) < kStopSpeed) {
      standstill_acceleration =
          std::min(standstill_acceleration,
                   full_stop_condition_.lockdown_acceleration());
      constexpr double kStopIncreaseBrakeRate =
          0.25;  // Increase brake with jerk 0.25 m/s^3 when stopped.
      full_stop_brake_cmd_integral_ = kStopIncreaseBrakeRate * kControlInterval;
    }
    if (vehicle_state.gear() == Chassis::GEAR_REVERSE) {
      const double acceleration_standstill =
          std::min(previous_acc_calibration_ + full_stop_brake_cmd_integral_,
                   -standstill_acceleration);
      acc_calibration = std::max(acc_calibration, acceleration_standstill);
    } else {
      const double acceleration_standstill =
          std::max(previous_acc_calibration_ - full_stop_brake_cmd_integral_,
                   standstill_acceleration);
      acc_calibration = std::min(acc_calibration, acceleration_standstill);
    }
    QLOG_EVERY_N_SEC(INFO, 10.0) << "Stop location reached";
  } else {
    constexpr double kInitAccelRate =
        1.0;  // m/s^3, initial acceleration changing rate.
    full_stop_brake_cmd_integral_ = kInitAccelRate * kControlInterval;
  }

  debug->set_acceleration_cmd(acceleration_cmd);
  cmd->set_acceleration(acceleration_cmd);
  previous_acceleration_cmd_ = acceleration_cmd;

  // Compute speed cmd
  const double speed_cmd =
      previous_speed_cmd_ + acceleration_cmd * kControlInterval;
  cmd->set_speed(speed_cmd);
  previous_speed_cmd_ = speed_cmd;

  // Estimate acceleration offset based on pose pitch.
  const double pitch = vehicle_state.pitch();
  const double sin_slope = -std::sin(pitch);
  controller_debug_proto->set_sin_slope(sin_slope);
  constexpr double kSinSlopeLimit = 0.2588;  // slope angle limit, 15 degrees.
  const double sin_slope_filtered = sin_slope_mean_filter_.Update(
      std::clamp(sin_slope, -kSinSlopeLimit, kSinSlopeLimit));

  const double acceleration_offset =
      sin_slope_filtered * kGravitationalAcceleration;
  controller_debug_proto->set_acceleration_offset(acceleration_offset);
  cmd->set_acceleration_offset(acceleration_offset);

  if (is_full_stop) {
    if (vehicle_state.gear() == Chassis::GEAR_REVERSE) {
      acc_calibration +=
          std::fabs(acceleration_offset);  // full stop when uphill
    } else {
      acc_calibration -=
          std::fabs(acceleration_offset);  // full stop when uphill
    }
  } else {
    acc_calibration += acceleration_offset;
  }

  // Update acc by speed mode

  if (control_conf_->enable_speed_mode_manager()) {
    if (vehicle_state.gear() == Chassis::GEAR_REVERSE) {
      acc_calibration = -1 * closed_loop_acc_.UpdateAccCommand(
                                 is_auto_mode, -acc_calibration,
                                 -acceleration_cmd, -speed_feedback_,
                                 acceleration_offset, controller_debug_proto);
    } else {
      acc_calibration = closed_loop_acc_.UpdateAccCommand(
          is_auto_mode, acc_calibration, acceleration_cmd, speed_feedback_,
          acceleration_offset, controller_debug_proto);
    }
  }

  cmd->set_acceleration_calibration(acc_calibration);
  previous_acc_calibration_ = acc_calibration;

  CalibrationValue calibration_value;
  calibration_manager_->UpdateCalibrationValue(speed_feedback_, acc_calibration,
                                               vehicle_state.gear(),
                                               &calibration_value);

  // Estimate vehicle states.
  if (control_conf_->state_estimator_setting().v_enable()) {
    state_estimator_.v += acceleration_cmd * 0.01;
    state_estimator_.v =
        std::clamp(state_estimator_.v, 0.0,
                   control_conf_->state_estimator_setting().v_threshold());
  }

  is_standstill_ =
      UpdateStandStillState(speed_feedback_, control_conf_->control_period(),
                            is_full_stop, is_standstill_, standstill_proto_);
  controller_debug_proto->mutable_speed_mode_debug_proto()
      ->set_standstill_counter(standstill_counter_);
  controller_debug_proto->mutable_speed_mode_debug_proto()->set_is_standstill(
      is_standstill_);

  double steer_rad_abs =
      std::abs(std::atan(steer_cmd_kappa_controller / wheel_base_));

  auto control_calibration_value = calibration_value.control_calibration_value;
  if (control_conf_->closed_loop_acc_conf().enable_closed_loop_acc()) {
    control_calibration_value = closed_loop_acc_.UpdateCalibrationCmd(
        is_auto_mode, is_full_stop, control_calibration_value,
        vehicle_state.linear_acceleration(), steer_rad_abs,
        controller_debug_proto);
  }

  CalibrationCmd calibration_cmd;
  calibration_manager_->ComputeLongitudinalCmd(control_calibration_value,
                                               &calibration_cmd);

  const double acceleration_idle = calibration_manager_->GetIdleAcceleration();
  const double acceleration_pure = calibration_manager_->GetPureAcceleration();
  controller_debug_proto->mutable_calibration_debug_proto()
      ->set_acceleration_idle(acceleration_idle);
  controller_debug_proto->mutable_calibration_debug_proto()
      ->set_acceleration_pure(acceleration_pure);
  debug->set_calibration_value(control_calibration_value);
  cmd->set_throttle(calibration_cmd.throttle_cmd);
  cmd->set_brake(calibration_cmd.brake_cmd);
  cmd->set_e_brake(calibration_cmd.ebrake_cmd);
  cmd->set_aeb_triggered(trajectory_interface.aeb_triggered());

  // SControl cmd post-processing.
  double kappa_cmd_mrac = steer_cmd_kappa_controller;
  if (control_conf_->has_mrac_conf() &&
      control_conf_->mrac_conf().enable_mrac()) {
    // Mrac control computer
    const KappaConstraint kappa_constraint = {
        .kappa_upper = steering_protection_result.kappa_output_upper()[0],
        .kappa_lower = steering_protection_result.kappa_output_lower()[0],
        .kappa_rate_upper = steering_protection_result.kappa_rate_upper(),
        .kappa_rate_lower = steering_protection_result.kappa_rate_lower(),
    };
    kappa_cmd_mrac = mrac_control_->MracComputer(
        is_auto_mode, steer_cmd_kappa_controller, vehicle_state.kappa(),
        speed_feedback_, kappa_constraint,
        controller_debug_proto->mutable_mrac_debug_proto());
  }

  previous_steering_cmd_ = kappa_cmd_mrac;
  cmd->set_curvature(kappa_cmd_mrac);

  // When av is in auto_mode and stops standstill and receives stationary
  // trajectory, enter stationary steering state and override curvature.
  const bool has_enter_stationary_steering_state =
      is_standstill_ && is_auto_mode &&
      trajectory_interface.GetIsStationaryTrajectory();
  if (has_enter_stationary_steering_state) {
    cmd->mutable_debug()->set_has_enter_stationary_steering(true);
    VLOG(2) << "Enter stationary steering state.";
    const double chassis_steer_angle =
        vehicle_state.front_wheel_steering_angle();
    const double av_kappa =
        FrontWheelAngle2Kappa(chassis_steer_angle, wheel_base_);
    const double ref_kappa =
        trajectory_interface.GetAllTrajPoints(false)[0].path_point().kappa();
    const double control_curvature = CalcStationarySteeringCmd(
        previous_steering_cmd_, av_kappa, ref_kappa, wheel_base_, steer_ratio_);
    cmd->set_curvature(control_curvature);
    previous_steering_cmd_ = control_curvature;
  }

  return absl::OkStatus();
}

void PolePlacementController::Reset(const Chassis &chassis,
                                    const VehicleStateProto &vehicle_state) {
  previous_speed_cmd_ = vehicle_state.linear_velocity();
}

void PolePlacementController::CalculateCurrError(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface,
    ControlError *control_error_debug) {  // For longitudinal control, do not
                                          // use the transition trajectory.
  double control_relative_time =
      vehicle_state.timestamp() -
      trajectory_interface.GetPlannerStartTime(false);

  const ApolloTrajectoryPointProto ref_traj_point =
      trajectory_interface.QueryTrajectoryPointByRelativeTime(
          false, control_relative_time);
  const Vec2d ref_xy(ref_traj_point.path_point().x(),
                     ref_traj_point.path_point().y());
  const Vec2d ref_tangent =
      Vec2d::UnitFromAngle(ref_traj_point.path_point().theta());
  const Vec2d vs_pos(vehicle_state.x(), vehicle_state.y());

  control_error_debug->set_station_error((vs_pos - ref_xy).dot(ref_tangent));

  control_error_debug->set_speed_error(vehicle_state.linear_velocity() -
                                       ref_traj_point.v());

  // Step 1: find the closest trajectory point from trajectory and past
  // points.
  ApolloTrajectoryPointProto closest_trajectory_point;
  constexpr bool kIsOnTransitionTraj = false;
  if (last_matched_point_s_ == 0.0) {
    VLOG(1) << "First search lateral control match point!";
    closest_trajectory_point =
        trajectory_interface.QueryNearestTrajectoryPointByPosition(
            kIsOnTransitionTraj, vehicle_state.x(), vehicle_state.y());
  } else {
    closest_trajectory_point =
        trajectory_interface.QueryNearestTrajectoryPointByPositionWithSRange(
            kIsOnTransitionTraj, vehicle_state.x(), vehicle_state.y(),
            last_matched_point_s_, speed_feedback_);
  }

  const auto &closest_path_point = closest_trajectory_point.path_point();
  // Render the closest_path_point in vantage

  const Vec2d closest_path_point_xy(closest_path_point.x(),
                                    closest_path_point.y());
  const Vec2d closest_path_point_tangent =
      Vec2d::UnitFromAngle(closest_path_point.theta());
  const Vec2d closest_path_point_normal = closest_path_point_tangent.Perp();

  const double ref_heading = closest_path_point.theta();
  control_error_debug->set_heading_error(
      NormalizeAngle(vehicle_state.yaw() - ref_heading));
  control_error_debug->set_lateral_error(
      (vs_pos - closest_path_point_xy).dot(closest_path_point_normal));
}

// T-control functions

void PolePlacementController::FindTControlReference(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface, SimpleMPCDebug *debug) {
  // For longitudinal control, do not use the transition trajectory.
  const bool on_transition_traj = false;
  t_control_relative_time_ =
      vehicle_state.timestamp() -
      trajectory_interface.GetPlannerStartTime(on_transition_traj);
  double target_relative_time = t_control_relative_time_;

  // i = 0: collect information for s-control use and calculate tracking
  // error. i = 1 to N: calculate t-control state reference information. i = 0
  // to (N-1): calculate t-control input reference information.
  for (int i = 0; i < kTControlHorizon + 1; ++i) {
    const ApolloTrajectoryPointProto ref_traj_point =
        trajectory_interface.QueryTrajectoryPointByRelativeTime(
            on_transition_traj, target_relative_time);

    if (i == 0) {
      // Collect information for s-control use and calculate tracking error.
      const Vec2d ref_xy(ref_traj_point.path_point().x(),
                         ref_traj_point.path_point().y());
      const Vec2d ref_tangent =
          Vec2d::UnitFromAngle(ref_traj_point.path_point().theta());
      const Vec2d vs_pos(vehicle_state.x(), vehicle_state.y());

      t_control_initial_s_ = ref_traj_point.path_point().s();
      const double station_error = (vs_pos - ref_xy).dot(ref_tangent);
      debug->set_station_error(station_error);
      if (longitudinal_error_integrator_) {
        longitudinal_error_integrator_->Integrate(station_error,
                                                  is_standstill_);
        const double longitudinal_error_integral =
            longitudinal_error_integrator_->GetIntegralValue();
        debug->set_longitudinal_error_integral(longitudinal_error_integral);
      }
      debug->set_speed_reference(ref_traj_point.v());
      debug->set_acceleration_reference(ref_traj_point.a());
    } else {
      // MPC state reference step is from 1 to N.
      t_control_state_reference_[i - 1](1) = ref_traj_point.path_point().s();
      t_control_state_reference_[i - 1](2) = ref_traj_point.v();
    }
    if (i < kTControlHorizon) {
      // MPC input reference step is from 0 to N-1.
      t_control_input_reference_[i](0) = ref_traj_point.a();
    }

    target_relative_time += ts_;
  }
}

void PolePlacementController::TControlComputeLongitudinalErrors(
    const VehicleStateProto &vehicle_state, SimpleMPCDebug *debug) {
  debug->set_speed_feedback(speed_feedback_);
  debug->set_acceleration_feedback(vehicle_state.linear_acceleration());

  debug->set_speed_error(debug->speed_feedback() - debug->speed_reference());
  debug->set_acceleration_error(vehicle_state.linear_acceleration() -
                                debug->acceleration_reference());
}

void PolePlacementController::TControlUpdateInitialStateAndMatrix(
    SimpleMPCDebug *debug) {
  t_initial_state_(0) = longitudinal_error_integrator_
                            ? longitudinal_error_integrator_->GetIntegralValue()
                            : 0;
  t_initial_state_(1) = debug->station_error() + t_control_initial_s_;
  t_initial_state_(2) = debug->speed_feedback();

  t_matrix_a_ << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  t_matrix_b_ << 0.0, 0.0, 1.0;

  const Matrix t_matrix_i =
      Matrix::Identity(kTControlStateNum, kTControlStateNum);
  const Matrix t_matrix_a_delta = t_matrix_a_ * ts_;
  const Matrix t_matrix_b_delta = t_matrix_b_ * ts_;

  t_matrix_ad_ =
      t_matrix_i + t_matrix_a_delta + 0.5 * t_matrix_a_delta * t_matrix_a_delta;
  t_matrix_bd_ = t_matrix_b_delta + 0.5 * t_matrix_a_delta * t_matrix_b_delta +
                 0.167 * t_matrix_a_delta * t_matrix_a_delta * t_matrix_b_delta;

  if (longitudinal_error_integrator_) {
    t_matrix_ad_(0, 0) =
        longitudinal_error_integrator_->GetIntegralAlphaValue();
  } else {
    t_matrix_ad_(0, 0) = 0.0;
    t_matrix_ad_(0, 1) = 0.0;
    t_matrix_ad_(0, 2) = 0.0;
    t_matrix_bd_(0, 0) = 0.0;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    t_matrix_ad_t_[i] = t_matrix_ad_;
  }
}

void PolePlacementController::TControlConstraintsSetup() {
  t_control_input_constraint_enable_ << 1.0;
  for (int i = 0; i < kTControlHorizon; ++i) {
    t_control_input_lower_[i](0) = max_deceleration_;
    t_control_input_upper_[i](0) = max_acceleration_;
  }
}

void PolePlacementController::TControlVLOG() {
  VLOG(1) << "---------------------------------------------------" << std::endl;
  VLOG(1) << "t_Ad = \n" << t_matrix_ad_ << std::endl;
  VLOG(1) << "t_Bd = \n" << t_matrix_bd_ << std::endl;
  VLOG(1) << "t_Cd = \n" << t_matrix_cd_ << std::endl;
  VLOG(1) << "t_Q = \n" << t_matrix_q_updated_ << std::endl;
  VLOG(1) << "t_R = \n" << t_matrix_r_updated_ << std::endl;
  VLOG(1) << "t_N = \n" << t_matrix_n_updated_ << std::endl;
  VLOG(1) << "t_control_input_constraint_enable_ = \n"
          << t_control_input_constraint_enable_ << std::endl;
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_input_lower_: "
            << t_control_input_lower_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_input_upper_: "
            << t_control_input_upper_[i].transpose() << std::endl;
  }
  VLOG(1) << "t_control_state_constraint_enable_ = \n"
          << t_control_state_constraint_enable_ << std::endl;
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_state_lower_: "
            << t_control_state_lower_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_state_upper_: "
            << t_control_state_upper_[i].transpose() << std::endl;
  }
  VLOG(1) << "t_initial_state_ = " << t_initial_state_.transpose() << std::endl;
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_state_reference_[" << i
            << "]: " << t_control_state_reference_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_input_reference_[" << i
            << "]: " << t_control_input_reference_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_output_: " << t_control_output_[i].transpose()
            << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_output_unconstrained: "
            << t_control_output_unconstrained_[i].transpose() << std::endl;
  }
  std::vector<Matrix> t_control_state(kTControlHorizon + 1, t_initial_state_);
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "t_control_error[" << i << "] = "
            << t_control_state[i].transpose() -
                   t_control_state_reference_[i].transpose()
            << std::endl;
    t_control_state[i + 1] =
        t_matrix_ad_ * t_control_state[i] + t_matrix_bd_ * t_control_output_[i];
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "delta_output_control [" << i << "] = "
            << t_control_output_[i].transpose() -
                   t_control_input_reference_[i].transpose()
            << std::endl;
  }
  for (int i = 0; i < kTControlHorizon; ++i) {
    VLOG(1) << "delta t_control_output_unconstrained: "
            << t_control_output_unconstrained_[i].transpose() -
                   t_control_input_reference_[i].transpose()
            << std::endl;
  }

  // TODO(Zhichao): potential bug of planner v, a matching
  for (int i = 1; i < kTControlHorizon; ++i) {
    VLOG(1) << "speed ref [" << i
            << "] is: " << t_control_state_reference_[i](1)
            << "; speed from a [" << i << "] is: "
            << t_control_state_reference_[i - 1](1) +
                   t_control_input_reference_[i](0) * 0.1
            << "; error is: "
            << t_control_state_reference_[i](1) -
                   t_control_state_reference_[i - 1](1) -
                   t_control_input_reference_[i](0) * 0.1;
  }
  for (int i = 1; i < kTControlHorizon; ++i) {
    VLOG(1) << "s ref [" << i << "] is: " << t_control_state_reference_[i](0)
            << "; s from v [" << i << "] is: "
            << t_control_state_reference_[i - 1](0) +
                   t_control_state_reference_[i - 1](1) * 0.1
            << "; error is: "
            << t_control_state_reference_[i](0) -
                   t_control_state_reference_[i - 1](0) -
                   t_control_state_reference_[i - 1](1) * 0.1;
  }
  VLOG(1) << "---------------------------------------------------------------"
          << std::endl;
}

void PolePlacementController::FindPoleControlReference(
    double x, double y, double yaw,
    const TrajectoryInterface &trajectory_interface, SimpleMPCDebug *debug,
    ControllerDebugProto *controller_debug_proto) {
  const Vec2d vs_pos(x, y);
  // const Vec2d vs_heading_tangent = Vec2d::UnitFromAngle(yaw);

  // Step 1: find the closest trajectory point from trajectory and past
  // points.
  ApolloTrajectoryPointProto closest_trajectory_point;
  constexpr bool kIsOnTransitionTraj = false;
  if (last_matched_point_s_ == 0.0) {
    VLOG(1) << "First search lateral control match point!";
    closest_trajectory_point =
        trajectory_interface.QueryNearestTrajectoryPointByPosition(
            kIsOnTransitionTraj, x, y);
    last_matched_point_s_ = closest_trajectory_point.path_point().s();
  } else {
    closest_trajectory_point =
        trajectory_interface.QueryNearestTrajectoryPointByPositionWithSRange(
            kIsOnTransitionTraj, x, y, last_matched_point_s_, speed_feedback_);
    last_matched_point_s_ = closest_trajectory_point.path_point().s();
  }

  controller_debug_proto->set_lat_ref_relative_time(
      closest_trajectory_point.relative_time());
  const auto &closest_path_point = closest_trajectory_point.path_point();
  // Render the closest_path_point in vantage
  Vec2dProto *closest_path_point_for_render =
      controller_debug_proto->mutable_mpc_debug_proto()
          ->add_mpc_reference_traj_point();
  closest_path_point_for_render->set_x(closest_path_point.x());
  closest_path_point_for_render->set_y(closest_path_point.y());

  const Vec2d closest_path_point_xy(closest_path_point.x(),
                                    closest_path_point.y());
  const Vec2d closest_path_point_tangent =
      Vec2d::UnitFromAngle(closest_path_point.theta());
  const Vec2d closest_path_point_normal = closest_path_point_tangent.Perp();
  const double current_s = closest_path_point.s();

  const double ref_heading = closest_path_point.theta();
  const double heading_error = NormalizeAngle(yaw - ref_heading);
  const double lateral_error =
      (vs_pos - closest_path_point_xy).dot(closest_path_point_normal);

  debug->set_ref_heading(ref_heading);
  debug->set_heading_error(heading_error);
  debug->set_lateral_error(lateral_error);

  // Reference kappa, considering steering delay time.
  constexpr bool kApplyTransitionTrajOnLatControl = true;
  double ref_kappa = trajectory_interface
                         .QueryTrajectoryPointBasedOnPathS(
                             kApplyTransitionTrajOnLatControl, current_s)
                         .path_point()
                         .kappa();

  debug->set_kappa_feedforward(ref_kappa);
}

bool PolePlacementController::FullStopState(
    double accel_planner, double speed_planner, double linear_speed,
    const FullStopProto &full_stop_condition,
    Chassis::GearPosition gear_position) const {
  // Entering control full stop conditions.
  // TODO(shijun): need to refine after launch freespace planner.
  if (gear_position == Chassis::GEAR_DRIVE && accel_planner > 0.0) {
    return false;
  }
  if (gear_position == Chassis::GEAR_REVERSE && accel_planner < 0.0) {
    return false;
  }
  if (std::abs(speed_planner) >
          full_stop_condition.abs_planner_speed_upperlimit() ||
      std::abs(linear_speed) >
          full_stop_condition.abs_linear_speed_upperlimit()) {
    return false;
  }
  return true;
}

bool PolePlacementController::UpdateStandStillState(
    double speed_measurement, double control_period, bool is_full_stop,
    bool is_standstill, const StandStillProto &standstill_proto) {
  const int standstill_counter_threshold =
      FloorToInt(standstill_proto.standstill_time_th() / control_period);
  if (std::fabs(speed_measurement) > standstill_proto.vel_move_th() ||
      !(is_full_stop)) {
    standstill_counter_ = 0;
    return false;
  }
  if (!is_standstill) {
    if (std::fabs(speed_measurement) < standstill_proto.vel_standstill_th()) {
      standstill_counter_ =
          std::min(standstill_counter_ + 1, standstill_counter_threshold);
      if (standstill_counter_ == standstill_counter_threshold) {
        return true;
      }
    } else {
      standstill_counter_ = std::max(standstill_counter_ - 1, 0);
    }
  }
  return is_standstill;
}

double PolePlacementController::PolePlacementFeedbackSteer(
    double lateral_error, double heading_error, double speed, double wheelbase,
    double ts, PolePlacementDebug *debug) {
  constexpr double kEpsilonSpeed = 0.1;
  constexpr double kPoleMinSpeed = 0.6;
  constexpr double kMaxLatErr = 0.5;
  constexpr double kMaxHeadingErr = 0.175;

  double pole = -1 * v_pole_plf_(std::max(std::abs(speed), kEpsilonSpeed));
  double v_limit = std::max(std::fabs(speed), kPoleMinSpeed);
  if (speed < 0.0) {
    v_limit = std::min(std::fabs(speed), -kPoleMinSpeed);
  }

  const double pole_exp = std::exp(pole * ts);

  double lat_gain = ((1 - 2 * pole_exp + Sqr(pole_exp)) * wheelbase) /
                    (Sqr(v_limit) * Sqr(ts));
  double heading_gain =
      ((3 - 2 * pole_exp - Sqr(pole_exp)) * wheelbase) / (2 * v_limit * ts);
  double lateral_error_limit =
      std::clamp(lateral_error, -kMaxLatErr, kMaxLatErr);

  double heading_error_limit =
      std::clamp(lateral_error, -kMaxHeadingErr, kMaxHeadingErr);

  double steer_feedback =
      -FrontWheelAngle2Kappa(std::atan(lateral_error_limit * lat_gain +
                                       heading_error_limit * heading_gain),
                             wheelbase);

  debug->set_lateral_err(lateral_error_limit);
  debug->set_heading_err(filted_heading_error_);
  debug->set_pole(pole);
  debug->set_pole_exp(pole_exp);
  return steer_feedback;
}

}  // namespace qcraft::control
