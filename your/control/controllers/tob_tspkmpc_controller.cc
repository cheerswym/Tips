#include "onboard/control/controllers/tob_tspkmpc_controller.h"

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

bool IsSteeringCmdBackToCenter(
    const std::vector<Eigen::VectorXd> &kappa_rate_prediction,
    double current_kappa_cmd, double ts) {
  // Consider next kKappaRateCmdSize + 1 kappa command prediction.
  constexpr int kKappaRateCmdSize = 4;
  QCHECK_GE(kappa_rate_prediction.size(), kKappaRateCmdSize);

  std::vector<double> kappa_cmd_prediction;
  kappa_cmd_prediction.reserve(kKappaRateCmdSize + 1);
  kappa_cmd_prediction.push_back(current_kappa_cmd);
  for (int i = 0; i < kKappaRateCmdSize; ++i) {
    const double kappa_rate = kappa_rate_prediction[i](0);
    const double next_kappa_cmd = current_kappa_cmd + kappa_rate * ts;
    kappa_cmd_prediction.push_back(next_kappa_cmd);
    current_kappa_cmd = next_kappa_cmd;
  }

  return control::IsCmdBackToZero(kappa_cmd_prediction);
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

}  // namespace

TobTsPkmpcController::TobTsPkmpcController() {}

absl::Status TobTsPkmpcController::LoadControlConf(
    const ControllerConf *control_conf,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  if (!control_conf->has_ts_pkmpc_controller_conf() ||
      !control_conf->ts_pkmpc_controller_conf().has_s_matrix_psi() ||
      !control_conf->ts_pkmpc_controller_conf().has_s_matrix_n_kappa()) {
    const auto error_msg =
        absl::StrCat("[TobTsPkmpcController] lacks the controller weights.");
    QLOG(ERROR) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }

  control_conf_ = control_conf;
  vehicle_drive_params_ = &vehicle_drive_params;

  ts_ = control_conf_->ts_pkmpc_controller_conf().ts();

  QCHECK_GT(ts_, 0.0) << "[TobTsPkmpcController] Invalid control time step "
                      << ts_;
  step_ratio_ = ts_ / kControlInterval;
  max_steer_angle_ = vehicle_drive_params.max_steer_angle();
  wheel_base_ = vehicle_geometry_params.wheel_base();
  steer_ratio_ = vehicle_drive_params.steer_ratio();

  max_acceleration_ = control_conf_->max_acceleration_cmd();
  max_deceleration_ = control_conf_->max_deceleration_cmd();

  VLOG(1) << "TOB STPKMPC conf loaded";
  return absl::OkStatus();
}

void TobTsPkmpcController::LogInitParameters() { VLOG(1) << "TS_PKMPC begin."; }

absl::Status TobTsPkmpcController::Init(
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

  // s-control matrix initialization

  s_initial_state_ = VecXd::Zero(kSControlStateNum);

  s_matrix_a_ = Matrix::Zero(kSControlStateNum, kSControlStateNum);
  s_matrix_ad_ = s_matrix_a_;
  s_matrix_ad_t_.assign(kSControlHorizon, s_matrix_ad_);
  s_matrix_b_ = Matrix::Zero(kSControlStateNum, 1);
  s_matrix_bd_ = s_matrix_b_;
  s_matrix_c_ = Matrix::Zero(kSControlStateNum, 1);
  s_matrix_cd_ = s_matrix_c_;
  s_matrix_cd_t_.assign(kSControlHorizon, s_matrix_cd_);

  s_matrix_q_ = Matrix::Zero(kSControlStateNum, kSControlStateNum);
  s_matrix_q_updated_ = s_matrix_q_;
  s_matrix_r_ = Matrix::Zero(1, 1);
  s_matrix_r_updated_ = s_matrix_r_;
  s_matrix_n_ = Matrix::Zero(kSControlStateNum, kSControlStateNum);
  s_matrix_n_updated_ = s_matrix_n_;

  s_control_state_reference_.assign(kSControlHorizon,
                                    VecXd::Zero(kSControlStateNum));
  s_control_input_reference_.assign(kSControlHorizon,
                                    VecXd::Zero(kSControlInputNum));
  s_control_input_lower_.assign(kSControlHorizon,
                                VecXd::Zero(kSControlInputNum));
  s_control_input_upper_.assign(kSControlHorizon,
                                VecXd::Zero(kSControlInputNum));
  s_control_state_lower_.assign(kSControlHorizon,
                                VecXd::Zero(kSControlStateNum));
  s_control_state_upper_.assign(kSControlHorizon,
                                VecXd::Zero(kSControlStateNum));
  s_control_output_.assign(kSControlHorizon, VecXd::Zero(kSControlInputNum));
  s_control_output_unconstrained_.assign(kSControlHorizon,
                                         VecXd::Zero(kSControlInputNum));

  s_control_input_constraint_enable_ =
      Matrix::Zero(kSControlInputNum, kSControlInputNum);
  s_control_state_constraint_enable_ =
      Matrix::Zero(kSControlStateNum, kSControlStateNum);

  // Load configuration parameter
  const int t_matrix_q_size =
      control_conf_->ts_pkmpc_controller_conf().t_matrix_q_size();
  const int t_matrix_r_size =
      control_conf_->ts_pkmpc_controller_conf().t_matrix_r_size();
  if (control_conf_->ts_pkmpc_controller_conf().has_t_matrix_q_integral()) {
    t_matrix_q_(0, 0) =
        control_conf_->ts_pkmpc_controller_conf().t_matrix_q_integral();
  }
  if (control_conf_->ts_pkmpc_controller_conf().has_t_matrix_n_integral()) {
    t_matrix_n_(0, 0) =
        control_conf_->ts_pkmpc_controller_conf().t_matrix_n_integral();
  }
  for (int i = 1; i < kTControlStateNum; ++i) {
    t_matrix_q_(i, i) =
        control_conf_->ts_pkmpc_controller_conf().t_matrix_q(i - 1);
    t_matrix_n_(i, i) =
        control_conf_->ts_pkmpc_controller_conf().t_matrix_n(i - 1);
  }
  for (int i = 0; i < t_matrix_r_size; ++i) {
    t_matrix_r_(i, i) = control_conf_->ts_pkmpc_controller_conf().t_matrix_r(i);
  }

  // Lateral state = [x, y, theta, (kappa_delay), kappa]^T;
  s_matrix_q_(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_xy();
  s_matrix_q_(1, 1) = s_matrix_q_(0, 0);
  s_matrix_q_(2, 2) = control_conf_->ts_pkmpc_controller_conf().s_matrix_yaw();
  s_matrix_q_(kSControlStateNum - 1, kSControlStateNum - 1) =
      control_conf_->ts_pkmpc_controller_conf().s_matrix_kappa();

  s_matrix_n_(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_n_xy();
  s_matrix_n_(1, 1) = s_matrix_n_(0, 0);
  s_matrix_n_(2, 2) =
      control_conf_->ts_pkmpc_controller_conf().s_matrix_n_yaw();
  s_matrix_n_(kSControlStateNum - 1, kSControlStateNum - 1) =
      control_conf_->ts_pkmpc_controller_conf().s_matrix_n_kappa();

  s_matrix_r_(0, 0) = control_conf_->ts_pkmpc_controller_conf().s_matrix_psi();

  QCHECK(kTControlStateNum >= t_matrix_q_size)
      << "ST-PKMPC controller error: matrix_q size in parameter file is larger "
         "than control state size";

  LoadGainScheduler(
      control_conf_->ts_pkmpc_controller_conf().t_control_gain_scheduler(),
      &t_control_gain_scheduler_plf_);
  VLOG(1) << "T control gain scheduler loaded.";
  LoadGainScheduler(
      control_conf_->ts_pkmpc_controller_conf().s_control_gain_scheduler(),
      &s_control_gain_scheduler_plf_);
  VLOG(1) << "S control gain scheduler loaded.";

  LogInitParameters();

  lon_postprocess_manager_ =
      std::make_unique<LonPostProcess>(control_conf_, vehicle_drive_params_);

  if (control_conf_->ts_pkmpc_controller_conf()
          .has_lateral_error_integrator()) {
    lateral_error_integrator_.reset(new AntiWindupIntegrator);
    lateral_error_integrator_->Init(
        control_conf_->ts_pkmpc_controller_conf().lateral_error_integrator(),
        control_conf_->ts_pkmpc_controller_conf().ts());
  }
  if (control_conf_->ts_pkmpc_controller_conf()
          .has_longitudinal_error_integrator() &&
      control_conf_->ts_pkmpc_controller_conf().has_t_matrix_q_integral() &&
      control_conf_->ts_pkmpc_controller_conf().has_t_matrix_n_integral()) {
    longitudinal_error_integrator_.reset(new AntiWindupIntegrator);
    longitudinal_error_integrator_->Init(
        control_conf_->ts_pkmpc_controller_conf()
            .longitudinal_error_integrator(),
        control_conf_->ts_pkmpc_controller_conf().ts());
  }

  // Mrac Init
  if (control_conf_->mrac_conf().enable_mrac()) {
    MracConfig mrac_config;
    mrac_config.mrac_conf = control_conf_->mrac_conf();
    mrac_config.steer_delay = control_conf_->steer_delay_time();
    mrac_control_ = std::make_unique<MracControl>(mrac_config);
    CHECK_OK(mrac_control_->Init());
  }

  VLOG(1) << "[TobTsPkmpcController] init done!";
  return absl::OkStatus();
}

void TobTsPkmpcController::Stop() {}

void TobTsPkmpcController::LoadGainScheduler(
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

absl::Status TobTsPkmpcController::ComputeControlCommand(
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

  double st_pkmpc_start_timestamp = ToUnixDoubleSeconds(Clock::Now());

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

  // Time-control part for computing acceleration cmd
  CalculateCurrError(vehicle_state, trajectory_interface, control_error_debug);

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
    SCOPED_QTRACE("VehicleControlModule::TSPKMPC_AccelerationController");
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
        qevent->AddField("controller_name", "tob_ts_pkmpc_controller")
            .AddField("controller_type", "longitudinal_control");
      });
      const std::string error_msg = absl::StrCat(
          "MPC solver failed, controller_name: tob_ts_pkmpc_controller ",
          " controller_type: longitudinal_control, detailed reason: ",
          status.ToString());
      return absl::InternalError(error_msg);
    } else {
      acceleration_cmd = t_control_output_[0](0);
      for (const auto &res : t_control_output_) {
        controller_debug_proto->mutable_mpc_debug_proto()
            ->add_t_control_mpc_result(res(0));
      }
    }
  }

  TControlVLOG();
  // Predict pose after steer delay
  VehState veh_state = {
      .steer_delay_time =
          cmd->debug().bias_estimation_debug().steer_delay_online(),
      .init_pose = {.x = vehicle_state.x(),
                    .y = vehicle_state.y(),
                    .v = speed_feedback_,
                    .heading = vehicle_state.yaw(),
                    // Todo: Considering Kalman filter.
                    .acc = 0.5 * (acceleration_cmd +
                                  vehicle_state.linear_acceleration())},
      .control_history_state_mgr = &control_history_state_mgr};
  VehPose veh_predicted_pose =
      PredictedPoseByConstAccKinematicModel(veh_state, wheel_base_);
  veh_predicted_pose.ToProto(
      controller_debug_proto->mutable_predicted_veh_pose_proto());

  // Space-control part for computing steering cmd
  ComputeStepLengthFromTControl(is_standstill_, vehicle_state.linear_velocity(),
                                vehicle_state.gear(), controller_debug_proto);

  FindSControlReference(veh_predicted_pose.x, veh_predicted_pose.y,
                        veh_predicted_pose.heading, trajectory_interface, debug,
                        controller_debug_proto);
  SControlUpdateInitialStateAndMatrix(veh_predicted_pose.x,
                                      veh_predicted_pose.y,
                                      veh_predicted_pose.heading, debug);

  if (control_conf_->enable_gain_scheduler() &&
      s_control_gain_scheduler_plf_.has_value()) {
    const double scheduler_gain =
        s_control_gain_scheduler_plf_->Evaluate(std::fabs(speed_feedback_));
    QCHECK_GT(scheduler_gain, 0.0) << "Lateral control loads negative gain "
                                      "schedular at the speed of "
                                   << speed_feedback_;
    s_matrix_q_updated_ = s_matrix_q_ * scheduler_gain;
    s_matrix_r_updated_ = s_matrix_r_;
    s_matrix_n_updated_ = s_matrix_n_ * scheduler_gain;
  } else {
    s_matrix_q_updated_ = s_matrix_q_;
    s_matrix_r_updated_ = s_matrix_r_;
    s_matrix_n_updated_ = s_matrix_n_;
  }

  SControlConstraintsSetup(steering_protection_result);

  double s_kappa_rate;
  {
    SCOPED_QTRACE("VehicleControlModule::TSPKMPC_SteeringController");
    const auto status = math::SolveLinearMPC(
        s_matrix_ad_t_, s_matrix_bd_, s_matrix_cd_t_, s_matrix_q_updated_,
        s_matrix_r_updated_, s_matrix_n_updated_,
        s_control_input_constraint_enable_, s_control_input_lower_,
        s_control_input_upper_, s_control_state_constraint_enable_,
        s_control_state_lower_, s_control_state_upper_, s_initial_state_,
        s_control_state_reference_, s_control_input_reference_,
        &s_control_output_, &s_control_output_unconstrained_);
    if (!status.ok()) {
      QEVENT("zhichao", "mpc_solver_fails", [&](QEvent *qevent) {
        qevent->AddField("controller_name", "tob_ts_pkmpc_controller")
            .AddField("controller_type", "lateral_control");
      });
      const std::string error_msg = absl::StrCat(
          "MPC solver failed, controller_name: tob_ts_pkmpc_controller ",
          " controller_type: lateral_control, detailed reason: ",
          status.ToString());
      return absl::InternalError(error_msg);
    } else {
      s_kappa_rate = s_control_output_[0](0);
      for (const auto &res : s_control_output_) {
        controller_debug_proto->mutable_mpc_debug_proto()
            ->add_s_control_mpc_result(res(0));
      }

      debug->set_is_steering_back_to_center(
          IsSteeringCmdBackToCenter(s_control_output_, s_kappa_cmd_, ts_));
    }
  }

  // TODO(shijun): move the curvature constraint into MPC state constraint.
  const double kappa_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               steering_protection_result.kappa_limit_wrt_speed());
  s_kappa_cmd_ = std::clamp(s_kappa_cmd_ + s_kappa_rate * kControlInterval,
                            -kappa_limit, kappa_limit);

  // Prepare controller debug proto.
  UpdateMPCDebugProto(s_control_state_reference_, s_initial_state_,
                      s_matrix_ad_t_, s_matrix_bd_, s_matrix_cd_t_,
                      s_control_output_, controller_debug_proto);

  SControlVLOG();

  double st_pkmpc_end_timestamp = ToUnixDoubleSeconds(Clock::Now());

  VLOG(1) << "MPC core algorithm: calculation time is: "
          << (st_pkmpc_end_timestamp - st_pkmpc_start_timestamp) * 1000
          << " ms.";

  // TControl cmd post-processing.
  // Estimate vehicle states.
  if (control_conf_->state_estimator_setting().v_enable()) {
    state_estimator_.v += (acceleration_cmd)*0.01;
    state_estimator_.v =
        std::clamp(state_estimator_.v, 0.0,
                   control_conf_->state_estimator_setting().v_threshold());
  }

  LonPostProcessInput lon_postprocess_input = {
      .is_auto_mode = vehicle_state.is_auto_mode(),
      .gear_fb = vehicle_state.gear(),
      .low_speed_freespace = trajectory_interface.GetIsLowSpeedFreespace(),
      .acc_target = acceleration_cmd,
      .acc_feedback = vehicle_state.linear_acceleration(),
      .acc_planner = debug->acceleration_reference(),
      .speed_feedback = speed_feedback_,
      .speed_planner = debug->speed_reference(),
      .pitch_pose = vehicle_state.pitch(),
      .steer_wheel_angle = vehicle_state.front_wheel_steering_angle(),
      .is_standstill = is_standstill_};
  lon_postprocess_manager_->Process(lon_postprocess_input, cmd,
                                    controller_debug_proto);
  is_standstill_ =
      controller_debug_proto->speed_mode_debug_proto().is_standstill();
  cmd->set_aeb_triggered(trajectory_interface.aeb_triggered());

  // SControl cmd post-processing.
  double kappa_cmd_mrac = s_kappa_cmd_;
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
        vehicle_state.is_auto_mode(), s_kappa_cmd_, vehicle_state.kappa(),
        speed_feedback_, kappa_constraint,
        controller_debug_proto->mutable_mrac_debug_proto());
  }

  cmd->set_curvature(kappa_cmd_mrac);

  // When av is in auto_mode and stops standstill and receives stationary
  // trajectory, enter stationary steering state and override curvature.
  const bool has_enter_stationary_steering_state =
      is_standstill_ && vehicle_state.is_auto_mode() &&
      trajectory_interface.GetIsStationaryTrajectory();
  if (has_enter_stationary_steering_state) {
    cmd->mutable_debug()->set_has_enter_stationary_steering(true);
    s_kappa_cmd_ = vehicle_state.kappa();

    VLOG(2) << "Enter stationary steering state.";
    const double chassis_steer_angle =
        vehicle_state.front_wheel_steering_angle();
    const double av_kappa =
        FrontWheelAngle2Kappa(chassis_steer_angle, wheel_base_);
    if (first_hit_stationary_steering_) {
      previous_stationary_steering_cmd_ = kappa_cmd_mrac;
      first_hit_stationary_steering_ = false;
    }
    const double ref_kappa =
        trajectory_interface.GetAllTrajPoints(false)[0].path_point().kappa();
    const double control_curvature =
        CalcStationarySteeringCmd(previous_stationary_steering_cmd_, av_kappa,
                                  ref_kappa, wheel_base_, steer_ratio_);
    cmd->set_curvature(control_curvature);
    previous_stationary_steering_cmd_ = control_curvature;
  } else {
    if (!first_hit_stationary_steering_) {
      first_hit_stationary_steering_ = true;
      s_kappa_cmd_ = previous_stationary_steering_cmd_;
      cmd->set_curvature(previous_stationary_steering_cmd_);
    }
  }
  return absl::OkStatus();
}

void TobTsPkmpcController::Reset(const Chassis &chassis,
                                 const VehicleStateProto &vehicle_state) {
  // Set control kappa cmd memory the same as chassis steering position.
  const double front_wheel_angle = SteeringPct2FrontWheelAngle(
      chassis.steering_percentage(), steer_ratio_, max_steer_angle_);
  const double chassis_kappa =
      FrontWheelAngle2Kappa(front_wheel_angle, wheel_base_);

  s_kappa_cmd_ = chassis_kappa;

  previous_speed_cmd_ = vehicle_state.linear_velocity();
}

// T-control functions
void TobTsPkmpcController::CalculateCurrError(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface,
    ControlError *control_error_debug) {
  // For longitudinal control, do not use the transition trajectory.
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

void TobTsPkmpcController::FindTControlReference(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface, SimpleMPCDebug *debug) {
  // For longitudinal control, do not use the transition trajectory.
  const bool on_transition_traj = false;
  t_control_relative_time_ =
      vehicle_state.timestamp() -
      trajectory_interface.GetPlannerStartTime(on_transition_traj);
  double target_relative_time = t_control_relative_time_;

  // i = 0: collect information for s-control use and calculate tracking error.
  // i = 1 to N: calculate t-control state reference information.
  // i = 0 to (N-1): calculate t-control input reference information.
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

void TobTsPkmpcController::TControlComputeLongitudinalErrors(
    const VehicleStateProto &vehicle_state, SimpleMPCDebug *debug) {
  debug->set_speed_feedback(speed_feedback_);
  debug->set_acceleration_feedback(vehicle_state.linear_acceleration());

  debug->set_speed_error(debug->speed_feedback() - debug->speed_reference());
  debug->set_acceleration_error(vehicle_state.linear_acceleration() -
                                debug->acceleration_reference());
}

void TobTsPkmpcController::TControlUpdateInitialStateAndMatrix(
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

void TobTsPkmpcController::TControlConstraintsSetup() {
  t_control_input_constraint_enable_ << 1.0;
  for (int i = 0; i < kTControlHorizon; ++i) {
    t_control_input_lower_[i](0) = max_deceleration_;
    t_control_input_upper_[i](0) = max_acceleration_;
  }
}

void TobTsPkmpcController::TControlVLOG() {
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
    VLOG(1) << "t_control_state[" << i
            << "] = " << t_control_state[i].transpose() << std::endl;
    VLOG(1) << "t_control_state_reference_[" << i
            << "] = " << t_control_state_reference_[i].transpose() << std::endl;
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

// S-control functions
void TobTsPkmpcController::ComputeStepLengthFromTControl(
    bool is_standstill, double av_speed, Chassis::GearPosition gear_position,
    ControllerDebugProto *debug) {
  t_control_s_.assign(kSControlHorizon, 0.0);
  if (is_standstill) {
    for (double s : t_control_s_) {
      debug->mutable_mpc_debug_proto()->add_step_length(s);
    }
    return;
  }

  double speed_pre = av_speed;
  double speed_next;
  for (int i = 0; i < kSControlHorizon; ++i) {
    speed_next = speed_pre + t_control_output_[0](0) * ts_;
    // Cutoff speed at zero wrt actual gear location.
    if (gear_position == Chassis::GEAR_DRIVE) {
      speed_next = std::max(0.0, speed_next);
    } else if (gear_position == Chassis::GEAR_REVERSE) {
      speed_next = std::min(0.0, speed_next);
    }
    if (i == 0) {
      t_control_s_[i] = 0.5 * (speed_pre + speed_next) * ts_;
    } else {
      t_control_s_[i] =
          t_control_s_[i - 1] + 0.5 * (speed_pre + speed_next) * ts_;
    }
    speed_pre = speed_next;
  }

  for (int i = 0; i < kSControlHorizon; ++i) {
    const double s0 = i == 0 ? 0 : t_control_s_[i - 1];
    const double s1 = t_control_s_[i];
    debug->mutable_mpc_debug_proto()->add_step_length(s1 - s0);
  }
}

void TobTsPkmpcController::FindSControlReference(
    double x, double y, double yaw,
    const TrajectoryInterface &trajectory_interface, SimpleMPCDebug *debug,
    ControllerDebugProto *controller_debug_proto) {
  const Vec2d vs_pos(x, y);

  // Step 1: find the closest trajectory point from trajectory and past points.
  ApolloTrajectoryPointProto closest_trajectory_point;
  constexpr bool kIsOnTransitionTraj = false;
  if (last_matched_point_s_ == 0.0) {
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

  VLOG(1) << "current_s = " << current_s << std::endl
          << "closest_path_point = \n"
          << closest_path_point.DebugString();

  constexpr bool kApplyTransitionTrajOnLatControl = true;
  for (int i = 0; i < kSControlHorizon; ++i) {
    const double t_control_s = current_s + t_control_s_[i];
    VLOG(1) << "t_control_s: " << t_control_s;
    const auto reference_path_point =
        trajectory_interface
            .QueryTrajectoryPointBasedOnPathS(kApplyTransitionTrajOnLatControl,
                                              t_control_s)
            .path_point();
    s_control_state_reference_[i](0) = reference_path_point.x();
    s_control_state_reference_[i](1) = reference_path_point.y();
    s_control_state_reference_[i](2) =
        NormalizeAngle(reference_path_point.theta() - yaw);
    s_control_state_reference_[i](kSControlStateNum - 1) =
        reference_path_point.kappa();
  }
  const double reference_kappa =
      s_control_state_reference_[0](kSControlStateNum - 1);
  debug->set_kappa_feedforward(reference_kappa);
}

void TobTsPkmpcController::SControlUpdateInitialStateAndMatrix(
    double x, double y, double yaw, SimpleMPCDebug *debug) {
  s_initial_state_(0) = x;
  s_initial_state_(1) = y;
  s_initial_state_(2) = 0;

  s_initial_state_(kSControlStateNum - 1) = s_kappa_cmd_;

  // TODO(shijun): replace speed_feedback_ with predict speed after checking in
  // lat mpc delay handing.
  const auto speed = CalcSControlHorizonSpeedSequence(
      speed_feedback_, t_control_output_[0](0), kSControlHorizon);

  for (int i = 0; i < kSControlHorizon; ++i) {
    s_matrix_ad_(0, 0) = 1.0;
    s_matrix_ad_(1, 1) = 1.0;
    s_matrix_ad_(2, 2) = 1.0;
    s_matrix_ad_(0, 2) = -speed[i] * std::sin(yaw) * ts_;
    s_matrix_ad_(1, 2) = speed[i] * std::cos(yaw) * ts_;
    s_matrix_ad_(2, 3) = speed[i] * ts_;

    s_matrix_ad_(kSControlStateNum - 1, kSControlStateNum - 1) = 1;
    s_matrix_ad_t_[i] = s_matrix_ad_;

    // state offset
    s_matrix_cd_(0) = speed[i] * std::cos(yaw) * ts_;
    s_matrix_cd_(1) = speed[i] * std::sin(yaw) * ts_;
    s_matrix_cd_t_[i] = s_matrix_cd_;
  }

  s_matrix_bd_(kSControlStateNum - 1) = ts_;
}

void TobTsPkmpcController::SControlConstraintsSetup(
    const SteeringProtectionResult &steering_protection_result) {
  s_control_input_constraint_enable_ << 1.0;
  for (int i = 0; i < kSControlHorizon; ++i) {
    s_control_input_lower_[i](0) =
        steering_protection_result.kappa_rate_lower();
    s_control_input_upper_[i](0) =
        steering_protection_result.kappa_rate_upper();
  }
}

void TobTsPkmpcController::SControlVLOG() {
  VLOG(1) << "-------------------------------------------------" << std::endl;
  VLOG(1) << "s_Ad = \n" << s_matrix_ad_ << std::endl;
  VLOG(1) << "s_Bd = \n" << s_matrix_bd_ << std::endl;
  VLOG(1) << "s_Cd = \n" << s_matrix_cd_ << std::endl;
  VLOG(1) << "s_Q = \n" << s_matrix_q_updated_ << std::endl;
  VLOG(1) << "s_R = \n" << s_matrix_r_updated_ << std::endl;
  VLOG(1) << "s_N = \n" << s_matrix_n_updated_ << std::endl;
  VLOG(1) << "s_control_input_constraint_enable_ = \n"
          << s_control_input_constraint_enable_ << std::endl;
  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_input_lower_: "
            << s_control_input_lower_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_input_upper_: "
            << s_control_input_upper_[i].transpose() << std::endl;
  }
  VLOG(1) << "s_control_state_constraint_enable_ = \n"
          << s_control_state_constraint_enable_ << std::endl;
  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_state_lower_: "
            << s_control_state_lower_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_state_upper_: "
            << s_control_state_upper_[i].transpose() << std::endl;
  }
  VLOG(1) << "s_initial_state_          : " << s_initial_state_.transpose()
          << std::endl;

  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_state_reference_: "
            << s_control_state_reference_[i].transpose() << std::endl;
  }
  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_input_reference_: "
            << s_control_input_reference_[i].transpose() << std::endl;
  }

  for (int i = 0; i < kSControlHorizon; ++i) {
    VLOG(1) << "s_control_output: " << s_control_output_[i](0) << std::endl;
  }
  VLOG(1) << std::endl << std::endl << std::endl;
}

void TobTsPkmpcController::UpdateMPCDebugProto(
    const std::vector<Eigen::VectorXd> &s_control_state_reference,
    const Eigen::VectorXd &s_initial_state,
    const std::vector<Eigen::MatrixXd> &s_matrix_ad_t,
    const Eigen::MatrixXd &s_matrix_bd,
    const std::vector<Eigen::MatrixXd> &s_matrix_cd_t,
    const std::vector<Eigen::VectorXd> &s_control_output,
    ControllerDebugProto *controller_debug_proto) const {
  controller_debug_proto->set_active_controller("tob_ts_pk_mpc");
  // Note: this is the av state rather than really predicted point. Just for
  // visualization.
  Vec2dProto *predicted_point =
      controller_debug_proto->mutable_mpc_debug_proto()
          ->add_mpc_predicted_traj_point();
  predicted_point->set_x(s_initial_state(0));
  predicted_point->set_y(s_initial_state(1));

  std::vector<Eigen::VectorXd> s_control_state_predict;
  s_control_state_predict.assign(kSControlHorizon,
                                 VecXd::Zero(kSControlStateNum));
  VecXd current_control_state = s_initial_state;
  VecXd next_control_state;
  double xy_cost = 0.0;
  double heading_cost = 0.0;
  double kappa_cost = 0.0;
  double psi_cost = 0.0;
  for (int i = 0; i < kSControlHorizon; ++i) {
    // Based on MPC output, update current_control_state and next_control_state.
    next_control_state = s_matrix_ad_t[i] * current_control_state +
                         s_matrix_bd * s_control_output[i] + s_matrix_cd_t[i];
    s_control_state_predict[i] = next_control_state;
    current_control_state = next_control_state;

    // Collect MPC predicted position.
    Vec2dProto *predicted_point =
        controller_debug_proto->mutable_mpc_debug_proto()
            ->add_mpc_predicted_traj_point();
    predicted_point->set_x(s_control_state_predict[i](0));
    predicted_point->set_y(s_control_state_predict[i](1));

    // Collect MPC reference position.
    Vec2dProto *reference_point =
        controller_debug_proto->mutable_mpc_debug_proto()
            ->add_mpc_reference_traj_point();
    reference_point->set_x(s_control_state_reference[i](0));
    reference_point->set_y(s_control_state_reference[i](1));

    // Collect lateral mpc costs info.
    if (i == kSControlHorizon - 1) {
      xy_cost += Sqr(s_matrix_n_updated_(0, 0)) * s_matrix_q_updated_(0, 0) *
                     Sqr(s_control_state_reference[i](0) -
                         s_control_state_predict[i](0)) +
                 Sqr(s_matrix_n_updated_(1, 1)) * s_matrix_q_updated_(1, 1) *
                     Sqr(s_control_state_reference[i](1) -
                         s_control_state_predict[i](1));
      heading_cost +=
          Sqr(s_matrix_n_updated_(2, 2)) * s_matrix_q_updated_(2, 2) *
          Sqr(s_control_state_reference[i](2) - s_control_state_predict[i](2));
      kappa_cost +=
          Sqr(s_matrix_n_updated_(kSControlStateNum - 1,
                                  kSControlStateNum - 1)) *
          s_matrix_q_updated_(kSControlStateNum - 1, kSControlStateNum - 1) *
          Sqr(s_control_state_reference[i](kSControlStateNum - 1) -
              s_control_state_predict[i](kSControlStateNum - 1));
    } else {
      xy_cost +=
          s_matrix_q_updated_(0, 0) * Sqr(s_control_state_reference[i](0) -
                                          s_control_state_predict[i](0)) +
          s_matrix_q_updated_(1, 1) * Sqr(s_control_state_reference[i](1) -
                                          s_control_state_predict[i](1));
      heading_cost +=
          s_matrix_q_updated_(2, 2) *
          Sqr(s_control_state_reference[i](2) - s_control_state_predict[i](2));
      kappa_cost +=
          s_matrix_q_updated_(kSControlStateNum - 1, kSControlStateNum - 1) *
          Sqr(s_control_state_reference[i](kSControlStateNum - 1) -
              s_control_state_predict[i](kSControlStateNum - 1));
    }
    psi_cost += s_matrix_r_updated_(0, 0) * Sqr(s_control_output[i](0));
  }

  for (const auto &[name, cost] : {std::pair{"state_xy", xy_cost},
                                   {"state_heading", heading_cost},
                                   {"state_kappa", kappa_cost},
                                   {"control_psi", psi_cost}}) {
    auto *costs = controller_debug_proto->mutable_mpc_debug_proto()
                      ->mutable_lateral_mpc_costs()
                      ->add_costs();
    costs->set_name(name);
    costs->set_cost(cost);
  }
}

std::vector<double> TobTsPkmpcController::CalcSControlHorizonSpeedSequence(
    double speed_at_beginning, double t_control_acc,
    int s_control_horizon) const {
  std::vector<double> speed;
  speed.reserve(s_control_horizon);
  speed.push_back(speed_at_beginning);
  for (int i = 0; i < s_control_horizon - 1; ++i) {
    double v = speed.back() + t_control_acc * ts_;
    // Cutoff cross-zero speed.
    if (v * speed_at_beginning < 0.0) {
      v = 0.0;
    }
    speed.push_back(v);
  }
  return speed;
}

}  // namespace qcraft::control
