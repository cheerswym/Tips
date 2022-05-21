#include "onboard/control/mrac_control/mrac_control.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "glog/logging.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {
absl::Status MracControl::Init() {
  if (!config_.mrac_conf.has_matrix_p_1() ||
      !config_.mrac_conf.has_state_weight() ||
      !config_.mrac_conf.has_input_weight() ||
      !config_.mrac_conf.has_cutoff_frequency() ||
      !config_.mrac_conf.has_damping_ratio() ||
      !config_.mrac_conf.has_ke_max_weight() ||
      !config_.mrac_conf.has_ke_min_weight() ||
      !config_.mrac_conf.has_max_error() ||
      !config_.mrac_conf.has_max_delta_kr() ||
      !config_.mrac_conf.has_max_delta_kx() ||
      !config_.mrac_conf.has_max_kr() || !config_.mrac_conf.has_min_kr() ||
      !config_.mrac_conf.has_speed_limit() ||
      !config_.mrac_conf.has_kappa_error_window() ||
      !config_.mrac_conf.has_min_kx() || !config_.mrac_conf.has_max_kx() ||
      !config_.mrac_conf.has_kappa_threshold()) {
    return absl::InvalidArgumentError("[MracControl] Config proto is empty!");
  }
  // Init config matrix.
  InitConfigMatrix();

  // Check system lyapunovs tability.
  if (!IsLyapunovStability()) {
    return absl::InternalError(
        "[MracControl] System is not lyapunov stability!");
  }
  Reset();
  // Mean filter for computer kappa rate
  kappa_error_filter_ = std::make_unique<apollo::common::MeanFilter>(
      std::max(config_.mrac_conf.kappa_error_window(), 1));

  // Consider steer delay.
  num_steer_delay_ =
      std::max(FloorToInt(config_.steer_delay / kControlInterval), 1);
  kappa_target_cache_.resize(num_steer_delay_);

  QLOG_EVERY_N_SEC(INFO, 1.0) << "[MracControl] Init Success!";
  return absl::OkStatus();
}

MracControl::MracControl(const MracConfig& config) { config_ = config; }

MracControl::~MracControl() {}

double MracControl::MracComputer(bool is_automode, double kappa_target,
                                 double av_kappa, double speed,
                                 const KappaConstraint& kappa_constraint,
                                 MracDebugProto* mrac_debug) {
  CHECK_NOTNULL(mrac_debug);

  if (!is_automode) {
    Reset();
    mrac_debug->set_mrac_state(false);
    return kappa_target;
  }

  // Mrac computer
  mrac_debug_.set_mrac_state(true);
  mrac_debug_.set_kappa_input(kappa_target);
  mrac_debug_.set_av_kappa(av_kappa);

  if (is_first_run_) {
    is_first_run_ = false;
    previous_kappa_cmd_ = kappa_target;
    kappa_target_cache_.assign(num_steer_delay_, kappa_target);
  }

  // Consider steer delay.
  kappa_target_cache_.push_back(kappa_target);
  const double kappa_target_delay = kappa_target_cache_.front();
  mrac_debug_.set_kappa_target_delay(kappa_target_delay);

  double kappa_cmd = kappa_target;
  if (std::fabs(speed) < config_.mrac_conf.speed_limit() ||
      std::fabs(kappa_target_delay) < config_.mrac_conf.kappa_threshold()) {
    kappa_cmd = ComputerKappaCmd(kappa_constraint, kappa_target);
  } else {
    ComputerErrorMatrix(kappa_target_delay);
    ComputerGainMatrix(kappa_target);
    kappa_cmd = ComputerKappaCmd(kappa_constraint, kappa_target);
  }
  previous_kappa_cmd_ = kappa_cmd;

  *mrac_debug = mrac_debug_;
  return kappa_cmd;
}

void MracControl::Reset() {
  ResetErrorState();

  kx_gain_matrix_.setZero(1, state_num_);
  kr_gain_matrix_.setOnes(1, input_num_);

  previous_kappa_cmd_ = 0.0;
  kappa_target_cache_.clear();

  is_first_run_ = true;
  QLOG_EVERY_N_SEC(INFO, 5.0) << "[MracControl] Reset!";
}

void MracControl::ResetErrorState() {
  state_cur_matrix_.setZero(state_num_, 1);
  state_ref_matrix_.setZero(state_num_, 1);
  error_matrix_.setZero(state_num_, 1);
}

void MracControl::InitConfigMatrix() {
  const double cutoff_frequency = config_.mrac_conf.cutoff_frequency();
  const double damp_ratio = config_.mrac_conf.damping_ratio();

  // StateMatrix
  A_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);
  A_matrix_(0, 1) = 1.0;
  A_matrix_(1, 0) = -Sqr(cutoff_frequency);
  A_matrix_(1, 1) = -2 * damp_ratio * cutoff_frequency;

  // ReferenceMatrix
  Am_matrix_ = Eigen::MatrixXd::Identity(state_num_, state_num_) +
               A_matrix_ * kControlInterval;

  Bm_matrix_ = Eigen::MatrixXd::Zero(state_num_, input_num_);
  Bm_matrix_(1, 0) = Sqr(cutoff_frequency) * kControlInterval;

  // GainMatrix
  gamma_x_matrix_ = config_.mrac_conf.state_weight() *
                    Eigen::MatrixXd::Identity(input_num_, input_num_);
  gamma_r_matrix_ = config_.mrac_conf.input_weight() *
                    Eigen::MatrixXd::Identity(input_num_, input_num_);

  ke_gain_matrix_ = Eigen::MatrixXd::Zero(input_num_, state_num_);

  state_ref_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);
  state_cur_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);
  error_matrix_ = Eigen::MatrixXd::Zero(state_num_, 1);

  P_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);
  Q_matrix_ = Eigen::MatrixXd::Zero(state_num_, state_num_);

  P_matrix_(0, 0) = config_.mrac_conf.matrix_p_1();
  constexpr double kMatrix_p_2_4 = 1.0;
  P_matrix_(0, 1) = kMatrix_p_2_4;
  P_matrix_(1, 0) = kMatrix_p_2_4;
  P_matrix_(1, 1) = kMatrix_p_2_4;

  Q_matrix_ = -P_matrix_ * A_matrix_ - A_matrix_.transpose() * P_matrix_;
}

bool MracControl::IsLyapunovStability() {
  // check matrix Q is or not symmetric and positive mratix
  Eigen::LLT<Eigen::MatrixXd> llt_matrix_Q(Q_matrix_);
  return (Q_matrix_.isApprox(Q_matrix_.transpose()) &&
          llt_matrix_Q.info() != Eigen::NumericalIssue);
}

void MracControl::ComputerErrorMatrix(double kappa_target_delay) {
  state_ref_matrix_ =
      Am_matrix_ * state_ref_matrix_ + Bm_matrix_ * kappa_target_delay;
  state_cur_matrix_(0, 0) = mrac_debug_.av_kappa();
  state_cur_matrix_(1, 0) = 0.0;

  error_matrix_ = state_ref_matrix_ - state_cur_matrix_;
  double kappa_error = error_matrix_(0, 0);
  kappa_error = kappa_error_filter_->Update(kappa_error);
  kappa_error = std::clamp(kappa_error, -config_.mrac_conf.max_error(),
                           config_.mrac_conf.max_error());
  error_matrix_(0, 0) = kappa_error;
  error_matrix_(1, 0) = 0.0;

  mrac_debug_.set_kappa_error(kappa_error);
}

void MracControl::ComputerGainMatrix(double kappa_target) {
  Eigen::MatrixXd delta_x_matrix(1, state_num_), delta_r_matrix(1, input_num_);

  delta_x_matrix = kControlInterval * gamma_x_matrix_ * Bm_matrix_.transpose() *
                   P_matrix_ * error_matrix_ * state_cur_matrix_.transpose();
  delta_r_matrix = kControlInterval * gamma_r_matrix_ * Bm_matrix_.transpose() *
                   P_matrix_ * error_matrix_ * kappa_target;
  const double max_delta_kx_step =
      config_.mrac_conf.max_delta_kx() * kControlInterval;
  const double max_delta_kr_step =
      config_.mrac_conf.max_delta_kx() * kControlInterval;
  delta_x_matrix(0, 0) =
      std::clamp(delta_x_matrix(0, 0), -max_delta_kx_step, max_delta_kx_step);
  delta_r_matrix(0, 0) =
      std::clamp(delta_r_matrix(0, 0), -max_delta_kr_step, max_delta_kr_step);

  kx_gain_matrix_ += delta_x_matrix;
  kx_gain_matrix_(0, 0) =
      std::clamp(kx_gain_matrix_(0, 0), config_.mrac_conf.min_kx(),
                 config_.mrac_conf.max_kx());

  kr_gain_matrix_ += delta_r_matrix;
  kr_gain_matrix_(0, 0) =
      std::clamp(kr_gain_matrix_(0, 0), config_.mrac_conf.min_kr(),
                 config_.mrac_conf.max_kr());

  ke_gain_matrix_(0, 0) = config_.mrac_conf.ke_max_weight();

  mrac_debug_.set_state_gain(kx_gain_matrix_(0, 0));
  mrac_debug_.set_state_rate_gain(kx_gain_matrix_(0, 1));
  mrac_debug_.set_input_gain(kr_gain_matrix_(0, 0));
  mrac_debug_.set_input_delta_gain(delta_r_matrix(0, 0) / kControlInterval);
}

double MracControl::ComputerKappaCmd(const KappaConstraint& kappa_constraint,
                                     double kappa_target) {
  Eigen::MatrixXd kappa_forward = kr_gain_matrix_ * kappa_target;
  Eigen::MatrixXd kappa_feedback = kx_gain_matrix_ * state_ref_matrix_;

  Eigen::MatrixXd kappa_offset = ke_gain_matrix_ * error_matrix_;

  double kappc_cmd =
      kappa_forward(0, 0) + kappa_feedback(0, 0) + kappa_offset(0, 0);

  kappc_cmd =
      std::clamp(kappc_cmd,
                 previous_kappa_cmd_ +
                     kappa_constraint.kappa_rate_lower * kControlInterval,
                 previous_kappa_cmd_ +
                     kappa_constraint.kappa_rate_upper * kControlInterval);
  kappc_cmd = std::clamp(kappc_cmd, kappa_constraint.kappa_lower,
                         kappa_constraint.kappa_upper);

  mrac_debug_.set_kappa_forward(kappa_forward(0, 0));
  mrac_debug_.set_kappa_feedback(kappa_feedback(0, 0));
  mrac_debug_.set_kappa_offset(kappa_offset(0, 0));
  mrac_debug_.set_kappa_cmd(kappc_cmd);
  return kappc_cmd;
}

}  // namespace control
}  // namespace qcraft
