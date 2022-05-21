#ifndef ONBOARD_CONTROL_MRAC_CONTROL_MRAC_CONTROL_H_
#define ONBOARD_CONTROL_MRAC_CONTROL_MRAC_CONTROL_H_

#include <memory>

#include "absl/status/status.h"
#include "boost/circular_buffer.hpp"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/math/eigen.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

struct MracConfig {
  MracConfProto mrac_conf;
  double steer_delay = 0.0;
};

struct KappaConstraint {
  double kappa_upper = 0.0;
  double kappa_lower = 0.0;
  double kappa_rate_upper = 0.0;
  double kappa_rate_lower = 0.0;
};
class MracControl {
 public:
  explicit MracControl(const MracConfig& config);
  ~MracControl();
  /**
   * @description: Initialize Mrac control API.
   * @param config {MracConfig include vehicle and mrac control config}
   * @return {Initialize status}
   */
  absl::Status Init();

  /**
   * @description: Mrac computer kappa_cmd.
   * @param kappa_target {kappa_target, from mpc controller}
   * @param av_kappa {av_kappa, from autovehicle location or chassis}
   * @param speed {vehicle current speed}
   * @param steer_wheel_angle {vehicle current swa, to computer swa_rate}
   * @param kappa_cmd {Mrac out: kappa_cmd}
   * @return {status}
   */
  double MracComputer(bool is_automode, double kappa_target, double av_kappa,
                      double speed, const KappaConstraint& kappa_constraint,
                      MracDebugProto* mrac_debug);

 private:
  MracConfig config_;
  MracDebugProto mrac_debug_;

  // Define state and input number
  const int state_num_ = 2;
  const int input_num_ = 1;

  // Define reference model matrix: A Am Bm
  Eigen::MatrixXd A_matrix_;   // continuous state matrix
  Eigen::MatrixXd Am_matrix_;  // discrete state matrix
  Eigen::MatrixXd Bm_matrix_;  // discrete input matrix

  // Define config matrix
  Eigen::MatrixXd gamma_x_matrix_;
  Eigen::MatrixXd gamma_r_matrix_;
  Eigen::MatrixXd P_matrix_;
  Eigen::MatrixXd Q_matrix_;

  // Define gain matrix
  Eigen::MatrixXd kx_gain_matrix_;  // Kx
  Eigen::MatrixXd kr_gain_matrix_;  // Kr
  Eigen::MatrixXd ke_gain_matrix_;  // Ke

  // Define state and error matrix
  Eigen::MatrixXd state_ref_matrix_;  // reference state
  Eigen::MatrixXd state_cur_matrix_;  // current state
  Eigen::MatrixXd error_matrix_;      // error state = reference-current

  int is_first_run_ = true;

  std::unique_ptr<apollo::common::MeanFilter> kappa_error_filter_;
  boost::circular_buffer<double> kappa_target_cache_;
  double previous_kappa_cmd_ = 0.0;
  int num_steer_delay_ = 0;

  /**
   * @description: Reset state and gain integral of mrac control API.
   */
  void Reset();

  void ResetErrorState();

  /**
   * @description: Update gamma_x gamma_r P Q.
   */
  void InitConfigMatrix();

  /**
   * @description: Check mrac control stability.
   */
  bool IsLyapunovStability();

  /**
   * @description: Computer error matrix: error_matrix_.
   */
  void ComputerErrorMatrix(double kappa_target_delay);

  /**
   * @description: Computer kx_gain_ kr_gain_ ke_gain_.
   */
  void ComputerGainMatrix(double kappa_target_delay);

  /**
   * @description: Computer total mrac cmd = feedback + forward + offset.
   */
  double ComputerKappaCmd(const KappaConstraint& kappa_constraint,
                          double kappa_target);
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_MRAC_CONTROL_MRAC_CONTROL_H_
