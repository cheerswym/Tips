#ifndef ONBOARD_CONTROL_CONTROLLERS_TOB_TSPKMPC_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_TOB_TSPKMPC_CONTROLLER_H_

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "onboard/ap_common/configs/proto/vehicle_config.pb.h"
#include "onboard/control/anti_windup_integrator/anti_windup_integrator.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/controllers/controller_base.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"
#include "onboard/control/mrac_control/mrac_control.h"
#include "onboard/math/filters/digital_filter.h"
#include "onboard/math/filters/digital_filter_coefficients.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/math/interpolation_2d.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft::control {

class TobTsPkmpcController : public ControllerBase {
 public:
  TobTsPkmpcController();

  absl::Status Init(
      const ControllerConf *control_conf,
      const VehicleGeometryParamsProto &vehicle_geometry_params,
      const VehicleDriveParamsProto &vehicle_drive_params) override;

  absl::Status ComputeControlCommand(
      const VehicleStateProto &vehicle_state,
      const TrajectoryInterface &trajectory_interface,
      const ControlConstraints &control_constraint,
      const ControlHistoryStateManager &control_history_state_mgr,
      ControlCommand *cmd,
      ControllerDebugProto *controller_debug_proto) override;

  void Reset(const Chassis &chassis,
             const VehicleStateProto &vehicle_state) override;

  void Stop() override;

  std::string_view Name() const override {
    return "Third-order Time-space Purely Kinematic MPC";
  }

 private:
  absl::Status LoadControlConf(
      const ControllerConf *control_conf,
      const VehicleGeometryParamsProto &vehicle_geometry_params,
      const VehicleDriveParamsProto &vehicle_drive_params);

  void LogInitParameters();

  void CalculateCurrError(const VehicleStateProto &vehicle_state,
                          const TrajectoryInterface &trajectory_interface,
                          ControlError *control_error_debug);

  // Functions for t-control use
  void FindTControlReference(const VehicleStateProto &vehicle_state,
                             const TrajectoryInterface &trajectory_interface,
                             SimpleMPCDebug *debug);

  void TControlComputeLongitudinalErrors(const VehicleStateProto &vehicle_state,
                                         SimpleMPCDebug *debug);

  void TControlUpdateInitialStateAndMatrix(SimpleMPCDebug *debug);

  void TControlConstraintsSetup();

  void TControlVLOG();

  // Function for s-control use
  void ComputeStepLengthFromTControl(bool is_standstill, double av_speed,
                                     Chassis::GearPosition gear_position,
                                     ControllerDebugProto *debug);

  void FindSControlReference(double x, double y, double yaw,
                             const TrajectoryInterface &trajectory_interface,
                             SimpleMPCDebug *debug,
                             ControllerDebugProto *controller_debug_proto);

  void SControlUpdateInitialStateAndMatrix(double x, double y, double yaw,
                                           SimpleMPCDebug *debug);

  void SControlConstraintsSetup(
      const SteeringProtectionResult &steering_protection_result);

  void SControlVLOG();

  void UpdateMPCDebugProto(
      const std::vector<Eigen::VectorXd> &s_control_state_reference,
      const Eigen::VectorXd &s_initial_state,
      const std::vector<Eigen::MatrixXd> &s_matrix_ad_t,
      const Eigen::MatrixXd &s_matrix_bd,
      const std::vector<Eigen::MatrixXd> &s_matrix_cd_t,
      const std::vector<Eigen::VectorXd> &s_control_output,
      ControllerDebugProto *controller_debug_proto) const;

  void LoadGainScheduler(
      const GainScheduler &gain_scheduler,
      std::optional<PiecewiseLinearFunction<double>> *gain_scheduler_plf);

  std::vector<double> CalcSControlHorizonSpeedSequence(
      double speed_at_beginning, double t_control_acc,
      int s_control_horizon) const;

  std::unique_ptr<LonPostProcess> lon_postprocess_manager_;

  // Kappa mrac function
  std::unique_ptr<MracControl> mrac_control_;

  // Timestep size
  double ts_ = 0.0;
  double step_ratio_ = 0.0;  // ts_ / kControlInterval.
  // the maximum turn of steer
  double max_steer_angle_ = 0.0;  // rad
  double wheel_base_ = 0.0;
  double steer_ratio_ = 0.0;

  std::optional<PiecewiseLinearFunction<double>> t_control_gain_scheduler_plf_;
  std::optional<PiecewiseLinearFunction<double>> s_control_gain_scheduler_plf_;

  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  // Control configuration.
  const ControllerConf *control_conf_ = nullptr;
  const VehicleDriveParamsProto *vehicle_drive_params_ = nullptr;

  double t_control_relative_time_;

  ////  For time-control use
  static constexpr int kTControlStateNum = 3;
  static constexpr int kTControlInputNum = 1;

  // state matrix
  Eigen::VectorXd t_initial_state_;
  std::vector<Eigen::VectorXd> t_control_output_;
  std::vector<Eigen::VectorXd> t_control_output_unconstrained_;

  // vehicle state matrix
  std::vector<Eigen::MatrixXd> t_matrix_ad_t_;
  Eigen::MatrixXd t_matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd t_matrix_ad_;
  // vehicle control matrix
  Eigen::MatrixXd t_matrix_b_;
  // vehicle control matrix (discrete-time)
  Eigen::MatrixXd t_matrix_bd_;
  // vehicle offset state
  Eigen::MatrixXd t_matrix_c_;
  // vehicle offset state (discrete-time)
  Eigen::MatrixXd t_matrix_cd_;

  // control authority weighting matrix
  Eigen::MatrixXd t_matrix_r_;
  // updated control authority weighting matrix
  Eigen::MatrixXd t_matrix_r_updated_;
  // state weighting matrix
  Eigen::MatrixXd t_matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd t_matrix_q_updated_;
  // Terminal state weighting matrix
  Eigen::MatrixXd t_matrix_n_;
  // updated terminal state weighting matrix
  Eigen::MatrixXd t_matrix_n_updated_;

  // time-control state: [s, v]
  std::vector<Eigen::VectorXd> t_control_state_reference_;
  std::vector<Eigen::VectorXd> t_control_input_reference_;
  Eigen::MatrixXd t_control_input_constraint_enable_;
  std::vector<Eigen::VectorXd> t_control_input_lower_;
  std::vector<Eigen::VectorXd> t_control_input_upper_;
  Eigen::MatrixXd t_control_state_constraint_enable_;
  std::vector<Eigen::VectorXd> t_control_state_lower_;
  std::vector<Eigen::VectorXd> t_control_state_upper_;

  double t_control_initial_s_;
  double previous_speed_cmd_ = 0.0;

  ////  For space-control use
  static constexpr int kSControlStateNum = 4;
  static constexpr int kSControlInputNum = 1;

  std::vector<double> s_dead_controls_;
  double s_kappa_cmd_ = 0.0;

  // step length estimation from t control output
  std::vector<double> t_control_s_;

  Eigen::VectorXd s_initial_state_;
  std::vector<Eigen::VectorXd> s_control_output_;
  std::vector<Eigen::VectorXd> s_control_output_unconstrained_;

  // vehicle state matrix
  Eigen::MatrixXd s_matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd s_matrix_ad_;
  // vehicle state matrix (time-variant)
  std::vector<Eigen::MatrixXd> s_matrix_ad_t_;
  // vehicle control matrix
  Eigen::MatrixXd s_matrix_b_;
  // vehicle control matrix (discrete-time)
  Eigen::MatrixXd s_matrix_bd_;
  // vehicle offset state
  Eigen::MatrixXd s_matrix_c_;
  // vehicle offset state (discrete-time)
  Eigen::MatrixXd s_matrix_cd_;
  // vehicle state matrix (time-variant)
  std::vector<Eigen::MatrixXd> s_matrix_cd_t_;

  // control authority weighting matrix
  Eigen::MatrixXd s_matrix_r_;
  // updated control authority weighting matrix
  Eigen::MatrixXd s_matrix_r_updated_;
  // state weighting matrix
  Eigen::MatrixXd s_matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd s_matrix_q_updated_;
  // Terminal state weighting matrix
  Eigen::MatrixXd s_matrix_n_;
  // updated terminal state weighting matrix
  Eigen::MatrixXd s_matrix_n_updated_;

  std::vector<Eigen::VectorXd> s_control_state_reference_;
  std::vector<Eigen::VectorXd> s_control_input_reference_;
  Eigen::MatrixXd s_control_input_constraint_enable_;
  std::vector<Eigen::VectorXd> s_control_input_lower_;
  std::vector<Eigen::VectorXd> s_control_input_upper_;
  Eigen::MatrixXd s_control_state_constraint_enable_;
  std::vector<Eigen::VectorXd> s_control_state_lower_;
  std::vector<Eigen::VectorXd> s_control_state_upper_;

  double speed_feedback_ = 0.0;
  double previous_stationary_steering_cmd_ = 0.0;
  bool first_hit_stationary_steering_ = true;

  struct StateEstimator {
    // TODO(zhichao, shijun): add s for full preset mode.
    double v = 0.0;
  };

  StateEstimator state_estimator_;

  double last_matched_point_s_ = 0.0;

  bool is_standstill_ = false;

  std::unique_ptr<AntiWindupIntegrator> lateral_error_integrator_;
  std::unique_ptr<AntiWindupIntegrator> longitudinal_error_integrator_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_TOB_TSPKMPC_CONTROLLER_H_
