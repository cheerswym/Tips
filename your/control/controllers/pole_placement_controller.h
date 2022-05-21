#ifndef ONBOARD_CONTROL_CONTROLLERS_POLE_PLACEMENT_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_POLE_PLACEMENT_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "onboard/ap_common/configs/proto/vehicle_config.pb.h"
#include "onboard/control/anti_windup_integrator/anti_windup_integrator.h"
#include "onboard/control/calibration/calibration_manager.h"
#include "onboard/control/closed_loop_acc/speed_mode_manager.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/controllers/controller_base.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/mrac_control/mrac_control.h"
#include "onboard/math/filters/digital_filter.h"
#include "onboard/math/filters/digital_filter_coefficients.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/math/interpolation_2d.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft::control {

class PolePlacementController : public ControllerBase {
 public:
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

  double PolePlacementFeedbackSteer(double lateral_error, double heading_error,
                                    double speed, double wheelbase, double ts,
                                    PolePlacementDebug *debug);

  void Reset(const Chassis &chassis,
             const VehicleStateProto &vehicle_state) override;

  void Stop() override;

  std::string_view Name() const override { return "Time-space Pole Placement"; }

 private:
  absl::Status LoadControlConf(
      const ControllerConf *control_conf,
      const VehicleGeometryParamsProto &vehicle_geometry_params,
      const VehicleDriveParamsProto &vehicle_drive_params);

  void InitializeFilters();

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

  bool FullStopState(double accel_planner, double speed_planner,
                     double linear_speed,
                     const FullStopProto &full_stop_condition,
                     Chassis::GearPosition gear_position) const;

  bool UpdateStandStillState(double speed_measurement, double control_period,
                             bool is_full_stop, bool is_standstill,
                             const StandStillProto &standstill_proto);

  void FindPoleControlReference(double x, double y, double heading,
                                const TrajectoryInterface &trajectory_interface,
                                SimpleMPCDebug *debug,
                                ControllerDebugProto *controller_debug_proto);

  void LoadGainScheduler(
      const GainScheduler &gain_scheduler,
      std::optional<PiecewiseLinearFunction<double>> *gain_scheduler_plf);

  std::unique_ptr<CalibrationManager> calibration_manager_;

  // Kappa mrac function
  std::unique_ptr<MracControl> mrac_control_;

  // Timestep size
  double ts_ = 0.0;
  double step_ratio_ = 0.0;  // ts_ / kControlInterval.

  apollo::common::DigitalFilter accel_ref_digital_filter_;
  apollo::common::MeanFilter speed_feedback_mean_filter_;
  apollo::common::MeanFilter sin_slope_mean_filter_;

  std::optional<PiecewiseLinearFunction<double>> t_control_gain_scheduler_plf_;
  std::optional<PiecewiseLinearFunction<double>> s_control_gain_scheduler_plf_;

  double throttle_lowerbound_ = 0.0;
  double brake_lowerbound_ = 0.0;
  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;
  double max_curvature_ = 0.0;
  double min_curvature_ = 0.0;
  double max_steer_angle_rate_ = 0.0;
  WheelDriveMode wheel_drive_mode_;
  double ratio_of_RAC_speed_over_linear_speed_ = 1.0;

  // Control configuration.
  const ControllerConf *control_conf_ = nullptr;

  double t_control_relative_time_;

  ////  For time-control use
  static constexpr int kTControlHorizon = 10;
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

  std::vector<double> t_control_s_;

  FullStopProto full_stop_condition_;
  StandStillProto standstill_proto_;

  double previous_acceleration_cmd_ = 0.0;
  double previous_acc_calibration_ = 0.0;
  double hard_brake_cmd_integral_ = 0.01;
  double full_stop_brake_cmd_integral_ = 0.01;
  double speed_feedback_ = 0.0;
  double previous_steering_cmd_ = 0.0;
  double previous_speed_cmd_ = 0.0;

  struct StateEstimator {
    // TODO(zhichao, shijun): add s for full preset mode.
    double v = 0.0;
  };

  StateEstimator state_estimator_;
  bool ebrake_enable_ = false;

  double last_matched_point_s_ = 0.0;
  double wheel_base_ = 0.0;
  double steer_ratio_ = 0.0;
  double steering_wheel_max_angle_ = 0.0;

  control::ClosedLoopAcc closed_loop_acc_;
  bool is_standstill_ = false;
  int standstill_counter_ = 0;

  std::unique_ptr<AntiWindupIntegrator> lateral_error_integrator_;
  std::unique_ptr<AntiWindupIntegrator> longitudinal_error_integrator_;
  PiecewiseLinearFunction<double> v_pole_plf_;
  double filted_heading_error_ = 0.0;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_POLE_PLACEMENT_CONTROLLER_H_
