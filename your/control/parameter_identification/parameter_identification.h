#ifndef ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_
#define ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_

#include <optional>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::control {

struct SteerDelay {
  double steer_delay = 0.0;  // Current steer delay time.
  double canbus_steer_delay = 0.0;

  PiecewiseLinearFunction<double> steer_delay_plf;

  void InitConf(const ControllerConf& control_conf) {
    const double steer_straight_th =
        control_conf.steer_deadzone_adaptor_conf().steer_straight_th();
    const double steer_delay_time_straight =
        control_conf.steer_deadzone_adaptor_conf().steer_straight_delay_time();
    const double steer_turn_th =
        control_conf.steer_deadzone_adaptor_conf().steer_turn_th();
    const double steer_delay_time_turn = control_conf.steer_delay_time();

    steer_delay_plf = PiecewiseLinearFunction(
        std::vector<double>{steer_straight_th, steer_turn_th},
        std::vector<double>{steer_delay_time_straight, steer_delay_time_turn});

    canbus_steer_delay =
        control_conf.bias_estimation_conf().steer_status_delay_time();
  }
};

struct ParameterIdentificationInput {
  double steer_cmd;
  double steer_pose;
  double steer_feedback;
  double speed_measurement;
  double lat_error;
  double heading_err;
  bool is_auto;
  const ControlHistoryStateManager* control_history_state_mgr;
};

class ParameterIdentificator {
 public:
  ParameterIdentificator(
      const VehicleDriveParamsProto& vehicle_drive_params,
      const VehicleGeometryParamsProto& vehicle_geometry_params,
      const ControllerConf& control_conf)
      : vehicle_drive_params_(vehicle_drive_params),
        vehicle_geometry_params_(vehicle_geometry_params) {
    InitEstimation(control_conf);
  }

  void Process(const PoseProto& pose, const Chassis& chassis,
               const AutonomyStateProto& autonomy_state,
               ControllerDebugProto* controller_debug_proto);

  int prev_valid_result_num() const { return prev_valid_result_num_; }

  // Steering bias method V2:
  // https://qcraft.feishu.cn/docs/doccn4isLXzD37Xdwc8QsyqY13b
  BiasEstimationDebug EstimateSteerBias(
      const ParameterIdentificationInput& input);

  void UpdateSteerDelay(const Chassis& chassis,
                        const AutonomyStateProto& autonomy_state);

  double GetSteerDelay() const { return steer_delay_.steer_delay; }

  double QueryCurrentSteerDelay() const { return steer_delay_.steer_delay; }

 private:
  void InitEstimation(const ControllerConf& control_conf);

  double CalculateSteerDelay(bool is_auto, double steer_cmd);

  struct SteerBiasIdentificationInputData {
    double front_wheel_angle;  // unit: rad
    double kappa;
  };
  struct SteerBiasStat {
    int stat_index = 0;
    double current_steering_bias = 0.0;
  };
  // Construct input data from pose, chassis and autonomy state, wrt time delay
  // between steering action and av pose
  std::optional<SteerBiasIdentificationInputData>
  AssembleSteerBiasIdentificationInputData(const PoseProto& pose,
                                           const Chassis& chassis);

  double CalculateSteerBias(const SteerBiasIdentificationInputData&
                                steer_bias_identification_input_data);

  void SteeringBiasStatistics(double current_steering_bias,
                              ControllerDebugProto* controller_debug_proto);

 private:
  double prev_steering_bias_ = 0.0;
  int prev_valid_result_num_ = 0;

  double calib_steering_ = 0.0;
  double calib_heading_ = 0.0;

  double ts_ = 0.01;
  double weight_update_steer_ = 0.0;
  double weight_update_heading_ = 0.0;

  BiasEstimationConf bias_estimation_conf_;
  SteerDelay steer_delay_;

  // Time delay between chassis steering action and av pose.
  static constexpr int kSteeringBiasDelayCacheSize = 11;  // time range = 0.1s
  boost::circular_buffer<double> pose_curvature_cache_{
      kSteeringBiasDelayCacheSize};

  VehicleDriveParamsProto vehicle_drive_params_;
  VehicleGeometryParamsProto vehicle_geometry_params_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_PARAMETER_IDENTIFICATION_PARAMETER_IDENTIFICATION_H_
