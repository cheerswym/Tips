#ifndef ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_
#define ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "onboard/control/control_check/wire_control_check.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/controller_agent.h"
#include "onboard/control/controllers/control_tracking_statistics.h"
#include "onboard/control/controllers/steer_calibration.h"
#include "onboard/control/openloop_control/openloop_control.h"
#include "onboard/control/parameter_identification/parameter_identification.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/lite/lite_module.h"
#include "onboard/params/param_manager.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft::control {

// The vehicle control module is communication with the vehicle, including
// sending commands and receiving status reports.
class VehicleControlModule : public LiteModule {
 public:
  explicit VehicleControlModule(LiteClientBase *lite_client);
  ~VehicleControlModule();

  void Proc();
  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  struct LocalView {
    std::shared_ptr<const AutonomyStateProto> autonomy_state;
    std::shared_ptr<const Chassis> chassis;
    std::shared_ptr<const TrajectoryProto> trajectory;
    std::shared_ptr<const PoseProto> pose;
  };

  // Upon receiving pad message
  void OnAutonomyState(
      std::shared_ptr<const AutonomyStateProto> autonomy_state);
  void OnChassis(std::shared_ptr<const Chassis> chassis);
  void OnTrajectory(std::shared_ptr<const TrajectoryProto> trajectory);
  void OnPoseProto(std::shared_ptr<const PoseProto> pose);

  void OnRemoteAssistToCarProto(
      std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car) {
    if (remote_assist_to_car->has_use_manual_control_cmd()) {
      use_manual_cmd_ = remote_assist_to_car->has_use_manual_control_cmd();
    }
  }

  absl::Status ProduceControlCommand(
      ControlCommand *control_command,
      ControllerDebugProto *controller_debug_proto);

  absl::Status UpdateInput(const LocalView &local_view,
                           ControllerDebugProto *controller_debug_proto);

  absl::Status CheckTimestamp(const LocalView &local_view, bool is_input_ready);

  // Record tracking error history and trigger related QEvent.
  void QEventTrackingError(const SimpleMPCDebug &controller_info);

  void LightControl(const TrajectoryProto &trajectory,
                    ControlCommand *control_command) const;
  void DoorControl(const TrajectoryProto &trajectory,
                   ControlCommand *control_command) const;
  void DrivingScenariosControl(const TrajectoryProto &trajectory,
                               ControlCommand *control_command) const;
  // Set gear command, need to ensure car stop standstill.
  absl::Status GearControl(const LocalView &local_view,
                           ControlCommand *control_command) const;
  void ReportControlError(const AutonomyStateProto &autonomy_state,
                          const SimpleMPCDebug &simple_mpc_debug,
                          const ControlError &control_error);

  ControllerAgent controller_agent_;
  ControllerConf control_conf_;
  ControlTrackingStatistics control_tracking_statistics_;
  std::optional<ParameterIdentificator> parameter_identificator_;
  VehicleStateProto vehicle_state_;
  TrajectoryInterface trajectory_interface_;
  std::unique_ptr<OpenloopControl> openloop_control_;
  std::unique_ptr<WireControlChecker> wire_control_checker_;
  PiecewiseLinearFunction<double> steering_gain_wrt_speed_plf_;

  VehicleGeometryParamsProto vehicle_geometry_params_;
  VehicleDriveParamsProto vehicle_drive_params_;
  DynamicParamProto dynamic_param_;
  std::unique_ptr<SteerCalibration> steer_calibration_;

  LocalView local_view_;

  struct TrackingError {
    double lat_error;
    double lon_error;
  };
  std::deque<TrackingError> tracking_error_queue_;
  bool enable_openloop_control_ = false;
  bool planner_trajectory_ready_ = false;
  bool is_input_ready_ = false;
  bool use_manual_cmd_ = false;
  double steer_angle_bias_ = 0.0;
  std::string car_id_;
  int auto_cruise_count_ = 0;
  double steer_delay_time_ = 0.0;
  ControlHistoryStateManager control_history_state_mgr_;
};

REGISTER_LITE_MODULE(VehicleControlModule);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_
