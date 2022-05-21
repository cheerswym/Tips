#ifndef ONBOARD_CONTROL_STEERING_PROTECTION_H_
#define ONBOARD_CONTROL_STEERING_PROTECTION_H_

#include <string>

#include "boost/circular_buffer.hpp"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

void WrapSteerConstraintChartData(
    double prev_kappa_cmd, const ControllerConf &controller_conf,
    const ControlCommand &control_command,
    const ControllerDebugProto &controller_debug_proto,
    vis::vantage::ChartsDataProto *chart_data);

// Limit maximum steering kappa rate and steering kappa for the safety.
class SteeringProtection {
 public:
  SteeringProtection(const VehicleGeometryParamsProto *vehicle_geometry_params,
                     const VehicleDriveParamsProto *vehicle_drive_params,
                     const ControllerConf *control_conf)
      : vehicle_geometry_params_(vehicle_geometry_params),
        vehicle_drive_params_(vehicle_drive_params),
        control_conf_(control_conf) {}

  SteeringProtectionResult CalcKappaAndKappaRateLimit(
      const AutonomyStateProto &autonomy_state, double av_speed,
      double front_wheel_angle, double previous_kappa_cmd,
      const ControlCommand &control_command) const;

  void FillDebugMessage(
      const VehicleStateProto &vehicle_state,
      const SteeringProtectionResult &steering_protection_result,
      std::string *debug_msg) const;

  bool IsProtectiveKickout(
      double av_speed,
      const boost::circular_buffer<ControlHistoryStateManager::ControlStateData>
          &control_state_cache,
      const ControllerDebugProto &controller_debug_proto,
      SteeringProtectionResult *steering_protection_result) const;

 private:
  const VehicleGeometryParamsProto *vehicle_geometry_params_;
  const VehicleDriveParamsProto *vehicle_drive_params_;
  const ControllerConf *control_conf_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_STEERING_PROTECTION_H_
