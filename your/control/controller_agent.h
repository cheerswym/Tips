#ifndef ONBOARD_CONTROL_CONTROLLER_AGENT_H_
#define ONBOARD_CONTROL_CONTROLLER_AGENT_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/control/controllers/controller_base.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

class ControllerAgent {
 public:
  absl::Status Init(const VehicleGeometryParamsProto &vehicle_geometry_params,
                    const VehicleDriveParamsProto &vehicle_drive_params,
                    const ControllerConf *control_conf);

  absl::Status ComputeControlCommand(
      const VehicleStateProto &vehicle_state,
      const TrajectoryInterface &trajectory_interface,
      const ControlConstraints &control_constraint,
      const ControlHistoryStateManager &control_history_state_mgr,
      ControlCommand *cmd, ControllerDebugProto *controller_debug_proto);

  void Reset(const Chassis &chassis, const VehicleStateProto &vehicle_state);

 private:
  std::vector<std::unique_ptr<ControllerBase>> controller_list_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLER_AGENT_H_
