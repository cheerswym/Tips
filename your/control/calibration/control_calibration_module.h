#ifndef ONBOARD_CONTROL_CALIBRATION_CONTROL_CALIBRATION_MODULE_H_
#define ONBOARD_CONTROL_CALIBRATION_CONTROL_CALIBRATION_MODULE_H_

#include <memory>

#include "absl/status/status.h"
#include "onboard/control/calibration/calibration_tools/calibration_tools.h"
#include "onboard/lite/lite_module.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace control {

struct ControlCalibrationInput {
  std::shared_ptr<const Chassis> chassis;
  std::shared_ptr<const PoseProto> pose;

  VehicleParamApi vehicle_params;
};

class ControlCalibrationModule : public LiteModule {
 public:
  explicit ControlCalibrationModule(LiteClientBase* client)
      : LiteModule(client) {}
  ~ControlCalibrationModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  void OnChassis(std::shared_ptr<const Chassis> chassis);
  void OnPoseProto(std::shared_ptr<const PoseProto> pose);

  void MainLoop();

  absl::Status Process(const ControlCalibrationInput& control_calibration_input,
                       ControlCalibrationTableV2* control_calibration_result);

  ControlCalibrationInput control_calibration_input_;
  ControlCalibrationTableV2 control_calibration_result_;

  std::unique_ptr<calibration::CalibrationTools> calibration_tools_;
};

REGISTER_LITE_MODULE(ControlCalibrationModule);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_CONTROL_CALIBRATION_MODULE_H_
