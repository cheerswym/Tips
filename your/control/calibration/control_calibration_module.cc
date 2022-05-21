#include "onboard/control/calibration/control_calibration_module.h"

#include <memory>
#include <utility>

#include "glog/logging.h"

DEFINE_string(savedir, "/hosthome/calibration", "save dir");
DEFINE_string(savename, "Q1002", "save name");
DEFINE_string(method, "IDLE", "IDLE, SLIDE, FORCE");
DEFINE_double(throttle_deadzone, 0.0, "throttle_deadzone");
DEFINE_double(brake_deadzone, 0.0, "brake_deadzone");
DEFINE_bool(is_save_sample, true, "is_save_sample");

namespace qcraft {
namespace control {

namespace {

// Control calibration module main loop running frequency.
constexpr absl::Duration kControlCalibrationInterval = absl::Milliseconds(20);

bool CheckInput(const ControlCalibrationInput& input) {
  if (!input.chassis || !input.pose) return false;
  return true;
}

}  // namespace

void ControlCalibrationModule::OnInit() {
  // Load vehicle params.
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  control_calibration_input_.vehicle_params = run_params.vehicle_params();

  calibration_tools_ = std::make_unique<calibration::CalibrationTools>(
      FLAGS_savedir, FLAGS_savename, FLAGS_method, FLAGS_throttle_deadzone,
      FLAGS_brake_deadzone, FLAGS_is_save_sample);
}

void ControlCalibrationModule::OnSubscribeChannels() {
  Subscribe(&ControlCalibrationModule::OnChassis, this);

  if (IsOnboardMode()) {
    Subscribe(&ControlCalibrationModule::OnPoseProto, this, "pose_proto");
  } else {
    Subscribe(&ControlCalibrationModule::OnPoseProto, this, "sensor_pose");
  }
}

void ControlCalibrationModule::OnChassis(
    std::shared_ptr<const Chassis> chassis) {
  control_calibration_input_.chassis = std::move(chassis);
}
void ControlCalibrationModule::OnPoseProto(
    std::shared_ptr<const PoseProto> pose) {
  control_calibration_input_.pose = std::move(pose);
}

ControlCalibrationModule::~ControlCalibrationModule() {
  calibration_tools_->SaveData();
}

void ControlCalibrationModule::OnSetUpTimers() {
  AddTimerOrDie("control_calibration_main_loop",
                std::bind(&ControlCalibrationModule::MainLoop, this),
                kControlCalibrationInterval,
                /*one_shot=*/false);
}

// Calibration main loop.
void ControlCalibrationModule::MainLoop() {
  if (!CheckInput(control_calibration_input_)) return;
  // Calibration main loop.
  // LOG(INFO) << "speed: " << control_calibration_input_.pose->speed();
  // LOG(INFO) << "acc: " << control_calibration_input_.pose->accel_body().x();
  // LOG(INFO) << "pitch: " << control_calibration_input_.pose->pitch();
  // LOG(INFO) << "throttle_percentage: "
  //           << control_calibration_input_.chassis->throttle_percentage();
  // LOG(INFO) << "brake_percentage: "
  //           << control_calibration_input_.chassis->brake_percentage();
  // LOG(INFO) << "gear_location: "
  //           << control_calibration_input_.chassis->gear_location();

  vis::vantage::ChartsDataProto chart_data;
  const auto success = calibration_tools_->Process(
      control_calibration_input_.pose->speed(),
      control_calibration_input_.pose->accel_body().x(),
      control_calibration_input_.pose->pitch(),
      control_calibration_input_.chassis->throttle_percentage(),
      control_calibration_input_.chassis->brake_percentage(),
      control_calibration_input_.chassis->gear_location(),
      chart_data.add_charts());
  QCHECK(success) << "[calibration]: process failed! ";

  QLOG_IF_NOT_OK(WARNING, Publish(chart_data));
}

absl::Status ControlCalibrationModule::Process(
    const ControlCalibrationInput& control_calibration_input,
    ControlCalibrationTableV2* control_calibration_result) {
  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
