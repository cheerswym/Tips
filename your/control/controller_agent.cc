#include "onboard/control/controller_agent.h"

#include <algorithm>
#include <utility>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/pole_placement_controller.h"
#include "onboard/control/controllers/tob_tspkmpc_controller.h"
#include "onboard/control/controllers/ts_pkmpc_controller.h"
#include "onboard/global/clock.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/math/util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace control {
namespace {

// Returns error status when controller's error is larger than given thresholds.
absl::Status ReportControlErrorIssues(const ControlCommand *cmd) {
  QCHECK_NOTNULL(cmd);
  if (!cmd->has_debug()) {
    return absl::FailedPreconditionError("Check control error: No debug field");
  }
  const auto &control_error = cmd->debug().control_error();
#define CHECK_CONTROL(msg)                                        \
  if (!control_error.has_##msg()) {                               \
    return absl::FailedPreconditionError(                         \
        absl::StrCat("Check control error field failed:", #msg)); \
  }
  CHECK_CONTROL(lateral_error);
  CHECK_CONTROL(heading_error);
  CHECK_CONTROL(speed_error);
  CHECK_CONTROL(station_error);

#undef CHECK_CONTROL

  if (control_error.station_error() <
      FLAGS_control_max_station_backward_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_STATION_BACKWARD_ERROR_TOO_LARGE,
        "station distance fallback too much",
        absl::StrFormat("Station error [%f] is less than %f",
                        control_error.station_error(),
                        FLAGS_control_max_station_backward_error));
  }
  if (control_error.station_error() > FLAGS_control_max_station_forward_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_STATION_FORWARD_ERROR_TOO_LARGE,
        "station distance advanced too much",
        absl::StrFormat("Station error [%f] is larger than range %f",
                        control_error.station_error(),
                        FLAGS_control_max_station_forward_error));
  }
  if (std::fabs(control_error.lateral_error()) >
      FLAGS_control_max_error_warning_factor *
          FLAGS_control_max_lateral_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ERROR_TOO_LARGE,
        "lateral error too large warning",
        absl::StrCat("Control lateral error too large warning. Lateral error: ",
                     control_error.lateral_error(), " is not within threshold ",
                     FLAGS_control_max_error_warning_factor *
                         FLAGS_control_max_lateral_error));
  }

  if (std::fabs(control_error.lateral_error()) >
      FLAGS_control_max_lateral_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ERROR_TOO_LARGE,
        "lateral error too large",
        absl::StrFormat("Lateral error [%f]'s abs value is larger than %f.",
                        control_error.lateral_error(),
                        FLAGS_control_max_lateral_error));
  }
  if (std::fabs(control_error.heading_error()) >
      FLAGS_control_max_heading_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_HEADING_ERROR_TOO_LARGE,
        "heading error too large",
        absl::StrFormat("Heading error [%f]'s abs value is larger than %f.",
                        control_error.heading_error(),
                        FLAGS_control_max_heading_error));
  }
  if (std::fabs(control_error.speed_error()) > FLAGS_control_max_speed_error) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_SPEED_ERROR_TOO_LARGE,
        "speed error too large",
        absl::StrFormat("Speed error [%f]'s abs value is larger than %f.",
                        control_error.speed_error(),
                        FLAGS_control_max_speed_error));
  }
  return absl::OkStatus();
}

std::unique_ptr<ControllerBase> CreateController(
    ControllerConf::ControllerType controller_type) {
  switch (controller_type) {
    case ControllerConf::TS_PKMPC_CONTROLLER:
      return std::make_unique<TsPkmpcController>();
    case ControllerConf::TOB_TSPKMPC_CONTROLLER:
      return std::make_unique<TobTsPkmpcController>();
    case ControllerConf::POLE_PLACEMENT_CONTROLLER:
      return std::make_unique<PolePlacementController>();
  }
}

std::vector<std::unique_ptr<ControllerBase>> InitializeConf(
    const ControllerConf &control_conf) {
  std::vector<std::unique_ptr<ControllerBase>> controller_list;
  QCHECK_GT(control_conf.active_controllers_size(), 0);
  for (const auto controller_type : control_conf.active_controllers()) {
    controller_list.push_back(CreateController(
        static_cast<ControllerConf::ControllerType>(controller_type)));
  }
  return controller_list;
}

}  // namespace

absl::Status ControllerAgent::Init(
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const ControllerConf *control_conf) {
  controller_list_ = InitializeConf(*control_conf);

  for (auto &controller : controller_list_) {
    if (controller == nullptr) {
      return absl::Status(absl::StatusCode::kNotFound, "Controller is null.");
    }
    if (!controller
             ->Init(control_conf, vehicle_geometry_params, vehicle_drive_params)
             .ok()) {
      LOG(ERROR) << "Controller <" << controller->Name() << "> init failed!";
      return absl::Status(
          absl::StatusCode::kNotFound,
          absl::StrCat("Failed to init Controller:", controller->Name()));
    }
    LOG(INFO) << "Controller <" << controller->Name() << "> init done!";
  }
  return absl::OkStatus();
}

absl::Status ControllerAgent::ComputeControlCommand(
    const VehicleStateProto &vehicle_state,
    const TrajectoryInterface &trajectory_interface,
    const ControlConstraints &control_constraint,
    const ControlHistoryStateManager &control_history_state_mgr,
    ControlCommand *cmd, ControllerDebugProto *controller_debug_proto) {
  for (const auto &controller : controller_list_) {
    VLOG(1) << "controller:" << controller->Name() << " processing ...";
    if (!controller
             ->ComputeControlCommand(
                 vehicle_state, trajectory_interface, control_constraint,
                 control_history_state_mgr, cmd, controller_debug_proto)
             .ok()) {
      const auto error_msg = absl::StrCat(
          controller->Name(), " fails to generate control command.");
      return absl::InternalError(error_msg);
    }

    // Errors must be filled inside the controller.
    QCHECK_OK(ReportControlErrorIssues(cmd));
  }
  return absl::OkStatus();
}

void ControllerAgent::Reset(const Chassis &chassis,
                            const VehicleStateProto &vehicle_state) {
  for (auto &controller : controller_list_) {
    VLOG(1) << "controller:" << controller->Name() << " reset...";
    controller->Reset(chassis, vehicle_state);
  }
}

}  // namespace control
}  // namespace qcraft
