#include "onboard/emergency_brake/emergency_brake_module.h"

#include <future>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/emergency_brake/util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/safe_unit.h"
#include "onboard/planner/planner_util.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace emergency_brake {

namespace {
absl::Status CheckInput(const EmergencyBrakeInput& input) {
  if (input.autonomy == nullptr) {
    return absl::NotFoundError("No autonomy_state.");
  }
  if (input.fen_detections == nullptr) {
    return absl::NotFoundError("No fen_detections.");
  }
  if (input.pose == nullptr) {
    return absl::NotFoundError("No pose.");
  }
  if (input.chassis == nullptr) {
    return absl::NotFoundError("No chassis.");
  }
  if (!input.chassis->has_steering_percentage() ||
      std::isnan(input.chassis->steering_percentage())) {
    return absl::InternalError(
        "chassis.steering_percentage() has no valid value.");
  }
  return absl::OkStatus();
}

}  // namespace

void EmergencyBrakeModule::OnInit() {
  // Load vehicle geometry params.
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  input_.vehicle_params = run_params.vehicle_params();
  planner::PlannerParams::Instance()->Init(run_params);
  input_.planner_params = planner::PlannerParams::Get();
}

absl::StatusOr<EmergencyBrakeProto> EmergencyBrakeModule::MainLoop(
    const EmergencyBrakeInput& input) const {
  SCOPED_QTRACE("EmergencyBrakeModule::MainLoop");
  EmergencyBrakeProto proto;
  proto.set_triggered(false);

  // ----------------------------------------------------------
  // Skip emergency stop if full stop.
  // ----------------------------------------------------------
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop =
      std::abs(input.pose->vel_body().x()) < kFullStopSpeedThreshold;
  if (full_stop) return proto;

  // ----------------------------------------------------------
  // Compute planner start point from pose.
  // ----------------------------------------------------------
  const auto vehicle_geom = input.vehicle_params.vehicle_geometry_params();
  const ApolloTrajectoryPointProto start_point =
      planner::ComputePlanStartPointAfterReset(
          /*prev_reset_planned_point=*/std::nullopt, *input.pose,
          *input.chassis, input.planner_params.motion_constraint_params(),
          vehicle_geom, input.vehicle_params.vehicle_drive_params());

  // ------------------------------------------------------------
  // Compute the braking trajectory within a given time duration.
  // ------------------------------------------------------------
  constexpr Duration kDt = Seconds(0.1);
  constexpr Duration kMaxDuration = Seconds(2.0);
  constexpr Acceleration kBrakeAccel = MetersPerSquaredSecond(-3.5);
  const auto brake_traj = ComputeConstAccelerationCircularMotion(
      start_point, kDt, kMaxDuration, kBrakeAccel);

  // ------------------------------------------------------------
  // Check if the trajectory has collision with any fen detection.
  // ------------------------------------------------------------
  const auto detected = ComputeFenCollisions(
      brake_traj, input.pose->timestamp(), vehicle_geom,
      input.vehicle_params.vehicle_drive_params(), *input.fen_detections);
  if (detected != std::nullopt) {
    proto.set_triggered(true);
    *proto.mutable_fen_detection() = detected.value();
    const double object_smoothed_x_coord = detected.value().bounding_box().x();
    const double object_smoothed_y_coord = detected.value().bounding_box().y();
    QEVENT_EVERY_N_SECONDS(
        "lidong", "aeb_signal_triggered", 1.0, [&](QEvent* qevent) {
          qevent->AddField("object_smoothed_x_coord", object_smoothed_x_coord)
              .AddField("object_smoothed_y_coord", object_smoothed_y_coord)
              .AddField("object_type", FenDetectionProto::DetectionType_Name(
                                           detected.value().type()));
        });
  }

  return proto;
}

void EmergencyBrakeModule::HandleFenDetections(
    std::shared_ptr<const FenDetectionsProto> fen_detections) {
  input_.fen_detections = std::move(fen_detections);

  if (const auto status = CheckInput(input_); !status.ok()) {
    QLOG_EVERY_N_SEC(ERROR, 3.0)
        << "Input not valid, AEB module skipped: " << status.ToString();
    return;
  }

  const absl::StatusOr<EmergencyBrakeProto> result = MainLoop(input_);
  if (!result.ok()) {
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_PLANNER_EMERGENCY_BRAKE,
                      "EmergencyBrake Error", result.status().ToString());
  } else {
    QLOG_IF_NOT_OK(WARNING, Publish(std::move(result).value()));
  }
}

}  // namespace emergency_brake
}  // namespace qcraft
