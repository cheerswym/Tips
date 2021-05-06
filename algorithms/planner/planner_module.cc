#include "onboard/planner/planner_module.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <optional>
#include <queue>
#include <sstream>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>
// IWYU pragma: no_include <cxxabi.h>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "common/proto/semantic_map_modifier.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/vis/ark/ark_server/ark_client_man.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/logging.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_io.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/manual_trajectory_util.h"
#include "onboard/planner/plan/plan_task_dispatcher.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_sim_flags.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/standby_state.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/prediction_runner.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/scheduler/scheduler.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"
// Example saved trajectory:
// onboard/control/testdata/test_trajectory_garage_circle.pb.txt
DEFINE_string(
    planner_load_saved_trajectory, "",
    "If not null, publish a saved trajectory once instead of planning online.");

DEFINE_bool(planner_reset_on_stop, true,
            "Reset whenever stopped (both planned and pose speed).");

DEFINE_int32(planner_debug, 0, "How much do you want to debug me?");

DEFINE_string(planner_inject_teleop_proto, "",
              "If non-empty, parsed as an InjectedTeleopProto that injects "
              "teleop protos into planner");

DEFINE_bool(
    planner_wait_for_pose_to_stablize, true,
    "Planner will refuse to plan before pose settles down (stationary).");

DEFINE_bool(route_send_canvas, false, "Send canvas for the route lanes.");

DEFINE_bool(est_planner_canvas, false, "Send canvas in est planner");

DEFINE_bool(planner_play_robobus_alert_audio, false,
            "Planner will request playing audio alerts for certain events in "
            "robobus operation, such as upon reaching stops or making a turn.");

DEFINE_int32(teleop_expire_seconds, 3, "Instruction expiration");

DEFINE_string(planner_semantic_map_modifier, "",
              "Proto file or text that defines the SemanticMapModifierProto.");

DEFINE_bool(
    planner_module_enable_cross_iteration_tf, false,
    "Whether to enable cross-interation smooth coordinate transformation to "
    "compensate for smooth coordinate origin drift.");

// Ref: https://qcraft.atlassian.net/wiki/spaces/WEIT/pages/1224015930/snapshot
DEFINE_bool(restore_from_snapshot, false,
            "Restore planner module state from planner_state_proto of playback "
            "source when engage is triggered in simulation.");

DEFINE_bool(run_prediction_only, false,
            "Only run prediction in open loop simulation.");

DEFINE_bool(run_prediction_feature_extraction_only, false,
            "Only run prediction feature extraction in open loop simulation.");

namespace qcraft::planner {
namespace {

absl::Status ValidateLiteHeader(const LiteHeader &header,
                                const absl::Duration &duration) {
  // backward compatibility
  if (header.timestamp() == 0) return absl::OkStatus();

  const auto delay = Clock::Now() - absl::FromUnixMicros(header.timestamp());
  if (delay > duration) {
    return absl::FailedPreconditionError(
        absl::StrCat("Header timestamp ", header.timestamp() * 1e-6, ", now ",
                     ToUnixDoubleSeconds(Clock::Now()), ", delay ",
                     absl::ToDoubleSeconds(delay), " seconds."));
  }
  return absl::OkStatus();
}

absl::Status ValidateRemoteAssistMessage(
    const RemoteAssistToCarProto &ra_to_car) {
  return ValidateLiteHeader(ra_to_car.header(),
                            absl::Seconds(FLAGS_teleop_expire_seconds));
}

absl::Status ValidateDriverActionMessage(const DriverAction &driver_action) {
  return ValidateLiteHeader(driver_action.header(),
                            absl::Seconds(FLAGS_teleop_expire_seconds));
}

LaneChangeStateProto InitializeLaneChangeState() {
  LaneChangeStateProto proto;
  proto.set_stage(LCS_NONE);
  proto.set_entered_target_lane(false);
  proto.set_lc_left(true);
  return proto;
}

void ConvertPreviousTrajectoryToCurrentSmooth(
    const CoordinateConverter &coordinate_converter,
    const std::vector<PlannerState::PosePoint> &previous_trajectory_global,
    const std::vector<PlannerState::PosePoint> &previous_past_trajectory_global,
    TrajectoryProto *previous_trajectory) {
  FUNC_QTRACE();

  QCHECK_EQ(previous_trajectory->trajectory_point_size(),
            previous_trajectory_global.size());
  QCHECK_EQ(previous_trajectory->past_points_size(),
            previous_past_trajectory_global.size());
  if (!previous_trajectory->trajectory_point().empty()) {
    for (int i = 0; i < previous_trajectory->trajectory_point_size(); ++i) {
      auto *path_point = previous_trajectory->mutable_trajectory_point(i)
                             ->mutable_path_point();
      const Vec2d smooth_point = coordinate_converter.GlobalToSmooth(
          previous_trajectory_global[i].pos);
      const double smooth_yaw =
          coordinate_converter.GlobalYawToSmoothNoNormalize(
              previous_trajectory_global[i].theta);
      path_point->set_x(smooth_point.x());
      path_point->set_y(smooth_point.y());
      path_point->set_theta(smooth_yaw);
    }
  }
  if (!previous_trajectory->past_points().empty()) {
    for (int i = 0; i < previous_trajectory->past_points_size(); ++i) {
      auto *path_point =
          previous_trajectory->mutable_past_points(i)->mutable_path_point();
      const Vec2d smooth_point = coordinate_converter.GlobalToSmooth(
          previous_past_trajectory_global[i].pos);
      const double smooth_yaw =
          coordinate_converter.GlobalYawToSmoothNoNormalize(
              previous_past_trajectory_global[i].theta);
      path_point->set_x(smooth_point.x());
      path_point->set_y(smooth_point.y());
      path_point->set_theta(smooth_yaw);
    }
  }
}

absl::Status LoadSavedTrajectory(
    const std::string &saved_traj_path,
    const CoordinateConverter &coordinate_converter,
    const AutonomyStateProto &autonomy_state,
    TrajectoryProto *prev_trajectory_in_global,
    TrajectoryProto *trajectory_in_smooth) {
  if (prev_trajectory_in_global->trajectory_point_size() == 0 &&
      prev_trajectory_in_global->past_points_size() == 0) {
    if (!file_util::FileToProto(saved_traj_path, prev_trajectory_in_global)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Unable to load trajectory file: ", saved_traj_path));
    }

    CompleteTrajectoryPastPoints(kTrajectoryTimeStep,
                                 prev_trajectory_in_global);
    UpdateTrajectoryPointAccel(prev_trajectory_in_global);
  }

  if (IS_AUTO_DRIVE(autonomy_state.autonomy_state())) {
    ShiftPreviousTrajectory(kPlannerMainLoopInterval,
                            prev_trajectory_in_global);
  }

  *trajectory_in_smooth = *prev_trajectory_in_global;

  ConvertGlobalTrajectoryToSmooth(coordinate_converter, trajectory_in_smooth);

  trajectory_in_smooth->set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(Clock::Now()));

  trajectory_in_smooth->mutable_header()->set_seq_number(2);
  return absl::OkStatus();
}

void FillInputIterationNumToPlannerState(
    const PlannerInput &input, PlannerStateProto *planner_state_proto) {
  FUNC_QTRACE();

  auto *input_seq_num = planner_state_proto->mutable_input_seq_num();
#define SET_SEQ_NUM(NAME)                                           \
  do {                                                              \
    if (input.NAME != nullptr) {                                    \
      input_seq_num->set_##NAME(input.NAME->header().seq_number()); \
    }                                                               \
  } while (false)

  SET_SEQ_NUM(pose);
  SET_SEQ_NUM(av_objects);
  SET_SEQ_NUM(real_objects);
  SET_SEQ_NUM(virtual_objects);
  SET_SEQ_NUM(autonomy_state);
  SET_SEQ_NUM(traffic_light_states);
  SET_SEQ_NUM(driver_action);
  SET_SEQ_NUM(remote_assist_to_car);
  SET_SEQ_NUM(chassis);
  SET_SEQ_NUM(localization_transform);
  SET_SEQ_NUM(prediction);
  SET_SEQ_NUM(route_mgr_output);
#undef SET_SEQ_NUM

  if (input.prev_planner_debug.has_header()) {
    input_seq_num->set_prev_planner_debug(
        input.prev_planner_debug.header().seq_number());
  }
}

// Reoder ObjectsPredictionProto repeat trajs fields with id asc.
void DebugOutPrediction(const PredictionDebugProto &prediction_debug,
                        ObjectsPredictionProto *prediction) {
  if (prediction->objects_size() > 0) {
    std::sort(prediction->mutable_objects()->begin(),
              prediction->mutable_objects()->end(),
              [](const ObjectPredictionProto &a,
                 const ObjectPredictionProto &b) { return a.id() < b.id(); });
    for (auto it = prediction->mutable_objects()->begin();
         it != prediction->mutable_objects()->end(); it++) {
      auto trajectories = it->mutable_trajectories();
      std::sort(
          trajectories->begin(), trajectories->end(),
          [](auto &a, auto &b) { return a.probability() > b.probability(); });
      int index = 0;
      std::for_each(trajectories->begin(), trajectories->end(),
                    [&index](auto &traj) { traj.set_index(index++); });
    }
  }
  static std::atomic<int> i{1};
  int seq = i;
  std::string prediction_file_path = absl::StrCat(
      FLAGS_dump_prediction_output_dir, "/prediction_", seq, ".txt");
  std::ofstream f1(prediction_file_path, std::ios::trunc);
  f1 << prediction->DebugString() << std::endl;
  std::string prediction_debug_file_path = absl::StrCat(
      FLAGS_dump_prediction_output_dir, "/prediction_debug_", seq, ".txt");
  std::ofstream f2(prediction_debug_file_path, std::ios::trunc);
  f2 << prediction_debug.DebugString() << std::endl;
  i++;
}

void ParsePlannerInputToDebugProto(PlannerDebugProto *mutable_debug,
                                   const PlannerInput &planner_input) {
  if (planner_input.pose != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_pose() =
        *planner_input.pose;
  }
  if (planner_input.chassis != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_chassis() =
        *planner_input.chassis;
  }
  if (planner_input.localization_transform != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_loc_transform() =
        *planner_input.localization_transform;
  }
  if (planner_input.real_objects != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_real_objects() =
        *planner_input.real_objects;
  }

  if (planner_input.prediction != nullptr) {
    *mutable_debug->mutable_planner_input()->mutable_prediction() =
        *planner_input.prediction;
  }
}

}  // namespace

absl::Status PlannerModule::CheckIfDriverCanEngage(
    const TrajectoryProto &trajectory) {
  if (trajectory.trajectory_point_size() == 0) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
            QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_EMPTY,
            "Trajectory points are empty.");
    return absl::NotFoundError("Empty trajectory, no trajectory points found.");
  }

  // When vehicle is stopped, allow to engage.
  constexpr double kLowSpeedToAllowEngage = 2.0;  // m/s.
  if (std::abs(trajectory.trajectory_point(0).v()) < kLowSpeedToAllowEngage) {
    return absl::OkStatus();
  }

  // Look forward check if the curvature is too high.
  // Find the point near kLookForwardTime.
  int i = 0;
  while (i < trajectory.trajectory_point_size() &&
         trajectory.trajectory_point(i).relative_time() <
             FLAGS_planner_check_trajectory_engage_condition_duration) {
    ++i;
  }
  if (i >= trajectory.trajectory_point_size()) {
    QISSUEX(
        QIssueSeverity::QIS_ERROR, QIssueType::QIT_PREVENT_ENGAGE,
        QIssueSubType::QIST_PLANNER_TRAJECTORY_TOO_SHORT,
        absl::StrCat("Trajectory's duration is less than ",
                     FLAGS_planner_check_trajectory_engage_condition_duration));
    return absl::NotFoundError("Trajectory duration is too short.");
  }

  // See https://qcraft.atlassian.net/browse/DEVTEST-719
  const PiecewiseLinearFunction<double> speed_to_max_kappa(
      /*x=*/{5.0, 10.0, 30.0},      // Speed.
      /*y=*/{0.015, 0.01, 0.004});  // Max kappa.

  // Do not allow engage if kappa is larger than a threshold.
  for (int j = 0; j < i; ++j) {
    const double abs_speed = std::abs(trajectory.trajectory_point(j).v());
    const double abs_kappa =
        std::abs(trajectory.trajectory_point(j).path_point().kappa());
    const double allowed_kappa = speed_to_max_kappa(abs_speed);
    if (abs_kappa > allowed_kappa) {
      const std::string args_msg = absl::StrFormat(
          "The trajectory point[%d]'s kappa %f (abs value) is larger than "
          "allowed_kappa [%f]. Change it to a larger value if you want to "
          "engage in this situation.",
          j, abs_kappa, allowed_kappa);
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR,
                        QIssueType::QIT_PREVENT_ENGAGE,
                        QIssueSubType::QIST_PLANNER_TRAJECTORY_KAPPA_TOO_LARGE,
                        "Trajectory's kappa is too large", args_msg);
      return absl::FailedPreconditionError(args_msg);
    }
  }

  // TODO(all): Add other cases that we should not allow engage.

  return absl::OkStatus();
}

void PlannerModule::OnInit() {
  DisableAutoOk();
  vantage_client_man::CreateVantageClientMan(param_manager());
  ark_client_man::CreateArkClientMan(param_manager());
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  PlannerParams::Instance()->Init(run_params);
  prediction_conflict_resolver_params_.LoadParams();

  if (FLAGS_planner_thread_pool_size > 0) {
    thread_pool_ = std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  }
  if (FLAGS_planner_map_patch_enable) {
    onboard_input_.semantic_map_manager =
        std::make_unique<SemanticMapManager>(mapping::LoadMapOption{
            .use_local_semantic_map = true,
        });
  } else {
    onboard_input_.semantic_map_manager =
        std::make_unique<SemanticMapManager>();
  }

  prediction_model_pool_ =
      std::make_unique<prediction::ModelPool>(param_manager());

  onboard_input_.traffic_light_states =
      std::make_shared<TrafficLightStatesProto>();

  onboard_input_.vehicle_params =
      PlannerParams::GetRunParams().vehicle_params();

  planner_state_.planner_frame_seq_num = 0;
  planner_state_.lane_change_state = InitializeLaneChangeState();

  planner_state_.last_audio_alert_time = absl::UnixEpoch();
  planner_state_.parking_brake_release_time = absl::InfinitePast();

  planner_state_to_proto_future_ = ScheduleFuture(
      static_cast<ThreadPool *>(nullptr),
      [this]() -> std::shared_ptr<PlannerStateProto> {
        auto planner_state_proto = std::make_shared<PlannerStateProto>();
        planner_state_.ToProto(planner_state_proto.get());
        return planner_state_proto;
      });

  // Initialize the first output state proto from context_state. The reason is
  // that the first output_state proto will be used to initialize the first
  // input.state_proto at the beginning of the MainLoop function.
  {
    absl::MutexLock lock(&output_mutex_);
    planner_state_.ToProto(onboard_output_->mutable_planner_state_proto());
  }

  mapping::SemanticMapModifierProto sm_mod_proto;
  if (!IsPlannerSnapshotMode()) {
    if (FLAGS_planner_map_patch_enable) {
      onboard_input_.planner_semantic_map_manager =
          std::make_unique<PlannerSemanticMapManager>(
              onboard_input_.semantic_map_manager.get());
    } else {
      // Only load prediction map when not in snapshot mode.
      onboard_input_.semantic_map_manager->LoadWholeMap().Build();
      onboard_input_.planner_semantic_map_manager =
          std::make_unique<PlannerSemanticMapManager>(
              onboard_input_.semantic_map_manager.get(),
              CreateSemanticMapModification(
                  *onboard_input_.semantic_map_manager, sm_mod_proto));
    }
  }

  planner_state_to_proto_future_.Wait();

  QLOG(INFO) << " Planner module OnInit() success.";
}

void PlannerModule::OnSubscribeChannels() {
  Subscribe(&PlannerModule::HandlePose, this);
  Subscribe(&PlannerModule::HandleLocalizationTransform, this);
  Subscribe(&PlannerModule::HandleObjects, this);
  Subscribe(&PlannerModule::HandleAutonomyState, this);
  Subscribe(&PlannerModule::HandleTrafficLightStates, this);
  Subscribe(&PlannerModule::HandleDriverAction, this);
  Subscribe(&PlannerModule::HandleRemoteAssistToCar, this);
  Subscribe(&PlannerModule::HandleRoutingManagerOutputResult, this);
  Subscribe(&PlannerModule::HandleChassis, this);
  Subscribe(&PlannerModule::HandleSemanticMapPatch, this);
  Subscribe(&PlannerModule::HandlePlannerState, this,
            "log_planner_state_proto");
  Subscribe(&PlannerModule::HandleOraclePrediction, this,
            "oracle_objects_prediction_proto");
  Subscribe(&PlannerModule::HandleOracleAVTrajectory, this,
            "oracle_trajectory_proto");
  Subscribe(&PlannerModule::HandleSensorFovs, this);
}

void PlannerModule::HandlePlannerState(
    std::shared_ptr<const PlannerStateProto> planner_state) {
  playback_planner_state_ = std::move(planner_state);
}

void PlannerModule::HandleOraclePrediction(
    std::shared_ptr<const ObjectsPredictionProto> oracle_prediction) {
  onboard_input_.log_prediction = std::move(oracle_prediction);
}

void PlannerModule::HandleOracleAVTrajectory(
    std::shared_ptr<const TrajectoryProto> av_traj) {
  onboard_input_.log_av_trajectory = std::move(av_traj);
}

void PlannerModule::HandleSensorFovs(
    std::shared_ptr<const SensorFovsProto> sensor_fovs) {
  onboard_input_.sensor_fovs = std::move(sensor_fovs);
}

void PlannerModule::OnSetUpTimers() {}

PlannerModule::PlannerModule(LiteClientBase *client) : LiteModule(client) {}

void PlannerModule::HandlePose(std::shared_ptr<const PoseProto> pose) {
  FUNC_QTRACE();
  onboard_input_.pose = std::move(pose);

  const auto now = Clock::Now();
  constexpr double kAllowedPoseDelay = 0.02;  // Seconds.
  if (ToUnixDoubleSeconds(now) - onboard_input_.pose->timestamp() >
      kAllowedPoseDelay) {
    // Wait for the next pose in queue.
    return;
  }
  const auto duration_since_last_iteration = now - last_iteration_time_;
  if (duration_since_last_iteration < kPlannerMainLoopInterval) {
    // Not ready for the next iteration.
    return;
  }
  if (absl::ToDoubleSeconds(duration_since_last_iteration) >
      FLAGS_planner_max_allowed_iteration_time) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PLANNER_PROCESS_TIMEOUT, "Planner timeout.");
  }

  MainLoop();
  const auto iteration_time = Clock::Now() - now;
  if (iteration_time > kPlannerMainLoopInterval + absl::Milliseconds(50.0)) {
    QEVENT("lidong", "planner_timeout", [iteration_time](QEvent *event) {
      event->AddField("duration", absl::ToDoubleSeconds(iteration_time));
    });
  }

  last_iteration_time_ = now;
}

void PlannerModule::HandleLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("PlannerModule::HandleLocalizationTransform");
  onboard_input_.localization_transform =
      std::move(localization_transform_proto);
  if (onboard_input_.localization_transform != nullptr) {
    prediction_context_.UpdateLocalizationTransform(
        onboard_input_.localization_transform);
  }
}

void PlannerModule::HandleObjects(std::shared_ptr<const ObjectsProto> objects) {
  FUNC_QTRACE();
  QCHECK(objects && objects->has_scope());
  if (onboard_input_.localization_transform == nullptr ||
      !prediction_context_.HasLocalizationTransform()) {
    return;
  }
  // The prediction context objects update has to be done here to have the raw
  // objects observations available later for prediction.
  absl::MutexLock lock(&prediction_mutex_);
  // Prediction V2 operation
  unprocessed_objects_queue_.emplace(*objects,
                                     *onboard_input_.localization_transform);
  switch (objects->scope()) {
    case ObjectsProto::SCOPE_REAL:
      onboard_input_.real_objects = std::move(objects);
      break;
    case ObjectsProto::SCOPE_VIRTUAL:
      onboard_input_.virtual_objects = std::move(objects);
      break;
    case ObjectsProto::SCOPE_AV:
      onboard_input_.av_objects = std::move(objects);
      break;
  }
}

void PlannerModule::HandleAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy) {
  onboard_input_.autonomy_state = std::move(autonomy);
}

void PlannerModule::HandleTrafficLightStates(
    std::shared_ptr<const TrafficLightStatesProto> traffic_light_states) {
  onboard_input_.traffic_light_states = std::move(traffic_light_states);
}

void PlannerModule::HandleDriverAction(
    std::shared_ptr<const DriverAction> driver_action) {
  onboard_input_.driver_action = std::move(driver_action);
  // TODO(lidong): Move the logic to run main loop function.
  if (const auto status =
          ValidateDriverActionMessage(*onboard_input_.driver_action);
      !status.ok()) {
    QLOG(WARNING) << "Driver action validation failed: " << status;
  } else {
    teleop_state_.pending_driver_actions.push(*onboard_input_.driver_action);
  }
  if (FLAGS_restore_from_snapshot && !IsOnboardMode() &&
      onboard_input_.driver_action->press_engage_button() &&
      playback_planner_state_ != nullptr &&
      playback_planner_state_->planner_frame_seq_num() != 0) {
    QLOG(INFO) << "Restoring planner module state with planner_state_proto of "
                  "seq num: "
               << playback_planner_state_->planner_frame_seq_num();
    onboard_input_.planner_state_proto = playback_planner_state_;
  }
}

void PlannerModule::HandleRemoteAssistToCar(
    std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car) {
  onboard_input_.remote_assist_to_car = std::move(remote_assist_to_car);
}

void PlannerModule::HandleRoutingManagerOutputResult(
    std::shared_ptr<const RouteManagerOutputProto> route_manager_output) {
  if (!FLAGS_planner_map_patch_enable) {
    onboard_input_.route_mgr_output = std::move(route_manager_output);
    return;
  }

  if (route_manager_output->update_id() != planner_state_.route_update_id) {
    if (async_load_route_.pending_route_manager_output == nullptr ||
        async_load_route_.pending_route_manager_output->update_id() !=
            route_manager_output->update_id()) {
      // A  new different route arrived
      async_load_route_.pending_route_manager_output =
          std::move(route_manager_output);
      async_load_route_.request_micros =
          async_load_route_.pending_route_manager_output->header().timestamp();
    }
  } else {
    onboard_input_.route_mgr_output = std::move(route_manager_output);
  }
}

void PlannerModule::HandleChassis(std::shared_ptr<const Chassis> chassis) {
  onboard_input_.chassis = std::move(chassis);
}

void PlannerModule::HandleSemanticMapPatch(
    std::shared_ptr<const SemanticMapModificationProto> semantic_map_mod) {
  onboard_input_.semantic_map_modification = std::move(semantic_map_mod);
}

absl::Status PlannerModule::PreprocessInput(PlannerInput *input) {
  FUNC_QTRACE();
  if (FLAGS_planner_map_patch_enable) {
    // Try to load new semantic map manager patches by route
    const RouteManagerOutputProto *route_output_ptr =
        async_load_route_.pending_route_manager_output.get();
    if (route_output_ptr == nullptr && input->route_mgr_output != nullptr) {
      route_output_ptr = input->route_mgr_output.get();
    }
    if (route_output_ptr != nullptr && route_output_ptr->has_route() &&
        route_output_ptr->route().has_route_section_sequence()) {
      if (async_load_route_.is_first_request &&
          input->localization_transform == nullptr) {
        return absl::FailedPreconditionError(
            "Need localization_transform_proto to do the first "
            "SemanticMapManager::UpdateRoute()");
      }
      auto update_id = route_output_ptr->update_id();
      const RouteSectionSequenceProto &sections_proto =
          route_output_ptr->route().route_section_sequence();
      for (const auto &lane_path :
           route_output_ptr->route().lane_path().lane_paths()) {
        VLOG(3) << "lane ids:" << absl::StrJoin(lane_path.lane_ids(), ",");
      }
      VLOG(3) << "section ids:"
              << absl::StrJoin(sections_proto.section_id(), ",");
      // Semantic map manager load sections, and convert to patch
      std::vector<mapping::SectionId> section_ids(
          sections_proto.section_id_size());
      std::copy(sections_proto.section_id().begin(),
                sections_proto.section_id().end(), section_ids.begin());
      update_route_future_ =
          input->semantic_map_manager->UpdateRoute(update_id, section_ids);
    }
    async_load_route_.is_loading =
        input->semantic_map_manager->IsLoadingMapOnRoute();
    input->semantic_map_manager->ApplyUpdate();
    if (async_load_route_.is_loading) {
      QLOG(INFO) << "Loading map patch for new route, id: ";
      // TODO(xiang): kickout if loading timeout.
    } else if (async_load_route_.pending_route_manager_output != nullptr) {
      input->route_mgr_output =
          std::move(async_load_route_.pending_route_manager_output);
      async_load_route_.pending_route_manager_output.reset();
      if (input->route_mgr_output != nullptr) {
        LOG(INFO) << "Loaded map patch for new route,Done. id:"
                  << input->route_mgr_output->update_id();
      }
    }
  }

  if (input->autonomy_state == nullptr) {
    return absl::NotFoundError("No autonomy status.");
  }

  // NOTE(lidong): Semantic map manager is mutated here. We have to move it
  // outside of the run main loop function.
  // Update localization transform.
  if (input->localization_transform != nullptr) {
    input->semantic_map_manager->UpdateLocalizationTransform(
        *input->localization_transform, thread_pool_.get());
    if (FLAGS_planner_map_patch_enable) {
      update_pos_future_ = input->semantic_map_manager->UpdateSmoothPos(
          Vec2d{input->pose->pos_smooth().x(), input->pose->pos_smooth().y()});
    }
  }

  if (FLAGS_planner_map_patch_enable) {
    RETURN_IF_ERROR(input->semantic_map_manager->ReadyStatus());
  }

  if (input->semantic_map_modification != nullptr &&
      input->semantic_map_modification->has_modifier()) {
    PlannerSemanticMapModification psmm_modifier =
        CreateSemanticMapModification(
            *input->semantic_map_manager,
            input->semantic_map_modification->modifier());
    input->semantic_map_manager->ApplySemanticMapModifier(
        input->semantic_map_modification->modifier());
    input->planner_semantic_map_manager =
        std::make_unique<PlannerSemanticMapManager>(
            input->semantic_map_manager.get(), psmm_modifier);
    input->semantic_map_modification.reset();
  }

  if (input->traffic_light_states == nullptr) {
    input->traffic_light_states = std::make_shared<TrafficLightStatesProto>();
  }

  if (input->prediction == nullptr) {
    if (prediction_future_.IsValid()) {
      SCOPED_QTRACE("GetPrediction");
      auto tuple = prediction_future_.Get();
      if (planner_state_to_proto_future_.IsValid()) {
        WaitForFuture(planner_state_to_proto_future_);
      }
      input->prediction = std::move(std::get<0>(tuple));
      planner_state_.prediction_state = std::move(*std::get<1>(tuple));
      planner_state_.prediction_state.SetObjectsPredicitonTimeSeq(
          input->prediction->header().timestamp(),
          input->prediction->header().seq_number());
      input->prediction_debug = std::move(std::get<2>(tuple));
    }
    // Do prediction feature extraction.
    // TODO(xiangjun): move to offboard snapshot.
    if (input->localization_transform != nullptr && input->pose != nullptr &&
        (input->real_objects != nullptr || input->virtual_objects != nullptr)) {
      if (FLAGS_run_prediction_feature_extraction_only &&
          input->log_prediction != nullptr &&
          input->log_av_trajectory != nullptr) {
        SCOPED_QTRACE("SchedulePredictionFeatureExtraction");
        // 1. Update prediction world view.
        absl::Time prediction_init_time = Clock::Now();
        // Update received objects.
        while (!unprocessed_objects_queue_.empty()) {
          auto top_data = std::move(unprocessed_objects_queue_.front());
          unprocessed_objects_queue_.pop();
          prediction_context_.UpdateObjects(top_data.first, top_data.second,
                                            thread_pool_.get());
        }
        QLOG_IF_NOT_OK(WARNING,
                       prediction_context_.Update(
                           prediction_init_time, *input->semantic_map_manager,
                           *input->traffic_light_states, *input->pose,
                           input->vehicle_params.vehicle_geometry_params()))
            << " Update prediction context failed.";
        const auto objects_proto =
            GetAllObjects(input->real_objects, input->virtual_objects);
        const auto objects_to_predict = prediction_context_.GetObjectsToPredict(
            *objects_proto);  // to be replaced by a
        // 2. Get prediction features.
        auto prediction_features = SchedulePredictionFeatureExtraction(
            prediction_context_, objects_to_predict, *input->log_prediction,
            *input->log_av_trajectory);
        *prediction_debug_.mutable_features() = std::move(prediction_features);
        // 3. Publish prediction feature extraction result.
        QLOG_IF_NOT_OK(WARNING, Publish(prediction_debug_));
      } else {
        // Do normal prediction.
        SCOPED_QTRACE("SchedulePrediction");
        absl::Time prediction_init_time = Clock::Now();
        auto prediction_input = prediction_context_.mutable_prediction_input();
        prediction_input->veh_geom_params =
            input->vehicle_params.vehicle_geometry_params();
        prediction_input->real_objects = input->real_objects;
        prediction_input->virtual_objects = input->virtual_objects;
        prediction_input->pose = input->pose;
        prediction_input->traffic_light_states = input->traffic_light_states;
        prediction_input->semantic_map_manager =
            input->semantic_map_manager.get();
        prediction_input->prediction_init_time = prediction_init_time;
        prediction_input->conflict_resolver_params =
            &prediction_conflict_resolver_params_;
        prediction_future_ = ScheduleFuture(
            thread_pool_.get(),
            [prediction_model_pool = prediction_model_pool_.get(),
             prediction_input, &prediction_debug = prediction_debug_,
             &prediction_context = prediction_context_,
             &prev_prediction_state = planner_state_.prediction_state,
             &mutex = prediction_mutex_, this]() {
              SCOPED_QTRACE("prediction");
              absl::MutexLock lock(&mutex);
              // Update received objects.
              while (!unprocessed_objects_queue_.empty()) {
                auto top_data = std::move(unprocessed_objects_queue_.front());
                unprocessed_objects_queue_.pop();
                prediction_context_.UpdateObjects(
                    top_data.first, top_data.second, thread_pool_.get());
              }
              const auto preprocess_status =
                  prediction::PreprocessPredictionInput(prediction_input);
              if (!preprocess_status.ok()) {
                QLOG(ERROR)
                    << "Preporcess error:" << preprocess_status.message();
              }
              auto prediction_or = prediction::ComputePrediction(
                  *prediction_input, &prediction_context,
                  prediction_model_pool_.get(), thread_pool_.get(),
                  &prediction_debug);
              auto prediction =
                  (prediction_or.ok())
                      ? std::move(prediction_or).value()
                      : std::make_shared<ObjectsPredictionProto>();
              if (FLAGS_dump_prediction_output_dir != "") {
                DebugOutPrediction(prediction_debug, prediction.get());
              }
              PredictionDebugProto prediction_saved_debug(prediction_debug);
              QLOG_IF_NOT_OK(WARNING, Publish(prediction_debug));
              auto prediction_state =
                  std::make_shared<prediction::PredictionState>(
                      prev_prediction_state);
              QLOG_IF_NOT_OK(WARNING,
                             prediction::FillPredictionState(
                                 *prediction_input, prediction_state.get()))
                  << " Update prediction state failed.";
              const auto header_statusor = Publish(*prediction);
              if (header_statusor.ok()) {
                prediction_state->SetObjectsPredicitonTimeSeq(
                    header_statusor->timestamp(),
                    header_statusor->seq_number());
                *(prediction->mutable_header()) = *header_statusor;
              } else {
                QLOG(WARNING) << "Failed to publish prediction";
              }
              return std::tuple{std::move(prediction),
                                std::move(prediction_state),
                                std::move(prediction_saved_debug)};
            });
      }
    }
  }
  planner_state_.planner_semantic_map_modifier =
      input->planner_semantic_map_manager->GetSemanticMapModifier();
  teleop_state_.ClearPendingQueue();
  if (input->remote_assist_to_car != nullptr) {
    if (const auto status = ProcessTeleopProto(*input->remote_assist_to_car);
        !status.ok()) {
      input->remote_assist_to_car.reset();  // Clear if not valid.
      QLOG(WARNING) << status;
    }
  }
  MaybeInjectTeleopProto();
  if (FLAGS_planner_map_patch_enable) {
    if (async_load_route_.is_first_request) {
      async_load_route_.is_first_request = false;
    }
  }
  return absl::OkStatus();
}

void PlannerModule::MainLoop() {
  SCOPED_QTRACE("PlannerModule::MainLoop");

  // Preprocess input before Run_Main_loop
  if (const auto status = PreprocessInput(&onboard_input_); !status.ok()) {
    QLOG(ERROR) << "Failed to process input: " << status;
    return;
  }

  // Prediction only mode
  if (FLAGS_run_prediction_only) {
    onboard_input_.BeforeNextIteration(thread_pool_.get());
    return;
  }

  // Wait until the publishing the output from the previous iteration is done.
  if (publish_output_future_.IsValid()) {
    WaitForFuture(publish_output_future_);
  }

  auto planner_state_proto = planner_state_to_proto_future_.Get();

  absl::Status status;
  {
    if (FLAGS_use_oracle_prediction_only) {
      QCHECK_NOTNULL(onboard_input_.log_prediction);
      onboard_input_.prediction = std::move(onboard_input_.log_prediction);
    }
    absl::MutexLock lock(&output_mutex_);
    planner_state_proto->set_planner_frame_seq_num(
        planner_state_.planner_frame_seq_num);  // Set to current frame.
    // Fill in the sequence number of this iteration to the planner state in
    // last iteration.
    FillInputIterationNumToPlannerState(onboard_input_,
                                        planner_state_proto.get());
    planner_state_.prediction_state.ToProto(
        planner_state_proto->mutable_prediction_state_proto());
    MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(onboard_input_.planner_state_proto);
    onboard_input_.planner_state_proto = planner_state_proto;
    // Swap previous planner's debug as the input of this iteration.
    onboard_input_.prev_planner_debug.Swap(
        onboard_output_->mutable_planner_debug());

    // Clear onboard_output_ asynchronously.
    MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(onboard_output_);
    onboard_output_ = std::make_unique<PlannerOutput>();

    status = RunMainLoop(onboard_input_, onboard_output_.get());
    if (!status.ok()) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_PLANNER_MAIN_LOOP_FAILED,
                        "Planner main loop error: ", status.ToString());
    } else {
      // TODO(lidong): Remove this condition.
      if (IsOnboardMode() &&
          QCHECK_NOTNULL(onboard_input_.autonomy_state)->autonomy_state() ==
              AutonomyStateProto::READY_TO_AUTO_DRIVE &&
          !CheckIfDriverCanEngage(onboard_output_->trajectory()).ok()) {
        QLOG(ERROR) << "Trajectory is not ready to engage.";
      } else {
        // Report ready status if we have planned successfully for once.
        LiteModule::Ok();
      }
    }

    WaitForFuture(
        publish_planner_state_future_);  // last async publish planner state

    // Clear some of the input fields before next iteration.
    onboard_input_.BeforeNextIteration(thread_pool_.get());
    onboard_output_->mutable_planner_debug()->set_planner_frame_seq_num(
        planner_state_.planner_frame_seq_num);
    planner_state_.planner_frame_seq_num++;

    if (FLAGS_planner_debug > 1) {
      ParsePlannerInputToDebugProto(onboard_output_->mutable_planner_debug(),
                                    onboard_input_);
    }
  }

  PublishOutputAsync(status);
}

// NOTE(lidong): Code inside this function shall depends on the provided
// `input` function.
absl::Status PlannerModule::RunMainLoop(const PlannerInput &input,
                                        PlannerOutput *output) {
  SCOPED_QTRACE("PlannerModule::RunMainLoop");
  ScopedMultiTimer timer("planner");

  const absl::Cleanup cleaner = [this, output] {
    SCOPED_QTRACE("cleanup");
    QCHECK(output->has_trajectory());
    if (output->planner_debug().planner_status().status() !=
        PlannerStatusProto::OK) {
      planner_state_.planner_skip_counter++;
    }
    planner_state_to_proto_future_ = ScheduleFuture(
        IsOnboardMode() ? thread_pool_.get() : nullptr,
        [this]() -> std::shared_ptr<PlannerStateProto> {
          auto planner_state_proto = std::make_shared<PlannerStateProto>();
          planner_state_.ToProto(planner_state_proto.get());
          return planner_state_proto;
        });
  };

  // Set the status to OK first.
  output->mutable_planner_debug()->mutable_planner_status()->set_status(
      PlannerStatusProto::OK);

  output->mutable_trajectory()->set_planner_state_seq_num(
      input.planner_state_proto->header().seq_number());

  if (IsPlannerSnapshotMode() ||
      (FLAGS_restore_from_snapshot && IsFirstSimulationFrame())) {
    RecoverPlannerStateFromProto(
        *input.planner_state_proto, *input.planner_semantic_map_manager,
        *input.route_mgr_output, input.vehicle_params.vehicle_geometry_params(),
        &planner_state_);
    simulation_frame_++;
  }

  VLOG(1) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
  VLOG(1) << "Planner iteration " << planner_state_.planner_frame_seq_num;

  const CoordinateConverter coordinate_converter =
      input.localization_transform
          ? CoordinateConverter::FromLocalizationTransform(
                *input.localization_transform)
          : CoordinateConverter::FromLocale();

  if (!FLAGS_planner_load_saved_trajectory.empty()) {
    return LoadSavedTrajectory(FLAGS_planner_load_saved_trajectory,
                               coordinate_converter, *input.autonomy_state,
                               &planner_state_.previous_trajectory,
                               output->mutable_trajectory());
  }

  const auto &vehicle_geom = input.vehicle_params.vehicle_geometry_params();

  if ((input.route_mgr_output == nullptr ||
       !IsValidRouteOutput(*input.route_mgr_output)) &&
      input.pose != nullptr) {
    return EnterStandbyState(*input.pose, vehicle_geom,
                             output->mutable_trajectory(),
                             output->mutable_planner_debug());
  }

  RouteManagerOutput route_mgr_output;
  if (input.route_mgr_output != nullptr) {
    route_mgr_output.FromProto(input.semantic_map_manager.get(),
                               *input.route_mgr_output);
  }

  if (const auto input_status = CheckInput(input); !input_status.ok()) {
    auto *planner_status =
        output->mutable_planner_debug()->mutable_planner_status();
    planner_status->set_status(PlannerStatusProto::INPUT_INCORRECT);
    planner_status->set_message(std::string(input_status.message()));
    return input_status;
  }

  // Collect objects and update object history manager.
  VLOG(2) << "Exporting objects from objects view.";
  const auto objects_proto =
      GetAllObjects(input.real_objects, input.virtual_objects);
  planner_state_.planner_skip_counter = 0;

  if (FLAGS_planner_module_enable_cross_iteration_tf) {
    // Convert previous trajectory coordinates to the current smooth origin.
    ConvertPreviousTrajectoryToCurrentSmooth(
        coordinate_converter, planner_state_.previous_trajectory_global,
        planner_state_.previous_past_trajectory_global,
        &planner_state_.previous_trajectory);
  }

  // TODO(jinyun) Add press_res_button() to reset it.
  teleop_state_.pending_driver_actions = {};
  if (planner_state_.preferred_lane_path.IsEmpty()) {
    teleop_state_.lane_change_direction = std::nullopt;
  }
  if (!teleop_state_.queued_lane_change_requests.empty()) {
    const auto lc_direction =
        teleop_state_.queued_lane_change_requests.front().direction();
    teleop_state_.lane_change_direction =
        lc_direction == LaneChangeRequestProto::CANCEL
            ? std::nullopt
            : std::optional(lc_direction);
  }

  absl::Time current_time = Clock::Now();
  // Publish planner state immediately before running main loop.
  // This is done asynchronously to unblock the main thread.
  // Smelly code with copy Arg.
  publish_planner_state_future_ = ScheduleFuture(
      thread_pool_.get(),
      [this, current_time](PlannerStateProto *proto) {
        SCOPED_QTRACE("publish_planner_state");
        proto->set_current_time(absl::ToUnixMicros(current_time));
        QLOG_IF_NOT_OK(WARNING, Publish(*proto));
      },
      const_cast<PlannerStateProto *>(input.planner_state_proto.get()));

  ReportTeleopStatus(teleop_state_, *input.semantic_map_manager,
                     coordinate_converter, output->mutable_teleop_status());

  if (const auto task_dispatcher_status = RunPlanTaskDispatcher(
          coordinate_converter, *PlannerParams::Instance(), input,
          route_mgr_output, std::as_const(objects_proto).get(), teleop_state_,
          planner_state_.initializer_state.end_info(), current_time,
          &planner_state_, output, thread_pool_.get(), this);
      !task_dispatcher_status.ok()) {
    return task_dispatcher_status;
  }

  // When av starting up, Check whether gear position is ready. If not, output
  // stationary trajectory.
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  if (std::abs(input.pose->vel_body().x()) < kFullStopSpeedThreshold &&
      input.chassis->gear_location() != output->trajectory().gear()) {
    QLOG(WARNING) << absl::StrCat(
        "Gear position is not ready, chassis gear position is:\t",
        input.chassis->gear_location(), ", trajectory gear position is:\t",
        output->trajectory().gear());
    return TranslateToStationaryTrajectory(*input.pose,
                                           output->mutable_trajectory());
  }

  return absl::OkStatus();
}

void PlannerModule::PublishOutput(const PlannerOutput &output,
                                  absl::Status traj_status) {
  SCOPED_QTRACE("PlannerModule::PublishOutput");
  // NOTE(lidong): Make the publish runs in parallel if too slow.
  ScopedMultiTimer timer("planner_publisher");

#define PUBLISH_MSG(msg)                            \
  if (output.has_##msg()) {                         \
    QLOG_IF_NOT_OK(WARNING, Publish(output.msg())); \
  }
  // Only publish trajectory when the trajectory status is OK.
  if (traj_status.ok()) {
    PUBLISH_MSG(trajectory);
  }
  PUBLISH_MSG(planner_debug);
  PUBLISH_MSG(hmi_content);
  PUBLISH_MSG(charts_data);
  PUBLISH_MSG(teleop_status);
  // We do not publish planner states now on purpose. This is because we want
  // to record the message sequence number of external inputs of the next
  // planner iteration in planner state. Therefore, `PlannerState` will be
  // published at the beginning of next iteration.
#undef PUBLISHMSG

  if (!IsOnboardMode()) {
    vantage_client_man::FlushAll();
  }
}

void PlannerModule::PublishOutputAsync(absl::Status traj_status) {
  publish_output_future_ = ScheduleFuture(
      IsOnboardMode() ? thread_pool_.get() : nullptr, [this, traj_status] {
        absl::MutexLock lock(&output_mutex_);
        PublishOutput(*onboard_output_, traj_status);
      });
}

// TODO(lidong): Make this function only depends on PlannerInput.
absl::Status PlannerModule::CheckInput(const PlannerInput &input) {
  SCOPED_QTRACE("PlannerModule::CheckInput");

  const double now = ToUnixDoubleSeconds(Clock::Now());

  // TODO(Fang) report qissue or deny engage when one of the
  // inputs is not available.
  if (planner_state_.planner_last_cycle_timeout) {
    planner_state_.planner_last_cycle_timeout = false;
    return absl::FailedPreconditionError(
        "Skip this cycle due to planner mainloop timing out last cycle.");
  }

  if (input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "pose is not available. Refuse to plan.");
  }

  if (input.traffic_light_states == nullptr) {
    return absl::FailedPreconditionError(
        "Traffic light state is not available. Refuse to plan.");
  }

  if (input.localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "localization transform is not available. Refuse to plan.");
  } else {
    constexpr double kMaxLocalizationTransformStaleness = 1.0;  // s
    // Need to handle legacy logs where there is no timestamp field in
    // localization transform.
    if (input.localization_transform->timestamp() <
        now - kMaxLocalizationTransformStaleness) {
      return absl::FailedPreconditionError(
          absl::StrFormat("localization transform is stale for %.3fs (max "
                          "staleness %.3f). Refuse to plan.",
                          now - input.localization_transform->timestamp(),
                          kMaxLocalizationTransformStaleness));
    }
  }

  if (FLAGS_planner_consider_objects && input.prediction == nullptr) {
    return absl::FailedPreconditionError("No prediction.");
  }

  // Pose quality checks.
  constexpr double kPoseLateralVelocityLimit = 1.0;  // m/s.
  if (std::abs(input.pose->vel_body().y()) > kPoseLateralVelocityLimit) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
            QIssueSubType::QIST_POSITION_POSE_VELOCITY_UNACCEPTABLE,
            "Check pose state: UNACCEPTABLE(lateral velocity)");
    return absl::FailedPreconditionError(
        "pose lateral velocity is above limit. Considering pose "
        "unreliable. Refusing to plan.");
  }

  if (const double time_diff = now - input.pose->timestamp();
      time_diff > FLAGS_planner_max_allowed_iteration_time) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PLANNER_PROTO_POSE_TIMEOUT,
            "Check pose state: OUTDATE(stale pose)");
    // In most cases stale poses are a result of planning iterations
    // being slow, which makes PlannerModule busy running the
    // PlannerModule::MainLoop() timer rather than processing pose
    // subscription callbacks. Aborting planning in such cases will
    // quickly drain the MainLoop timer backlog and allow pose
    // subscription to come through.
    return absl::FailedPreconditionError(
        absl::StrFormat("latest pose is too old: %.3f clock now: %.3f, time "
                        "diff is: %.3f. Very likely this is caused by planner "
                        "iteration timeout.",
                        input.pose->timestamp(), now, time_diff));
  }
  if (input.autonomy_state == nullptr) {
    return absl::FailedPreconditionError("autonomy_state is not available.");
  }
  if (input.real_objects == nullptr && input.virtual_objects == nullptr) {
    return absl::FailedPreconditionError(
        "perception objects message is not available.");
  }

  return absl::OkStatus();
}

absl::Status PlannerModule::ProcessTeleopProto(
    const RemoteAssistToCarProto &teleop_proto) {
  FUNC_QTRACE();
  RETURN_IF_ERROR(ValidateRemoteAssistMessage(teleop_proto));

  teleop_state_.FromProto(teleop_proto);

  if (teleop_proto.has_door_override()) {
    planner_state_.last_door_override_time =
        MicroSecondsToSeconds(teleop_proto.header().timestamp());
  }
  return absl::OkStatus();
}

// TODO(lidong): Change to return Status.
void PlannerModule::MaybeInjectTeleopProto() {
  FUNC_QTRACE();
  InjectedTeleopProto injected_teleop_proto;
  QCHECK(file_util::StringToProto(FLAGS_planner_inject_teleop_proto,
                                  &injected_teleop_proto));
  for (int i = 0; i < injected_teleop_proto.frames_size(); ++i) {
    if (injected_teleop_proto.frames(i) ==
        planner_state_.planner_frame_seq_num) {
      QLOG(INFO) << "Inject Teleop proto at frame "
                 << planner_state_.planner_frame_seq_num << ": "
                 << injected_teleop_proto.DebugString();
      if (const auto status =
              ProcessTeleopProto(injected_teleop_proto.contents(i));
          !status.ok()) {
        QLOG(ERROR) << status;
      }
    }
  }
}

PlannerModule::~PlannerModule() {
  // Print prediction debug information.
  // TODO(lidong): Move to function callback such as LiteModule::NotifyStop().
  if (prediction_debug_.has_timer_max()) {
    const TimerDebugProto &max_duation_timer = prediction_debug_.timer_max();
    if (max_duation_timer.has_whole_timer()) {
      QLOG(INFO) << absl::StrFormat(
          "\n \n ####### Max Total Stat Info #########");
      prediction::PrintMultiTimerReportStat(max_duation_timer.whole_timer());
    }
    if (max_duation_timer.has_engine_timer()) {
      QLOG(INFO) << absl::StrFormat("\n \n ####### Engine Stat Info #########");
      prediction::PrintMultiTimerReportStat(max_duation_timer.engine_timer());
    }
    if (max_duation_timer.has_vehicle_timer()) {
      QLOG(INFO) << absl::StrFormat(
          "\n \n ####### Vehilce Stat info #########");
      prediction::PrintMultiTimerReportStat(max_duation_timer.vehicle_timer());
    }
  }
  // Wait for prediction future to finish.
  if (prediction_future_.IsValid()) {
    prediction_future_.Get();
  }

  if (publish_planner_state_future_.IsValid()) {
    publish_planner_state_future_.Get();
  }
  if (planner_state_to_proto_future_.IsValid()) {
    planner_state_to_proto_future_.Get();
  }
  if (publish_output_future_.IsValid()) {
    publish_output_future_.Get();
  }
}
std::string PlannerModule::DebugString() const {
  std::stringstream ss;
  ss << "input:" << onboard_input_.DebugString() << "\n";
  ss << "teleop_state:" << teleop_state_.DebugString() << "\n";
  ss << "planner_snapshot_mode_:" << planner_snapshot_mode_;
  return ss.str();
}

}  // namespace qcraft::planner
