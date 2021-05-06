#ifndef ONBOARD_PLANNER_PLANNER_MODULE_H_
#define ONBOARD_PLANNER_PLANNER_MODULE_H_

#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_output.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_recorder_module.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/prediction_state.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/semantic_map_modification.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {

struct AsyncLoadRoute {
  std::shared_ptr<const RouteManagerOutputProto> pending_route_manager_output;
  bool is_loading;  // loading SemanticMap
  int64_t request_micros;
  bool is_first_request = true;
};

class PlannerModule : public LiteModule {
 public:
  explicit PlannerModule(LiteClientBase *client);
  ~PlannerModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

  // Preprocess planner input before calling RunMainLoop, also used in snapshot
  // runner
  absl::Status PreprocessInput(PlannerInput *input);

  // The planner main loop that reads planner input and runs in planner
  // onboard/offboard environment.
  // TODO(lidong): Make this a stateless function.
  absl::Status RunMainLoop(const PlannerInput &input, PlannerOutput *output);

  // Publish planner outputs. Lite message publishing function requires the
  // message is a mutable.
  void PublishOutput(const PlannerOutput &output, absl::Status traj_status);

  // Publish planner output asynchronously.
  void PublishOutputAsync(absl::Status traj_status);

  // Getters and setters
  bool IsPlannerSnapshotMode() const { return this->planner_snapshot_mode_; }

  std::string DebugString() const;

  // Getters&Setters
  const PlannerInput &GetPlannerInput() const { return onboard_input_; }

  const PlannerOutput GetPlannerOutput() const {
    absl::MutexLock guard(&output_mutex_);
    return *onboard_output_;
  }

 private:
  void HandlePose(std::shared_ptr<const PoseProto> pose);
  void HandleLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);
  void HandleObjects(std::shared_ptr<const ObjectsProto> objects);
  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy);
  void HandleTrafficLightStates(
      std::shared_ptr<const TrafficLightStatesProto> tl_states);
  void HandleDriverAction(std::shared_ptr<const DriverAction> driver_action);
  void HandleRemoteAssistToCar(
      std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car);
  void HandleRoutingManagerOutputResult(
      std::shared_ptr<const RouteManagerOutputProto> route_manager_output);
  void HandleChassis(std::shared_ptr<const Chassis> chassis);
  void HandlePlannerState(
      std::shared_ptr<const PlannerStateProto> planner_state);
  void HandleSemanticMapPatch(
      std::shared_ptr<const SemanticMapModificationProto> semantic_map_mod);
  void HandleOraclePrediction(
      std::shared_ptr<const ObjectsPredictionProto> oracle_prediction);
  void HandleOracleAVTrajectory(std::shared_ptr<const TrajectoryProto> av_traj);
  void HandleSensorFovs(std::shared_ptr<const SensorFovsProto> sensor_fovs);
  void MaybeInjectTeleopProto();

  absl::Status ProcessTeleopProto(const RemoteAssistToCarProto &teleop_proto);

  absl::Status CheckInput(const PlannerInput &input);

  // The onboard wrapper function.
  void MainLoop();

  bool IsFirstSimulationFrame() const { return simulation_frame_ == 0; }

  // This function checks if a trajectory computed in non-auto mode is ready to
  // enter auto mode. Returns OK if driver can engage.
  absl::Status CheckIfDriverCanEngage(const TrajectoryProto &trajectory);

  // Planner main thread pool.
  std::unique_ptr<ThreadPool> thread_pool_;

  // Contains all the input data to start an iteration of planner.
  // NOTE(lidong): Move all the input to this data structure.
  PlannerInput onboard_input_;

  // Contains the output data of planner module. It should include published
  // proto messages, produced events, issues, canvas etc.
  // NOTE(lidong): Move all the output data to this structure.
  std::unique_ptr<PlannerOutput> onboard_output_ GUARDED_BY(output_mutex_) =
      std::make_unique<PlannerOutput>();
  mutable absl::Mutex output_mutex_;

  // All futue members.
  Future<std::tuple<std::shared_ptr<ObjectsPredictionProto>,
                    std::shared_ptr<prediction::PredictionState>,
                    PredictionDebugProto>>
      prediction_future_;
  Future<void> publish_planner_state_future_;
  Future<std::shared_ptr<PlannerStateProto>> planner_state_to_proto_future_;
  Future<void> publish_output_future_;

  AsyncLoadRoute async_load_route_;
  Future<void> update_pos_future_;
  Future<void> update_route_future_;

  PlannerState planner_state_;
  TeleopState teleop_state_;

  // If input_ has prediction, we don't need to use the following members
  // to compute prediction result. Therefore, when restoring planner state from
  // snapshot, we don't need to restore the states of the following classes.
  PredictionDebugProto prediction_debug_;
  absl::Mutex prediction_mutex_;

  // Prediction V2
  prediction::PredictionContext prediction_context_;
  std::unique_ptr<prediction::ModelPool> prediction_model_pool_;
  prediction::ConflictResolverParams prediction_conflict_resolver_params_;
  std::queue<std::pair<ObjectsProto, LocalizationTransformProto>>
      unprocessed_objects_queue_;

  bool planner_snapshot_mode_ = false;
  std::shared_ptr<const PlannerStateProto> playback_planner_state_;
  std::string semantic_map_dir_;
  int simulation_frame_ = 0;

  absl::Time last_iteration_time_;
};

REGISTER_LITE_MODULE(PlannerModule);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_MODULE_H_
