#include "onboard/prediction/conflict_resolver/conflict_resolver.h"

#include <optional>

#include "onboard/async/thread_pool.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_input.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_util.h"
#include "onboard/prediction/conflict_resolver/constraint_builder.h"
#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"
#include "onboard/prediction/conflict_resolver/object_conflict_resolver.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"

namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionResult> ResolveConflict(
    const std::map<ObjectIDType, ObjectPredictionResult>& predictions,
    const PredictionContext& prediction_context,
    const ConflictResolverParams& params,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool) {
  debug_proto->Clear();
  if (FLAGS_prediction_conflict_resolver_on == false) {
    return predictions;
  }
  SCOPED_QTRACE("ResolveConflict");
  const auto start_time = absl::Now();
  const auto& smm = *prediction_context.semantic_map_manager();
  // Generate as mush common infomation as possible before resolving conflicts
  // for each trajectory.

  // 1. Create Object conflict manager.
  auto ptr_obj_conflict_mgr = std::make_unique<ObjectConflictManager>(
      &prediction_context, &predictions, &params.GetGeneralConfig());
  const auto& obj_con_mgr = *ptr_obj_conflict_mgr;

  // 1.5 Currently focus on build "StoplineProto" based on TL info. Just need to
  // know tl red lane ids.
  const auto& tl_mgr = prediction_context.traffic_light_manager();
  const auto tl_red_lanes = GetRedTrafficLightLanes(
      prediction_context.semantic_map_manager(), &tl_mgr);

  // 2. Create object conflict resolver input in series.
  // TODO(changqing): Check parallel vs series.
  const int object_trajs_size = obj_con_mgr.object_trajs_size();
  std::vector<planner::DiscretizedPath> path_container;
  path_container.reserve(object_trajs_size);
  std::vector<ObjectConflictResolverInput> object_resolver_inputs;
  object_resolver_inputs.reserve(object_trajs_size);

  for (int i = 0, n = object_trajs_size; i < n; ++i) {
    const auto& object_traj = obj_con_mgr.object_traj_ids()[i];
    const auto predicted_traj_or =
        obj_con_mgr.GetPredictedTrajectoryByTrajId(object_traj);

    if (!predicted_traj_or.ok()) {
      continue;  // Should not come here. path_container[i] would be empty.
    }

    // Filter out stationary trajectories.
    if ((*predicted_traj_or)->type() == PT_STATIONARY) {
      continue;
    }

    // Separate predicted trajectory to pure spatial path and reference speed
    // vector.
    const auto& predicted_trajectory = *(*predicted_traj_or);
    auto pair =
        PredictedTrajectoryToPurePathAndSpeedVector(predicted_trajectory);
    path_container.push_back(std::move(pair.first));
    const auto& ref_speed = pair.second;

    // Find lane path here.
    const auto lane_path_or = FindConsecutiveLanesForDiscretizedPath(
        smm, path_container.back(), predicted_trajectory.is_reversed());

    object_resolver_inputs.push_back(ObjectConflictResolverInput({
        .obj_con_mgr = ptr_obj_conflict_mgr.get(),
        .predicted_trajectory = *predicted_traj_or,
        .ego_path = &(path_container.back()),
        .ref_speed = &ref_speed,
        .possible_lane_path = lane_path_or.ok() ? &(*lane_path_or) : nullptr,
        .tl_red_lanes = &tl_red_lanes,
        .params = &params,
        .context = &prediction_context,
    }));
    const auto modified_predicted_trajectory_or = ResolveObjectConflict(
        object_resolver_inputs.back(), object_traj, thread_pool, debug_proto);
    if (modified_predicted_trajectory_or.ok()) {
      const auto& origin_points = predicted_trajectory.points();
      const auto& new_points = modified_predicted_trajectory_or->points();
      if (!origin_points.empty() && !new_points.empty()) {
        QEVENT_EVERY_N_SECONDS(
            "changqing", "prediction_conflict_resolver_modifying_trajectory",
            10.0, [&](QEvent* qevent) {
              qevent->AddField("object_traj_id", object_traj);
              qevent->AddField("origin_length", origin_points.back().s());
              qevent->AddField("modified_length", new_points.back().s());
            });
      }
      // Add to object conflict manager to collect results.
      ptr_obj_conflict_mgr->AddModifiedTrajectory(
          object_traj, (*modified_predicted_trajectory_or));
    }
  }

  // Get all modified predcitions from object conflict manager.
  const auto modified_predictions_or =
      ptr_obj_conflict_mgr->ModifiedPredictions();
  if (modified_predictions_or.ok()) {
    QEVENT_EVERY_N("changqing", "prediction_conflict_resolver_time_spent", 20,
                   [&](QEvent* qevent) {
                     qevent->AddField("time", absl::ToDoubleMilliseconds(
                                                  absl::Now() - start_time));
                   });
    return *modified_predictions_or;  // Results are returned here.
  }
  QLOG(WARNING) << absl::StrFormat(
      "Conflict resolution in prediction fail: %s. Return original "
      "prediction result.",
      modified_predictions_or.status().message());
  return predictions;
}
}  // namespace prediction
}  // namespace qcraft
