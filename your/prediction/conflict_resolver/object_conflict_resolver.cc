#include "onboard/prediction/conflict_resolver/object_conflict_resolver.h"

#include "onboard/planner/discretized_path.h"
#include "onboard/prediction/util/trajectory_developer.h"

namespace qcraft::prediction {

// Resolve conflict for one PredictedTrajectory for one specified object.
absl::StatusOr<PredictedTrajectory> ResolveObjectConflict(
    const ObjectConflictResolverInput& input, const std::string& traj_id,
    ThreadPool* thread_pool, ConflictResolverDebugProto* debug_proto) {
  SCOPED_QTRACE("ResolveObjectConlict");
  QCHECK_NOTNULL(input.ego_path);
  QCHECK_NOTNULL(input.obj_con_mgr);
  QCHECK_NOTNULL(input.predicted_trajectory);
  QCHECK_NOTNULL(input.ref_speed);
  QCHECK_NOTNULL(input.params);
  QCHECK_NOTNULL(input.context);
  QCHECK_NOTNULL(input.tl_red_lanes);

  const auto& path = *input.ego_path;
  const auto& obj_con_mgr = *input.obj_con_mgr;
  const auto& params = *input.params;
  const auto& tl_red_lanes = *input.tl_red_lanes;
  const auto& predicted_trajectory = *input.predicted_trajectory;
  const auto& ref_speed = *input.ref_speed;
  auto mutable_predicted_trajectory = predicted_trajectory;  // Make a copy?

  // Step 1. Build st map.
  ASSIGN_OR_RETURN(const auto* ptr_object_proto,
                   obj_con_mgr.GetObjectProtoByTrajId(traj_id));
  const auto& object_proto = *ptr_object_proto;
  std::unique_ptr<ObjectStMap> st_map =
      std::make_unique<ObjectStMap>(obj_con_mgr, object_proto, path, ref_speed,
                                    predicted_trajectory, &params, thread_pool);
  const auto& object_id = obj_con_mgr.GetObjectIdFromTrajectoryId(traj_id);
  const auto stationary_traj_ids = obj_con_mgr.FindStationaryObjects(object_id);
  const auto stationary_map_status =
      st_map->MapStationaryObjects(stationary_traj_ids);
  if (!stationary_map_status.ok()) {
    return absl::InternalError(
        absl::StrFormat("Map stationary objects for path of %s error: %s",
                        traj_id, stationary_map_status.message()));
  }
  if (input.possible_lane_path != nullptr) {
    st_map->MapTLRedLanes(tl_red_lanes, *input.possible_lane_path);
  }
  if (!st_map->HasConflict()) {
    // No need for resolution.
    return absl::NotFoundError("Do not have conflict.");
  }
  VLOG(5) << st_map->DebugString();

  // Step 2. Build cost provider.
  const auto& cost_config = params.GetCostConfig();
  const auto& object_config = params.GetConfigByObjectType(object_proto.type());
  const auto stoplines = st_map->QueryStoplines();
  const auto stationary_objects = st_map->QueryStationaryObjects();
  const auto moving_objects = st_map->QueryMovingObjects();
  SvtCostProviderInput cost_provider_input({
      .stationary_objects = &stationary_objects,
      .stoplines = &stoplines,
      .moving_objects = &moving_objects,
      .ref_speed = &ref_speed,
      .cost_config = &cost_config,
      .stationary_follow_distance = object_config.stationary_follow_distance(),
      .dynamic_follow_distance = object_config.dynamic_follow_distance(),
  });
  std::unique_ptr<SvtCostProvider> cost_provider =
      std::make_unique<SvtCostProvider>(cost_provider_input);

  // Step 3. Solve.
  const auto& general_config = params.GetGeneralConfig();
  ObjectSvtGraphInput svt_graph_input({
      .traj_id = traj_id,
      .object_proto = &object_proto,
      .stoplines = &stoplines,
      .stationary_objects = &stationary_objects,
      .ref_speed = &ref_speed,
      .cost_provider = cost_provider.get(),
      .general_config = &general_config,
      .object_config = &object_config,
  });
  std::unique_ptr<ObjectSvtGraph> svt_graph =
      std::make_unique<ObjectSvtGraph>(svt_graph_input, thread_pool);
  const auto new_speed_vec_or = svt_graph->Search();

  // Step 4. Collect result and add annotations.
  const auto& origin_annotation = predicted_trajectory.annotation();
  if (new_speed_vec_or.ok()) {
    VLOG(3) << absl::StrFormat("Modifying %s trajectory: ", traj_id);
    auto modified_points = CombinePathAndSpeedForPredictedTrajectoryPoints(
        path, *new_speed_vec_or);
    if (!predicted_trajectory.points().empty() && !modified_points.empty()) {
      std::string annotation =
          absl::StrFormat("%s. CR: %f m -> %f m. %s mapped.", origin_annotation,
                          predicted_trajectory.points().back().s(),
                          modified_points.back().s(), st_map->Annotation());
      *mutable_predicted_trajectory.mutable_points() =
          std::move(modified_points);
      mutable_predicted_trajectory.set_annotation(std::move(annotation));
    } else {
      mutable_predicted_trajectory.set_annotation(
          absl::StrCat(origin_annotation, "CR: Modified points empty!"));
    }
    if (FLAGS_prediction_conflict_resolver_visual_on) {
      svt_graph->ToProto(debug_proto);
    }
  } else {
    mutable_predicted_trajectory.set_annotation(absl::StrCat(
        origin_annotation, "CR: Speed vec fail: ", st_map->Annotation()));
    return new_speed_vec_or.status();
  }
  return mutable_predicted_trajectory;
}
}  // namespace qcraft::prediction
