#include "onboard/planner/scene/scene_understanding.h"

#include <utility>

#include "onboard/planner/planner_flags.h"
#include "onboard/planner/scene/occluded_objects_reasoning.h"
#include "onboard/planner/scene/traffic_flow_reasoning.h"
#include "onboard/utils/status_macros.h"
namespace qcraft::planner {

absl::StatusOr<SceneOutputProto> RunSceneUnderstanding(
    const SceneUnderstandInput &input, ThreadPool *thread_pool) {
  SCOPED_QTRACE("SceneReasoning");
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.prediction);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.lane_paths);
  QCHECK_NOTNULL(input.route_sections);

  const auto &psmm = *input.psmm;
  const auto &prediction = *input.prediction;
  const auto &tl_info_map = *input.tl_info_map;
  const auto &lane_paths = *input.lane_paths;
  const auto &route_sections = *input.route_sections;
  SceneOutputProto scene_output;

  // Run Traffic flow understanding.
  // Output: traffic waiting queue , stall objects.
  const TrafficFlowReasoningInput traffic_flow_input{
      .psmm = &psmm,
      .prediction = &prediction,
      .lane_paths = &lane_paths,
      .tl_info_map = &tl_info_map};

  ASSIGN_OR_RETURN(auto traffic_flow_output,
                   RunTrafficFlowReasoning(traffic_flow_input, thread_pool));

  for (auto &traffic_waiting_queue :
       traffic_flow_output.traffic_waiting_queues) {
    *scene_output.add_traffic_waiting_queue() =
        std::move(traffic_waiting_queue);
  }
  for (auto &object_annotation : traffic_flow_output.object_annotations) {
    *scene_output.add_objects_annotation() = std::move(object_annotation);
  }

  // Run Occluded objects reasoning when sensor fov not empty.
  if (FLAGS_planner_enable_occluded_objects_inference &&
      input.sensor_fovs != nullptr) {
    const auto sensor_fov =
        sensor_fov::BuildLidarViewSensorFov(*input.sensor_fovs);
    const OccludedObjectsReasoningInput occluded_objects_input{
        .psmm = &psmm,
        .sensor_fov = &sensor_fov,
        .route_sections = &route_sections,
        .prediction = &prediction};
    ASSIGN_OR_RETURN(auto occluded_objects_output,
                     RunOccludedObjectsReasoning(occluded_objects_input));
    scene_output.mutable_inferred_objects()->Reserve(
        occluded_objects_output.inferred_objects.size());
    for (auto &inferred_object : occluded_objects_output.inferred_objects) {
      *scene_output.add_inferred_objects() = std::move(inferred_object);
    }
  }

  return scene_output;
}

}  // namespace qcraft::planner
