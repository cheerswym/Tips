#ifndef ONBOARD_PLANNER_SCENE_OCCLUDED_OBJECTS_REASONING_H_
#define ONBOARD_PLANNER_SCENE_OCCLUDED_OBJECTS_REASONING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/perception/sensor_fov/sensor_fov.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
namespace qcraft::planner {

struct OccludedObjectsReasoningInput {
  const PlannerSemanticMapManager *psmm = nullptr;
  // Sensor fov is a necessary condition to reason occluded objects.
  const sensor_fov::SensorFov *sensor_fov = nullptr;
  // Collect crosswalks along route sections.
  const RouteSections *route_sections = nullptr;
  // TODO(jiayu): Remove prediction. Consider sensor_fov only, in the first
  // version. Occluded objects are more likely exist around stationary objects.
  const ObjectsPredictionProto *prediction = nullptr;
};

struct OccludedObjectsReasoningOutput {
  std::vector<InferredObjectProto> inferred_objects;
};

absl::StatusOr<OccludedObjectsReasoningOutput> RunOccludedObjectsReasoning(
    const OccludedObjectsReasoningInput &input);

}  // namespace qcraft::planner
#endif
