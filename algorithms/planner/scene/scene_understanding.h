#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct SceneUnderstandInput {
  const PlannerSemanticMapManager *psmm = nullptr;
  const ObjectsPredictionProto *prediction = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  // The lane path must in order, from left to right.
  const std::vector<mapping::LanePath> *lane_paths = nullptr;
  const SensorFovsProto *sensor_fovs = nullptr;
  const RouteSections *route_sections = nullptr;
};

absl::StatusOr<SceneOutputProto> RunSceneUnderstanding(
    const SceneUnderstandInput &input, ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCENE_SCENE_UNDERSTAND_H_
