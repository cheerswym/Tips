#include "onboard/prediction/container/prediction_context.h"

#include <memory>
#include <utility>

#include "onboard/async/thread_pool.h"
#include "onboard/global/clock.h"
#include "onboard/prediction/container/prediction_input.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kTTLSteps = 100;
constexpr double kShortTermDt = 0.05;
constexpr double kLongTermDt = 0.5;
constexpr double kLongTermHistoryLen = 20.0;
}  // namespace

PredictionContext::PredictionContext()
    : prediction_input_(kTTLSteps, kHistoryLen, /*min_dt=*/kShortTermDt),
      obj_long_term_hist_mgr_(kTTLSteps, kLongTermHistoryLen,
                              /*min_dt=*/kLongTermDt) {}

absl::Status PredictionContext::Update(
    const absl::Time prediction_init_time,
    const SemanticMapManager& semantic_map_manager,
    const TrafficLightStatesProto& tl_states, const PoseProto& av_pose,
    const VehicleGeometryParamsProto& veh_geom_params) {
  FUNC_QTRACE();
  prediction_input_.prediction_init_time = prediction_init_time;
  prediction_input_.veh_geom_params = veh_geom_params;
  prediction_input_.semantic_map_manager = &semantic_map_manager;
  prediction_input_.av_context->Update(
      av_pose, *prediction_input_.localization_transform, veh_geom_params);
  traffic_light_manager_.UpdateTlStateMap(semantic_map_manager, tl_states);
  return absl::OkStatus();
}
void PredictionContext::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto> localization_transform) {
  prediction_input_.localization_transform = std::move(localization_transform);
}
void PredictionContext::UpdateObjects(
    const ObjectsProto& objects_proto,
    const LocalizationTransformProto& localization_transform,
    ThreadPool* thread_pool) {
  obj_long_term_hist_mgr_.Update(objects_proto, localization_transform,
                                 thread_pool);
  prediction_input_.objects_history->Update(
      objects_proto, localization_transform, thread_pool);
}

std::vector<const ObjectHistory*> PredictionContext::GetObjectsToPredict(
    const ObjectsProto& objects_proto) {
  FUNC_QTRACE();
  std::vector<const ObjectHistory*> object_to_predict;
  object_to_predict.reserve(objects_proto.objects().size());
  for (const auto& object : objects_proto.objects()) {
    const auto& object_id = object.id();
    auto& objects_history = *prediction_input_.objects_history;
    if (!objects_history.Contains(object_id)) {
      continue;
    }
    auto history = objects_history[object_id].GetHistory();
    if (history.ok()) {
      object_to_predict.push_back(&objects_history[object_id]);
    }
  }
  return object_to_predict;
}

}  // namespace prediction
}  // namespace qcraft
