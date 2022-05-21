#include "onboard/prediction/container/prediction_runner.h"

#include <algorithm>
#include <fstream>
#include <utility>
#include <vector>

#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/prediction/scheduler/scheduler.h"
// Used for checking prediciton state, should be removed later.
namespace qcraft::prediction {

namespace {

[[maybe_unused]] void UpdateObjectHistory(const ObjectsProto &objects_proto,
                                          PredictionInput *input,
                                          ThreadPool *thread_pool) {
  input->objects_history->Update(objects_proto, *input->localization_transform,
                                 thread_pool);
}

[[maybe_unused]] void UpdateAvContext(const PoseProto &av_pose,
                                      PredictionInput *input) {
  input->av_context->Update(av_pose, *input->localization_transform,
                            input->veh_geom_params);
}

}  // namespace

absl::Status CheckInput(const PredictionInput &input) {
  if (input.localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "The localization_transform is nullptr.");
  }
  return absl::OkStatus();
}

absl::Status PreprocessPredictionInput(PredictionInput *input) {
  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<ObjectsPredictionProto>> ComputePrediction(
    const PredictionInput &input, PredictionContext *prediction_context,
    ModelPool *model_pool, ThreadPool *thread_pool,
    PredictionDebugProto *debug) {
  FUNC_QTRACE();
  ScopedMultiTimer prediction_timer("prediction_debug");

  const auto objects_proto =
      planner::GetAllObjects(input.real_objects, input.virtual_objects);
  // Run motion prediction.
  VLOG(2) << "Running prediction.";
  prediction_timer.Mark("prediction start");

  auto prediction = std::make_shared<ObjectsPredictionProto>();
  // If no object is addted to the prediction context cache, do not make
  // prediction.
  if (!prediction_context->HasObjects()) {
    return prediction;
  }
  const auto &vehicle_geom = input.veh_geom_params;
  const auto &pose = *input.pose;
  const auto &traffic_light_states = *input.traffic_light_states;
  const auto &semantic_map_manager = *input.semantic_map_manager;

  // Prediction V2
  // 1. Update prediction world view.
  RETURN_IF_ERROR(prediction_context->Update(
      input.prediction_init_time, semantic_map_manager, traffic_light_states,
      pose, vehicle_geom));
  // 2. Get objects that need prediction and make prediction.
  const auto objects_to_predict = prediction_context->GetObjectsToPredict(
      *objects_proto);  // to be replaced by a
  auto object_prediction_results = SchedulePrediction(
      *prediction_context, *model_pool, *input.conflict_resolver_params,
      objects_to_predict, thread_pool, debug);

  // 3. Assemble prediction proto.
  prediction->Clear();
  std::vector<ObjectPredictionResult> objects;
  objects.reserve(object_prediction_results.size());
  prediction->mutable_objects()->Reserve(object_prediction_results.size());
  for (auto &key_val : object_prediction_results) {
    objects.push_back(std::move(key_val.second));
  }
  std::sort(objects.begin(), objects.end(),
            [](const auto &a, const auto &b) { return a.id < b.id; });
  for (const auto &object : objects) {
    ObjectPredictionProto obj;
    object.ToProto(&obj);
    *prediction->add_objects() = std::move(obj);
  }

  // 4. Update object prediction history, note that results are moved. Do not
  // use this vector afterwards.
  // TODO(xiangjun) add object prediction history update.
  return prediction;
}
}  // namespace qcraft::prediction
