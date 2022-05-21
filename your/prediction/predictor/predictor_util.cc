#include "onboard/prediction/predictor/predictor_util.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <queue>
#include <string>
#include <utility>

#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
namespace {
struct ObjectKeyFilterField {
  const ObjectHistory *object_hist;
  double dist_to_ego;
};

ObjectKeyFilterField CreateObjectKeyFilterField(const ObjectHistory &obj_hist,
                                                const Box2d &ego_box) {
  const auto &hist_or = obj_hist.GetHistory();
  const auto &hist = hist_or.value();
  const double dist_to_ego = ego_box.DistanceTo(hist.back().val.bounding_box());
  return ObjectKeyFilterField{
      .object_hist = &obj_hist,
      .dist_to_ego = dist_to_ego,
  };
}

// Closer object has higher priority to be selected as Prophnet object.
struct ObjectKeyFilterFieldCmp {
  bool operator()(const ObjectKeyFilterField &object_a,
                  const ObjectKeyFilterField &object_b) {
    return object_a.dist_to_ego > object_b.dist_to_ego;
  }
};
}  // namespace.

// Closer object has higher priority to be considered as prophnet object.
std::vector<const ObjectHistory *> ScreenPredictObjectsByDistance(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    int max_objects_num) {
  std::vector<const ObjectHistory *> pred_objects;
  std::priority_queue<ObjectKeyFilterField, std::vector<ObjectKeyFilterField>,
                      ObjectKeyFilterFieldCmp>
      object_pq;
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    object_pq.push(CreateObjectKeyFilterField(*objs_to_predict[i], ego_box));
  }
  while (!object_pq.empty()) {
    pred_objects.push_back(object_pq.top().object_hist);
    if (pred_objects.size() == max_objects_num) {
      return pred_objects;
    }
    object_pq.pop();
  }
  return pred_objects;
}

std::vector<const ObjectHistory *> ScreenPredictObjectsByType(
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const absl::flat_hash_set<ObjectType> &pred_types) {
  std::vector<const ObjectHistory *> pred_objects;
  for (const ObjectHistory *const object_history : objs_to_predict) {
    if (pred_types.count(object_history->type())) {
      pred_objects.push_back(object_history);
    }
  }
  return pred_objects;
}

double GetCurrentTimeStamp(absl::Span<const ObjectHistory *const> objs) {
  return objs.empty() ? 0.0
                      : std::accumulate(objs.begin(), objs.end(), 0.0,
                                        [](const double sum, const auto &obj) {
                                          return sum + obj->timestamp();
                                        }) /
                            objs.size();
}

ResampledObjectsHistory GetResampledHistory(
    const ObjectHistorySpan &av_object,
    absl::Span<const ObjectHistory *const> objs, double current_ts,
    int history_num) {
  ResampledObjectsHistory resampled_objects_history;
  resampled_objects_history.reserve(objs.size() + 1);
  for (const auto *obj : objs) {
    resampled_objects_history.emplace_back(
        ResampleObjectHistorySpan(obj->GetHistory().value(), current_ts,
                                  kPredictionTimeStep, history_num + 1));
  }
  resampled_objects_history.emplace_back(ResampleObjectHistorySpan(
      av_object, current_ts, kPredictionTimeStep, history_num + 1));
  return resampled_objects_history;
}
}  // namespace prediction
}  // namespace qcraft
