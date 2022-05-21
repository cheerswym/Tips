#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/time/time.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/object_prediction_history_span.h"
#include "onboard/prediction/container/object_predictions_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft {
namespace prediction {

// TODO(xiang): this class should be removed totally.
class PredictionContext {
 public:
  PredictionContext();
  absl::Status Update(absl::Time prediction_init_time,
                      const SemanticMapManager& semantic_map_mgr,
                      const TrafficLightStatesProto& tl_states,
                      const PoseProto& av_pose,
                      const VehicleGeometryParamsProto& veh_geom_params);
  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto> localization_transform);
  void UpdateObjects(const ObjectsProto& objects_proto,
                     const LocalizationTransformProto& localization_transform,
                     ThreadPool* thread_pool);

  const ObjectsHistory& object_long_term_history_manager() const {
    return obj_long_term_hist_mgr_;
  }
  const ObjectsHistory& object_history_manager() const {
    return *prediction_input_.objects_history;
  }
  const AvContext& av_context() const { return *prediction_input_.av_context; }
  const VehicleGeometryParamsProto& vehicle_geometry_params() const {
    return prediction_input_.veh_geom_params;
  }
  const TrafficLightManager& traffic_light_manager() const {
    return traffic_light_manager_;
  }
  const SemanticMapManager* semantic_map_manager() const {
    return prediction_input_.semantic_map_manager;
  }
  bool HasObjects() const {
    return !prediction_input_.objects_history->empty();
  }
  bool HasLocalizationTransform() const {
    return prediction_input_.localization_transform != nullptr;
  }
  absl::Time PredictionInitTime() const {
    return prediction_input_.prediction_init_time;
  }
  std::vector<const ObjectHistory*> GetObjectsToPredict(
      const ObjectsProto& objects_proto);
  // TODO(xiang): Remove later.
  PredictionInput* mutable_prediction_input() { return &prediction_input_; }

 private:
  // TODO(xiang): Can be created for each iteration, need to remove.
  TrafficLightManager traffic_light_manager_;
  PredictionInput prediction_input_;
  ObjectsHistory obj_long_term_hist_mgr_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_
