#ifndef ONBOARD_PREDICTION_PREDICTOR_TNT_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_TNT_PREDICTOR_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
std::map<std::string, const mapping::IntersectionInfo *>
SelectTntPredictedObjects(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const absl::flat_hash_set<ObjectType> &pred_types,
    const int max_objects_num, const SemanticMapManager &semantic_map_manager);

std::vector<VehicleTNTFeature> PrepareTntFeature(
    const std::map<std::string, const mapping::IntersectionInfo *>
        &predicted_obj_ids,
    const ResampledObjectsHistory &resampled_objects_history,
    const SemanticMapManager &semantic_map_manager,
    const std::map<mapping::ElementId, std::vector<Vec2d>>
        &intersection_exits_map,
    const TrafficLightManager::TLStateHashMap &tl_states);

void MakeTntPrediction(const AvContext &av_context,
                       absl::Span<const ObjectHistory *const> objects_history,
                       const absl::flat_hash_set<ObjectType> &pred_types,
                       const int max_objects_num,
                       const SemanticMapManager &semantic_map_manager,
                       const TrafficLightManager::TLStateHashMap &tl_states);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_TNT_PREDICTOR_H_
