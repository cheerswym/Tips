#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_PROPHNET_FEATURE_EXTRACTOR_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_PROPHNET_FEATURE_EXTRACTOR_H_

#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/nets/trt/prophnet.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft {
namespace prediction {
// The detection region is consistent with fen classifier.
constexpr float kProphnetMapHalfWidth = 51.2f;
constexpr float kProphnetMapRegionFront = 112.0f;
constexpr float kProphnetMapRegionBehind = 67.2f;
constexpr float kProphnetObjectPredictionHalfWidth = 45.2f;
constexpr float kProphnetObjectPredictionRegionFront = 100.0f;
constexpr float kProphnetObjectPredictionRegionBehind = 50.0f;
// For one ObjectHistory, the back is the latest.
prophnet::ProphnetFeature ExtractProphnetFeature(
    const ResampledObjectsHistory& objects_history,
    const TrafficLightManager::TLStateHashMap& tl_states,
    const SemanticMapManager* semantic_map_manager);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_PROPHNET_FEATURE_EXTRACTOR_H_
