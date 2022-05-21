#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_VEHICLE_TNT_FEATURE_EXTRACTOR_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_VEHICLE_TNT_FEATURE_EXTRACTOR_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
// For one ObjectHistory, the back is the latest.
VehicleTNTFeature ExtractVehicleTNTFeature(
    const ResampledObjectsHistory &objects_history,
    const SemanticMapManager &semantic_map_manager,
    const std::vector<ObjectProto> &ego_history,
    const qcraft::mapping::IntersectionInfo &intersection,
    const std::map<mapping::ElementId, std::vector<Vec2d>>
        &intersection_exits_map,
    const TrafficLightManager::TLStateHashMap &tl_states);

void ToVehicleTNTDumpedFeatureProto(
    const VehicleTNTFeature &tnt_feature,
    VehicleTNTDumpedFeatureProto *const feature_proto);

// Filter out objects without valid ground truth trajectory (go beyond
// intersection)
std::map<std::string, Vec2d> SelectObjectsWithValidGroundTruth(
    const SemanticMapManager &semantic_map_manager,
    const std::map<ObjectIDType, const ObjectPredictionProto *> &log_pred_map,
    std::map<std::string, const mapping::IntersectionInfo *>
        *object_intersection_map);

std::optional<std::pair<VehicleTNTDumpedFeatureProto::GroundTruthProto,
                        VehicleTNTDumpedFeatureProto::LabelsProto>>
ExtractTntGroundTruth(const Vec2d &ref_pos, double rot_rad,
                      const ObjectPredictionProto &object_prediction_proto,
                      absl::Span<const float> targets, const double ts,
                      const std::map<std::string, Vec2d> &obj_exits_map);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_VEHICLE_TNT_FEATURE_EXTRACTOR_H_
