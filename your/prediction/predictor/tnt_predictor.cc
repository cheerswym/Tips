#include "onboard/prediction/predictor/tnt_predictor.h"

#include <math.h>

#include <algorithm>
#include <map>
#include <memory>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature_extractor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/lane_path_finder.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kExtendAngleDiffLimit = M_PI_4;  // rad.
constexpr double kForwardLen = 20.0;
inline ResampledObjectsHistory GetResampledHistory(
    const ObjectHistorySpan &av_object,
    absl::Span<const ObjectHistory *const> objs, double current_ts) {
  return GetResampledHistory(av_object, objs, current_ts,
                             kVehicleTNTConfig.history_num);
}
}  // namespace

// Closer object has higher priority to be considered as prophnet object.
std::map<std::string, const mapping::IntersectionInfo *>
SelectTntPredictedObjects(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const absl::flat_hash_set<ObjectType> &pred_types,
    const int max_objects_num, const SemanticMapManager &semantic_map_manager) {
  std::vector<const ObjectHistory *> screened_objs;
  std::map<std::string, const mapping::IntersectionInfo *>
      object_intersection_map;
  const std::vector<const ObjectHistory *> screened_objs_by_type =
      ScreenPredictObjectsByType(objs_to_predict, pred_types);
  screened_objs.reserve(screened_objs_by_type.size());
  for (auto *object_history : screened_objs_by_type) {
    if (object_history->empty()) {
      continue;
    }
    const auto &pos = object_history->back().val.pos();
    // Check if target point is inside the intersection.
    const auto &level = semantic_map_manager.GetLevel();
    const auto *intersection_ptr =
        semantic_map_manager.GetNearestIntersectionInfoAtLevel(level, pos);
    if (intersection_ptr == nullptr) {
      continue;
    }
    if (intersection_ptr->polygon_smooth.IsPointIn(pos)) {
      screened_objs.push_back(object_history);
      VLOG(2) << "Object Id " << object_history->id()
              << "is approaching Junction " << intersection_ptr->id;
      object_intersection_map.try_emplace(object_history->id(),
                                          intersection_ptr);
      continue;
    }
    const auto heading = object_history->back().val.heading();
    const auto nearest_lane_id_or =
        FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
            semantic_map_manager, pos, heading,
            /*boundary_distance_limit=*/0.0, kExtendAngleDiffLimit);
    if (!nearest_lane_id_or.has_value()) {
      continue;
    }
    ASSIGN_OR_CONTINUE(
        const auto closest_lane_pt,
        FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
            semantic_map_manager.GetLevel(), semantic_map_manager, pos,
            std::vector<mapping::ElementId>{*nearest_lane_id_or},
            /*heading=*/heading,
            /*heading_penality_weight=*/0.0));

    const auto intersection_ptr_or = FindIntersectionsAlongTheLaneByDistance(
        semantic_map_manager, kForwardLen, nearest_lane_id_or.value(),
        closest_lane_pt.fraction());
    if (!intersection_ptr_or.has_value()) {
      continue;
    }
    screened_objs.push_back(object_history);
    VLOG(2) << "Object Id " << object_history->id()
            << "is approaching Junction " << intersection_ptr->id;
    object_intersection_map.try_emplace(object_history->id(),
                                        intersection_ptr_or.value());
  }
  if (screened_objs.size() > max_objects_num) {
    screened_objs =
        ScreenPredictObjectsByDistance(ego_box, screened_objs, max_objects_num);
  }
  std::map<std::string, const mapping::IntersectionInfo *> result;
  std::for_each(
      screened_objs.begin(), screened_objs.end(),
      [&result, &object_intersection_map](const ObjectHistory *object_history) {
        result.try_emplace(object_history->id(),
                           object_intersection_map[object_history->id()]);
      });
  return result;
}

std::vector<VehicleTNTFeature> PrepareTntFeature(
    const std::map<std::string, const mapping::IntersectionInfo *>
        &predicted_obj_ids,
    const ResampledObjectsHistory &resampled_objects_history,
    const SemanticMapManager &semantic_map_manager,
    const std::map<mapping::ElementId, std::vector<Vec2d>>
        &intersection_exits_map,
    const TrafficLightManager::TLStateHashMap &tl_states) {
  std::vector<VehicleTNTFeature> features;
  features.reserve(predicted_obj_ids.size());
  for (const auto &resampled_object_history : resampled_objects_history) {
    const auto &id = resampled_object_history.back().id();
    if (predicted_obj_ids.count(id) == 0) {
      continue;
    }
    VLOG(2) << "Extracting Object id " << id << " junction id "
            << predicted_obj_ids.at(id)->id;
    features.push_back(ExtractVehicleTNTFeature(
        resampled_objects_history, semantic_map_manager,
        resampled_object_history, *predicted_obj_ids.at(id),
        intersection_exits_map, tl_states));
  }
  return features;
}

void MakeTntPrediction(const AvContext &av_context,
                       absl::Span<const ObjectHistory *const> objects_history,
                       const absl::flat_hash_set<ObjectType> &pred_types,
                       const int max_objects_num,
                       const SemanticMapManager &semantic_map_manager,
                       const std::map<mapping::ElementId, std::vector<Vec2d>>
                           &intersection_exits_map,
                       const TrafficLightManager::TLStateHashMap &tl_states) {
  const ObjectHistorySpan av_history =
      av_context.GetAvObjectHistory().GetHistory().value();
  const auto predicted_objs = SelectTntPredictedObjects(
      av_history.back().val.bounding_box(), objects_history, pred_types,
      max_objects_num, semantic_map_manager);
  const double current_ts = GetCurrentTimeStamp(objects_history);
  const auto resampled_objects_history =
      GetResampledHistory(av_history, objects_history, current_ts);
  const auto tnt_features = PrepareTntFeature(
      predicted_objs, resampled_objects_history, semantic_map_manager,
      intersection_exits_map, tl_states);
}
}  // namespace prediction
}  // namespace qcraft
