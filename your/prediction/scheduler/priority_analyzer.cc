#include "onboard/prediction/scheduler/priority_analyzer.h"

#include "onboard/prediction/prediction_util.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kScanAreaLength = 80.0;                    // m.
constexpr double kScanAreaWidth = 12.0;                     // m.
constexpr double kNearLaneDistanceThreshold = 4.0;          // m.
constexpr double kNearIntersectionDistanceThreshold = 2.0;  // m.
constexpr double kBackDistIgnorePed = -4.0;                 // m.
std::map<ObjectIDType, ObjectPredictionPriority> FindStationaryOrIgnoredObjects(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& obj_scenarios) {
  std::map<ObjectIDType, ObjectPredictionPriority> res;
  const auto& obj =
      prediction_context.av_context().GetAvObjectHistory().back().val;
  const Vec2d& ego_pos = obj.pos();
  const Vec2d ego_heading_vec = Vec2d::UnitFromAngle(obj.heading());
  const Box2d scan_area(ego_pos, obj.heading(), kScanAreaLength,
                        kScanAreaWidth);
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto* obj = objs_to_predict[i];
    const auto hist_or = obj->GetHistory();
    QCHECK(hist_or.ok());
    const auto& hist = hist_or.value();
    const std::string id = obj->id();
    const auto& scenario = obj_scenarios.at(obj->id());
    const bool is_in_scan_box = scan_area.IsPointIn(hist.back().val.pos());
    const bool is_on_road = (scenario.road_status() == ORS_ON_ROAD);
    const Vec2d ego_to_obstacle_vec = hist.back().val.pos() - ego_pos;
    double s = ego_to_obstacle_vec.Dot(ego_heading_vec);
    const bool is_static = hist.back().val.IsStationary();
    bool maybe_vru_in_front_near_lanes = false;
    if (scenario.has_abs_dist_to_nearest_lane()) {
      maybe_vru_in_front_near_lanes =
          (s > kBackDistIgnorePed) && MaybeVRU(hist.type()) &&
          (scenario.abs_dist_to_nearest_lane() < kNearLaneDistanceThreshold);
    }
    bool is_near_intersection = (scenario.intersection_status() ==
                                 ObjectIntersectionStatus::OIS_IN_INTERSECTION);
    if (scenario.has_abs_dist_to_nearest_intersection()) {
      is_near_intersection = (scenario.abs_dist_to_nearest_intersection() <
                              kNearIntersectionDistanceThreshold) ||
                             is_near_intersection;
    }
    const bool need_consider =
        (is_in_scan_box || is_on_road || maybe_vru_in_front_near_lanes ||
         is_near_intersection) &&
        (!is_static);
    VLOG(2) << "obj " << hist.id() << " is in scan box: " << is_in_scan_box
            << "; is on road: " << is_on_road
            << "; is ped like in front near lanes: "
            << maybe_vru_in_front_near_lanes
            << "; is near intersection: " << is_near_intersection;
    if (need_consider) {
      res[id] = OPP_P2;
    } else {
      res[id] = OPP_P3;  // Other objs.
    }
  }
  return res;
}

}  // namespace
std::map<ObjectIDType, ObjectPredictionPriority> AnalyzePriorities(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& obj_scenarios) {
  return FindStationaryOrIgnoredObjects(prediction_context, objs_to_predict,
                                        obj_scenarios);
}
}  // namespace prediction
}  // namespace qcraft
