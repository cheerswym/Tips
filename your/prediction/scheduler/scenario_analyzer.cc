#include "onboard/prediction/scheduler/scenario_analyzer.h"

#include <algorithm>
#include <limits>

#include "onboard/maps/semantic_map_util.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kDistanceEpsilon = 1e-2;  // m.
}  // namespace
std::map<ObjectIDType, ObjectPredictionScenario> AnalyzeScenarios(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict) {
  const auto* semantic_map_mgr = prediction_context.semantic_map_manager();
  QCHECK_NOTNULL(semantic_map_mgr);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto hist_or = objs_to_predict[i]->GetHistory();
    QCHECK(hist_or.ok());
    const std::string id = objs_to_predict[i]->id();
    scenarios[id] = AnalyzeScenarioWithSemanticMapAndObjectProto(
        *semantic_map_mgr, hist_or->back().val.object_proto());
    VLOG(2) << "Scenario: " << id << " " << scenarios[id].DebugString();
  }
  return scenarios;
}

ObjectPredictionScenario AnalyzeScenarioWithSemanticMapAndObjectProto(
    const SemanticMapManager& semantic_map_mgr, const ObjectProto& obj_proto) {
  ObjectPredictionScenario scenario;
  const Vec2d obj_pos = Vec2dFromProto(obj_proto.pos());
  const Box2d bbox(obj_proto.bounding_box());
  scenario.set_abs_dist_to_nearest_lane(std::numeric_limits<double>::max());
  scenario.set_abs_dist_to_nearest_intersection(
      std::numeric_limits<double>::max());

  if (const auto nearest_lane_point_or =
          FindClosestLanePointToSmoothPointAtLevel(semantic_map_mgr.GetLevel(),
                                                   semantic_map_mgr, obj_pos);
      !nearest_lane_point_or.ok()) {
    scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  } else {
    scenario.set_nearest_lane_id(nearest_lane_point_or->lane_id());
    // Check if object bbox is on road.
    bool is_off_road = true;
    double abs_dist_to_lane = std::numeric_limits<double>::max();
    for (const auto& corner_pt : bbox.GetCornersCounterClockwise()) {
      const double invasion_dist = GetDistOfPointInvasionLaneSupport(
          corner_pt, semantic_map_mgr, nearest_lane_point_or->lane_id());
      if (invasion_dist < 0) {  // Not on lane.
        abs_dist_to_lane = std::min(abs_dist_to_lane, std::fabs(invasion_dist));
      } else {  // Already in lane.
        abs_dist_to_lane = 0.0;
        is_off_road = false;
        break;
      }
    }
    scenario.set_abs_dist_to_nearest_lane(abs_dist_to_lane);
    if (is_off_road) {
      scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
    } else {
      QCHECK_LE(abs_dist_to_lane, 1e-3);
      scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
    }
  }
  const auto* intersection_ptr =
      semantic_map_mgr.GetNearestIntersectionInfoAtLevel(
          semantic_map_mgr.GetLevel(), obj_pos);
  if (intersection_ptr != nullptr) {
    scenario.set_nearest_intersection_id(intersection_ptr->id);
    bool is_out_intersection = true;
    double abs_dist_to_intersection = std::numeric_limits<double>::max();
    // Check if object bbox is in intersection.
    for (const auto& corner_pt : bbox.GetCornersCounterClockwise()) {
      const double cur_distance =
          intersection_ptr->polygon_smooth.DistanceTo(corner_pt);
      abs_dist_to_intersection =
          std::min(abs_dist_to_intersection, cur_distance);
      if (cur_distance < kDistanceEpsilon) {
        is_out_intersection = false;
        break;
      }
    }
    scenario.set_abs_dist_to_nearest_intersection(abs_dist_to_intersection);
    if (is_out_intersection) {
      scenario.set_intersection_status(
          ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
    } else {
      scenario.set_intersection_status(
          ObjectIntersectionStatus::OIS_IN_INTERSECTION);
    }
  } else {
    scenario.set_intersection_status(
        ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  }
  return scenario;
}
}  // namespace prediction
}  // namespace qcraft
