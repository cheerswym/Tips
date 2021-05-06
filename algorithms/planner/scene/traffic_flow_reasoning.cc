#include "onboard/planner/scene/traffic_flow_reasoning.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/types/span.h"
#include "onboard/async/parallel_for.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft::planner {
namespace {
// Threshold value to ignore OT_UNKNOWN_STATIC object, which is far from AV.
constexpr double kIgnoreUnknownStaticDist = 100.0;  // m.
// Threshold value to check traffic chain is continuous.
constexpr double kMaxLonDistBetweenObjects = 10.0;  // m.
// Min threshold value to check object is waiting traffic light.
constexpr double kTrafficLightDistMinThreshold = 10.0;  // m.
// Max threshold value to check object is waiting traffic light.
constexpr double kTrafficLightDistMaxThreshold = 80.0;  // m
// Default stop time threshold for checking stall objects.
constexpr double kDefaultStopTimeThreshold = 10.0;  // s.
// Threshold value to check special zone effective.
constexpr double kSpecialZoneDistThreshold = 30.0;  // m
// Default stall object probability threshold.
constexpr double kStalledProbThreshold = 0.6;  // ratio.
// If the tl state is TL_RED , front object less likely to be stall.
constexpr double kDefaultRedTrafficLightMultiplier = 1.5;  // ratio.
using LanePath = mapping::LanePath;
using ObjectsOnLane =
    std::vector<std::pair<double, const ObjectPredictionProto *>>;

struct AnalyzeOnLaneOutput {
  TrafficWaitingQueueProto traffic_waiting_queue;
  std::vector<ObjectAnnotationProto> object_annotations;
};

double HalfLength(const Box2dProto &box_proto) {
  return box_proto.length() * 0.5;
}

double GetObjectFrontS(double object_s, const ObjectProto &object) {
  return object_s + HalfLength(object.bounding_box());
}

double GetObjectBackS(double object_s, const ObjectProto &object) {
  return object_s - HalfLength(object.bounding_box());
}

bool IsStationary(const ObjectPredictionProto &object_pred) {
  return object_pred.trajectories_size() == 0 ||
         prediction::IsStationaryPrediction(object_pred);
}

// TODO(jiayu): Lane path maybe too short to get traffic light info, extend lane
// path to sufficient length.
// Return all traffic light stop line distance which in front of AV.
std::vector<double> CollectTrafficLightStopLine(
    const PlannerSemanticMapManager &psmm, const LanePath &lane_path) {
  std::vector<double> tl_stop_lines;
  for (auto it = lane_path.begin(); it != lane_path.end(); ++it) {
    const auto *lane_proto = psmm.FindLaneByIdOrNull((*it).lane_id);
    if (lane_proto == nullptr) continue;

    if (!lane_proto->startpoint_associated_traffic_lights().empty()) {
      const double stop_line_s =
          lane_path.LaneIndexPointToArclength({it.lane_index(), 0.0});
      tl_stop_lines.push_back(stop_line_s);
    }

    for (const auto &multi_tl_control_point :
         lane_proto->multi_traffic_light_control_points()) {
      const double stop_line_s = lane_path.LaneIndexPointToArclength(
          {it.lane_index(), multi_tl_control_point.lane_fraction()});
      tl_stop_lines.push_back(stop_line_s);
    }

    // Extract next lane stop line info of end lane in lane path.
    const auto &lane_info = psmm.FindLaneInfoOrDie((*it).lane_id);
    if (!lane_info.outgoing_lane_indices.empty()) {
      for (const auto out_lane_index : lane_info.outgoing_lane_indices) {
        const auto &out_lane_info = psmm.lane_info()[out_lane_index];
        if (!out_lane_info.traffic_lights.empty()) {
          tl_stop_lines.push_back((*it).end_s);
          break;
        }
      }
    }
  }

  return tl_stop_lines;
}

// TODO(jiayu): Refactor this function and add comments.
std::vector<double> FindSpecialZonesOnLanePath(
    const PlannerSemanticMapManager &psmm, const LanePath &lane_path) {
  std::vector<double> arc_lens;
  auto collect_zones = [&arc_lens](const LanePath::LaneSegment &seg,
                                   const auto &zones, bool *connect_prev) {
    if (zones.empty() || zones.front().second[0] > 0.0) {
      *connect_prev = false;
    }
    for (const auto &[_, frac] : zones) {
      if (*connect_prev && frac[0] == 0.0) continue;

      if (seg.start_fraction <= frac[0] && frac[1] <= seg.end_fraction) {
        arc_lens.push_back(seg.start_s +
                           seg.length() * (frac[0] - seg.start_fraction));
      }
    }
    if (!zones.empty() && zones.back().second[1] == 1.0) {
      *connect_prev = true;
    }
  };

  bool connect_prev_intersect = false, connect_prev_crosswalk = false;
  for (const auto &seg : lane_path) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(seg.lane_id);
    // Intersections.
    collect_zones(seg, lane_info.intersections, &connect_prev_intersect);
    // Crosswalks.
    collect_zones(seg, lane_info.crosswalks, &connect_prev_crosswalk);
  }
  std::sort(arc_lens.begin(), arc_lens.end());
  return arc_lens;
}

bool IsObjectOnLanePathByPose(const PlannerSemanticMapManager &psmm,
                              const LanePath &lane_path,
                              const ObjectProto &object, double *arc_len) {
  if (lane_path.IsEmpty()) return false;

  // Check whether object pose on the lane path.
  const bool is_on_lane = IsPointOnLanePathAtLevel(
      psmm.GetLevel(), *psmm.semantic_map_manager(),
      Vec2dFromProto(object.pos()), lane_path, arc_len);

  // Ignore far unknown static object.
  if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
      *arc_len >= kIgnoreUnknownStaticDist) {
    return false;
  }

  return is_on_lane;
}

// “arc_len” represents the object pose projection distance on lane path.
// Check whether object on lane path by bouding box.
bool IsObjectOnLanePathByBoundingBox(const PlannerSemanticMapManager &psmm,
                                     const LanePath &lane_path,
                                     const ObjectProto &object,
                                     double *arc_len) {
  if (lane_path.IsEmpty()) return false;

  const auto closest_lane_point_or =
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), *psmm.semantic_map_manager(),
          Vec2dFromProto(object.pos()), lane_path, object.yaw());
  if (!closest_lane_point_or.ok()) return false;

  *arc_len =
      lane_path.FirstOccurrenceOfLanePointToArclength(*closest_lane_point_or);

  // Ignore far unknown static object.
  if (object.type() == ObjectType::OT_UNKNOWN_STATIC &&
      *arc_len >= kIgnoreUnknownStaticDist) {
    return false;
  }

  const auto bounding_box =
      ComputeObjectContour(object).BoundingBoxWithHeading(object.yaw());
  for (const auto &pt : bounding_box.GetCornersCounterClockwise()) {
    double s;
    if (IsPointOnLanePathAtLevel(psmm.GetLevel(), *psmm.semantic_map_manager(),
                                 pt, lane_path, &s)) {
      return true;
    }
  }

  return false;
}

/*
Two scenarios are considered:
1. If the dist from stop line to front edge belongs to [0.0, 10.0], regard as
traffic waiting.
    stop line     ------------
                       __
                      |  |
    object box        |  |
                      |__|

2. If the stop line crosses an object, regard as traffic waiting.
                       __
                      |  |
                      |  |
                  ------------
                      |__|
*/
bool IsTrafficWaiting(absl::Span<const double> tls_s,
                      double traffic_chain_front_object_front_s,
                      double traffic_chain_front_object_back_s,
                      double object_back_s) {
  const auto it_tl_front =
      std::upper_bound(tls_s.begin(), tls_s.end(), object_back_s);
  // No traffic light ahead, return false;
  if (it_tl_front == tls_s.end()) return false;

  const double dist_to_front_edge =
      *it_tl_front - traffic_chain_front_object_front_s;
  const double dist_to_back_edge =
      *it_tl_front - traffic_chain_front_object_back_s;
  return (0.0 < dist_to_front_edge &&
          dist_to_front_edge < kTrafficLightDistMinThreshold) ||
         (dist_to_front_edge * dist_to_back_edge < 0.0);
}

// Calculate multiplying factor between 1.0 and <max_factor>, when special zone
// ahead. Consider crosswalk/intersection/traffic light control  area as special
// zone. Param <s_vec> record the start_s of all special zones. Param
// <front_object_s> represent the  first front object's back_edge_s of current
// object chain. Param <current_object_s> represent current object's
// back_edge_s.
double CalculateFactorForSpecialZone(absl::Span<const double> s_vec,
                                     double front_object_s,
                                     double current_object_s,
                                     double distance_threshold,
                                     double max_factor) {
  double factor = 1.0;
  for (const double s : {current_object_s, front_object_s}) {
    const auto it_prev_special_zone_s =
        std::upper_bound(s_vec.begin(), s_vec.end(), s);
    if (it_prev_special_zone_s == s_vec.end()) continue;
    const double distance = *it_prev_special_zone_s - s;
    if (distance > distance_threshold) continue;
    // The more close to special zone, the more greater factor is.
    factor = std::max(factor, 1.0 + (max_factor - 1.0) *
                                        (1.0 - distance / distance_threshold));
  }
  return factor;
}

// The more moving time of neighbor object is, the greater stall probability is.
double CalculateFactorForMovingNeighbor(
    const ObjectStopTimeProto &stop_time_info) {
  const double factor =
      std::max(0.0, stop_time_info.previous_stop_time_duration() -
                        stop_time_info.last_move_time_duration()) /
      stop_time_info.last_move_time_duration();
  return std::min(1.0, factor);
}

std::vector<const ObjectPredictionProto *> CollectContinuousObjectsAheadOfS(
    const ObjectsOnLane &objects, double s) {
  std::vector<const ObjectPredictionProto *> objects_ahead;
  if (objects.empty() ||
      GetObjectFrontS(objects.back().first,
                      objects.back().second->perception_object()) < s) {
    return objects_ahead;
  }

  auto start_it = objects.rbegin(), end_it = objects.rbegin() + 1;
  for (; end_it != objects.rend(); ++end_it) {
    const double object_front_s =
        GetObjectFrontS(end_it->first, end_it->second->perception_object());
    if (object_front_s < s) break;

    const double prev_object_back_s = GetObjectBackS(
        (end_it - 1)->first, (end_it - 1)->second->perception_object());

    if (prev_object_back_s - object_front_s > kMaxLonDistBetweenObjects) {
      start_it = end_it;
    }
  }

  const double end_back_s = GetObjectBackS(
      (end_it - 1)->first, (end_it - 1)->second->perception_object());
  if (end_back_s - s > kMaxLonDistBetweenObjects) {
    return objects_ahead;
  }

  objects_ahead.reserve(end_it - start_it);
  for (auto it = start_it; it != end_it; ++it) {
    objects_ahead.push_back(it->second);
  }
  return objects_ahead;
}

bool CheckWhetherLanePathControlledByRedTrafficLight(
    const TrafficLightInfoMap &tl_info_map,
    const mapping::LanePath &lane_path) {
  for (const auto id : lane_path.lane_ids()) {
    const auto iter = tl_info_map.find(id);
    if (iter == tl_info_map.end()) continue;
    const auto &tl_info = iter->second;
    switch (tl_info.tl_control_type()) {
      case TrafficLightControlType::SINGLE_DIRECTION:
        return tl_info.tls().at(TrafficLightDirection::UNMARKED).tl_state ==
               TrafficLightState::TL_STATE_RED;
      case TrafficLightControlType::LEFT_WAITING_AREA:
        return tl_info.tls().at(TrafficLightDirection::LEFT).tl_state ==
               TrafficLightState::TL_STATE_RED;
    }
  }
  return false;
}

AnalyzeOnLaneOutput AnalyzeOnLane(
    const PlannerSemanticMapManager &psmm, const ObjectsOnLane &objects_on_lane,
    const LanePath &lane_path, const TrafficLightInfoMap &tl_info_map,
    absl::Span<const ObjectsOnLane *const> neighbors) {
  const auto tls_s = CollectTrafficLightStopLine(psmm, lane_path);
  const auto zones_s = FindSpecialZonesOnLanePath(psmm, lane_path);

  // Record all stall analysis temporary result, key object_id; value stall
  // probability.
  absl::flat_hash_map<std::string, double> stall_analysis;

  // Record all traffic waiting queue temporary result, first: object_s ;
  // second: object_id.
  std::vector<std::pair<double, std::string>> traffic_waiting_analysis;

  // First front vehicle in a continuous traffic chain.
  auto prev_cont_it = objects_on_lane.rbegin();
  bool has_stalled_ahead = false;
  // Analyze objects from far to near
  for (auto it = objects_on_lane.rbegin(); it != objects_on_lane.rend(); ++it) {
    const auto [object_s, object_pred_ptr] = *it;
    const auto &object = object_pred_ptr->perception_object();
    const double object_half_length = HalfLength(object.bounding_box());
    const double object_front_s = object_s + object_half_length;
    const double object_back_s = object_s - object_half_length;
    // NOTE(jiayu): whether traffic chain is continuous, only depends on the
    // distance between two adjacent objects, with no relevance to object motion
    // state or type.
    if (it != objects_on_lane.rbegin()) {
      const auto it_prev = it - 1;
      const double prev_back_s =
          GetObjectBackS(it_prev->first, it_prev->second->perception_object());
      if (prev_back_s - object_front_s > kMaxLonDistBetweenObjects) {
        prev_cont_it = it;
        has_stalled_ahead = false;
      }
    }

    // Filter moving object.
    if (!IsStationary(*object_pred_ptr)) continue;
    const auto &object_id = object.id();
    // Ignore all unknown static due to heavy mis-detection from perception.
    if (object.type() == ObjectType::OT_UNKNOWN_STATIC) {
      stall_analysis[object_id] = 0.0;
      continue;
    }
    // Cones and barriers must be stalled.
    if (object.type() == ObjectType::OT_BARRIER ||
        object.type() == ObjectType::OT_CONE ||
        object.type() == ObjectType::OT_WARNING_TRIANGLE) {
      stall_analysis[object_id] = 1.0;
      has_stalled_ahead = true;
      continue;
    }

    // Check object is waiting traffic light turn green.
    // 1. There is no stall object in current traffic chain.
    // 2. There is traffic light ahead.
    // 3. The vehicle in the front of traffic chain is waiting traffic or just
    // getting started.
    const double traffic_chain_front_object_front_s = GetObjectFrontS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    const double traffic_chain_front_object_back_s = GetObjectBackS(
        prev_cont_it->first, prev_cont_it->second->perception_object());
    if (!has_stalled_ahead && !tls_s.empty() &&
        IsTrafficWaiting(tls_s, traffic_chain_front_object_front_s,
                         traffic_chain_front_object_back_s, object_back_s)) {
      traffic_waiting_analysis.push_back({object_s, object_id});
    }
    const auto &smm = *psmm.semantic_map_manager();
    const bool is_leftmost_lane = IsLeftMostLane(smm, lane_path, object_s);
    const bool is_rightmost_lane = IsRightMostLane(smm, lane_path, object_s);
    // Set default stop_time_threshold.
    double stop_time_threshold = is_rightmost_lane
                                     ? kDefaultStopTimeThreshold
                                     : 3.0 * kDefaultStopTimeThreshold;

    // Intersection or crosswalk ahead, less likely to be stalled.
    stop_time_threshold *= CalculateFactorForSpecialZone(
        zones_s, traffic_chain_front_object_back_s, object_back_s,
        kSpecialZoneDistThreshold,
        /*max_factor= */ 2.0);

    // Traffic light ahead, less likely to be stalled.
    const double max_factor_for_tl =
        (!is_leftmost_lane && !is_rightmost_lane) ? 13.0 : 10.0;

    stop_time_threshold *= CalculateFactorForSpecialZone(
        tls_s, traffic_chain_front_object_back_s, object_back_s,
        kTrafficLightDistMaxThreshold, /*max_factor= */ max_factor_for_tl);

    // Laterally neighboring vehicles, less likely to be stalled.
    for (const auto &neighbor_lane : neighbors) {
      const auto neighbor_objects = CollectContinuousObjectsAheadOfS(
          *neighbor_lane, object_s - object_half_length);
      for (const auto &neighbor_ptr : neighbor_objects) {
        double factor = 0.0;
        const auto &neighbor_stop_time_info = neighbor_ptr->stop_time();
        if (IsStationary(*neighbor_ptr)) {
          factor = neighbor_stop_time_info.time_duration_since_stop() /
                   kDefaultStopTimeThreshold;
        } else {
          factor = CalculateFactorForMovingNeighbor(neighbor_stop_time_info);
        }
        stop_time_threshold *= 1.0 + std::min(1.0, factor);
      }
    }

    // The traffic light state is TL_RED, less likely to be stalled.
    // Strategy may conflict with line:308.
    if (CheckWhetherLanePathControlledByRedTrafficLight(tl_info_map,
                                                        lane_path)) {
      stop_time_threshold *= kDefaultRedTrafficLightMultiplier;
    }

    // Has moved in the recent past, less likely to be stalled.
    const auto &stop_time_info = object_pred_ptr->stop_time();
    if (stop_time_info.last_move_time_duration() > 0.0) {
      stop_time_threshold *=
          std::clamp(stop_time_info.last_move_time_duration() /
                         stop_time_info.time_duration_since_stop(),
                     1.0, 3.0);
    }

    // Has something stalled ahead, more likely to be stalled.
    if (has_stalled_ahead) {
      stop_time_threshold *= 0.3;
    }

    // Stops in parking area, more likely to be stalled.
    if (object.has_parked() && object.parked()) {
      stop_time_threshold *= 0.3;
    }

    // Multiple objects ahead, not on rightmost lane, less likely to be
    // stalled.
    if (!is_rightmost_lane) {
      for (auto it_front = prev_cont_it; it_front != it; ++it_front) {
        const auto front_ptr = it_front->second;
        if (IsStationary(*front_ptr)) {
          stop_time_threshold *=
              1.0 + (it - it_front) * std::min(1.0, FindOrDie(stall_analysis,
                                                              front_ptr->id()));
        } else {
          stop_time_threshold *=
              1.0 + (it - it_front) * CalculateFactorForMovingNeighbor(
                                          front_ptr->stop_time());
        }
      }
    }

    // Check stopped time at last.
    const double stalled_prob =
        stop_time_info.time_duration_since_stop() / stop_time_threshold;
    if (stalled_prob >= kStalledProbThreshold) {
      has_stalled_ahead = true;
    }

    stall_analysis[object_id] = std::min(1.0, stalled_prob);
  }

  // Generate traffic waiting queue.
  TrafficWaitingQueueProto traffic_waiting_queue;
  std::sort(traffic_waiting_analysis.begin(), traffic_waiting_analysis.end());
  for (const auto &tw_info : traffic_waiting_analysis) {
    // Note(jiayu): The id which in the traffic_waiting_queue was contained by
    // stall_analysis either .
    if (stall_analysis[tw_info.second] > kStalledProbThreshold) {
      continue;
    }
    traffic_waiting_queue.add_object_id(tw_info.second);
  }
  lane_path.ToProto(traffic_waiting_queue.mutable_lane_path());

  std::vector<ObjectAnnotationProto> object_annotations;
  object_annotations.reserve(stall_analysis.size());
  // Generate object annotation.
  for (const auto &[object_id, stall_probability] : stall_analysis) {
    ObjectAnnotationProto object_annotation;
    object_annotation.set_object_id(object_id);
    object_annotation.set_stalled_vehicle_likelyhood(stall_probability);
    object_annotation.set_depart_soon_likelyhood(1.0 - stall_probability);
    object_annotations.push_back(std::move(object_annotation));
  }

  return AnalyzeOnLaneOutput{
      .traffic_waiting_queue = std::move(traffic_waiting_queue),
      .object_annotations = std::move(object_annotations)};
}

TrafficFlowReasoningOutput MergeMultiAnalyzeOutput(
    absl::Span<const AnalyzeOnLaneOutput> analyze_outputs) {
  TrafficFlowReasoningOutput output;
  absl::flat_hash_map<std::string, ObjectAnnotationProto> objects_analysis_map;

  // Collect all traffic waiting queues and build objects analysis map.
  for (const auto &analysis : analyze_outputs) {
    output.traffic_waiting_queues.push_back(analysis.traffic_waiting_queue);

    for (const auto &object_annotation : analysis.object_annotations) {
      const auto &id = object_annotation.object_id();
      // Choose object annotation which stalled likelyhood greater.
      if (objects_analysis_map.find(id) == objects_analysis_map.end()) {
        objects_analysis_map[id] = object_annotation;
      } else {
        const auto &pre_object_annotation = objects_analysis_map[id];

        if (pre_object_annotation.stalled_vehicle_likelyhood() <
            object_annotation.stalled_vehicle_likelyhood()) {
          objects_analysis_map[id] = object_annotation;
        }
      }
    }
  }

  // Collect all objects annotation.
  for (auto &pair : objects_analysis_map) {
    output.object_annotations.push_back(std::move(pair.second));
  }

  return output;
}
}  // namespace

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput &input, ThreadPool *thread_pool) {
  SCOPED_QTRACE("TrafficFlowReasoning");
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.prediction);
  QCHECK_NOTNULL(input.lane_paths);
  QCHECK_NOTNULL(input.tl_info_map);

  const auto &psmm = *input.psmm;
  const auto &prediction = *input.prediction;
  // The lane path must in order, from left to right.
  const auto &lane_paths = *input.lane_paths;
  const auto &tl_info_map = *input.tl_info_map;
  // Check lane path set not empty.
  if (lane_paths.empty()) {
    return absl::InternalError("No Lane Path to make scene understanding");
  }
  // Classify object according to pos, and associate with lane path.
  std::vector<ObjectsOnLane> objects_on_lane_vec;
  const int lane_paths_size = lane_paths.size();
  objects_on_lane_vec.resize(lane_paths_size);
  for (auto &objects_on_lane : objects_on_lane_vec) {
    objects_on_lane.reserve(prediction.objects_size());
  }

  // TODO(jiayu): associate one object with multi lane path.
  // Classify objects from right to left.
  for (const auto &object_pred : prediction.objects()) {
    const auto &object = object_pred.perception_object();
    // Ignore vegetation, pedestrian, cyclist in traffic flow reasoning.
    if (object.type() == ObjectType::OT_VEGETATION ||
        object.type() == ObjectType::OT_PEDESTRIAN ||
        object.type() == ObjectType::OT_CYCLIST) {
      continue;
    }

    // Associate object with lane path by pose.
    bool has_associated = false;
    for (int i = lane_paths_size - 1; i >= 0; --i) {
      double s;
      const auto &lane_path = lane_paths[i];
      if (IsObjectOnLanePathByPose(psmm, lane_path, object, &s)) {
        objects_on_lane_vec[i].push_back({s, &object_pred});
        has_associated = true;
        break;
      }
    }

    // Associate object with lane path by bouding box. If associate failed by
    // object pose.
    if (has_associated == false) {
      for (int i = lane_paths_size - 1; i >= 0; --i) {
        double s;
        const auto &lane_path = lane_paths[i];
        if (IsObjectOnLanePathByBoundingBox(psmm, lane_path, object, &s)) {
          objects_on_lane_vec[i].push_back({s, &object_pred});
          break;
        }
      }
    }
  }

  // Sort object according s, from near to far.
  for (auto &objects_on_lane : objects_on_lane_vec) {
    std::sort(objects_on_lane.begin(), objects_on_lane.end());
  }

  std::vector<AnalyzeOnLaneOutput> analyze_outputs(lane_paths_size);
  ParallelFor(0, lane_paths_size, thread_pool, [&](int i) {
    std::vector<const ObjectsOnLane *> neighbors;
    // Not leftmost lane, has left neighbor.
    if (i != 0) {
      neighbors.push_back(&objects_on_lane_vec[i - 1]);
    }
    // Not rightmost lane, has right neighbor.
    if (i != lane_paths_size - 1) {
      neighbors.push_back(&objects_on_lane_vec[i + 1]);
    }
    analyze_outputs[i] = AnalyzeOnLane(psmm, objects_on_lane_vec[i],
                                       lane_paths[i], tl_info_map, neighbors);
  });

  return MergeMultiAnalyzeOutput(analyze_outputs);
}
}  // namespace qcraft::planner
