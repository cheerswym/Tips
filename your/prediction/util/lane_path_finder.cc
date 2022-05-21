#include "onboard/prediction/util/lane_path_finder.h"

#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>

#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxSearchRadiusForNearestLane = 5.0;  // m.
constexpr int kMaxChildrenSearchLayer = 5;

// Approximate a remaining length to search lane path.
constexpr double kPathLengthRelaxationFactor = 2.0;
constexpr int kMaxPathSamplePoints = 10;

template <class T>
[[maybe_unused]] std::string FlatHashSetDebugString(const std::set<T>& set) {
  std::string res = "";
  for (const auto& ele : set) {
    res += (std::to_string(ele) + " ");
  }
  return res;
}

template <class T>
[[maybe_unused]] std::string VectorDebugString(const std::vector<T>& set) {
  std::string res = "";
  for (const auto& ele : set) {
    res += (std::to_string(ele) + " ");
  }
  return res;
}

std::set<mapping::ElementId> FindConsecutiveLaneIds(
    mapping::ElementId id, const mapping::SemanticMapManager& semantic_map_mgr,
    bool is_reversed, int max_level) {
  std::set<mapping::ElementId> res;
  if (id == mapping::kInvalidElementId) {
    return res;
  }
  res.insert(id);
  if (max_level <= 0) {
    return res;
  }
  const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(id);
  const auto* next_idxes = &lane_info.outgoing_lane_indices;
  if (is_reversed) {
    next_idxes = &lane_info.incoming_lane_indices;
  }
  for (const auto& next_idx : *next_idxes) {
    const auto next_id = semantic_map_mgr.lane_info()[next_idx].id;
    if (next_id == mapping::kInvalidElementId) {
      continue;
    }
    const auto next_set = FindConsecutiveLaneIds(next_id, semantic_map_mgr,
                                                 is_reversed, max_level - 1);
    for (const auto& next_next_id : next_set) {
      res.insert(next_next_id);
    }
  }
  return res;
}

struct LaneSequence {
  std::vector<mapping::ElementId> ids;
  double accumulated_len;
};

LaneSequence SearchMostStraightLaneSequence(
    const mapping::ElementId id,
    const mapping::SemanticMapManager& semantic_map_mgr, double start_len,
    double forward_len, bool is_reversed) {
  LaneSequence cur_sequence{{id}, start_len};
  while (cur_sequence.ids.size() != kMaxChildrenSearchLayer &&
         cur_sequence.accumulated_len <= forward_len) {
    const auto& cur_lane_id = cur_sequence.ids.back();
    const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(cur_lane_id);
    const auto* next_idxes_ptr = &lane_info.outgoing_lane_indices;
    if (is_reversed) {
      next_idxes_ptr = &lane_info.incoming_lane_indices;
    }
    if (next_idxes_ptr->empty()) {
      break;
    }
    double max_unit_dot = -1.0;
    auto min_angle_diff_idx = next_idxes_ptr->at(0);
    for (const auto& next_idx : *next_idxes_ptr) {
      const auto& next_lane_info = semantic_map_mgr.lane_info()[next_idx];
      Vec2d start_tangent, end_tangent;
      next_lane_info.GetTangent(0.0, &start_tangent);
      next_lane_info.GetTangent(1.0, &end_tangent);
      const double dot_prod = start_tangent.Dot(end_tangent);
      if (max_unit_dot < dot_prod) {
        max_unit_dot = dot_prod;
        min_angle_diff_idx = next_idx;
      }
    }
    const auto next_id = semantic_map_mgr.lane_info()[min_angle_diff_idx].id;
    cur_sequence.ids.push_back(next_id);
    cur_sequence.accumulated_len +=
        semantic_map_mgr.FindLaneInfoOrDie(next_id).length();
  }
  if (is_reversed) {
    std::reverse(cur_sequence.ids.begin(), cur_sequence.ids.end());
  }
  return cur_sequence;
}

[[maybe_unused]] bool isChild(const SemanticMapManager& smm,
                              const mapping::ElementId& curr,
                              const mapping::ElementId& child_maybe,
                              bool is_reversed) {
  const auto& lane_info = smm.FindLaneInfoOrDie(curr);
  auto* candidates = &lane_info.outgoing_lane_indices;
  if (is_reversed) {
    candidates = &lane_info.incoming_lane_indices;
  }
  for (const auto& lane_index : *candidates) {
    if (child_maybe == smm.lane_info()[lane_index].id) {
      return true;
    }
  }
  return false;
}

std::vector<const mapping::LaneInfo*> PruneChildren(
    const SemanticMapManager& smm,
    const std::vector<const mapping::LaneInfo*>& lanes, bool is_reversed) {
  std::set<mapping::ElementId> to_delete;
  for (int i = 0; i < lanes.size(); ++i) {
    for (int j = i + 1; j < lanes.size(); ++j) {
      const auto* curr = lanes[i];
      const auto* child_maybe = lanes[j];
      if (isChild(smm, curr->id, child_maybe->id, is_reversed)) {
        to_delete.insert(child_maybe->id);
      }

      if (isChild(smm, child_maybe->id, curr->id, is_reversed)) {
        to_delete.insert(curr->id);
      }
    }
  }
  std::vector<const mapping::LaneInfo*> ret;
  for (const auto* lane_info : lanes) {
    if (to_delete.count(lane_info->id) == 0) {
      ret.push_back(lane_info);
    }
  }

  return ret;
}

void AddNextLayer(const SemanticMapManager& smm, bool is_reversed,
                  std::vector<std::set<mapping::ElementId>>* graph,
                  int remain_level, double max_remain_s) {
  if (max_remain_s < 0.0) {
    return;
  }

  if (remain_level <= 0) {
    return;
  }
  const auto& prev_layer = graph->back();
  std::set<mapping::ElementId> layer_to_add;
  double min_length_cur_layer = std::numeric_limits<double>::infinity();
  for (const auto& lane_id : prev_layer) {
    const auto& lane_info = smm.FindLaneInfoOrDie(lane_id);
    auto* children_lane_indices = &lane_info.outgoing_lane_indices;
    if (is_reversed) {
      children_lane_indices = &lane_info.incoming_lane_indices;
    }

    for (const auto& child_lane_index : *children_lane_indices) {
      const auto child_lane_id = smm.lane_info()[child_lane_index].id;
      if (child_lane_id == mapping::kInvalidElementId) {
        continue;
      }
      layer_to_add.insert(child_lane_id);
      min_length_cur_layer = std::min(min_length_cur_layer, lane_info.length());
    }
  }

  graph->push_back(std::move(layer_to_add));
  AddNextLayer(smm, is_reversed, graph, remain_level - 1,
               max_remain_s - min_length_cur_layer);
}

}  // namespace

std::optional<mapping::ElementId> FindNearestLaneIdWithBoundaryDistanceLimit(
    const mapping::SemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double boundary_distance_limit) {
  const auto* lane_info = semantic_map_mgr.GetNearestLaneInfoAtLevel(
      semantic_map_mgr.GetLevel(), pt);
  if (lane_info && lane_info->id != mapping::kInvalidElementId) {
    if (mapping::GetDistOfPointInvasionLaneSupport(
            pt, semantic_map_mgr, lane_info->id) > -boundary_distance_limit) {
      return lane_info->id;
    }
  }
  return std::nullopt;
}

std::optional<mapping::ElementId>
FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
    const mapping::SemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double heading, double boundary_distance_limit, double max_heading_diff) {
  const auto* lane_info = semantic_map_mgr.GetNearestLaneInfoWithHeadingAtLevel(
      semantic_map_mgr.GetLevel(), pt, heading, kMaxSearchRadiusForNearestLane,
      max_heading_diff);
  if (lane_info && lane_info->id != mapping::kInvalidElementId) {
    if (mapping::GetDistOfPointInvasionLaneSupport(
            pt, semantic_map_mgr, lane_info->id) > -boundary_distance_limit) {
      return lane_info->id;
    }
  }
  return std::nullopt;
}

std::set<mapping::ElementId>
FindNearestLaneIdsByBBoxWithBoundaryDistLimitAndHeadingDiffLimit(
    const mapping::SemanticMapManager& semantic_map_mgr, const Box2d& box,
    double boundary_distance_limit, double max_heading_diff) {
  std::set<mapping::ElementId> id_sets;
  const double heading = box.heading();
  for (const auto& pt : box.GetCornersCounterClockwise()) {
    const auto id_or =
        FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
            semantic_map_mgr, pt, heading, boundary_distance_limit,
            max_heading_diff);
    if (id_or.has_value()) {
      id_sets.insert(*id_or);
    }
  }
  return id_sets;
}

std::set<mapping::ElementId> FilterLanesByConsecutiveRelationship(
    const mapping::SemanticMapManager& semantic_map_mgr,
    const std::set<mapping::ElementId>& lane_ids) {
  absl::flat_hash_map<mapping::ElementId, std::set<mapping::ElementId>>
      id_children_map;
  for (const auto& id : lane_ids) {
    id_children_map[id] = FindConsecutiveLaneIds(
        id, semantic_map_mgr, /*is_reversed=*/false, kMaxChildrenSearchLayer);
  }
  std::set<mapping::ElementId> res;
  for (const auto& id : lane_ids) {
    bool is_child = false;
    for (const auto& other_id : lane_ids) {
      if (other_id == id) {
        continue;
      }
      if (id_children_map[other_id].count(id) != 0) {
        is_child = true;
        break;
      }
    }
    if (!is_child) {
      res.insert(id);
    }
  }
  return res;
}

std::vector<mapping::LanePath> SearchLanePath(
    const Vec2d& pos, const mapping::SemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving) {
  struct LaneSequence {
    std::vector<mapping::ElementId> ids;
    double accumulated_len;
  };
  const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(lane_id);
  auto start_lane_length = lane_info.length();
  if (const auto closest_lane_pt_or =
          FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
              semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
              std::vector<mapping::ElementId>({lane_id}), /*heading=*/0.0,
              /*heading_penality_weight=*/0.0);
      closest_lane_pt_or.ok()) {
    start_lane_length -= closest_lane_pt_or->fraction() * start_lane_length;
  }
  std::vector<LaneSequence> cur_sequences{
      LaneSequence{{lane_id}, start_lane_length}};
  std::vector<LaneSequence> final_seqs;
  while (!cur_sequences.empty()) {
    std::vector<LaneSequence> new_seqs;
    for (const auto& lane_seq : cur_sequences) {
      const auto& last_lane = lane_seq.ids.back();
      const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(last_lane);
      const auto* next_idxes_ptr = &lane_info.outgoing_lane_indices;
      if (is_reverse_driving) {
        next_idxes_ptr = &lane_info.incoming_lane_indices;
      }
      if (next_idxes_ptr->empty()) {
        final_seqs.push_back(lane_seq);
        continue;
      }
      for (const auto& next_idx : *next_idxes_ptr) {
        const auto next_id = semantic_map_mgr.lane_info()[next_idx].id;
        if (next_id == mapping::kInvalidElementId) {
          continue;
        }
        LaneSequence new_seq = lane_seq;
        new_seq.ids.push_back(next_id);
        new_seq.accumulated_len +=
            semantic_map_mgr.FindLaneInfoOrDie(next_id).length();
        if (new_seq.ids.size() == kMaxChildrenSearchLayer ||
            new_seq.accumulated_len > forward_len) {
          final_seqs.push_back(new_seq);
          continue;
        }
        new_seqs.push_back(std::move(new_seq));
      }
    }
    cur_sequences = std::move(new_seqs);
  }
  std::vector<mapping::LanePath> ret;
  for (auto& seq : final_seqs) {
    if (is_reverse_driving) {
      std::reverse(seq.ids.begin(), seq.ids.end());
    }
    ret.push_back(mapping::LanePath(&semantic_map_mgr, seq.ids, 0.0, 1.0));
  }
  return ret;
}

mapping::LanePath SearchMostStraightLanePath(
    const Vec2d& pos, const mapping::SemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving) {
  const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(lane_id);
  auto start_lane_length = lane_info.length();
  if (const auto closest_lane_pt_or =
          FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
              semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
              std::vector<mapping::ElementId>({lane_id}), /*heading=*/0.0,
              /*heading_penality_weight=*/0.0);
      closest_lane_pt_or.ok()) {
    start_lane_length -= closest_lane_pt_or->fraction() * start_lane_length;
  }
  const auto cur_sequence = SearchMostStraightLaneSequence(
      lane_id, semantic_map_mgr, start_lane_length, forward_len,
      is_reverse_driving);
  return mapping::LanePath(&semantic_map_mgr, cur_sequence.ids, 0.0, 1.0);
}

mapping::LanePath ExtendMostStraightLanePath(
    const mapping::LanePath& lp,
    const mapping::SemanticMapManager& semantic_map_mgr, double forward_len,
    bool is_reversed) {
  const auto& ids = lp.lane_ids();
  QCHECK_GT(ids.size(), 0);
  mapping::ElementId start_id = mapping::kInvalidElementId;
  if (is_reversed) {
    start_id = ids.front();
  } else {
    start_id = ids.back();
  }
  auto cur_sequence = SearchMostStraightLaneSequence(start_id, semantic_map_mgr,
                                                     /*start_lane_length=*/0.0,
                                                     forward_len, is_reversed);
  std::vector<mapping::ElementId> final_ids;
  if (is_reversed) {
    std::copy(cur_sequence.ids.begin(), cur_sequence.ids.end(),
              std::back_inserter(final_ids));
    std::copy(std::next(ids.begin(), 1), ids.end(),
              std::back_inserter(final_ids));
  } else {
    std::copy(ids.begin(), ids.end(), std::back_inserter(final_ids));
    std::copy(std::next(cur_sequence.ids.begin(), 1), cur_sequence.ids.end(),
              std::back_inserter(final_ids));
  }
  return mapping::LanePath(&semantic_map_mgr, final_ids, 0.0, 1.0);
}

absl::StatusOr<std::vector<mapping::ElementId>>
FindConsecutiveLanesForDiscretizedPath(
    const mapping::SemanticMapManager& semantic_map_mgr,
    const planner::DiscretizedPath& path, bool is_reversed) {
  QCHECK_GE(path.size(), 1);
  std::vector<mapping::ElementId> ret;
  const auto& cur_point = path.front();

  // Collect layers of lanes and the number of sampled path points closest to
  // it.
  std::vector<std::set<mapping::ElementId>> graph;
  std::map<mapping::ElementId, int> point_lane_match_count;

  auto start_lanes = semantic_map_mgr.GetLanesInfoWithHeadingAtLevel(
      semantic_map_mgr.GetLevel(), Vec2d(cur_point.x(), cur_point.y()),
      cur_point.theta(), kMaxSearchRadiusForNearestLane, M_PI_2);

  if (start_lanes.empty()) {
    return absl::NotFoundError(
        "Cannot find nearest lane for the start of the path!");
  }

  start_lanes = PruneChildren(semantic_map_mgr, start_lanes, is_reversed);

  std::set<mapping::ElementId> first_layer;
  for (const auto* start_lane_info : start_lanes) {
    first_layer.insert(start_lane_info->id);
  }

  graph.push_back(std::move(first_layer));
  AddNextLayer(semantic_map_mgr, is_reversed, &graph,
               kMaxChildrenSearchLayer - 1,
               path.back().s() * kPathLengthRelaxationFactor);

  VLOG(5) << "Create lane graph with count mapping:";
  for (const auto& layer : graph) {
    VLOG(5) << "-----------";
    for (const auto& id : layer) {
      VLOG(5) << id;
    }
  }

  // Taking samples of points and add to point_lane_match_count counter.
  const int min_points = std::min<int>(kMaxPathSamplePoints, path.size());
  const int step = FloorToInt(path.size() / static_cast<float>(min_points));
  for (int i = 0; i < path.size(); i += step) {
    const auto& cur_point = path[i];
    const mapping::LaneInfo* lane = semantic_map_mgr.GetNearestLaneInfoAtLevel(
        semantic_map_mgr.GetLevel(), Vec2d(cur_point.x(), cur_point.y()));
    if (lane == nullptr) {
      continue;
    }
    point_lane_match_count[lane->id]++;
  }

  VLOG(5) << "Points on lane counter: ";
  for (const auto& pair : point_lane_match_count) {
    VLOG(5) << absl::StrFormat("Lane %d has %d point(s).", pair.first,
                               pair.second);
  }

  // Build lane path for this discretized path. Do not require semantic lane
  // connection correctness since agents might move differently.
  for (const auto& layer : graph) {
    mapping::ElementId max_id = mapping::kInvalidElementId;
    int max = 0;
    for (const auto& lane : layer) {
      auto* ptr_second = FindOrNull(point_lane_match_count, lane);
      if (ptr_second == nullptr) {
        continue;
      }
      const auto count = *ptr_second;
      if (count > max) {
        max = count;
        max_id = lane;
      }
    }
    if (max_id != mapping::kInvalidElementId) {
      ret.push_back(max_id);
    }
  }
  // Returned path size might be less than the graph size.
  if (ret.empty()) {
    return absl::NotFoundError(
        "Cannot project any other sampled points on path to this local lane "
        "graph!");
  }
  return ret;
}

std::optional<const mapping::IntersectionInfo*>
FindIntersectionsAlongTheLaneByDistance(
    const mapping::SemanticMapManager& semantic_map_mgr,
    const double forward_len, const mapping::ElementId lane_id,
    const double start_lane_fraction) {
  std::set<mapping::ElementId> visited_lane_ids;
  const auto& lane_info = semantic_map_mgr.FindLaneInfoOrDie(lane_id);
  for (const auto& intersection_info : lane_info.intersections) {
    if (start_lane_fraction <= intersection_info.second.y()) {
      if (std::max(0.0, start_lane_fraction - intersection_info.second.x()) *
              lane_info.length() <
          forward_len) {
        return &semantic_map_mgr.IntersectionAt(intersection_info.first);
      }
    }
  }
  auto comp = [](std::pair<double, const mapping::LaneInfo*> a,
                 std::pair<double, const mapping::LaneInfo*> b) {
    return a.first > b.first;
  };
  std::priority_queue<std::pair<double, const mapping::LaneInfo*>,
                      std::vector<std::pair<double, const mapping::LaneInfo*>>,
                      decltype(comp)>
      pq(comp);
  visited_lane_ids.insert(lane_id);
  double remain_forward_len =
      forward_len - lane_info.length() * (1.0 - start_lane_fraction);
  if (remain_forward_len > 0.0) {
    pq.push(std::make_pair(remain_forward_len, &lane_info));
  }

  while (!pq.empty()) {
    const auto& cur_lane = pq.top();
    pq.pop();
    const auto* next_idxes_ptr = &cur_lane.second->outgoing_lane_indices;
    if (next_idxes_ptr->empty()) {
      continue;
    }
    for (const auto& next_idx : *next_idxes_ptr) {
      const auto next_id = semantic_map_mgr.lane_info()[next_idx].id;
      if (next_id == mapping::kInvalidElementId) {
        continue;
      }
      if (visited_lane_ids.count(next_id) != 0) {
        continue;
      }
      visited_lane_ids.insert(next_id);
      const auto& next_lane_info = semantic_map_mgr.FindLaneInfoOrDie(next_id);
      const double next_lane_len = next_lane_info.length();
      const double remain_forward_len_fraction = cur_lane.first / next_lane_len;
      for (const auto& intersection_info : next_lane_info.intersections) {
        if (intersection_info.second.x() < remain_forward_len_fraction) {
          return &semantic_map_mgr.IntersectionAt(intersection_info.first);
        }
      }
      const double next_forward_len = cur_lane.first - next_lane_len;
      if (next_forward_len > 0.0) {
        pq.push(std::make_pair(next_forward_len, &next_lane_info));
      }
    }
  }
  return std::nullopt;
}
}  // namespace prediction
}  // namespace qcraft
