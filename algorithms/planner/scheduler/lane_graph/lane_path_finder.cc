#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"

#include <algorithm>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/scheduler/lane_graph/dijkstra.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

inline BasicGraph::VertexId vert_id(int layer, mapping::ElementId lane_id) {
  return absl::StrFormat("%d-%d", layer, lane_id);
}

inline std::pair<int, mapping::ElementId> from_vert_id(
    const BasicGraph::VertexId &vert_id) {
  const auto split_pos = vert_id.find('-');
  if (split_pos == std::string::npos) return {-1, mapping::kInvalidElementId};

  const int layer = std::stoi(vert_id.substr(0, split_pos));
  const mapping::ElementId lane_id = std::stoi(vert_id.substr(split_pos + 1));
  return {layer, lane_id};
}

double GetSectionsLength(const PlannerSemanticMapManager &psmm,
                         const std::vector<mapping::ElementId> &lane_ids,
                         double start_frac, double end_frac) {
  if (lane_ids.empty()) return 0.0;

  const auto get_section_len = [&psmm](mapping::ElementId lane_id) {
    return psmm.FindSectionInfoOrDie(psmm.FindLaneInfoOrDie(lane_id).section_id)
        .average_length;
  };

  double accum_len = 0.0;
  for (const auto &lane_id : lane_ids) {
    accum_len += get_section_len(lane_id);
  }
  accum_len -= get_section_len(lane_ids.front()) * start_frac;
  accum_len -= get_section_len(lane_ids.back()) * (1.0 - end_frac);

  // Handle numerical error.
  return std::max(0.0, accum_len);
}

LanePathInfo ExtendLanePath(const PlannerSemanticMapManager &psmm,
                            const RouteSectionsInfo &sections_info,
                            double path_cost, int last_sec_idx,
                            std::vector<mapping::ElementId> lane_ids,
                            double start_frac, double end_frac) {
  const double local_horizon =
      sections_info.planning_horizon() + kLocalMapExtension;
  const auto &sections = sections_info.section_segments();
  const double length_along_lane_path = std::min(
      local_horizon, GetSectionsLength(psmm, lane_ids, start_frac, end_frac));

  const mapping::LanePath raw_lane_path(psmm.semantic_map_manager(), lane_ids,
                                        start_frac, end_frac);
  if (raw_lane_path.length() > local_horizon) {
    return LanePathInfo(raw_lane_path.BeforeArclength(local_horizon),
                        length_along_lane_path, path_cost,
                        *psmm.semantic_map_manager());
  }

  double accum_len = raw_lane_path.length();
  mapping::ElementId cur_lane_id = lane_ids.back();
  if (end_frac < 1.0) lane_ids.pop_back();
  while (accum_len < local_horizon) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(cur_lane_id);
    if (end_frac == 1.0) {
      if (last_sec_idx + 1 == sections.size()) break;
      const auto &next_sec = sections[++last_sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      for (const auto lane_idx : lane_info.outgoing_lane_indices) {
        const auto next_lane_id = psmm.lane_info()[lane_idx].id;
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          outgoing_lane_id = next_lane_id;
          break;
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      end_frac = 0.0;
    } else {
      lane_ids.emplace_back(cur_lane_id);
      const double full_len = lane_info.length();
      const double len =
          full_len * (sections[last_sec_idx].end_fraction - end_frac);
      if (accum_len + len > local_horizon) {
        end_frac += (local_horizon - accum_len) / full_len;
        break;
      }
      if (last_sec_idx + 1 == sections.size()) {
        end_frac = sections.back().end_fraction;
        break;
      }
      accum_len += len;
      end_frac = 1.0;
    }
  }

  return LanePathInfo(
      mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                        start_frac, end_frac),
      length_along_lane_path, path_cost, *psmm.semantic_map_manager());
}

// Including the target but not the start id.
std::vector<mapping::ElementId> DirectlyConnectedLanesBetween(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info,
    const LaneGraph::LaneGraphLayers &layers, int start_layer, int target_layer,
    mapping::ElementId start_id, mapping::ElementId target_id) {
  if (target_id == mapping::kInvalidElementId) return {};

  const auto &sections = sections_info.section_segments();
  const int start_sec_idx = layers[start_layer].first,
            target_sec_idx = layers[target_layer].first;
  if (start_sec_idx == target_sec_idx) return {};

  std::queue<std::pair<int, mapping::ElementId>> q;
  absl::flat_hash_map<mapping::ElementId, mapping::ElementId> prev_map;
  q.push({start_sec_idx, start_id});
  while (!q.empty()) {
    const auto [cur_sec_idx, cur_id] = q.front();
    q.pop();

    const auto &lane_info = psmm.FindLaneInfoOrDie(cur_id);
    const auto &next_sec = sections[cur_sec_idx + 1];
    for (const auto next_idx : lane_info.outgoing_lane_indices) {
      const auto out_lane_id = psmm.lane_info()[next_idx].id;
      if (out_lane_id == target_id) {
        std::vector<mapping::ElementId> lane_ids{target_id};
        auto rev_id = cur_id;
        while (rev_id != start_id) {
          lane_ids.push_back(rev_id);
          rev_id = FindOrDie(prev_map, rev_id);
        }
        std::reverse(lane_ids.begin(), lane_ids.end());
        return lane_ids;
      }

      if (cur_sec_idx + 1 < target_sec_idx &&
          next_sec.id_idx_map.contains(out_lane_id)) {
        q.push({cur_sec_idx + 1, out_lane_id});
        prev_map[out_lane_id] = cur_id;
      }
    }
  }
  return {};
}

LanePathInfo RecoverLanePathInfo(const PlannerSemanticMapManager &psmm,
                                 const RouteSectionsInfo &sections_info,
                                 const LaneGraph &lane_graph,
                                 const BasicGraph::TravelPath &travel_path,
                                 int *last_lk_idx = nullptr) {
  const auto &layers = lane_graph.layers;
  const auto &vertices = travel_path.vertices;
  std::vector<mapping::ElementId> lane_ids;
  double end_frac = 0.0;
  auto [cur_layer, cur_id] = from_vert_id(vertices[0]);
  lane_ids.push_back(cur_id);
  for (int i = 1; i + 1 < vertices.size();) {
    auto [next_layer, next_id] = from_vert_id(vertices[i]);
    while (next_id == cur_id) {
      std::tie(next_layer, next_id) = from_vert_id(vertices[++i]);
    }

    const auto lanes_between = DirectlyConnectedLanesBetween(
        psmm, sections_info, layers, cur_layer, next_layer, cur_id, next_id);
    if (lanes_between.empty()) {
      end_frac = layers[from_vert_id(vertices[i - 1]).first].second;
      if (last_lk_idx != nullptr) *last_lk_idx = i - 1;
      break;
    }
    lane_ids.insert(lane_ids.end(), lanes_between.begin(), lanes_between.end());
    cur_layer = next_layer;
    cur_id = next_id;
  }

  return ExtendLanePath(psmm, sections_info, travel_path.total_cost,
                        layers[cur_layer].first, std::move(lane_ids),
                        layers[0].second, end_frac);
}

std::vector<LanePathInfo> FindBestLanePathsFrom(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, const LaneGraph &lane_graph,
    mapping::ElementId lane_id) {
  ASSIGN_OR_RETURN(
      const auto path,
      Dijkstra(lane_graph.graph, vert_id(0, lane_id), LaneGraph::kTargetVertex),
      std::vector<LanePathInfo>());
  int last_lk_idx = 0;
  std::vector<LanePathInfo> lp_infos{
      RecoverLanePathInfo(psmm, sections_info, lane_graph, path, &last_lk_idx)};

  const auto &vertices = path.vertices;
  for (int i = 0; i < last_lk_idx; ++i) {
    if (lane_graph.fork_edges.contains({vertices[i], vertices[i + 1]})) {
      // Try to find the second-best path from the first diverging point.
      auto new_graph = lane_graph.graph;
      new_graph.RemoveEdge(vertices[i], vertices[i + 1]);
      ASSIGN_OR_RETURN(
          auto fork_path,
          Dijkstra(new_graph, vertices[i], LaneGraph::kTargetVertex), lp_infos);

      auto &fork_vertices = fork_path.vertices;
      fork_vertices.insert(fork_vertices.begin(), vertices.begin(),
                           vertices.begin() + i);
      for (int j = 0; j < i; ++j) {
        fork_path.total_cost +=
            lane_graph.graph.edge_cost(vertices[j], vertices[j + 1]);
      }

      lp_infos.emplace_back(
          RecoverLanePathInfo(psmm, sections_info, lane_graph, fork_path));
      break;  // Only consider the first diverging point.
    }
  }
  return lp_infos;
}

}  // namespace

// Find the best lane paths starting from each lane in the first section.
std::vector<LanePathInfo> FindBestLanePathsFromStart(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, const LaneGraph &lane_graph,
    ThreadPool *thread_pool) {
  SCOPED_QTRACE("FindBestLanePathsFromStart");

  const auto &start_ids = sections_info.front().lane_ids;
  std::vector<std::vector<LanePathInfo>> results(start_ids.size());
  ParallelFor(
      0, start_ids.size(), thread_pool,
      [&psmm, &sections_info, &lane_graph, &start_ids, &results](int i) {
        results[i] = FindBestLanePathsFrom(psmm, sections_info, lane_graph,
                                           start_ids[i]);
      });

  std::vector<LanePathInfo> lp_infos;
  lp_infos.reserve(start_ids.size() * 2);  // At most one fork per start lane.
  for (auto &result : results) {
    for (auto &lp_info_from_lane : result) {
      if (!lp_info_from_lane.empty()) {
        lp_infos.emplace_back(std::move(lp_info_from_lane));
      }
    }
  }

  return lp_infos;
}

}  // namespace qcraft::planner
