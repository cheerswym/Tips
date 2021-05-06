#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_sections_util.h"

namespace qcraft::planner {

namespace {

// Extend local map here to prevent sudden change of route look ahead cost.
constexpr double kExtendedHorizonOverLocalMap = 5.0 * kMinLcLaneLength;

constexpr int kVertexNumPerMinLcLen = 2;
constexpr double kVertexSampleDist = kMinLcLaneLength / kVertexNumPerMinLcLen;

constexpr double kCrossSolidWhiteCost = 1000.0;
constexpr double kOrdinaryLaneChangeCost = 100.0;
constexpr double kStalledObjectCost = 2.0 * kCrossSolidWhiteCost;
constexpr double kLaneLengthCost = 1e-3;  // per meter.
// Should be larger than cross-solid lc cost after multiplying normal lk cost:
constexpr double kAvoidLaneCostFactor =
    1.2 * kCrossSolidWhiteCost / (kLaneLengthCost * kVertexSampleDist);

using AvoidLanes = absl::flat_hash_set<mapping::ElementId>;

inline BasicGraph::VertexId vert_id(int layer, mapping::ElementId lane_id) {
  return absl::StrFormat("%d-%d", layer, lane_id);
}

bool HasOutgoingLane(
    const PlannerSemanticMapManager &psmm, mapping::ElementId lane_id,
    const RouteSectionsInfo::RouteSectionSegmentInfo &next_sec) {
  const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
  for (const auto lane_idx : lane_info.outgoing_lane_indices) {
    const auto out_lane_id = psmm.lane_info()[lane_idx].id;
    if (next_sec.id_idx_map.contains(out_lane_id)) {
      return true;
    }
  }
  return false;
}

std::vector<mapping::ElementId> FindDirectlyConnectedLanesInSection(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, int sec_idx,
    mapping::ElementId from_id, int target_sec_idx) {
  const auto &sections = sections_info.section_segments();
  std::vector<mapping::ElementId> lane_ids;
  std::queue<std::pair<int, mapping::ElementId>> q;
  q.push({sec_idx, from_id});
  while (!q.empty()) {
    const auto [cur_sec_idx, cur_id] = q.front();
    q.pop();
    if (cur_sec_idx == target_sec_idx) {
      lane_ids.push_back(cur_id);
      continue;
    }
    const auto &lane_info = psmm.FindLaneInfoOrDie(cur_id);
    const auto &next_sec = sections[cur_sec_idx + 1];
    for (const auto next_idx : lane_info.outgoing_lane_indices) {
      const auto out_lane_id = psmm.lane_info()[next_idx].id;
      if (next_sec.id_idx_map.contains(out_lane_id)) {
        q.push({cur_sec_idx + 1, out_lane_id});
      }
    }
  }
  return lane_ids;
}

double GetLaneKeepCost(const PlannerSemanticMapManager &psmm,
                       const AvoidLanes &avoid_lanes,
                       mapping::ElementId lane_id, double from_frac,
                       double to_frac) {
  const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
  double lane_cost =
      kLaneLengthCost * lane_info.length() * (to_frac - from_frac);
  if (avoid_lanes.contains(lane_id) ||
      lane_info.IsPassengerVehicleAvoidLaneType()) {
    lane_cost *= kAvoidLaneCostFactor;
  }

  return lane_cost;
}

double GetMixedLaneKeepCost(const PlannerSemanticMapManager &psmm,
                            const AvoidLanes &avoid_lanes,
                            mapping::ElementId from_id, double from_frac,
                            mapping::ElementId to_id, double to_frac) {
  return GetLaneKeepCost(psmm, avoid_lanes, from_id, from_frac, 1.0) +
         GetLaneKeepCost(psmm, avoid_lanes, to_id, 0.0, to_frac);
}

double GetLaneChangeCost(const PlannerSemanticMapManager &psmm, int layer_idx,
                         mapping::ElementId from_id, bool lc_left,
                         mapping::ElementId to_id) {
  constexpr double kEarlyLaneChangeCostFull = 0.5 * kOrdinaryLaneChangeCost;

  const auto lc_cost = [&psmm, layer_idx](mapping::ElementId lane_id,
                                          bool lc_left) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
    const auto &neighbors = lc_left ? lane_info.lane_neighbors_on_left
                                    : lane_info.lane_neighbors_on_right;
    if (neighbors.empty()) return BasicGraph::kInfiniteCost;

    // Only consider solid white since lane graph edges do not cross yellow
    // boundaries or curbs.
    const double base_cost = neighbors.front().lane_boundary_type ==
                                     mapping::LaneBoundaryProto::SOLID_WHITE
                                 ? kCrossSolidWhiteCost
                                 : kOrdinaryLaneChangeCost;
    // Add an extra cost to punish too early lc.
    return base_cost +
           kEarlyLaneChangeCostFull / static_cast<double>(layer_idx + 1);
  };

  return std::max(lc_cost(from_id, lc_left), lc_cost(to_id, !lc_left));
}

LaneGraph::LaneGraphLayers SplitSectionsAsVertices(
    const RouteSectionsInfo &sections_info, int last_sec_idx,
    double last_sec_frac) {
  const auto &sections = sections_info.section_segments();
  LaneGraph::LaneGraphLayers layers;
  layers.reserve(CeilToInt(sections_info.length_between(0, last_sec_idx + 1) /
                           kVertexSampleDist) +
                 1);
  double accum_len = sections[0].average_length * sections[0].start_fraction;
  for (int i = 0; i < last_sec_idx; ++i) {
    while (accum_len < sections[i].average_length) {
      const double fraction = accum_len / sections[i].average_length;
      layers.push_back({i, fraction});
      accum_len += kVertexSampleDist;
    }
    accum_len -= sections[i].average_length;
  }

  const auto &last_sec = sections[last_sec_idx];
  const double last_sec_len = last_sec.average_length * last_sec_frac;
  while (accum_len < last_sec_len) {
    if (last_sec_len - accum_len < 0.1 * kVertexSampleDist) break;

    const double fraction = accum_len / last_sec.average_length;
    layers.push_back({last_sec_idx, fraction});
    accum_len += kVertexSampleDist;
  }
  layers.push_back({last_sec_idx, last_sec_frac});

  return layers;
}

void ConnectEdgesWithinSections(const PlannerSemanticMapManager &psmm,
                                const RouteSectionsInfo &sections_info,
                                const AvoidLanes &avoid_lanes,
                                const LaneGraph::LaneGraphLayers &layers,
                                BasicGraph *graph) {
  const auto &sections = sections_info.section_segments();
  for (int i = 0; i + 1 < layers.size(); ++i) {
    if (layers[i].first != layers[i + 1].first) continue;

    // Directly connected edges within sections.
    const auto [sec_idx, from_frac] = layers[i];
    const auto &sec_lanes = sections[sec_idx].lane_ids;
    for (const auto lane_id : sec_lanes) {
      graph->AddEdge(vert_id(i, lane_id), vert_id(i + 1, lane_id),
                     GetLaneKeepCost(psmm, avoid_lanes, lane_id, from_frac,
                                     layers[i + 1].second));
    }

    const int to_layer = i + kVertexNumPerMinLcLen;
    if (to_layer >= layers.size() ||
        layers[i].first != layers[to_layer].first) {
      continue;
    }
    // Lc edges within sections.
    for (int lane_idx = 0; lane_idx < sec_lanes.size(); ++lane_idx) {
      const auto from_id = sec_lanes[lane_idx];
      if (lane_idx > 0) {
        const auto to_id = sec_lanes[lane_idx - 1];
        graph->AddEdge(
            vert_id(i, from_id), vert_id(to_layer, to_id),
            GetLaneChangeCost(psmm, i, from_id, /*lc_left=*/true, to_id));
      }
      if (lane_idx + 1 < sec_lanes.size()) {
        const auto to_id = sec_lanes[lane_idx + 1];
        graph->AddEdge(
            vert_id(i, from_id), vert_id(to_layer, to_id),
            GetLaneChangeCost(psmm, i, from_id, /*lc_left=*/false, to_id));
      }
    }
  }
}

void ConnectEdgesAcrossSections(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, const AvoidLanes &avoid_lanes,
    const LaneGraph::LaneGraphLayers &layers, int last_sec_idx,
    BasicGraph *graph, absl::flat_hash_set<BasicGraph::EdgeType> *fork_edges) {
  const auto add_lc_edges = [&](int sec_idx, mapping::ElementId cur_id,
                                mapping::ElementId neighbor_id, bool lc_left,
                                int start_layer, int end_layer) {
    for (int j = start_layer; j < end_layer; ++j) {
      const int to_layer = j + kVertexNumPerMinLcLen;
      const auto target_lanes = FindDirectlyConnectedLanesInSection(
          psmm, sections_info, sec_idx, neighbor_id, layers[to_layer].first);
      for (const auto target_id : target_lanes) {
        graph->AddEdge(vert_id(j, cur_id), vert_id(to_layer, target_id),
                       GetLaneChangeCost(psmm, j, cur_id, lc_left, target_id));
      }
    }
  };

  const auto &sections = sections_info.section_segments();
  const int n = layers.size();
  int layer_idx = 0;
  for (int sec_idx = 0; sec_idx < last_sec_idx; ++sec_idx) {
    if (layers[layer_idx].first != sec_idx) {
      // The current section contains no layer.
      continue;
    }

    while (layer_idx + kVertexNumPerMinLcLen < n &&
           layers[layer_idx + kVertexNumPerMinLcLen].first == sec_idx) {
      layer_idx += 1;
    }
    // Lk edges to the next section start from last layer in this section.
    int lk_layer_idx = layer_idx;
    while (layers[lk_layer_idx].first == sec_idx) ++lk_layer_idx;
    // Lc edges start among layers [layer_idx, last_lc_layer).
    const int last_lc_layer = std::min(lk_layer_idx, n - kVertexNumPerMinLcLen);

    const auto &cur_lanes = sections[sec_idx].lane_ids;
    // Expand along each lane to the next section.
    for (int lane_idx = 0; lane_idx < cur_lanes.size(); ++lane_idx) {
      const auto cur_id = cur_lanes[lane_idx];
      if (!HasOutgoingLane(psmm, cur_id, sections[sec_idx + 1])) continue;

      // Add directly connected outgoing edges.
      const auto target_lanes = FindDirectlyConnectedLanesInSection(
          psmm, sections_info, sec_idx, cur_id, layers[lk_layer_idx].first);
      for (const auto target_id : target_lanes) {
        graph->AddEdge(
            vert_id(lk_layer_idx - 1, cur_id), vert_id(lk_layer_idx, target_id),
            GetMixedLaneKeepCost(psmm, avoid_lanes, cur_id,
                                 layers[lk_layer_idx - 1].second, target_id,
                                 layers[lk_layer_idx].second));
      }
      // If multiple outgoing lanes exist, mark as forking edges.
      if (target_lanes.size() > 1) {
        for (const auto target_id : target_lanes) {
          fork_edges->insert({vert_id(lk_layer_idx - 1, cur_id),
                              vert_id(lk_layer_idx, target_id)});
        }
      }
      // Add lc edges to neighbors' outgoing lanes.
      if (lane_idx > 0) {
        add_lc_edges(sec_idx, cur_id, cur_lanes[lane_idx - 1],
                     /*lc_left=*/true, layer_idx, last_lc_layer);
      }
      if (lane_idx + 1 < cur_lanes.size()) {
        add_lc_edges(sec_idx, cur_id, cur_lanes[lane_idx + 1],
                     /*lc_left=*/false, layer_idx, last_lc_layer);
      }
    }
    layer_idx = lk_layer_idx;
  }
}

void ProjectStationaryObjects(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, bool target_in_horizon,
    const PlannerObjectManager &obj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const LaneGraph::LaneGraphLayers &layers, BasicGraph *graph) {
  std::vector<Box2d> obj_boxes;
  obj_boxes.reserve(stalled_objects.size());
  for (const auto obj_ptr : obj_mgr.stationary_objects()) {
    if (!stalled_objects.contains(obj_ptr->id())) continue;

    constexpr double kObjectBoxBuffer = 0.8;  // m.
    auto obj_box = obj_ptr->bounding_box();
    obj_box.LongitudinalExtend(kObjectBoxBuffer);
    obj_box.LateralExtend(kObjectBoxBuffer);
    ASSIGN_OR_CONTINUE(const auto obj_proj,
                       FindSmoothPointOnRouteSectionsByLateralOffset(
                           psmm, sections_info, obj_box.center()));

    if (!IsLanePathBlockedByBox2dAtLevel(
            psmm.GetLevel(), *psmm.semantic_map_manager(), obj_box,
            mapping::LanePath(psmm.semantic_map_manager(), {obj_proj.lane_id},
                              /*start_fraction=*/0.0, /*end_fraction=*/1.0),
            0.0)) {
      continue;
    }

    obj_boxes.push_back(std::move(obj_box));
  }
  if (obj_boxes.empty()) return;

  const auto &sections = sections_info.section_segments();
  absl::flat_hash_map<BasicGraph::VertexId, Vec2d> vertex_pos;
  // If destination is within horizon, leave the last several layers to handle
  // stalled objects on route end that may block all lane paths.
  const int last_layer_idx = target_in_horizon
                                 ? layers.size() - (kVertexNumPerMinLcLen + 1)
                                 : layers.size();
  for (int i = 0; i < last_layer_idx; ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      vertex_pos[vert_id(i, lane_id)] =
          mapping::LanePoint(lane_id, frac)
              .ComputePos(*psmm.semantic_map_manager());
    }
  }

  for (const auto &from_vert : graph->vertices()) {
    if (!vertex_pos.contains(from_vert)) continue;

    for (const auto &[to_vert, edge_cost] : graph->edges_from(from_vert)) {
      if (!vertex_pos.contains(to_vert)) continue;

      const Segment2d edge_seg(vertex_pos[from_vert], vertex_pos[to_vert]);
      for (const auto &box : obj_boxes) {
        if (box.HasOverlap(edge_seg)) {
          graph->ModifyEdge(from_vert, to_vert, edge_cost + kStalledObjectCost);
          break;
        }
      }
    }
  }
}

}  // namespace

LaneGraph BuildLaneGraph(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, const PlannerObjectManager &obj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const AvoidLanes &avoid_lanes) {
  SCOPED_QTRACE("BuildLaneGraph");

  const double extended_local_map = sections_info.planning_horizon() +
                                    kLocalMapExtension +
                                    kExtendedHorizonOverLocalMap;
  const auto &sections = sections_info.section_segments();
  // Find the last section and its end fraction.
  const bool target_in_horizon = sections_info.length() <= extended_local_map;
  int last_sec_idx = 0;
  double last_sec_frac = 0.0;
  if (target_in_horizon) {
    last_sec_idx = sections_info.size() - 1;
    last_sec_frac = sections_info.back().end_fraction;
  } else {
    double accum_len = 0.0;
    for (; last_sec_idx < sections_info.size(); ++last_sec_idx) {
      const auto &section = sections[last_sec_idx];
      if (accum_len + section.length() >= extended_local_map) {
        last_sec_frac =
            section.start_fraction +
            (extended_local_map - accum_len) / section.average_length;
        break;
      }
      accum_len += section.length();
    }
  }

  BasicGraph graph;
  auto layers =
      SplitSectionsAsVertices(sections_info, last_sec_idx, last_sec_frac);
  // Add vertices.
  for (int i = 0; i < layers.size(); ++i) {
    for (const auto lane_id : sections[layers[i].first].lane_ids) {
      graph.AddVertex(vert_id(i, lane_id));
    }
  }

  ConnectEdgesWithinSections(psmm, sections_info, avoid_lanes, layers, &graph);

  absl::flat_hash_set<BasicGraph::EdgeType> fork_edges;
  ConnectEdgesAcrossSections(psmm, sections_info, avoid_lanes, layers,
                             last_sec_idx, &graph, &fork_edges);

  // Project stationary objects onto lane graph and modify edge costs.
  ProjectStationaryObjects(psmm, sections_info, target_in_horizon, obj_mgr,
                           stalled_objects, layers, &graph);

  // Add the target vertex and edges to it.
  graph.AddVertex(LaneGraph::kTargetVertex);
  const auto last_layer = layers.size() - 1;
  if (target_in_horizon) {
    // From the target point.
    graph.AddEdge(vert_id(last_layer, sections_info.destination().lane_id()),
                  LaneGraph::kTargetVertex, 0.0);
  } else {
    // From vertices of the last layer that extends beyond horizon.
    const auto &last_sec = sections[last_sec_idx];
    for (const auto lane_id : last_sec.lane_ids) {
      graph.AddEdge(vert_id(last_layer, lane_id), LaneGraph::kTargetVertex,
                    0.0);
    }
  }

  return LaneGraph{.layers = std::move(layers),
                   .graph = std::move(graph),
                   .fork_edges = std::move(fork_edges)};
}

}  // namespace qcraft::planner
