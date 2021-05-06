#include "onboard/planner/scheduler/lane_graph/dijkstra.h"

#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace qcraft::planner {

namespace {
using VertexId = BasicGraph::VertexId;

std::vector<VertexId> RebuildPath(
    const absl::flat_hash_map<VertexId, VertexId> &prev, VertexId goal_id) {
  std::vector<VertexId> path;
  path.push_back(goal_id);
  while (prev.at(goal_id) != BasicGraph::kInvalidVertexId) {
    path.push_back(goal_id = prev.at(goal_id));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

using CostIdPair = std::pair<double, VertexId>;

}  // namespace

absl::StatusOr<BasicGraph::TravelPath> Dijkstra(const BasicGraph &graph,
                                                VertexId src_id,
                                                VertexId goal_id) {
  if (!graph.has_vertex(src_id)) {
    return absl::NotFoundError(
        absl::StrCat("Source vertex ", src_id, " does not exist!"));
  }
  if (!graph.has_vertex(goal_id)) {
    return absl::NotFoundError(
        absl::StrCat("Target vertex ", goal_id, " does not exist!"));
  }

  absl::flat_hash_map<VertexId, double> cost;
  absl::flat_hash_map<VertexId, VertexId> prev;
  for (const auto &vert_id : graph.vertices()) {
    cost[vert_id] = BasicGraph::kInfiniteCost;
    prev[vert_id] = BasicGraph::kInvalidVertexId;
  }
  cost[src_id] = 0.0;

  std::priority_queue<CostIdPair, std::vector<CostIdPair>,
                      std::greater<CostIdPair>>
      q;
  q.push({0.0, src_id});
  while (!q.empty()) {
    const auto [cur_cost, cur_id] = q.top();
    q.pop();
    if (cost[cur_id] != cur_cost) continue;  // Not the first visit.
    if (cur_id == goal_id) {
      return BasicGraph::TravelPath{.vertices = RebuildPath(prev, goal_id),
                                    .total_cost = cur_cost};
    }

    for (const auto &[to_id, edge_cost] : graph.edges_from(cur_id)) {
      double new_cost = cur_cost + edge_cost;
      if (new_cost < cost[to_id]) {
        cost[to_id] = new_cost;
        prev[to_id] = cur_id;
        q.push({new_cost, to_id});
      }
    }
  }

  return absl::NotFoundError("Path not found!");
}

}  // namespace qcraft::planner
