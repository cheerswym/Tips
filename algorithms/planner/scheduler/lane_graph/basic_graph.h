#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_BASIC_GRAPH_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_BASIC_GRAPH_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "onboard/lite/logging.h"

namespace qcraft::planner {

class BasicGraph {
 public:
  using VertexId = std::string;
  using EdgeType = std::pair<VertexId, VertexId>;
  static constexpr auto kInvalidVertexId = "";
  static constexpr double kInfiniteCost = std::numeric_limits<double>::max();

  struct TravelPath {
    std::vector<VertexId> vertices;
    double total_cost = kInfiniteCost;
  };

  // Maps to_id to edge cost.
  using VertexEdgeMap = absl::flat_hash_map<VertexId, double>;
  using VertexSet = absl::flat_hash_set<VertexId>;

 public:
  void AddVertex(const VertexId &vert_id) {
    if (vertices_.contains(vert_id)) {
      QLOG(WARNING) << vert_id << " already exists!";
      return;
    }
    vertices_.insert(vert_id);
    edges_.emplace(vert_id, VertexEdgeMap());
  }
  void AddEdge(const VertexId &from_id, const VertexId &to_id, double cost) {
    if (!vertices_.contains(from_id) || !vertices_.contains(to_id)) {
      QLOG(WARNING) << absl::StrFormat(
          "Edge (%s, %s): please insert vertices first", from_id, to_id);
      return;
    }
    if (cost == kInfiniteCost) return;

    if (edges_.at(from_id).contains(to_id)) {
      edges_[from_id][to_id] = std::min(edges_.at(from_id).at(to_id), cost);
    } else {
      edges_[from_id].emplace(to_id, cost);
    }
  }
  void RemoveEdge(const VertexId &from_id, const VertexId &to_id) {
    if (!IsEdgeValid(from_id, to_id)) {
      QLOG(WARNING) << absl::StrFormat("Invalid edge (%s, %s)!", from_id,
                                       to_id);
      return;
    }
    edges_[from_id].erase(to_id);
  }
  void ModifyEdge(const VertexId &from_id, const VertexId &to_id, double cost) {
    if (!IsEdgeValid(from_id, to_id)) {
      QLOG(WARNING) << absl::StrFormat("Invalid edge (%s, %s)!", from_id,
                                       to_id);
      return;
    }
    edges_[from_id][to_id] = cost;
  }

  bool has_vertex(const VertexId &vert_id) const {
    return vertices_.contains(vert_id);
  }
  double edge_cost(const VertexId &from_id, const VertexId &to_id) const {
    if (IsEdgeValid(from_id, to_id)) return edges_.at(from_id).at(to_id);
    return kInfiniteCost;
  }
  const VertexSet &vertices() const { return vertices_; }
  const VertexEdgeMap &edges_from(const VertexId &from_id) const {
    QCHECK(vertices_.contains(from_id)) << "Invalid vertex id " << from_id;
    return edges_.at(from_id);
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << vertices_.size() << " vertices in total\n";
    for (const auto &vertex : vertices_) {
      ss << "  " << vertex << ":\n";
      for (const auto &[to_id, cost] : edges_.at(vertex)) {
        ss << "    edge to " << to_id << " with cost " << cost << std::endl;
      }
    }
    return ss.str();
  }

 private:
  bool IsEdgeValid(const VertexId &from_id, const VertexId &to_id) const {
    return vertices_.contains(from_id) && vertices_.contains(to_id) &&
           edges_.at(from_id).contains(to_id);
  }

  VertexSet vertices_;
  absl::flat_hash_map<VertexId, VertexEdgeMap> edges_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_BASIC_GRAPH_H_
