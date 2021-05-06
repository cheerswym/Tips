#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_H_

#include <utility>
#include <vector>

#include "onboard/planner/scheduler/lane_graph/basic_graph.h"

namespace qcraft::planner {

struct LaneGraph {
  using LaneGraphLayers = std::vector<std::pair<int, double>>;
  static constexpr auto kTargetVertex = "target";

  // Maps a layer to the corresponding section index and fraction.
  LaneGraphLayers layers;
  BasicGraph graph;
  absl::flat_hash_set<BasicGraph::EdgeType> fork_edges;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_H_
