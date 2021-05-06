#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_DIJKSTRA_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_DIJKSTRA_H_

#include "absl/status/statusor.h"
#include "onboard/planner/scheduler/lane_graph/basic_graph.h"

namespace qcraft::planner {

absl::StatusOr<BasicGraph::TravelPath> Dijkstra(const BasicGraph &graph,
                                                BasicGraph::VertexId src_id,
                                                BasicGraph::VertexId tgt_id);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_DIJKSTRA_H_
