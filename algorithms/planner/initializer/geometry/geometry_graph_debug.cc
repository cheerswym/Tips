#include "onboard/planner/initializer/geometry/geometry_graph_debug.h"

namespace qcraft::planner {

void ParseResampleResultToProto(
    const std::vector<ResampleReason> &resample_result,
    GeometryGraphDebugProto *graph_debug_proto) {
  for (int i = 0, n = resample_result.size(); i < n; i++) {
    auto *new_result = graph_debug_proto->add_resample_result();
    new_result->set_layer_idx(i + 1);
    switch (resample_result[i]) {
      case ResampleReason::RESAMPLED:
        new_result->set_resample_reason(
            GeometryGraphDebugProto_ResampleResult::RESAMPLED);
        break;
      case ResampleReason::NR_ZERO_REACHABLE:
        new_result->set_resample_reason(
            GeometryGraphDebugProto_ResampleResult::NR_ZERO_REACHABLE);
        break;
      case ResampleReason::NR_ALL_REACHABLE:
        new_result->set_resample_reason(
            GeometryGraphDebugProto_ResampleResult::NR_ALL_REACHABLE);
        break;
      case ResampleReason::NR_INVALID_RANGE:
        new_result->set_resample_reason(
            GeometryGraphDebugProto_ResampleResult::NR_INVALID_RANGE);
        break;
      case ResampleReason::NR_LATERAL_RESOLUTION:
        new_result->set_resample_reason(
            GeometryGraphDebugProto_ResampleResult::NR_LATERAL_RESOLUTION);
        break;
      case ResampleReason::NOT_INITIALIZED:
        break;
    }
  }
}

void ParseEdgeDebugInfoToGeometryGraphDebugProto(
    const EdgeDebugInfo &cp, GeometryGraphDebugProto *graph_debug_proto) {
  auto *new_process = graph_debug_proto->add_connection_process();
  auto *start = new_process->mutable_start();
  start->set_layer_index(cp.start_layer_idx);
  start->set_node_on_layer_index(cp.start_node_on_layer_idx);
  start->set_station_index(cp.start_station_idx);

  auto *end = new_process->mutable_end();
  end->set_layer_index(cp.end_layer_idx);
  end->set_node_on_layer_index(cp.end_node_on_layer_idx);
  end->set_station_index(cp.end_station_idx);
}

std::string ConstraintAccumSDebug(double constraint_stop_sampling_s,
                                  double max_deceleration_accumulated_s,
                                  double leading_obj_caused_target_s,
                                  double stopline_accumulated_s) {
  std::string result =
      "\nconstraint_stop_sampling_s\tmax_deceleration_accumulated_s\tleading_"
      "obj_cause_target_s\tstopline_accumulated_s\n";
  result += absl::StrCat(
      constraint_stop_sampling_s, "\t", max_deceleration_accumulated_s, "\t",
      leading_obj_caused_target_s, "\t", stopline_accumulated_s);
  return result;
}

absl::Status CheckGeometryGraphConnectivity(const GeometryGraph &graph) {
  const auto &nodes = graph.nodes();
  // Check reachable and outgoing edges of the nodes should comply.
  for (const auto &node : nodes) {
    const auto &outgoing_edges_per_node = graph.GetOutgoingEdges(node.index);
    if (outgoing_edges_per_node.size() > 0) {
      if (!node.reachable) {
        std::string result = absl::StrFormat(
            "Node %d has non-zero outgoing edges but is recorded as "
            "unreachable. It is resampled? %d.",
            node.index.value(), node.resampled);
        return absl::InternalError(result);
      }
    }
  }

  // Check query node effect.
  for (int i = 0; i < graph.nodes_layers().size(); ++i) {
    const auto &layer = graph.nodes_layers()[i];
    for (const auto &node_idx : layer) {
      const auto &node = graph.GetNode(node_idx);
      const auto &outgoing_edges_per_node = graph.GetOutgoingEdges(node_idx);
      if (outgoing_edges_per_node.size() > 0) {
        if (!node.reachable) {
          std::string result = absl::StrFormat(
              "From nodes_layers: Node %d has non-zero outgoing edges but is "
              "recorded as unreachable. It is resampled? %d. It is at layer "
              "idx: %d",
              node.index.value(), node.resampled, i);
          return absl::InternalError(result);
        }
      }
    }
  }
  return absl::OkStatus();
}
}  // namespace qcraft::planner
