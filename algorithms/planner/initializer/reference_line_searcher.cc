#include "onboard/planner/initializer/reference_line_searcher.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft::planner {

namespace {
constexpr double kActivationBoundaryWidthRatio = 0.25;  // (LaneKeeping)
constexpr int kDownSampleStep = 5;

std::vector<double> AddCost(absl::Span<const double> vec1,
                            absl::Span<const double> vec2) {
  const int vec1_size = vec1.size();
  std::vector<double> res(vec1_size);
  QCHECK_EQ(vec1_size, vec2.size());
  for (int i = 0; i < vec1_size; ++i) {
    res[i] = vec1[i] + vec2[i];
  }
  return res;
}

void ComputeEdgeCost(
    const RefLineCostProvider* cost_provider, const GeometryEdge& edge,
    bool terminating,
    ReferenceLineSearcherOutput::EdgeInfo* ptr_edge_cost_info) {
  auto& edge_cost_info = *ptr_edge_cost_info;
  edge_cost_info.feature_costs.resize(cost_provider->weights().size());
  cost_provider->ComputeRefLineCost(
      edge.geometry, terminating, absl::MakeSpan(edge_cost_info.feature_costs));
}

void Initialize(
    const GeometryGraph& graph,
    GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo>* ptr_cost_nodes) {
  // If the node has no outgoing edges, it means that
  // it is a terminating node on the graph.
  auto& cost_nodes = *ptr_cost_nodes;
  for (const auto& node : graph.nodes()) {
    if (!node.reachable) {
      VLOG(5) << node.index.value() << " unreachable";
      continue;  // Ignore nodes that are not reachable.
    }
    const auto& outgoing_edges = graph.GetOutgoingEdges(node.index);
    if (outgoing_edges.size() == 0) {
      VLOG(5) << "set " << node.index.value() << "min_cost to 0.0";
      cost_nodes[node.index].min_cost = 0.0;
    }
  }
}

absl::Status BuildConnection(
    const GeometryGraph& graph,
    const GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo>& cost_nodes,
    const GeometryEdgeVector<ReferenceLineSearcherOutput::EdgeInfo>& cost_edges,
    const RefLineCostProvider& cost_provider,
    std::vector<GeometryNodeIndex>* ptr_nodes_list,
    std::vector<GeometryEdgeIndex>* ptr_edges_list,
    std::vector<double>* ptr_feature_costs) {
  const auto& start_node = graph.GetStartNode();
  GeometryNodeIndex cur_node_idx = start_node.index;
  auto& nodes = *ptr_nodes_list;
  auto& edges = *ptr_edges_list;
  auto& feature_costs = *ptr_feature_costs;
  feature_costs.resize(cost_provider.cost_names().size());
  std::fill(feature_costs.begin(), feature_costs.end(), 0.0);
  while (cur_node_idx != GeometryNodeVector<GeometryNode>::kInvalidIndex) {
    nodes.push_back(cur_node_idx);
    const auto& node_info = cost_nodes[cur_node_idx];
    const auto& edge_idx = node_info.outgoing_edge_idx;
    if (edge_idx != GeometryEdgeVector<GeometryEdge>::kInvalidIndex) {
      edges.push_back(edge_idx);
      feature_costs =
          AddCost(feature_costs, cost_edges[edge_idx].feature_costs);
    }
    cur_node_idx = node_info.prev_node_idx;
    if (nodes.size() > graph.nodes_layers().size()) {
      return absl::InternalError(
          "Linkage of nodes has size greater than layer size of geometry "
          "graph, linkage error! Check for loops.");
    }
  }
  return absl::OkStatus();
}

std::vector<PathPoint> GenerateRefLinePoints(
    const GeometryGraph* graph,
    const std::vector<GeometryEdgeIndex>& edges_list) {
  std::vector<PathPoint> ref_line_points;
  for (const auto& edge_idx : edges_list) {
    if (edge_idx == GeometryEdgeVector<GeometryEdge>::kInvalidIndex) continue;
    const auto& edge = graph->GetEdge(edge_idx);
    const auto* geom_form = edge.geometry;
    const auto states = geom_form->Sample(0.5);
    for (const auto& state : states) {
      PathPoint point;
      point.set_x(state.xy.x());
      point.set_y(state.xy.y());
      point.set_z(0);
      point.set_theta(state.h);
      point.set_kappa(state.k);
      point.set_s(0.0);
      ref_line_points.emplace_back(std::move(point));
    }
  }
  return ref_line_points;
}

PiecewiseLinearFunction<double, double> GetActivationRadiusPlf(
    const PathSlBoundary& path_sl, const GeometryGraph& geom_graph,
    const std::vector<GeometryNodeIndex>& node_idx_list) {
  // Generate Plf: accumulated_s <-> activation radius (for each layer) since sl
  // boundary width can be large within intersection.
  std::vector<double> accum_s;
  std::vector<double> activate_r;
  accum_s.reserve(node_idx_list.size());
  activate_r.reserve(node_idx_list.size());

  for (const auto& index : node_idx_list) {
    const auto& node = geom_graph.GetNode(index);
    accum_s.push_back(node.accumulated_s);
    // <right, left>
    const auto boundary_l = path_sl.QueryBoundaryL(node.accumulated_s);
    // Open up one fourth of the width with respect to the outer boundary.
    const double activate_radius = 0.5 * kActivationBoundaryWidthRatio *
                                   (boundary_l.second - boundary_l.first);
    activate_r.push_back(activate_radius);
  }
  VLOG(1) << "Activate width plf:";
  VLOG(1) << "accum_s: " << absl::StrJoin(accum_s, ", ");
  VLOG(1) << "r: " << absl::StrJoin(activate_r, ", ");

  return PiecewiseLinearFunction<double, double>(accum_s, activate_r);
}

}  // namespace

absl::StatusOr<ReferenceLineSearcherOutput> SearchReferenceLine(
    const ReferenceLineSearcherInput& input, InitializerDebugProto* debug_proto,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("SearchReferenceLine");

  QCHECK_NOTNULL(input.geometry_graph);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.planner_params);
  QCHECK_NOTNULL(input.vehicle_drive);
  QCHECK_NOTNULL(input.vehicle_geom);

  const GeometryGraph& graph = *input.geometry_graph;
  const SpacetimeTrajectoryManager& st_traj_mgr = *input.st_traj_mgr;
  const DrivePassage& drive_passage = *input.drive_passage;
  const PlannerParamsProto& planner_params = *input.planner_params;
  const PathSlBoundary& path_sl = *input.sl_boundary;
  const VehicleDriveParamsProto& vehicle_drive = *input.vehicle_drive;
  const VehicleGeometryParamsProto& vehicle_geom = *input.vehicle_geom;

  // Create reference line cost provider.
  const double relaxed_center_max_curvature =
      GetRelaxedCenterMaxCurvature(vehicle_geom, vehicle_drive);

  std::unique_ptr<RefLineCostProvider> cost_provider =
      std::make_unique<RefLineCostProvider>(
          &st_traj_mgr, &drive_passage, &path_sl, graph.GetMaxAccumulatedS(),
          relaxed_center_max_curvature, planner_params);

  // Start from last layer of geometry graph. Update min_cost to each node layer
  // by layer.
  GeometryNodeVector<ReferenceLineSearcherOutput::NodeInfo> cost_nodes;
  GeometryEdgeVector<ReferenceLineSearcherOutput::EdgeInfo> cost_edges;
  cost_nodes.resize(graph.nodes().size());
  cost_edges.resize(graph.edges().size());
  Initialize(graph, &cost_nodes);
  // Searching.
  const auto& nodes_layers = graph.nodes_layers();
  for (int layer_idx = nodes_layers.size() - 1; layer_idx >= 0; --layer_idx) {
    VLOG(5) << "layer_idx: " << layer_idx;
    const auto& nodes_on_cur_layer = nodes_layers[layer_idx];

    for (const auto& node_idx : nodes_on_cur_layer) {
      // Calculate all outgoing edge costs for this node.
      VLOG(5) << ">>>>>>>>> node_idx: " << node_idx.value();
      const auto& geom_node = graph.GetNode(node_idx);
      if (!geom_node.reachable) {
        VLOG(5) << "should be unreachable?";
        continue;  // Ignore unreachable nodes.
      }
      const auto& outgoing_edges_node = graph.GetOutgoingEdges(node_idx);

      for (int i = 0; i < outgoing_edges_node.size(); ++i) {
        const auto& edge_idx = outgoing_edges_node[i];
        const auto& edge = graph.GetEdge(edge_idx);
        const auto& next_edges = graph.GetOutgoingEdges(edge.end);
        const bool terminating = next_edges.size() == 0 ? true : false;
        ReferenceLineSearcherOutput::EdgeInfo cost_info;
        ComputeEdgeCost(cost_provider.get(), edge, terminating, &cost_info);
        cost_info.sum_cost = absl::c_accumulate(cost_info.feature_costs, 0.0);
        cost_edges[edge_idx] = std::move(cost_info);
      }

      // Update min_cost for this node.
      for (const auto& edge_idx : outgoing_edges_node) {
        const auto& edge_sum_cost = cost_edges[edge_idx].sum_cost;
        const auto& edge = graph.GetEdge(edge_idx);
        const auto& prev_node_cost_info = cost_nodes[edge.end];
        VLOG(5) << ">>> edge: " << edge_idx.value()
                << " has cost: " << edge_sum_cost;
        if (std::isinf(prev_node_cost_info.min_cost)) {
          VLOG(5) << edge.end.value()
                  << " node has inf cost, terminating edge idx "
                  << edge_idx.value() << "edge_sum_cost:" << edge_sum_cost;
          return absl::InternalError(
              "prev_node_cost infinity, initialization on geometry node cost "
              "might have failed!");
        }
        const double cost_through_this_edge =
            edge_sum_cost + prev_node_cost_info.min_cost;
        if (cost_through_this_edge < cost_nodes[node_idx].min_cost) {
          // Update current node cost infos.

          VLOG(5) << "prev_node: " << edge.end.value() << "has min_cost"
                  << prev_node_cost_info.min_cost;
          VLOG(5) << "cost through this edge: " << cost_through_this_edge;
          VLOG(5) << "cur node min cost : " << cost_nodes[node_idx].min_cost
                  << " so update.";

          auto& cur_node_cost_info = cost_nodes[node_idx];
          cur_node_cost_info.min_cost = cost_through_this_edge;
          cur_node_cost_info.outgoing_edge_idx = edge_idx;
          cur_node_cost_info.prev_node_idx = edge.end;
        }
      }
    }
  }

  // Build output.
  std::vector<GeometryNodeIndex> nodes_list;
  std::vector<GeometryEdgeIndex> edges_list;
  std::vector<double> feature_costs;

  RETURN_IF_ERROR(BuildConnection(graph, cost_nodes, cost_edges, *cost_provider,
                                  &nodes_list, &edges_list, &feature_costs));
  const auto ref_line_points = GenerateRefLinePoints(&graph, edges_list);

  if (ref_line_points.size() <= kDownSampleStep) {
    return absl::NotFoundError(
        "Reference line points size less than down sample size.");
  }

  const double min_cost = cost_nodes[nodes_list.front()].min_cost;
  ReferenceLineSearcherOutput search_result{
      .nodes_list = std::move(nodes_list),
      .edges_list = std::move(edges_list),
      .ref_line_points = std::move(ref_line_points),
      .total_cost = min_cost,
      .feature_costs = std::move(feature_costs),
      .ptr_cost_provider = std::move(cost_provider),
      .cost_edges = std::move(cost_edges),
  };
  return search_result;
}

absl::Status ActivateGeometryGraph(
    const ReferenceLineSearcherOutput& search_result,
    const PathSlBoundary& path_sl, GeometryGraph* mutable_geometry_graph) {
  SCOPED_QTRACE("ActivateGeometryGraph");
  const auto& ref_points = search_result.ref_line_points;
  std::vector<Vec2d> downsampled_ref_points;
  downsampled_ref_points.reserve(round(ref_points.size() / kDownSampleStep));

  for (int i = 0; i < ref_points.size(); i += kDownSampleStep) {
    i = std::min(i, static_cast<int>(ref_points.size()) - 1);
    const auto& p = ref_points[i];
    downsampled_ref_points.emplace_back(p.x(), p.y());
  }
  ASSIGN_OR_RETURN(const auto ref_frame,
                   BuildBruteForceFrenetFrame(downsampled_ref_points));

  // Get Plf.
  const auto accum_s_r_plf = GetActivationRadiusPlf(
      path_sl, *mutable_geometry_graph, search_result.nodes_list);

  int activated_count = 0;
  for (const auto& node : mutable_geometry_graph->nodes()) {
    const auto l = std::fabs(ref_frame.XYToSL(node.xy).l);
    if (l < accum_s_r_plf(node.accumulated_s)) {
      mutable_geometry_graph->ActivateNode(node.index);
      activated_count++;
    }
  }

  if (activated_count <= search_result.nodes_list.size()) {
    return absl::InternalError(
        "Cannot activate extra nodes according to this reference line.");
  }

  // Deactivate geometry edges if the start node or the end node is not active.
  for (const auto& edge : mutable_geometry_graph->edges()) {
    const auto& start_idx = edge.start;
    const auto& end_idx = edge.end;
    if (!mutable_geometry_graph->IsActive(start_idx) ||
        !mutable_geometry_graph->IsActive(end_idx)) {
      mutable_geometry_graph->DeactivateEdge(edge.index);
    }
  }

  return absl::OkStatus();
}

void ParseReferenceLineResultToProto(const ReferenceLineSearcherOutput& result,
                                     GeometryGraphProto* graph_proto) {
  auto* proto = graph_proto->mutable_reference_line();
  for (const auto& node_idx : result.nodes_list) {
    proto->add_node_idxes(node_idx.value());
  }
  for (const auto& edge_idx : result.edges_list) {
    proto->add_edge_idxes(edge_idx.value());
  }
  for (const auto& name : result.ptr_cost_provider->cost_names()) {
    proto->add_cost_names(name);
  }
  proto->set_total_cost(result.total_cost);
  for (const auto& feature_cost : result.feature_costs) {
    proto->add_feature_costs(feature_cost);
  }

  if (FLAGS_planner_initializer_debug_level >= 2) {
    for (const auto& point : result.ref_line_points) {
      auto* new_point = proto->add_ref_line_points();
      *new_point = point;
    }
    const auto& cost_edges = result.cost_edges;
    for (const auto index : result.cost_edges.index_range()) {
      const auto& edge_info = cost_edges[index];
      auto* new_edge_costs = proto->add_edge_costs();
      GeometryGraphProto::ReferenceLine::EdgeCost info_proto;
      info_proto.set_edge_idx(index.value());
      for (const auto& feature_cost : edge_info.feature_costs) {
        info_proto.add_feature_costs(feature_cost);
      }
      info_proto.set_total_cost(edge_info.sum_cost);
      *new_edge_costs = std::move(info_proto);
    }
  }
}
}  // namespace qcraft::planner
