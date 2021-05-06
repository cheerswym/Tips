#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {

struct MotionSearchOutput {
  // TODO(lidong): Add returning a list of motion forms and Edge indices.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  MotionEdgeIndex best_last_edge_index;

  struct SearchCost {
    std::vector<double> feature_cost;  // Accumulated feature cost
    double cost_to_come = 0.0;
    double TotalCost() const { return cost_to_come; }
  };

  MotionEdgeVector<SearchCost> cost;
  double min_cost;
  std::vector<std::string> leading_objs;

  std::unique_ptr<MotionGraph> motion_graph;

  // Geometries from sdc pose to geometry graph.
  std::vector<std::unique_ptr<GeometryForm>> start_geometries;

  std::unique_ptr<RefSpeedTable> ref_speed_table;

  std::unique_ptr<CostProvider> cost_provider;

  // Debug only (FLAGS_planner_initializer_debug_level >= 2)
  // Optimal trajectory (among all the leading objs)
  std::vector<MotionEdgeIndex> search_queue;
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<MotionEdgeIndex> top_k_edges;

  // Trajectories with multiple leading objects.
  struct TrajWithLeadingObject {
    std::vector<ApolloTrajectoryPointProto> trajectory;
    std::vector<std::string> leading_obj_ids;
    // The rest appplies when FLAGS_planner_initializer_debug_level >= 2.
    double total_cost;
    MotionEdgeIndex last_edge_index;
    std::vector<double> feature_costs;
    double final_cost;
  };
  int choice;
  std::vector<TrajWithLeadingObject> trajs_with_lead_obj;

  // Data Dumping only
  struct TrajectoryEvaluationDumping {
    double weighted_total_cost;  // weighted by onboard initializer's weights
    std::vector<double> dumped_weights;  // onboard initializer's weights
    std::vector<double> feature_costs;   // unweighted
    std::vector<ApolloTrajectoryPointProto> traj;
  };
  TrajectoryEvaluationDumping expert_evaluation;
  std::vector<TrajectoryEvaluationDumping> candidates_evaluation;
};

struct InitializerOutput {
  MotionSearchOutput search_result;
  InitializerStateProto initializer_state;
  std::unique_ptr<ConstraintProto::LeadingObjectProto> blocking_static_obj =
      nullptr;
  std::optional<std::vector<ConstraintProto::LeadingObjectProto>> lc_targets =
      std::nullopt;
};

struct ReferenceLineSearcherOutput {
  struct NodeInfo {
    double min_cost = std::numeric_limits<double>::infinity();
    GeometryEdgeIndex outgoing_edge_idx =
        GeometryEdgeVector<GeometryEdge>::kInvalidIndex;
    GeometryNodeIndex prev_node_idx =
        GeometryNodeVector<GeometryNode>::kInvalidIndex;
  };

  struct EdgeInfo {
    std::vector<double> feature_costs;
    double sum_cost = 0.0;
  };

  std::vector<GeometryNodeIndex> nodes_list;
  std::vector<GeometryEdgeIndex> edges_list;
  std::vector<PathPoint> ref_line_points;
  double total_cost = 0.0;
  std::vector<double> feature_costs;

  std::unique_ptr<RefLineCostProvider> ptr_cost_provider;
  GeometryEdgeVector<EdgeInfo> cost_edges;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
