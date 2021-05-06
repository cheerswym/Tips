#ifndef ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_
#define ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_join.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/math/rss_formulas.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/target_lane_clearance.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct SingleTrajDebugInfo {
  std::vector<MotionEdgeIndex> search_queue;
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<MotionEdgeIndex> top_k_edges;
};

struct SingleTrajInfo {
  // Different trajectory should be determined by different grouping of leading
  // objects based on lane change state. Need logic to choose leading objects
  // while changing lanes.
  std::vector<std::string> lead_objs;  // Leading objects' traj_ids.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  MotionEdgeIndex last_edge_index;
  MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;
  double total_cost;
  std::unique_ptr<MotionGraph> motion_graph;
  std::unique_ptr<CostProvider> cost_provider;

  SingleTrajDebugInfo debug_info;
  std::string GetLeadingObjTrajId() const {
    if (lead_objs.empty()) {
      return "No leading objects.";
    } else {
      return absl::StrJoin(lead_objs, ", ");
    }
  }
};

std::vector<InitializerSearchConfig> BuildSearchConfig(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ConstraintManager& constraint_manager,
    const std::optional<ClearanceCheckOutput>* lc_clearance,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ConstraintProto::LeadingObjectProto* blocking_static_obj,
    const GeometryGraph& geom_graph,
    const VehicleGeometryParamsProto& vehicle_geom, bool is_lane_change,
    bool lc_left, bool lc_multiple_traj);

int EvaluateMultiTrajs(const SpacetimeTrajectoryManager* st_traj_mgr,
                       const DrivePassage* passage,
                       const PlannerParamsProto& planner_params,
                       const std::vector<SingleTrajInfo>& multi_trajs,
                       const std::optional<ClearanceCheckOutput>* lc_clearance,
                       const VehicleGeometryParamsProto* vehicle_geom,
                       ThreadPool* thread_pool,
                       InitializerDebugProto* debug_proto);

}  // namespace qcraft::planner

#endif
