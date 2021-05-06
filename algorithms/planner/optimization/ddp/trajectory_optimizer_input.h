#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_

#include <vector>

#include "absl/types/span.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct TrajectoryOptimizerInput {
  absl::Span<const ApolloTrajectoryPointProto> trajectory;
  absl::Span<const ApolloTrajectoryPointProto> previous_trajectory;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_mgr = nullptr;
  ApolloTrajectoryPointProto plan_start_point;
  absl::Time plan_start_time;
  int plan_id = 0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
