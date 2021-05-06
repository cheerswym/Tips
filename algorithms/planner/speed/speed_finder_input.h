#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct SpeedFinderInput {
  std::string base_name;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const RouteSections* route_sections_from_start = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const DiscretizedPath* path = nullptr;
  ApolloTrajectoryPointProto plan_start_point;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_INPUT_H_
