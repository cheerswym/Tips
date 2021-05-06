#ifndef ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_
#define ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

// TODO(ping): Delete this file after refactoring speed finder.
struct FreespaceSpeedFinderInput {
  const SemanticMapManager* semantic_map_manager;
  const SpacetimeTrajectoryManager* obj_mgr;
  const ConstraintManager* constraint_mgr;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const DiscretizedPath* path;
  bool forward;
  ApolloTrajectoryPointProto plan_start_point;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_
