#ifndef ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft {
namespace planner {

struct StandstillDistanceDeciderInput {
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const absl::flat_hash_set<std::string>* stalled_object_ids = nullptr;
  const mapping::LanePath* lane_path = nullptr;  // Could be nullptr.
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
};

// Set standstill distance for st-boundary according to its type and some
// special rules.
void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDeciderInput& input,
    StBoundaryWithDecision* st_boundary_wd);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_STANDSTILL_DISTANCE_DECIDER_H_
