#ifndef ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_

#include <vector>

#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct IgnoreDeciderInput {
  const std::vector<StPoint>* emergency_stop_points = nullptr;
  const DiscretizedPath* path = nullptr;
  // Could be empty but not null.
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
};

void MakeIgnoreDecisionForStBoundary(const IgnoreDeciderInput& input,
                                     StBoundaryWithDecision* st_boundary_wd);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_
