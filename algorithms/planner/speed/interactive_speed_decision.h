#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "onboard/async/thread_pool.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/decider/post_st_boundary_modifier.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/speed_profile.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

absl::Status MakeInteractiveSpeedDecision(
    const VehicleGeometryParamsProto& vehicle_geom,
    const MotionConstraintParamsProto& motion_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpeedLimit& speed_limit, const DiscretizedPath& path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps, SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
