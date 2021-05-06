#ifndef ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_

#include <memory>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
struct SpacetimeTrajectoryManagerBuilderInput {
  const DrivePassage *passage;
  const PathSlBoundary *sl_boundary;
  const PlannerObjectManager *obj_mgr;
  const VehicleGeometryParamsProto *veh_geom;
  const ApolloTrajectoryPointProto *plan_start_point;
  int st_planner_trajectories_start_index;
  const LaneChangeStateProto *lane_change_state;
  const SpacetimePlannerTrajectories *prev_st_trajs;
};

std::unique_ptr<SpacetimeTrajectoryManager> BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput &input,
    ThreadPool *thread_pool);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
