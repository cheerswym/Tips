#ifndef ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_CLEARANCE_H_
#define ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_CLEARANCE_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/math/rss_formulas.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

// RSS constants.
const PiecewiseLinearFunction kEgoMinBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{1.2, 1.7, 2.5});
const PiecewiseLinearFunction kEgoMaxBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.95, 1.2, 1.4});
const PiecewiseLinearFunction kVehicleMaxBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.95, 0.8, 0.6});

absl::StatusOr<ClearanceCheckOutput> CheckTargetLaneClearance(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePath &target_lane_path,
    const ApolloTrajectoryPointProto &plan_start_point,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerParamsProto &planner_params, double aggr_factor,
    ThreadPool *thread_pool);

ClearanceCheckOutput::ObjectDecision DecideClearanceForMovingObject(
    const RssLongitudialFormulas::VehicleState &ego_state,
    const RssLongitudialFormulas::VehicleState &obj_state, double aggr_factor);

double ComputeObjectResponseTime(double aggr, double accel);

double ComputeVehicleMinBrake(double v, double aggr);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_CLEARANCE_H_
