#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_

#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_output.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput& input,
    const PlannerParamsProto& planner_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    TrajectoryOptimizerDebugProto* optimizer_debug,
    vis::vantage::ChartDataBundleProto* charts_data, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
