#ifndef ONBOARD_PLANNER_FREESPACE_TOB_PATH_SMOOTHER_H
#define ONBOARD_PLANNER_FREESPACE_TOB_PATH_SMOOTHER_H

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<DirectionalPath> SmoothLocalPath(
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const FreespaceLocalSmootherParamsProto &local_smoother_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::string &owner, const FreespaceMap &static_map,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_object_ids,
    const DirectionalPath &input_path, const TrajectoryPoint &plan_start_point,
    bool reset, const std::vector<PathPoint> &prev_path,
    FreespaceLocalSmootherDebugProto *debug_info,
    vis::vantage::ChartsDataProto *charts_data);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_TOB_PATH_SMOOTHER_H
