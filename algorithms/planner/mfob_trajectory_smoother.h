#ifndef ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
#define ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/trajectory_point.h"

namespace qcraft {
namespace planner {

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    const std::vector<TrajectoryPoint> &ref_traj, const std::string &owner,
    const TrajectorySmootherCostWeightParamsProto &smoother_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
