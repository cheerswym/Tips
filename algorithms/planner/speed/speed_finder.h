#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_

#include "absl/status/statusor.h"
#include "onboard/planner/speed/freespace_speed_finder_input.h"
#include "onboard/planner/speed/freespace_speed_finder_output.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"

DECLARE_bool(planner_send_dp_speed_chart_data);
DECLARE_bool(planner_send_speed_path_chart_data);
DECLARE_bool(planner_send_interactive_speed_to_chart);
DECLARE_bool(planner_send_sampling_dp_speed_to_chart);

namespace qcraft {
namespace planner {
absl::StatusOr<SpeedFinderOutput> FindSpeed(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool);

absl::StatusOr<FreespaceSpeedFinderOutput> FindFreespaceSpeed(
    const FreespaceSpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_H_
