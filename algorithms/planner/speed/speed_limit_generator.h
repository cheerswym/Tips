#ifndef ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

#include <tuple>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

absl::flat_hash_map<SpeedFinderParamsProto::SpeedLimitType, SpeedLimit>
GetSpeedLimitMap(
    const DiscretizedPath& discretized_points, double max_speed_limit,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage& drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedFinderParamsProto::SpeedLimitParamsProto& speed_limit_config);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_
