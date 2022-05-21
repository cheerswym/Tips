#ifndef ONBOARD_PERCEPTION_MEASUREMENT_UTIL_H_
#define ONBOARD_PERCEPTION_MEASUREMENT_UTIL_H_

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace measurement_util {

std::vector<bool> GetOccludedMeasurementIndex(
    const VehiclePose& pose,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    const MeasurementsProto& measurements);

bool ShouldRefineLaserDetectionBBox(
    const MeasurementProto& measurement,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    const VehiclePose& pose, bool is_occluded);

void RefineLaserMeasurementBoundingBox(
    const VehiclePose& pose,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    MeasurementProto* measurement);
}  // namespace measurement_util
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_MEASUREMENT_UTIL_H_
