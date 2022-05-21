#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_TEST_UTIL_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_TEST_UTIL_H_

#include <memory>
#include <utility>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/sensor_fov/sensor_fov_builder.h"

namespace qcraft::sensor_fov {

std::unique_ptr<SensorFovBuilder> BuildSensorFovBuilder();

std::pair<SegmentedClusters, ObstacleRefVector> BuildSegmentedClusters(
    const VehiclePose& pose = VehiclePose());

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_TEST_UTIL_H_
