#ifndef ONBOARD_PERCEPTION_UTILS_SENSOR_VIEW_UTILS_H_
#define ONBOARD_PERCEPTION_UTILS_SENSOR_VIEW_UTILS_H_

#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

bool IsCameraAndLidarViewMatch(const CameraId& camera_id,
                               const LidarId& lidar_id);

}  // namespace qcraft
#endif  // ONBOARD_PERCEPTION_UTILS_SENSOR_VIEW_UTILS_H_
