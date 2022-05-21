#include "onboard/perception/utils/sensor_view_utils.h"

namespace qcraft {

bool IsCameraAndLidarViewMatch(const CameraId& camera_id,
                               const LidarId& lidar_id) {
  if (lidar_id == LDR_CENTER) {
    return true;
  }
  if ((camera_id == CAM_L_FRONT_LEFT || camera_id == CAM_L_FRONT ||
       camera_id == CAM_L_FRONT_RIGHT) &&
      (lidar_id == LDR_FRONT_LEFT || lidar_id == LDR_FRONT_LEFT_BLIND)) {
    return true;
  }
  if ((camera_id == CAM_R_FRONT_LEFT || camera_id == CAM_R_FRONT ||
       camera_id == CAM_R_FRONT_RIGHT) &&
      (lidar_id == LDR_FRONT_RIGHT || lidar_id == LDR_FRONT_RIGHT_BLIND)) {
    return true;
  }
  return false;
}
}  // namespace qcraft
