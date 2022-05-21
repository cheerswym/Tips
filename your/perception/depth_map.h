#ifndef ONBOARD_PERCEPTION_DEPTH_MAP_H_
#define ONBOARD_PERCEPTION_DEPTH_MAP_H_

#include "onboard/camera/camera_params.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/image_overlay.pb.h"

namespace qcraft {

// Compute the depth map for the given lidar frame and camera. The vehicle poses
// on the image top/bottom are provided in order to estimate the pose of
// corresponding pixel for each projected point.
void ComputeDepthMap(const LidarFrame& lidar_frame,
                     const LidarExtrinsicsProto& lidar_extrinsics,
                     const CameraParams& camera_params,
                     const VehiclePose& image_top_pose,
                     const VehiclePose& image_bottom_pose,
                     DepthMapProto* depth_map_proto);

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_DEPTH_MAP_H_
