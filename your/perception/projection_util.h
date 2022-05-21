#ifndef ONBOARD_PERCEPTION_PROJECTION_UTIL_H_
#define ONBOARD_PERCEPTION_PROJECTION_UTIL_H_

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "onboard/camera/camera_params.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/camera.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::projection_util {

// Finds mid timestamp of ref lidar and return average mid timestamp if no ref
// lidar.
// TODO(fengyang): remove this function
double FindRefLidarMidTimestamp(const CameraParams& camera_param,
                                const std::vector<LidarFrame>& lidar_frames);

AffineTransformation GetSmoothToCameraCoordTransform(
    const CameraParams& camera_param,
    const LidarParametersProto& ref_lidar_params,
    const AffineTransformation& pose_transform);

AffineTransformation GetSmoothToCameraCoordTransform(
    const CameraParams& camera_param,
    const LidarParametersProto& ref_lidar_params, const VehiclePose& pose);

// Convert a point in smooth coordinate into image position. Return nullopt if
// the point falls outside of the image.
std::optional<Vec2i> SmoothPointToImagePos(
    const Vec3d& point, const AffineTransformation& smooth_to_camera_transform,
    const CameraParams& camera_params, bool out_of_image_coord = false);

// Same as above but point provided is in camera frame.
std::optional<Vec2i> CameraPointToImagePos(const Vec3d& point,
                                           const CameraParams& camera_params,
                                           bool out_of_image_coord);

// Inverse perspective mapping. With flat world assumption, convert image
// position to point in vehicle frame
Vec3d ImagePosToVehiclePoint(const Vec2i& pos,
                             const CameraParams& camera_params);

Vec3d GetLidarViewPointInVehicleCoord(
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params);

std::optional<AffineTransformation> FindClosestLocalToCameraTransformation(
    const boost::circular_buffer<CameraImageWithTransform>&
        camera_image_with_t_buffer,
    const double target_trigger_timestamp);

}  // namespace qcraft::projection_util

#endif  // ONBOARD_PERCEPTION_PROJECTION_UTIL_H_
