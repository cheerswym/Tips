#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/perception/sensor_fov/sensor_fov_grid.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::sensor_fov {

class SensorFov;
using SensorFovs = std::vector<SensorFov>;
using SensorFovPtr = const SensorFov*;
using SensorFovPtrs = std::vector<SensorFovPtr>;
using SensorFovRef = std::shared_ptr<SensorFov>;
using SensorFovRefs = std::vector<SensorFovRef>;

class SensorFov {
  friend class SensorFovBuilder;

 public:
  static SensorFovProto SensorFovToProto(const SensorFov& sensor_fov);
  static SensorFov ProtoToSensorFov(const SensorFovProto& proto);

  explicit SensorFov(const SensorFovProto& proto)
      : SensorFov(ProtoToSensorFov(proto)) {}
  // Not default constructible.
  SensorFov() = delete;
  // NOTE(dong): Point should be in the smooth coordiante.
  absl::StatusOr<bool> IsOccluded(const Vec2d& point, float height) const;
  // NOTE(dong): Box should be in the smooth coordinate.
  absl::StatusOr<bool> IsOccluded(const Box2d& box, float height) const;
  // NOTE(dong): Polygon should be a convex polygon in the smooth coordinate.
  absl::StatusOr<bool> IsOccluded(const Polygon2d& polygon, float height) const;
  // TODO(dong): Not implement yet.
  std::vector<Polygon2d> ComputeAllBlindZones(float height);

  SensorFovViewType view_type() const { return view_type_; }
  // Only vaild if view type is VT_CAMERA.
  CameraId camera_id() const;

 private:
  SensorFov(const double range_front, const double range_rear,
            const double range_left, const double range_right,
            const double diameter, const SensorFovViewType view_type,
            const std::map<std::string, Vec3d>& ref_sensors,
            const VehiclePoseWithTimestamp& pose)
      : sf_grid_(range_front, range_rear, range_left, range_right, diameter),
        view_type_(view_type),
        ref_sensors_(ref_sensors),
        pose_(pose),
        vehicle_to_smooth_(pose_.vehicle_pose.ToTransform()),
        smooth_to_vehicle_(vehicle_to_smooth_.Inverse()) {
    if (view_type_ == VT_CAMERA) {
      QCHECK_EQ(ref_sensors_.size(), 1);
    }
    sf_grid_.InitializeWithPose(pose_.vehicle_pose);
  }

 private:
  SensorFovGrid<SFPillar> sf_grid_;

  SensorFovViewType view_type_ = VT_LIDAR;

  std::map<std::string, Vec3d> ref_sensors_;

  VehiclePoseWithTimestamp pose_;

  AffineTransformation vehicle_to_smooth_;

  AffineTransformation smooth_to_vehicle_;
};

SensorFov BuildLidarViewSensorFov(const SensorFovsProto& sensor_fovs_proto);

std::map<CameraId, SensorFov> BuildCameraViewSensorFovs(
    const SensorFovsProto& sensor_fovs_proto);

SensorFovRef GetLidarViewSensorFov(const SensorFovRefs& sensor_fovs);

std::map<CameraId, SensorFovRef> GetCameraViewSensorFovs(
    const SensorFovRefs& sensor_fovs);

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_H_
