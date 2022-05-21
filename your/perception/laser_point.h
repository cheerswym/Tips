#ifndef ONBOARD_PERCEPTION_LASER_POINT_H_
#define ONBOARD_PERCEPTION_LASER_POINT_H_

#include <vector>

#include "onboard/lidar/intensity_util.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/lidar.pb.h"

namespace qcraft {

struct LaserPoint {
  Vec3d coord() const { return {x, y, z}; }
  Vec3d normal() const {
    return {normal_x * (1.0 / 128.0), normal_y * (1.0 / 128.0),
            normal_z * (1.0 / 128.0)};
  }
  // normalize intensity to [0, 1)
  float normalized_intensity() const { return intensity * (1.0f / 255.f); }

  double timestamp = 0.0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float range = 0.0f;
  int8_t normal_x = 0.0f;
  int8_t normal_y = 0.0f;
  int8_t normal_z = 0.0f;
  bool has_return_behind = false;
  uint8_t intensity = 0;
  uint8_t planarity = 0;
  uint8_t beam_index = 0;
  uint8_t return_index = 0;
  // Scan index in Spin and point index in PointCloud.
  uint32_t scan_or_point_index = 0;
  static_assert(LidarId_MAX < (1 << 8));
  LidarId lidar_id : 8;
  static_assert(LidarType_MAX < (1 << 8));
  LidarModel lidar_type : 8;
};
static_assert(sizeof(LaserPoint) == 40);

inline bool CompareLaserPointAtZ(const LaserPoint& lhs, const LaserPoint& rhs) {
  // using x/y as 2nd/3rd sort factor is to keep sort result stable
  return std::make_tuple(lhs.z, lhs.x, lhs.y) <
         std::make_tuple(rhs.z, rhs.x, rhs.y);
}

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_LASER_POINT_H_
