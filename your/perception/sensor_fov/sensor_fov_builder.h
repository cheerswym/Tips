#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_BUILDER_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_BUILDER_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/sensor_fov/sensor_fov.h"
#include "onboard/perception/sensor_fov/sensor_fov_grid.h"

namespace qcraft::sensor_fov {

struct SFRay {
  struct Point {
    Point(uint16_t x, uint16_t y, float range)
        : x(x), y(y), range(range), range_inv(1.f / range) {}
    Point() = default;

    uint16_t x = 0;
    uint16_t y = 0;
    float range = 0.f;
    float range_inv = 0.f;
  };

  std::vector<Point> points;
};

using SFRays = std::vector<SFRay>;

class SensorFovBuilder {
 public:
  SensorFovBuilder(const RunParamsProtoV2& run_params, ThreadPool* thread_pool);

  SensorFovBuilder() = delete;
  // Compute lidar & camera view sensor fov.
  SensorFovRefs Compute(const ObstaclePtrs& obstacle_ptrs,
                        const SegmentedClusters& clusters,
                        const VehiclePoseWithTimestamp& pose);

 private:
  void InitAllRays();

  SensorFov ComputeLidarView(const SegmentedClusters& clusters,
                             const VehiclePoseWithTimestamp& pose);

  SensorFov ComputeCameraView(const std::string& camera_id_name,
                              const ObstaclePtrs& obstacle_ptrs,
                              const VehiclePoseWithTimestamp& pose);

  SensorFovRefs ComputeCameraViews(const ObstaclePtrs& obstacle_ptrs,
                                   const VehiclePoseWithTimestamp& pose);

  SensorFov BuildSensorFov(const SensorFovGrid<SFGridInfo>& sf_grid,
                           const SensorFovViewType view_type,
                           const std::map<std::string, Vec3d>& ref_sensors,
                           const VehiclePoseWithTimestamp& pose) const;

 private:
  ThreadPool* const thread_pool_ = nullptr;

  float av_height_ = 0.f;

  std::map<std::string, Vec3d> ref_lidars_;

  std::map<std::string, Vec3d> ref_cameras_;

  std::map<std::string, SFRays> all_rays_;

  SensorFovGrid<SFGridInfo> lidar_view_sf_grid_;

  std::map<std::string, SensorFovGrid<SFGridInfo>> camera_view_sf_grids_;
};

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_BUILDER_H_
