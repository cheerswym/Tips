#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"

#include "onboard/params/param_manager.h"
#include "onboard/perception/test_util/cluster_builder.h"

namespace qcraft::sensor_fov {

SensorFovPtr GetLidarViewSensorFovPtr(const SensorFovs& sensor_fovs) {
  SensorFovPtr sensor_fov_ptr = nullptr;
  for (const auto& sensor_fov : sensor_fovs) {
    if (sensor_fov.view_type() == VT_LIDAR) {
      sensor_fov_ptr = &sensor_fov;
      break;
    }
  }
  return QCHECK_NOTNULL(sensor_fov_ptr);
}

std::unique_ptr<SensorFovBuilder> BuildSensorFovBuilder() {
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  // Init sensor fov
  return std::make_unique<SensorFovBuilder>(run_params, nullptr);
}

std::pair<SegmentedClusters, ObstacleRefVector> BuildSegmentedClusters(
    const VehiclePose& pose) {
  const AffineTransformation vehicle_to_smooth = pose.ToTransform();
  SegmentedClusters clusters;
  ObstacleRefVector obstacle_refs;
  ObstaclePtrs obstacle_ptrs;
  for (float x = 30.f; x < 40.f; x += 0.25f) {
    for (float y = -30.f; y < -20.f; y += 0.25f) {
      const Vec2d coord =
          vehicle_to_smooth.TransformPoint(Vec3d(x, y, 0)).head<2>();
      auto obstacle_ref =
          std::make_unique<Obstacle>(Obstacle(ObstacleBuilder()
                                                  .set_x(coord.x())
                                                  .set_y(coord.y())
                                                  .set_min_z(0.2)
                                                  .set_max_z(1.5)
                                                  .set_ground_z(0)
                                                  .Build()));
      obstacle_refs.emplace_back(std::move(obstacle_ref));
      obstacle_ptrs.emplace_back(obstacle_refs.back().get());
    }
  }
  clusters.emplace_back(0, ClusterBuilder()
                               .set_type(MT_VEHICLE)
                               .set_obstacles(std::move(obstacle_ptrs))
                               .Build());
  obstacle_ptrs.clear();
  for (float x = 40.f; x < 50.f; x += 0.25f) {
    for (float y = 20.f; y < 30.f; y += 0.25f) {
      const Vec2d coord =
          vehicle_to_smooth.TransformPoint(Vec3d(x, y, 0)).head<2>();
      auto obstacle_ref =
          std::make_unique<Obstacle>(Obstacle(ObstacleBuilder()
                                                  .set_x(coord.x())
                                                  .set_y(coord.y())
                                                  .set_min_z(0.2)
                                                  .set_max_z(1.2)
                                                  .set_ground_z(0)
                                                  .Build()));
      obstacle_refs.emplace_back(std::move(obstacle_ref));
      obstacle_ptrs.emplace_back(obstacle_refs.back().get());
    }
  }
  clusters.emplace_back(0, ClusterBuilder()
                               .set_type(MT_VEHICLE)
                               .set_obstacles(std::move(obstacle_ptrs))
                               .Build());

  obstacle_ptrs.clear();
  for (float x = -20.f; x < -10.f; x += 0.25f) {
    for (float y = 20.f; y < 30.f; y += 0.25f) {
      const Vec2d coord =
          vehicle_to_smooth.TransformPoint(Vec3d(x, y, 0)).head<2>();
      auto obstacle_ref =
          std::make_unique<Obstacle>(Obstacle(ObstacleBuilder()
                                                  .set_x(coord.x())
                                                  .set_y(coord.y())
                                                  .set_min_z(0.2)
                                                  .set_max_z(1.0)
                                                  .set_ground_z(0)
                                                  .Build()));
      obstacle_refs.emplace_back(std::move(obstacle_ref));
      obstacle_ptrs.emplace_back(obstacle_refs.back().get());
    }
  }
  clusters.emplace_back(0, ClusterBuilder()
                               .set_type(MT_VEHICLE)
                               .set_obstacles(std::move(obstacle_ptrs))
                               .Build());

  obstacle_ptrs.clear();
  for (float x = -20.f; x < -10.f; x += 0.25f) {
    for (float y = -30.f; y < -20.f; y += 0.25f) {
      const Vec2d coord =
          vehicle_to_smooth.TransformPoint(Vec3d(x, y, 0)).head<2>();
      auto obstacle_ref =
          std::make_unique<Obstacle>(Obstacle(ObstacleBuilder()
                                                  .set_x(coord.x())
                                                  .set_y(coord.y())
                                                  .set_min_z(0.2)
                                                  .set_max_z(1.2)
                                                  .set_ground_z(0)
                                                  .Build()));
      obstacle_refs.emplace_back(std::move(obstacle_ref));
      obstacle_ptrs.emplace_back(obstacle_refs.back().get());
    }
  }
  clusters.emplace_back(0, ClusterBuilder()
                               .set_type(MT_VEHICLE)
                               .set_obstacles(std::move(obstacle_ptrs))
                               .Build());

  return {std::move(clusters), std::move(obstacle_refs)};
}

}  // namespace qcraft::sensor_fov
