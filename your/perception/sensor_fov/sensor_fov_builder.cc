#include "onboard/perception/sensor_fov/sensor_fov_builder.h"

#include <algorithm>
#include <atomic>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/camera/camera_params.h"
#include "onboard/global/trace.h"
#include "onboard/math/util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/sensor_fov/line_iterator.h"
#include "onboard/perception/sensor_fov/sensor_fov_constants.h"
#include "onboard/utils/map_util.h"
#include "snappy/snappy.h"

DEFINE_bool(enable_camera_view_sensor_fov, false,
            "Whether to enable camera view sensor fov.");

namespace qcraft::sensor_fov {

namespace {

static const std::set<LidarId> kRefLidarIds = {LDR_FRONT_LEFT, LDR_FRONT_RIGHT,
                                               LDR_FRONT, LDR_CENTER};

static const std::set<CameraId> kRefCameraIds = {
    CAM_PBQ_FRONT_WIDE, CAM_PBQ_FRONT_LEFT, CAM_PBQ_FRONT_RIGHT, CAM_L_FRONT,
    CAM_L_FRONT_LEFT,   CAM_L_FRONT_RIGHT,  CAM_R_FRONT};

float ClampHeight(const float height) {
  return std::clamp(height, 0.f, kSFDetectionRangeHeight);
}

std::map<std::string, Vec3d> ComupteRefLidars(
    const RunParamsProtoV2& run_params) {
  std::map<std::string, Vec3d> ref_lidars;
  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    const auto& lidar_id = lidar_param.installation().lidar_id();
    const auto& extrinsics = lidar_param.installation().extrinsics();
    if (ContainsKey(kRefLidarIds, lidar_id)) {
      ref_lidars.emplace(LidarId_Name(lidar_id),
                         Vec3d(extrinsics.x(), extrinsics.y(), extrinsics.z()));
    }
  }

  return ref_lidars;
}

std::map<std::string, Vec3d> ComupteRefCameras(
    const RunParamsProtoV2& run_params) {
  std::map<std::string, Vec3d> ref_cameras;
  const auto all_camera_params =
      ComputeAllCameraParams(run_params.vehicle_params());
  for (const auto& [camera_id, camera_param] : all_camera_params) {
    const auto& extrinsics = camera_param.camera_to_vehicle_extrinsics();
    if (ContainsKey(kRefCameraIds, camera_id)) {
      ref_cameras.emplace(CameraId_Name(camera_id),
                          Vec3d(extrinsics.x, extrinsics.y, extrinsics.z));
    }
  }

  return ref_cameras;
}

std::map<std::string, SensorFovGrid<SFGridInfo>> ComputeCameraViewSFGrids(
    const std::map<std::string, Vec3d>& ref_cameras) {
  std::map<std::string, SensorFovGrid<SFGridInfo>> camera_view_sf_grids;
  for (const auto& [camera_id_name, _] : ref_cameras) {
    // TODO(dong): Should use customized detection range for each camera.
    camera_view_sf_grids.try_emplace(
        camera_id_name, kSFDetectionRangeFront, kSFDetectionRangeRear,
        kSFDetectionRangeLateral, kSFDetectionRangeLateral, kSFGridDiameter);
  }

  return camera_view_sf_grids;
}

SFRays ComputeSensorRays(const SensorFovGrid<SFGridInfo>& sf_grid,
                         const Vec3d& ref_sensor) {
  const int height = sf_grid.height();
  const int width = sf_grid.width();
  std::vector<std::vector<Vec2i>> contour_lines;
  contour_lines.reserve(4);
  contour_lines.emplace_back(
      ComputeAllLinePoints({0, 0}, {height - 1, 0}, /*connectivity*/ 4));
  contour_lines.emplace_back(ComputeAllLinePoints(
      {height - 1, 0}, {height - 1, width - 1}, /*connectivity*/ 4));
  contour_lines.emplace_back(ComputeAllLinePoints(
      {height - 1, width - 1}, {0, width - 1}, /*connectivity*/ 4));
  contour_lines.emplace_back(
      ComputeAllLinePoints({0, width - 1}, {0, 0}, /*connectivity*/ 4));
  SFRays rays;
  const auto [sensor_row, sensor_col] =
      sf_grid.VehicleCoordToRC(ref_sensor.head<2>());
  const Vec2i start_point(sensor_row, sensor_col);
  for (const auto& contour_line : contour_lines) {
    for (const auto& line_point : contour_line) {
      const auto line_points =
          ComputeAllLinePoints(start_point, line_point, /*connectivity*/ 8);
      SFRay ray;
      ray.points.reserve(line_points.size());
      // Skip near sensor point.
      for (int i = 2; i < line_points.size(); ++i) {
        const auto& line_point = line_points[i];
        const float range =
            Hypot(static_cast<float>(line_point.x() - start_point.x()),
                  static_cast<float>(line_point.y() - start_point.y()));
        ray.points.emplace_back(line_point.x(), line_point.y(), range);
      }
      rays.emplace_back(std::move(ray));
    }
  }

  return rays;
}

bool IsTargetCluster(const SegmentedCluster& cluster) {
  if (cluster.type() == MT_VEHICLE || cluster.type() == MT_MOTORCYCLIST ||
      cluster.type() == MT_PEDESTRIAN || cluster.type() == MT_CYCLIST) {
    return true;
  }
  return false;
}

bool IsTargetObstacle(const Obstacle& obstacle, const float av_height) {
  if (obstacle.type == ObstacleProto::IGNORED ||
      obstacle.type == ObstacleProto::VEGETATION) {
    return false;
  }
  if (obstacle.clearance > av_height) {
    return false;
  }

  return true;
}

template <typename T>
void CompareAndUpdateToMin(std::atomic<T>* min_value, const T value) {
  T prev_value = *min_value;
  while (prev_value > value &&
         !min_value->compare_exchange_weak(prev_value, value)) {
  }
}

float ComputeSlope(const float pillar_height, const float sensor_height,
                   const float pillar_to_sensor_range_inv) {
  return (pillar_height - sensor_height) * pillar_to_sensor_range_inv;
}

float ComputeOccludedHeightAtPoint(const float point_to_sensor_range,
                                   const float sensor_height,
                                   const float slope) {
  const float occluded_height = sensor_height + slope * point_to_sensor_range;
  return ClampHeight(occluded_height);
}

void UpdateOccupiedGrid(const Obstacle& obstacle,
                        const AffineTransformation& smooth_to_vehicle,
                        SensorFovGrid<SFGridInfo>* sf_grid) {
  const Vec3d peak =
      smooth_to_vehicle.TransformPoint(Vec3d(obstacle.coord(), obstacle.max_z));
  const Vec3d bottom = smooth_to_vehicle.TransformPoint(
      Vec3d(obstacle.coord(), obstacle.ground_z + obstacle.clearance));
  const auto [row, col] = sf_grid->VehicleCoordToRC(peak.head<2>());
  if (!sf_grid->IsValidRC({row, col})) return;
  auto& info = (*sf_grid)(row, col);
  info.state = SF_OCCUPIED;
  const float peak_z = ClampHeight(peak.z());
  info.max_invisable_elevation =
      info.max_invisable_elevation == std::numeric_limits<float>::max()
          ? peak_z
          : std::max(info.max_invisable_elevation, peak_z);
  const float bottom_z = ClampHeight(bottom.z());
  info.min_invisable_elevation =
      info.min_invisable_elevation == 0.f
          ? bottom_z
          : std::min(info.min_invisable_elevation, bottom_z);
}

void UpdateOccupiedGrid(const SegmentedCluster& cluster,
                        const AffineTransformation& smooth_to_vehicle,
                        SensorFovGrid<SFGridInfo>* sf_grid) {
  for (const auto* obstacle : cluster.obstacles()) {
    UpdateOccupiedGrid(*obstacle, smooth_to_vehicle, sf_grid);
  }
}

void UpdateOccludedGrid(const std::string& sensor_name, const Vec3d& ref_sensor,
                        const SFRays& rays,
                        SensorFovGrid<SFGridInfo>* sf_grid) {
  SCOPED_QTRACE_ARG1("SensorFovBuilder::UpdateOccludedGrid", "sensor_name",
                     sensor_name);
  const auto [sensor_row, sensor_col] =
      sf_grid->VehicleCoordToRC(ref_sensor.head<2>());
  const float sensor_height = ref_sensor.z();
  const Vec2i start_point(sensor_row, sensor_col);
  for (const auto& ray : rays) {
    std::optional<std::pair<float, float>> slopes;
    for (const auto& point : ray.points) {
      auto& info = (*sf_grid)(point.x, point.y);
      if (info.state == SF_OCCUPIED) {
        if (!slopes) {
          slopes = {ComputeSlope(info.max_invisable_elevation, sensor_height,
                                 point.range_inv),
                    ComputeSlope(info.min_invisable_elevation, sensor_height,
                                 point.range_inv)};
        } else {
          const auto [upper_slope, lower_slope] = *slopes;
          const float new_upper_slope = ComputeSlope(
              info.max_invisable_elevation, sensor_height, point.range_inv);
          const float new_lower_slope = ComputeSlope(
              info.min_invisable_elevation, sensor_height, point.range_inv);
          slopes = {std::max(upper_slope, new_upper_slope),
                    std::min(lower_slope, new_lower_slope)};

          if (new_upper_slope <= upper_slope) {
            const float occluded_upper_height = ComputeOccludedHeightAtPoint(
                point.range, sensor_height, upper_slope);
            DCHECK(occluded_upper_height >= info.max_invisable_elevation);
            info.max_invisable_elevation = occluded_upper_height;
            info.state = SF_OCCLUDED;
          }
          if (new_lower_slope >= lower_slope && info.state == SF_OCCLUDED) {
            const float occluded_lower_height = ComputeOccludedHeightAtPoint(
                point.range, sensor_height, lower_slope);
            DCHECK(occluded_lower_height <= info.min_invisable_elevation);
            info.min_invisable_elevation = occluded_lower_height;
          }
        }
        continue;
      }

      if (!slopes) {
        info.state = SF_FREE;
        continue;
      }

      const auto [upper_slope, lower_slope] = *slopes;
      const float occluded_upper_height =
          ComputeOccludedHeightAtPoint(point.range, sensor_height, upper_slope);
      info.max_invisable_elevation =
          std::min(info.max_invisable_elevation, occluded_upper_height);
      const float occluded_lower_height =
          ComputeOccludedHeightAtPoint(point.range, sensor_height, lower_slope);
      info.min_invisable_elevation =
          std::max(info.min_invisable_elevation, occluded_lower_height);

      if (info.max_invisable_elevation == 0.f &&
          info.min_invisable_elevation == 0.f) {
        info.state = SF_FREE;
        slopes.reset();
      } else if (info.max_invisable_elevation <= info.min_invisable_elevation) {
        info.state = SF_FREE;
      } else {
        info.state = SF_OCCLUDED;
      }
    }
  }
}

}  // namespace

SensorFovBuilder::SensorFovBuilder(const RunParamsProtoV2& run_params,
                                   ThreadPool* thread_pool)
    : thread_pool_(thread_pool),
      av_height_(obstacle_util::ComputeAvHeight(run_params)),
      ref_lidars_(ComupteRefLidars(run_params)),
      ref_cameras_(ComupteRefCameras(run_params)),
      lidar_view_sf_grid_(kSFDetectionRangeFront, kSFDetectionRangeRear,
                          kSFDetectionRangeLateral, kSFDetectionRangeLateral,
                          kSFGridDiameter),
      camera_view_sf_grids_(ComputeCameraViewSFGrids(ref_cameras_)) {
  InitAllRays();
}

void SensorFovBuilder::InitAllRays() {
  for (const auto& [lidar_id_name, ref_lidar] : ref_lidars_) {
    all_rays_[lidar_id_name] =
        ComputeSensorRays(lidar_view_sf_grid_, ref_lidar);
  }
  for (const auto& [camera_id_name, ref_camera] : ref_cameras_) {
    all_rays_[camera_id_name] = ComputeSensorRays(
        FindOrDie(camera_view_sf_grids_, camera_id_name), ref_camera);
  }
}

SensorFovRefs SensorFovBuilder::Compute(const ObstaclePtrs& obstacle_ptrs,
                                        const SegmentedClusters& clusters,
                                        const VehiclePoseWithTimestamp& pose) {
  SCOPED_QTRACE_ARG2("SensorFovBuilder::Compute", "num_obstacles",
                     obstacle_ptrs.size(), "num_clusters", clusters.size());
  SensorFovRefs sensor_fovs;
  sensor_fovs.reserve(ref_cameras_.size() + 1);
  auto lidar_view_sensor_fov_future = ScheduleFuture(
      FLAGS_enable_camera_view_sensor_fov ? thread_pool_ : nullptr, [&] {
        return std::make_shared<SensorFov>(ComputeLidarView(clusters, pose));
      });
  if (FLAGS_enable_camera_view_sensor_fov) {
    auto camera_view_sensor_fovs = ComputeCameraViews(obstacle_ptrs, pose);
    std::move(camera_view_sensor_fovs.begin(), camera_view_sensor_fovs.end(),
              std::back_inserter(sensor_fovs));
  }
  sensor_fovs.emplace_back(lidar_view_sensor_fov_future.Get());

  return sensor_fovs;
}

SensorFov SensorFovBuilder::ComputeLidarView(
    const SegmentedClusters& clusters, const VehiclePoseWithTimestamp& pose) {
  SCOPED_QTRACE("SensorFovBuilder::ComputeLidarView");

  lidar_view_sf_grid_.InitializeWithPose(pose.vehicle_pose);

  if (clusters.empty()) {
    return BuildSensorFov(lidar_view_sf_grid_, VT_LIDAR, ref_lidars_, pose);
  }

  const auto vehicle_to_smooth = pose.vehicle_pose.ToTransform();
  const auto smooth_to_vehicle = vehicle_to_smooth.Inverse();

  {
    SCOPED_QTRACE("SensorFovBuilder::UpdateOccupiedGrid");
    for (const auto& cluster : clusters) {
      if (IsTargetCluster(cluster)) {
        UpdateOccupiedGrid(cluster, smooth_to_vehicle, &lidar_view_sf_grid_);
      }
    }
  }

  {
    SCOPED_QTRACE("SensorFovBuilder::UpdateOccludedGrid");
    for (const auto& [lidar_id_name, ref_lidar] : ref_lidars_) {
      UpdateOccludedGrid(lidar_id_name, ref_lidar,
                         FindOrDie(all_rays_, lidar_id_name),
                         &lidar_view_sf_grid_);
    }
  }

  auto sensor_fov =
      BuildSensorFov(lidar_view_sf_grid_, VT_LIDAR, ref_lidars_, pose);
  // Clear grid info in sf grid asynchronously.
  lidar_view_sf_grid_.ResetAsync();

  return sensor_fov;
}

SensorFov SensorFovBuilder::ComputeCameraView(
    const std::string& camera_id_name, const ObstaclePtrs& obstacle_ptrs,
    const VehiclePoseWithTimestamp& pose) {
  SCOPED_QTRACE_ARG1("SensorFovBuilder::ComputeCameraView", "camera_id_name",
                     camera_id_name);

  auto& camera_view_sf_grid = FindOrDie(camera_view_sf_grids_, camera_id_name);

  camera_view_sf_grid.InitializeWithPose(pose.vehicle_pose);

  if (obstacle_ptrs.empty()) {
    return BuildSensorFov(camera_view_sf_grid, VT_CAMERA, ref_lidars_, pose);
  }

  const auto vehicle_to_smooth = pose.vehicle_pose.ToTransform();
  const auto smooth_to_vehicle = vehicle_to_smooth.Inverse();

  {
    SCOPED_QTRACE("SensorFovBuilder::UpdateOccupiedGrid");
    for (const auto* obstacle : obstacle_ptrs) {
      if (IsTargetObstacle(*obstacle, av_height_)) {
        UpdateOccupiedGrid(*obstacle, smooth_to_vehicle, &camera_view_sf_grid);
      }
    }
  }

  {
    SCOPED_QTRACE("SensorFovBuilder::UpdateOccludedGrid");
    UpdateOccludedGrid(camera_id_name, FindOrDie(ref_cameras_, camera_id_name),
                       FindOrDie(all_rays_, camera_id_name),
                       &camera_view_sf_grid);
  }

  const std::pair<std::string, Vec3d> ref_camera = {
      camera_id_name, FindOrDie(ref_cameras_, camera_id_name)};
  auto sensor_fov =
      BuildSensorFov(camera_view_sf_grid, VT_CAMERA, {ref_camera}, pose);
  // Clear grid info in sf grid asynchronously.
  camera_view_sf_grid.ResetAsync();

  return sensor_fov;
}

SensorFovRefs SensorFovBuilder::ComputeCameraViews(
    const ObstaclePtrs& obstacle_ptrs, const VehiclePoseWithTimestamp& pose) {
  SensorFovRefs sensor_fovs;
  sensor_fovs.resize(ref_cameras_.size());
  std::vector<std::string> camera_id_names;
  for (const auto& [camera_id_name, _] : ref_cameras_) {
    camera_id_names.push_back(camera_id_name);
  }
  ParallelFor(0, sensor_fovs.size(), thread_pool_, [&](size_t i) {
    sensor_fovs[i] = std::make_shared<SensorFov>(
        ComputeCameraView(camera_id_names[i], obstacle_ptrs, pose));
  });

  return sensor_fovs;
}

SensorFov SensorFovBuilder::BuildSensorFov(
    const SensorFovGrid<SFGridInfo>& sf_grid, const SensorFovViewType view_type,
    const std::map<std::string, Vec3d>& ref_sensors,
    const VehiclePoseWithTimestamp& pose) const {
  QCHECK(!ref_sensors.empty());
  SCOPED_QTRACE_ARG1("SensorFovBuilder::BuildSensorFov", "view_type",
                     ref_sensors.begin()->first);
  const auto& detection_range = sf_grid.detection_range();
  SensorFov sensor_fov(detection_range.front, detection_range.behind,
                       detection_range.left, detection_range.right,
                       sf_grid.diameter(), view_type, ref_sensors, pose);
  // Build sf grid.
  // NOTE(dong): Use memcpy here to copy vector<SFGridInfo> to vector<SFPillar>
  // because they are both POD types and have exactly the same layout by now.
  // Need to be adapted if we change SFGridInfo later.
  QCHECK_EQ(sizeof(SFGridInfo), sizeof(SFPillar));
  QCHECK_EQ(sf_grid.info_grid_.size(), sensor_fov.sf_grid_.info_grid_.size());
  std::memcpy(sensor_fov.sf_grid_.info_grid_.data(), sf_grid.info_grid_.data(),
              sizeof(SFGridInfo) * sf_grid.info_grid_.size());

  return sensor_fov;
}

}  // namespace qcraft::sensor_fov
