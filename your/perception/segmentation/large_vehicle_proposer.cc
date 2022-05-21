#include "onboard/perception/segmentation/large_vehicle_proposer.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/perception/semantic_segmentation_result.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(enable_image_instance_result, true,
            "Enable the result of instance segmentation");

DEFINE_bool(large_vehicle_proposer_cvs, false,
            "Enable large vehicle proposer cvs");

DEFINE_bool(cluster_polygon_cvs, false, "Enable cluster's polygon");

namespace qcraft::segmentation {
namespace {
struct Compare {
  bool operator()(const Vec2d v1, const Vec2d v2) const {
    if (v1.x() == v2.x()) {
      return v1.y() < v2.y();
    }
    return v1.x() < v2.x();
  }
};

enum Position : int {
  FRONT_P = 0,
  FRONT_V = 1,
  BACK = 2,
  LEFT_FRONT = 3,
  RIGHT_FRONT = 4,
  LEFT_BACK = 5,
  RIGHT_BACK = 6,
  RIGHT = 7,
  LEFT = 8,
  MIDDLE_BACK = 9,
  MIDDLE_FRONT = 10
};

Vec3d GetCenterPointOfCluster(const ProposedCluster& cluster) {
  Vec3d center_point = {0, 0, 0};
  int size = 0;
  for (const auto& obstacle_ptr : cluster.obstacles()) {
    for (const auto& point : obstacle_ptr->points) {
      size++;
      center_point = {center_point.x() + point.x, center_point.y() + point.y,
                      center_point.z() + point.z};
    }
  }
  const double size_inv = 1 / static_cast<double>(size);
  center_point = center_point * size_inv;
  return center_point;
}

double GetDistanceSqrOfVehicleWithEgoCar(
    const Vec2d& coor, const ProposedCluster& current_cluster) {
  const Vec3d point = GetCenterPointOfCluster(current_cluster);
  return Sqr(point.x() - coor.x()) + Sqr(point.y() - coor.y());
}

int GetClusterPointsNum(const ProposedCluster& cluster) {
  int size = 0;
  for (const auto& obstacle_ptr : cluster.obstacles()) {
    size += obstacle_ptr->points.size();
  }
  return size;
}

Box2d GetMinAreaBox(const Cluster& cluster) {
  const auto polygon = cluster_util::ComputeContour(cluster);
  return polygon.MinAreaBoundingBox();
}

// Get the heading of a cluster
Vec2d GetHeadingDir(const ProposedCluster& cluster) {
  const auto min_area_bounding_box = GetMinAreaBox(cluster);
  constexpr double kRatioWidthToLength = 5;
  if (min_area_bounding_box.width() >
      min_area_bounding_box.length() * kRatioWidthToLength) {
    return Vec2d::FastUnitFromAngle(min_area_bounding_box.heading() + M_PI_2);
  }
  return Vec2d::FastUnitFromAngle(min_area_bounding_box.heading());
}

Position GetClusterPosition(const Vec2d current_cluster_center_point,
                            const double heading,
                            const Vec2d relative_cluster_center_point,
                            const Vec2d relative_heading) {
  const AffineTransformation transform =
      AffineTransformation::FromTranslation(current_cluster_center_point.x(),
                                            current_cluster_center_point.y(),
                                            0.0)
          .ApplyYawPitchRoll(heading, 0.0, 0.0)
          .Inverse();
  const auto coord_of_relative_transform =
      transform.TransformPoint(Vec3d(relative_cluster_center_point, 0));

  // The road's width
  constexpr double kRoadWidth = 2;  // m
  constexpr double kHeadingCosValue = 0.9;

  const Vec2d current_heading = Vec2d::FastUnitFromAngle(heading);
  // In the same lane
  if (std::abs(coord_of_relative_transform.y()) < kRoadWidth) {
    const double heading_with_heading_cos =
        current_heading.dot(relative_heading);
    if (std::abs(heading_with_heading_cos) > kHeadingCosValue) {
      if (coord_of_relative_transform.x() > 0) {
        return FRONT_P;
      } else {
        return BACK;
      }
    } else {
      return FRONT_V;
    }
  }

  const double current_to_relative_dist_sqr =
      (Sqr((current_cluster_center_point.x() -
            relative_cluster_center_point.x())) +
       Sqr(current_cluster_center_point.y() -
           relative_cluster_center_point.y()));
  constexpr double kDistance = Sqr(15.0);     // m2
  constexpr double kDistanceUpper = Sqr(30);  // m2
  // In adjacent road
  if (coord_of_relative_transform.x() < 0) {
    if (current_to_relative_dist_sqr < kDistance) {
      if (coord_of_relative_transform.y() > 0) {
        return LEFT;
      } else {
        return RIGHT;
      }
    } else if (current_to_relative_dist_sqr < kDistanceUpper) {
      return MIDDLE_BACK;  // RIGHT_BACK and LEFT_BACK is the same
    } else {
      if (coord_of_relative_transform.y() > 0) {
        return LEFT_BACK;
      } else {
        return RIGHT_BACK;
      }
    }
  } else {
    if (current_to_relative_dist_sqr < kDistance) {
      if (coord_of_relative_transform.y() > 0) {
        return LEFT;
      } else {
        return RIGHT;  // RIGHT and LEFT is the same
      }
    } else if (current_to_relative_dist_sqr < kDistanceUpper) {
      return MIDDLE_FRONT;  // RIGHT_FRONT and LEFT_FRONT is the same
    } else {
      if (coord_of_relative_transform.y() > 0) {
        return LEFT_FRONT;
      } else {
        return RIGHT_FRONT;
      }
    }
  }
}

bool CheckRelativeObstacleInCurrentCluster(
    const Vec2d& current_heading, const Vec2d& current_cluster_center_point,
    const Box2d& current_box, const ProposedCluster& current_cluster,
    const ProposedCluster& relative_cluster) {
  const double road_width_with_obstacle =
      0.05 + current_box.width() * 0.5;  // m   1.75  2.5
  constexpr double kInsideRatio = 0.8;
  int in_road_counter = 0;
  const int obstacle_size = relative_cluster.obstacles().size();
  for (const auto& obstacle_ptr : relative_cluster.obstacles()) {
    const Vec2d obstacle_center_point = obstacle_ptr->coord();
    const Vec2d center_point_dir =
        obstacle_center_point - current_cluster_center_point;
    const double center_point_dir_norm = center_point_dir.norm();
    const double center_point_dir_norm_inv = 1.0 / center_point_dir_norm;
    const double cur_heading_with_obstacle_cos_value =
        current_heading.dot(center_point_dir) * center_point_dir_norm_inv;
    const double cur_heading_with_obstacle_sin_value =
        std::sqrt(1 - cur_heading_with_obstacle_cos_value *
                          cur_heading_with_obstacle_cos_value);
    const double cur_center_with_obstacle_width =
        center_point_dir_norm * cur_heading_with_obstacle_sin_value;
    if (cur_center_with_obstacle_width < road_width_with_obstacle) {
      in_road_counter++;
    }
  }

  const double real_ratio =
      in_road_counter / static_cast<double>(obstacle_size);
  if (real_ratio < kInsideRatio) {
    return false;
  }

  return true;
}

bool IsTheSameLaneWithRelativeClusterObstacle(
    const Vec2d& current_cluster_center_point, const Vec2d& current_heading,
    const ProposedCluster& relative_cluster) {
  constexpr double kRoadWidthWithObstacle = 1.75;  // m
  constexpr double kInsideRatio = 0.9;
  int in_road_counter = 0;
  const int obstacle_size = relative_cluster.obstacles().size();
  for (const auto& obstacle_ptr : relative_cluster.obstacles()) {
    const Vec2d obstacle_center_point = obstacle_ptr->coord();
    const Vec2d center_point_dir =
        obstacle_center_point - current_cluster_center_point;
    const double center_point_dir_norm = center_point_dir.norm();
    const double center_point_dir_norm_inv = 1.0 / center_point_dir_norm;
    const double cur_heading_with_obstacle_cos_value =
        current_heading.dot(center_point_dir) * center_point_dir_norm_inv;
    const double cur_heading_with_obstacle_sin_value =
        std::sqrt(1 - cur_heading_with_obstacle_cos_value *
                          cur_heading_with_obstacle_cos_value);
    const double cur_center_with_obstacle_width =
        center_point_dir_norm * cur_heading_with_obstacle_sin_value;
    if (cur_center_with_obstacle_width < kRoadWidthWithObstacle) {
      in_road_counter++;
    }
  }

  const double real_ratio =
      in_road_counter / static_cast<double>(obstacle_size);
  if (real_ratio < kInsideRatio) {
    VLOG(2) << "the false reason is the real_ratio is " << real_ratio;
    return false;
  }

  return true;
}

bool AreInTheSameLane(const ProposedCluster& current_cluster,
                      const ProposedCluster& relative_cluster) {
  QCHECK(current_cluster.bounding_box());
  const auto current_box = current_cluster.bounding_box();
  const double cur_heading = current_box->heading();

  const Vec2d point = current_box->center();
  const AffineTransformation transform =
      AffineTransformation::FromTranslation(point.x(), point.y(), 0.0)
          .ApplyYawPitchRoll(cur_heading, 0.0, 0.0)
          .Inverse();

  constexpr double kMaxTheSameLaneWidth = 1.75;  // m
  int in_road_count = 0;
  for (const auto obs_ptr : relative_cluster.obstacles()) {
    const auto point = transform.TransformPoint(Vec3d(obs_ptr->coord(), 1));
    if (std::abs(point.y()) < kMaxTheSameLaneWidth) {
      in_road_count++;
    }
  }

  constexpr double kMinInRoadCountRatio = 0.6;

  return in_road_count >
         relative_cluster.obstacles().size() * kMinInRoadCountRatio;
}

// Get the positional relationship between two clusters
Position GetClusterRelativePosition(const Vec2d& av_coord,
                                    const double av_heading,
                                    const ProposedCluster& relative_cluster) {
  Vec2d relative_heading;
  std::optional<Box2d> relative_box = relative_cluster.bounding_box();
  if (relative_box) {
    relative_heading = Vec2d::FastUnitFromAngle(relative_box->heading());
  } else {
    relative_box = GetMinAreaBox(relative_cluster);
    relative_heading = Vec2d::FastUnitFromAngle(relative_box->heading());
  }

  return GetClusterPosition(av_coord, av_heading, relative_box->center(),
                            relative_heading);
}

SegmentationType GetPointSegmentationType(const LaserPoint& point,
                                          const cv::Mat& mask,
                                          const VehiclePose& pose,
                                          const CameraParams& camera_params,
                                          const double sem_seg_output_scale) {
  const auto smooth_to_camera_trans =
      (pose.ToTransform() *
       camera_params.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();
  const auto& image_pose = projection_util::SmoothPointToImagePos(
      point.coord(), smooth_to_camera_trans, camera_params);
  if (image_pose) {
    return static_cast<SegmentationType>(
        mask.data[static_cast<int>(image_pose->y() / sem_seg_output_scale) *
                      mask.cols +
                  static_cast<int>(image_pose->x() / sem_seg_output_scale)]);
  } else {
    return ST_DONTCARE;
  }
}

int GetPointIsSameInstance(const LaserPoint& point, const cv::Mat& mask,
                           const VehiclePose& pose,
                           const CameraParams& camera_params,
                           const double sem_seg_output_scale) {
  const auto smooth_to_camera_trans =
      (pose.ToTransform() *
       camera_params.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();
  const auto& image_pose = projection_util::SmoothPointToImagePos(
      point.coord(), smooth_to_camera_trans, camera_params);
  if (image_pose) {
    return mask.data[static_cast<int>(image_pose->y() / sem_seg_output_scale) *
                         mask.cols +
                     static_cast<int>(image_pose->x() / sem_seg_output_scale)];
  } else {
    return -1;
  }
}

SegmentationType GetClusterSegmentationType(const ProposedCluster& cluster,
                                            const cv::Mat& mask,
                                            const VehiclePose& image_pose,
                                            const CameraParams& camera_params,
                                            const double sem_seg_output_scale) {
  const auto obstacles_ptr = cluster.obstacles();
  std::array<int, SegmentationType_ARRAYSIZE>
      cluster_points_segmentation_type_array = {};
  for (const auto& obstacle_ptr : obstacles_ptr) {
    for (const auto& laser_point : obstacle_ptr->points) {
      const auto point_segmentation_type = GetPointSegmentationType(
          laser_point, mask, image_pose, camera_params, sem_seg_output_scale);
      cluster_points_segmentation_type_array[point_segmentation_type]++;
    }
  }

  const auto max_point_type =
      std::max_element(cluster_points_segmentation_type_array.begin(),
                       cluster_points_segmentation_type_array.end());

  return static_cast<SegmentationType>(std::distance(
      cluster_points_segmentation_type_array.begin(), max_point_type));
}

// NOTE(zhangtao): Judge the cluster whether behind ego car.
bool IsBehindEgoCar(const ProposedCluster& cluster,
                    const std::optional<VehiclePose> pose) {
  QCHECK(!(pose == std::nullopt));
  for (const auto* obstacle : cluster.obstacles()) {
    const Vec2d& obstacle_coor_in_vehicle =
        Vec2d(obstacle->x - pose->x, obstacle->y - pose->y)
            .FastRotate(-pose->yaw);
    if (obstacle_coor_in_vehicle.x() < 0) {
      return true;
    }
  }

  return false;
}

std::optional<int> GetClusterInstanceType(
    const ProposedCluster& cluster, const cv::Mat& mask,
    const VehiclePose& image_pose, const CameraParams& camera_params,
    const double sem_seg_output_scale, const std::optional<VehiclePose> pose) {
  if (IsBehindEgoCar(cluster, pose)) {
    return std::nullopt;
  }
  absl::flat_hash_map<int, int> cluster_points_type_map;
  for (const auto& obstacle_ptr : cluster.obstacles()) {
    for (int i = obstacle_ptr->above_ground_points_start_index();
         i < obstacle_ptr->points.size(); i++) {
      const auto point_instance_type =
          GetPointIsSameInstance(obstacle_ptr->points[i], mask, image_pose,
                                 camera_params, sem_seg_output_scale);
      auto& num_points =
          LookupOrInsert(&cluster_points_type_map, point_instance_type, 0);
      num_points++;
    }
  }

  int instance_type = -1;
  int max_instance_type = -1;
  constexpr int kBackGroundTypeValue = 0;
  for (const auto& it : cluster_points_type_map) {
    if (max_instance_type < it.second) {
      if (it.first == kBackGroundTypeValue) {
        continue;
      }
      instance_type = it.first;
      max_instance_type = it.second;
    }
  }

  return instance_type == -1 ? std::nullopt
                             : std::make_optional<int>(instance_type);
}

bool IsSameInstance(const ProposedCluster& current_cluster,
                    const ProposedCluster& relative_cluster,
                    const std::optional<cv::Mat> mask,
                    const std::optional<VehiclePose> image_pose,
                    const std::optional<CameraParams> camera_params,
                    const std::optional<double> sem_seg_output_scale,
                    const std::optional<VehiclePose> pose) {
  // Note(zhangtao): if the current cluster and relative cluster are all
  // vehicle, needn't merge two clusters.
  const auto box_1 = current_cluster.bounding_box();
  const auto box_2 = relative_cluster.bounding_box();
  // TODO(zhangtao): Add the pillar semantic instance result as reference
  if (box_1 && box_2) {
    constexpr double kMaxGapWithTwoDetectionBox = 1.0;  // m
    if (!(box_1->HasOverlap(*box_2)) &&
        box_1->DistanceTo(*box_2) > kMaxGapWithTwoDetectionBox) {
      const auto current_velocity = current_cluster.fen_velocity();
      const auto relative_velocity = relative_cluster.fen_velocity();
      QCHECK(current_velocity && relative_velocity);
      const double kMaxVelocity = 5.0;  // m/s
      const auto max_velocity = std::min(
          std::max(current_velocity->norm(), relative_velocity->norm()),
          kMaxVelocity);
      if (max_velocity > 2.0) {
        constexpr double kSafeInterval = 2.0;
        if (!(box_1->DistanceTo(*box_2) < kSafeInterval * max_velocity)) {
          return false;
        }
      } else {
        return false;
      }
    }
  } else {
    // Note(zhangtao):In order to avoid under segmentation caused by a noise
    // point cloud behind a vehicle or a traffic sign
    const Vec3d current_center_point = GetCenterPointOfCluster(current_cluster);
    const Vec3d relative_center_point =
        GetCenterPointOfCluster(relative_cluster);
    const Vec2d ego_car_coord = pose->coord2d();
    const double current_cluster_to_vehicle_dist_sqr =
        Sqr(current_center_point.x() - ego_car_coord.x()) +
        Sqr(current_center_point.y() - ego_car_coord.y());
    const double relative_cluster_to_vehicle_dist_sqr =
        Sqr(relative_center_point.x() - ego_car_coord.x()) +
        Sqr(relative_center_point.y() - ego_car_coord.y());
    const auto current_box = GetMinAreaBox(current_cluster);
    const auto relative_box = GetMinAreaBox(relative_cluster);

    constexpr double kMaxBoundingBoxArea = 2.0;         // m2
    constexpr double kMinDisOfCurrentToRelative = 1.0;  // m

    if (relative_cluster_to_vehicle_dist_sqr <
            current_cluster_to_vehicle_dist_sqr &&
        relative_cluster.obstacles().size() <
            current_cluster.obstacles().size()) {
      if (relative_box.area() < kMaxBoundingBoxArea &&
          current_box.DistanceTo(relative_box) > kMinDisOfCurrentToRelative) {
        return false;
      }
    } else if (relative_cluster_to_vehicle_dist_sqr >
                   current_cluster_to_vehicle_dist_sqr &&
               relative_cluster.obstacles().size() >
                   current_cluster.obstacles().size()) {
      if (current_box.area() < kMaxBoundingBoxArea &&
          current_box.DistanceTo(relative_box) > kMinDisOfCurrentToRelative) {
        return false;
      }
    }
  }
  if (!mask || !image_pose || !camera_params || !sem_seg_output_scale) {
    VLOG(2) << "the params is nullptr";
    return false;
  }
  const std::optional<int> current_instance_type =
      GetClusterInstanceType(current_cluster, *mask, *image_pose,
                             *camera_params, *sem_seg_output_scale, pose);
  const std::optional<int> relative_instance_type =
      GetClusterInstanceType(relative_cluster, *mask, *image_pose,
                             *camera_params, *sem_seg_output_scale, pose);

  // Note(zhangtao): In order to avoid some problems of instance
  // segmentation, resulting in wrong merge
  if (!current_instance_type || !relative_instance_type) {
    return false;
  }

  constexpr int kBackGroundType = 0;
  if (current_instance_type == kBackGroundType ||
      relative_instance_type == kBackGroundType) {
    return false;
  }

  return current_instance_type == relative_instance_type;
}

bool IsLargeVehicle(const ProposedCluster& cluster) {
  auto vehicle_box = cluster.bounding_box();
  if (!vehicle_box) {
    return false;
  }
  const double length = vehicle_box->length();
  constexpr double kLargeVehicleLength = 6.0;  // m
  if (length > kLargeVehicleLength) {
    return true;
  }
  return false;
}

bool IsConnectOfTwoClustersWithPoint(const ProposedCluster& cluster1,
                                     const ProposedCluster& cluster2) {
  const auto polygon1 = cluster_util::ComputeContour(cluster1);
  const auto polygon2 = cluster_util::ComputeContour(cluster2);

  constexpr double kDistance = 0.05;  // m
  const double distance = polygon1.DistanceTo(polygon2);
  if (distance > kDistance) {
    return false;
  }
  return true;
}

float MeanIntensityOfObstacle(const Obstacle& obs) {
  uint8_t intensity_mean = 0.0;
  for (int i = obs.above_ground_points_start_index(); i < obs.points.size();
       i++) {
    intensity_mean += obs.points[i].intensity;
  }
  return intensity_mean / static_cast<float>(obs.num_points_above_ground);
}

float MeanHeightOfObstacle(const Obstacle& obs) {
  float height_mean = 0.0;
  for (int i = obs.above_ground_points_start_index(); i < obs.points.size();
       i++) {
    height_mean += obs.points[i].z;
  }

  return height_mean / static_cast<float>(obs.num_points_above_ground);
}

bool IsSimilarObstacles(const Obstacle& obs1, const Obstacle& obs2) {
  const float intensity_1 = MeanIntensityOfObstacle(obs1);
  const float intensity_2 = MeanIntensityOfObstacle(obs2);

  constexpr float kMaxIntensityDev = 10.0;
  const bool flag_intensity =
      (std::abs(intensity_1 - intensity_2) < kMaxIntensityDev);
  const float height_1 = MeanHeightOfObstacle(obs1);
  const float height_2 = MeanHeightOfObstacle(obs2);
  constexpr float kMaxHeightDev = 0.5;  // m

  const bool flag_height = (std::abs(height_1 - height_2) < kMaxHeightDev);

  return flag_height || flag_intensity;
}

// Note(zhangtao): the near is the distance of vehicle bounding box to ego
// car less than 15m, the threshold will maybe change in the future
bool IsConnectOfTwoClustersWithObstacleNear(const ProposedCluster& cluster1,
                                            const ProposedCluster& cluster2,
                                            const int radius) {
  const auto obstacles = cluster1.obstacles();
  const obstacle_util::LocalObstacleGrid local_obstacle_grid(obstacles);
  int count_obstacle_points = 0;
  absl::flat_hash_set<int> connected_obstacle_indexes;
  for (const auto& obstacle_ptr : cluster2.obstacles()) {
    const auto neighbor_raw_indices = local_obstacle_grid.FindNearestInRadius(
        obstacle_ptr->row, obstacle_ptr->col, radius);

    // Wheather the nearest obstacle's property is the same.
    for (const int& index : neighbor_raw_indices) {
      if (IsSimilarObstacles(*obstacles[index], *obstacle_ptr)) {
        if (!ContainsKey(connected_obstacle_indexes, index)) {
          connected_obstacle_indexes.insert(index);
          count_obstacle_points += (*obstacles[index]).points.size();
        }
      }
    }
  }

  constexpr int kMinNumPointsTimeNumObstacle = 50;
  constexpr int kMinNumObstacle = 3;
  constexpr int kMinNumPointsGtMinNumObstacle = 12;

  // Note(zhangtao): in order to avoid the lidar reflection points with two cars
  // lead to be merged with two cars.
  return (connected_obstacle_indexes.size() >= kMinNumObstacle &&
          count_obstacle_points >= kMinNumPointsGtMinNumObstacle) ||
         count_obstacle_points * connected_obstacle_indexes.size() >=
             kMinNumPointsTimeNumObstacle;
}

// Note(zhangtao): the near is the distance of vehicle bounding box to ego
// car less than 30m, the threshold will maybe change in the future.
bool IsConnectOfTwoClustersWithObstacleMiddle(const ProposedCluster& cluster1,
                                              const ProposedCluster& cluster2,
                                              const int radius) {
  const auto obstacles = cluster1.obstacles();
  const obstacle_util::LocalObstacleGrid local_obstacle_grid(obstacles);
  int count = 0;
  for (const auto& obstacle_ptr : cluster2.obstacles()) {
    const auto neighbor_raw_indices = local_obstacle_grid.FindNearestInRadius(
        obstacle_ptr->row, obstacle_ptr->col, radius);

    // Wheather the nearest obstacle's property is the same.
    for (const int& index : neighbor_raw_indices) {
      if (IsSimilarObstacles(*obstacles[index], *obstacle_ptr)) {
        count += 1;
      }
    }
  }

  return count >= 1;
}

bool IsConnectOfTwoClustersWithObstacle(const ProposedCluster& cluster1,
                                        const ProposedCluster& cluster2,
                                        const int radius) {
  const obstacle_util::LocalObstacleGrid local_obstacle_grid(
      cluster1.obstacles());
  absl::flat_hash_set<int> connected_obstacle_indexes;
  for (const auto& obstacle_ptr : cluster2.obstacles()) {
    const auto neighbor_raw_indices = local_obstacle_grid.FindNearestInRadius(
        obstacle_ptr->row, obstacle_ptr->col, radius);
    for (const auto& i : neighbor_raw_indices) {
      if (!ContainsKey(connected_obstacle_indexes, i)) {
        connected_obstacle_indexes.insert(i);
      }
    }
  }

  int min_limit_connected_obstacles_quantity = 20;
  // Note(zhangtao): in order to avoid two large vehicle be merged.
  const std::optional<Box2d> box1 = cluster1.bounding_box();
  const std::optional<Box2d> box2 = cluster2.bounding_box();
  constexpr double kMaxDistWithTwoClusterBoundingbox = 0.5;
  if (box1 && box2) {
    if (box1->HasOverlap(*box2)) {
      min_limit_connected_obstacles_quantity = 0;
    } else if (box1->DistanceTo(*box2) < kMaxDistWithTwoClusterBoundingbox) {
      // Note(zhangtao): if the pillar instance result is valid, replace this
      // hack with pillar instance result
      min_limit_connected_obstacles_quantity = 10;
    } else {
      min_limit_connected_obstacles_quantity = 17;
    }
  } else if (radius == 3) {
    min_limit_connected_obstacles_quantity = 2;
  } else if (radius == 5) {
    min_limit_connected_obstacles_quantity = 0;
  }

  return connected_obstacle_indexes.size() >
         min_limit_connected_obstacles_quantity;
}

bool IsBarrier(const ProposedCluster& cluster) {
  if (cluster.type() == MT_BARRIER) {
    return true;
  }
  int barrier_count = 0;
  for (const auto& obstacle_ptr : cluster.obstacles()) {
    if (obstacle_ptr->type == ObstacleProto::BARRIER) {
      barrier_count++;
    }
  }

  if (barrier_count > cluster.obstacles().size() * 0.5) {
    return true;
  }

  return false;
}

std::map<Vec2d, int> GetClustersCenterMap(const ProposedClusters& clusters,
                                          std::vector<Vec2d>* center_points) {
  std::map<Vec2d, int> center_point_map;
  for (int i = 0; i < clusters.size(); i++) {
    const auto center_point_3d = GetCenterPointOfCluster(clusters[i]);
    const Vec2d center_point = {center_point_3d.x(), center_point_3d.y()};
    center_points->push_back({center_point.x(), center_point.y()});
    // Judge whether the cluster is noise
    if (clusters[i].HasProperty(PP_NOISE)) {
      VLOG(2) << "the cluster[" << i << "] is a PP_NOISE type";
      continue;
    }

    // Judge whether the cluster is person or cyclist
    if (clusters[i].type() == MT_CYCLIST ||
        clusters[i].type() == MT_PEDESTRIAN) {
      continue;
    }

    if (clusters[i].type() == MT_VEGETATION) {
      continue;
    }

    // Judge whether the cluster is barrier, some vehicle may merge the
    // barrier cluster
    if (IsBarrier(clusters[i])) {
      VLOG(2) << "the cluster[" << i << "] is kBarrier type.";
      continue;
    }
    center_point_map.insert(std::make_pair(center_point, i));
  }

  return center_point_map;
}

std::map<Vec2d, int> GetClusterContourMap(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters,
    std::vector<Vec2d>* contour_vertex_points) {
  std::map<Vec2d, int> contour_vertex_map;
  for (int i = 0; i < clusters.size(); i++) {
    // Judge whether the cluster is noise
    if (clusters[i].HasProperty(PP_NOISE)) {
      VLOG(2) << "the cluster[" << i << "] is a PP_NOISE type";
      continue;
    }

    // Judge whether the cluster is person or cyclist
    if (clusters[i].type() == MT_CYCLIST ||
        clusters[i].type() == MT_PEDESTRIAN) {
      continue;
    }

    if (clusters[i].type() == MT_VEGETATION) {
      continue;
    }

    // Judge whether the cluster is barrier, some vehicle may merge the
    // barrier cluster
    if (IsBarrier(clusters[i])) {
      VLOG(2) << "the cluster[" << i << "] is kBarrier type.";
      continue;
    }
    const auto contour = cluster_util::ComputeContour(clusters[i]);
    const std::vector<Vec2d> vertices = contour.GetAllVertices();
    for (const Vec2d& vertex : vertices) {
      contour_vertex_map.insert(std::make_pair(vertex, i));
      contour_vertex_points->push_back(vertex);
    }

    if (FLAGS_cluster_polygon_cvs) {
      constexpr double kBoundingBoxHeight = 6.0;  // m
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/cluster_polygon_cvs");
      canvas.DrawPolygon(contour, env_info.pose().z + kBoundingBoxHeight,
                         vis::Color::kMiddleBlueGreen);

      const auto box = contour.MinAreaBoundingBox();
      canvas.DrawBox({box.center(), env_info.pose().z + kBoundingBoxHeight},
                     box.heading(), {box.length(), box.width()},
                     vis::Color::kYellow);
    }
  }

  return contour_vertex_map;
}

// Determine the search radius according to the position relationship between
// the cluster and the cluster
std::set<int> FindNearestInRaius(const Vec2d& point,
                                 const ProposedCluster& cluster,
                                 const Position& position,
                                 const KDTree<Vec2d> kdtree) {
  double length = 0;
  std::optional<Box2d> bounding_box = cluster.bounding_box();
  if (bounding_box) {
    length = bounding_box->length();
  } else {
    bounding_box = std::make_optional(GetMinAreaBox(cluster));
    length = bounding_box->length();
  }
  double radius = 0;
  switch (position) {
    case FRONT_P: {
      radius = std::clamp(length * 1.5, 6.0, 20.0);
      break;
    }
    case FRONT_V: {
      radius = length * 1.5;
      break;
    }
    case BACK: {
      radius = length;
      break;
    }
    case MIDDLE_FRONT:
    case MIDDLE_BACK:
    case RIGHT:
    case LEFT: {
      radius = length * 1.5;
      break;
    }
    case RIGHT_BACK:
    case RIGHT_FRONT:
    case LEFT_BACK:
    case LEFT_FRONT: {
      radius = length * 2;
      break;
    }
  }

  constexpr double kMinRadius = 2;  // m 6.0

  if (radius < kMinRadius) {
    radius *= 2;
  }

  std::set<int> nearest = kdtree.FindNearestInRadius(point, radius * radius);

  return nearest;
}

bool IsSameVehicle(const Position& position,
                   const ProposedCluster& current_cluster,
                   const ProposedCluster& relative_cluster,
                   const std::optional<cv::Mat> mask,
                   const std::optional<VehiclePose> image_pose,
                   const std::optional<CameraParams> camera_params,
                   const std::optional<double> sem_seg_output_scale,
                   const std::optional<VehiclePose> pose) {
  int radius = 1;  // one obstacle gird
  switch (position) {
    case FRONT_P: {
      // The large vehicle is in front fo our car, it is directly judged by the
      // image whether it is in the same instance
      if (FLAGS_enable_image_instance_result) {
        return IsSameInstance(current_cluster, relative_cluster, mask,
                              image_pose, camera_params, sem_seg_output_scale,
                              pose);
      } else {
        return false;
      }
    }
    case FRONT_V: {
      radius = 5;
      break;
    }
    case RIGHT:
    case LEFT: {
      return IsConnectOfTwoClustersWithObstacleNear(current_cluster,
                                                    relative_cluster, radius);
    }
    case BACK: {
      radius = 5;
      break;
    }
    case MIDDLE_FRONT:
    case MIDDLE_BACK: {
      radius = 3;
      break;
    }
    case RIGHT_BACK: {
      radius = 5;
      break;
    }
    case RIGHT_FRONT: {
      radius = 5;
      break;
    }
    case LEFT_BACK: {
      radius = 5;
      break;
    }
    case LEFT_FRONT: {
      radius = 5;
      break;
    }
  }
  return IsConnectOfTwoClustersWithObstacle(current_cluster, relative_cluster,
                                            radius);
}

absl::flat_hash_set<int> DeDuplicationClusterIndex(
    const std::set<int>& indexs, const KDTree<Vec2d>& kdtree,
    const std::map<Vec2d, int>& contour_vertex_map) {
  absl::flat_hash_set<int> de_duplicate_indexs;
  for (const auto& it : indexs) {
    const Vec2d point = kdtree.PointAt(it);
    int index = -1;
    if (FindCopy(contour_vertex_map, point, &index)) {
      de_duplicate_indexs.insert(index);
    }
  }
  return de_duplicate_indexs;
}

// Note(zhangtao): center_point_map is the cluster center point corresponding to
// cluster index map, and contour vertex_map is the cluster's contour's vertex
// corresponding to cluster index map
std::vector<int> GetVehicleClusters(
    const VehiclePose& pose,
    const std::map<Vec2d, int>& center_point_to_index_map,
    const std::map<Vec2d, int>& contour_vertex_map,
    const std::vector<Vec2d>& index_to_center_point_map,
    const ProposedCluster& cluster, const ProposedClusters& clusters,
    const KDTree<Vec2d>& kdtree, const std::optional<cv::Mat> mask,
    const std::optional<VehiclePose> image_pose,
    const std::optional<CameraParams> camera_params,
    const std::optional<double> sem_seg_output_scale) {
  QCHECK(cluster.bounding_box());
  const auto center_point_3d = GetCenterPointOfCluster(cluster);
  const Vec2d center_point = {center_point_3d.x(), center_point_3d.y()};
  int index_of_cluster = -1;
  if (!FindCopy(center_point_to_index_map, center_point, &index_of_cluster)) {
    QLOG(WARNING)
        << "the point not find in center_point_to_index_map, the point is : ("
        << center_point.x() << ", " << center_point.y() << ")";
  }
  std::deque<Vec2d> point_deque;
  point_deque.push_back(center_point);
  std::vector<int> cluster_indices;

  std::vector<bool> visit(clusters.size(), false);
  // Note(zhangtao): set current cluster to be visited
  if (index_of_cluster != -1) {
    visit[index_of_cluster] = true;
  }

  // The coordinate of the vehicle
  const Vec2d vehicle_coord = pose.coord2d();

  // Here only need to consider the relationship between the self-vehicle and
  // the current cluster position
  const Position position =
      GetClusterRelativePosition(vehicle_coord, pose.yaw, cluster);

  if (position == BACK) {
    return std::vector<int>();
  }

  while (!point_deque.empty()) {
    const auto current_center_point = point_deque.front();
    point_deque.pop_front();
    int index = -1;
    if (!FindCopy(center_point_to_index_map, current_center_point, &index)) {
      QLOG(WARNING)
          << "the point not find in center_point_to_index_map, the point is : ("
          << current_center_point.x() << ", " << current_center_point.y()
          << ")";
      continue;
    }
    const auto current_cluster = clusters[index];

    const std::set<int> nearest = FindNearestInRaius(
        current_center_point, current_cluster, position, kdtree);

    const absl::flat_hash_set<int> de_duplicate_nearest =
        DeDuplicationClusterIndex(nearest, kdtree, contour_vertex_map);
    for (const auto& i : de_duplicate_nearest) {
      // maybe find the point itself, need to filter it
      if (i == index) {
        continue;
      }

      // already visited
      if (visit[i]) {
        continue;
      }

      // Judge whether the cluster in front of ego car and the the distance is
      // less than 30, avoid under segmentation occur
      constexpr double kMinFrontDistanceToBeMerged = Sqr(20.0);  // m
      if (position == FRONT_P &&
          GetDistanceSqrOfVehicleWithEgoCar(vehicle_coord, clusters[i]) <
              kMinFrontDistanceToBeMerged) {
        continue;
      }

      if (!AreInTheSameLane(cluster, clusters[i])) {
        continue;
      }

      const Position relative_position =
          GetClusterRelativePosition(vehicle_coord, pose.yaw, clusters[i]);

      // Judge whether two clusters belong to the same vehicle
      const bool is_same_vehicle =
          IsSameVehicle(relative_position, current_cluster, clusters[i], mask,
                        image_pose, camera_params, sem_seg_output_scale, pose);

      // Whether the current cluster is connected to near_cluster
      if (!is_same_vehicle) {
        continue;
      }

      // avoid some cluster not merge
      visit[i] = true;
      QCHECK_LT(i, index_to_center_point_map.size());
      point_deque.push_back(index_to_center_point_map[i]);
      cluster_indices.push_back(i);
    }
  }

  return cluster_indices;
}

KDTree<Vec2d> InitKdtree(const std::vector<Vec2d>& points) {
  QCHECK(!points.empty());
  return KDTree(points);
}

const std::unordered_set<CameraId>& GetLlnProposerRefCameras() {
  static const std::unordered_set<CameraId> kLlnProposerRefCameras = {
      CAM_L_FRONT, CAM_FRONT};
  return kLlnProposerRefCameras;
}

std::optional<Box2d> GenerateBoundingBoxCluster(
    const ProposedClusters& clusters, const double& heading) {
  std::vector<Vec2d> corners;
  for (const auto& cluster : clusters) {
    std::optional<Box2d> box = cluster.bounding_box();
    if (!box) {
      for (const auto* obstacle : cluster.obstacles()) {
        const auto obstacle_contour_points =
            obstacle_util::ComputeContour(*obstacle);
        corners.insert(corners.end(), obstacle_contour_points.begin(),
                       obstacle_contour_points.end());
      }
    } else {
      const std::vector<Vec2d> current_corners =
          box->GetCornersCounterClockwise();
      corners.insert(corners.end(), current_corners.begin(),
                     current_corners.end());
    }
  }

  Polygon2d polygon;
  if (Polygon2d::ComputeConvexHull(corners, &polygon)) {
    return polygon.BoundingBoxWithHeading(heading);
  }

  return std::nullopt;
}

std::vector<int> CheckAndRemoveUnmatchedClusterIndexes(
    const ProposedClusters& clusters, const std::vector<int>& cluster_indexs,
    const std::vector<bool>& visit) {
  QCHECK(!cluster_indexs.empty());
  std::vector<int> res_indexs;
  constexpr double kMinVehicleWidthToBeMerged = 2.50;  // m
  std::vector<ObstaclePtr> obstacle_ptrs;
  for (const auto& i : cluster_indexs) {
    if (visit[i]) {
      continue;
    }
    const auto box = clusters[i].bounding_box();
    if (box) {
      if (box->width() < kMinVehicleWidthToBeMerged) {
        continue;
      }
    }
    obstacle_ptrs.insert(obstacle_ptrs.end(), clusters[i].obstacles().begin(),
                         clusters[i].obstacles().end());
    res_indexs.push_back(i);
  }

  // Note(zhangtao): In order to avoid the vehicle too long.
  constexpr double kMaxClusterLength = 25.0;  // m
  constexpr double kMaxClusterWidth = 25.0;   // m
  if (!obstacle_ptrs.empty()) {
    const auto box = GetMinAreaBox(Cluster(obstacle_ptrs));
    if (box.length() > kMaxClusterLength || box.width() > kMaxClusterWidth) {
      return {};
    }
  }

  return res_indexs;
}

}  // namespace

ProposedClusters LargeVehicleProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  std::optional<SemanticSegmentationResult> ref_camera_result = std::nullopt;
  const auto& semantic_segmentation_results = env_info.context().ll_net_result;
  const auto& kLlnProposerRefCameras = GetLlnProposerRefCameras();
  for (const auto& semantic_segmentation_result :
       semantic_segmentation_results) {
    if (ContainsKey(kLlnProposerRefCameras,
                    semantic_segmentation_result->camera_id())) {
      ref_camera_result = std::make_optional(*semantic_segmentation_result);
    }
  }

  std::optional<VehiclePose> image_pose = std::nullopt;
  std::optional<CameraId> camera_id = std::nullopt;
  std::optional<cv::Mat> instance_mask = std::nullopt;
  std::optional<double> sem_seg_output_scale = std::nullopt;

  if (ref_camera_result) {
    image_pose = ref_camera_result->pose();
    camera_id = std::make_optional(ref_camera_result->camera_id());
    instance_mask = std::make_optional(ref_camera_result->instance_mat());
    sem_seg_output_scale =
        std::make_optional(ref_camera_result->sem_seg_output_scale());
  }

  std::optional<CameraParams> camera_params = std::nullopt;
  if (camera_id) {
    const auto* camera_params_ptr = env_info.GetCameraParams(*camera_id);
    camera_params = camera_params_ptr == nullptr
                        ? std::nullopt
                        : std::make_optional(*camera_params_ptr);
  }

  std::optional<LidarParametersProto> ref_lidar_param = std::nullopt;
  if (camera_params) {
    const auto* ref_lidar_param_ptr =
        env_info.GetLidarParams(camera_params->ref_lidar());
    ref_lidar_param = ref_lidar_param_ptr != nullptr
                          ? std::make_optional(*ref_lidar_param_ptr)
                          : std::nullopt;
  }

  std::vector<Vec2d> center_points;
  const std::map<Vec2d, int> center_point_map =
      GetClustersCenterMap(clusters, &center_points);

  std::vector<Vec2d> contour_vertex_points;
  const std::map<Vec2d, int> contour_vertex_map =
      GetClusterContourMap(env_info, clusters, &contour_vertex_points);

  if (contour_vertex_points.empty() || center_points.empty()) {
    VLOG(2) << "either the center_points or contour_vertex_points is empty.";
    return clusters;
  }

  const auto kdtree = InitKdtree(contour_vertex_points);

  const int size = clusters.size();
  ProposedClusters proposed_clusters;
  std::vector<bool> process(size, false);

  for (int i = 0; i < size; i++) {
    if (!IsLargeVehicle(clusters[i])) {
      continue;
    }
    if (process[i]) {
      continue;
    }
    std::vector<int> cluster_indexs = GetVehicleClusters(
        env_info.pose(), center_point_map, contour_vertex_map, center_points,
        clusters[i], clusters, kdtree, instance_mask, image_pose, camera_params,
        sem_seg_output_scale);
    if (cluster_indexs.empty()) {
      continue;
    }

    cluster_indexs.push_back(i);

    cluster_indexs = CheckAndRemoveUnmatchedClusterIndexes(
        clusters, cluster_indexs, process);

    if (cluster_indexs.empty()) {
      continue;
    }

    int obstacles_size = 0;
    ProposedClusters temp_clusters;
    temp_clusters.reserve(cluster_indexs.size());
    std::vector<int> dump_indexs;
    int last_unprocess = -1;
    for (const int index : cluster_indexs) {
      if (process[index]) {
        continue;
      }
      process[index] = true;
      obstacles_size += clusters[index].obstacles().size();
      temp_clusters.push_back(clusters[index]);
      dump_indexs.push_back(index);
      last_unprocess = index;
    }

    ObstaclePtrs current_cluster_obstacle_ptrs;
    current_cluster_obstacle_ptrs.reserve(obstacles_size);

    for (const auto& temp_cluster : temp_clusters) {
      for (const auto& obstacle_ptr : temp_cluster.obstacles()) {
        current_cluster_obstacle_ptrs.push_back(obstacle_ptr);
      }
    }

    if (temp_clusters.size() == 1 && last_unprocess != -1) {
      VLOG(2) << "the temp_clusters only has one cluster, no need merge.";
      process[last_unprocess] = false;
      continue;
    }

    std::optional<Box2d> box = std::nullopt;

    if (clusters[i].bounding_box()) {
      box = GenerateBoundingBoxCluster(temp_clusters,
                                       clusters[i].bounding_box()->heading());
      if (!box) {
        for (const int idx : dump_indexs) {
          process[idx] = false;
        }
        continue;
      }
    }

    proposed_clusters.emplace_back(
        ProposedCluster::InheritFrom(temp_clusters)
            .ConstructBase(std::move(current_cluster_obstacle_ptrs)));
    proposed_clusters.back().set_type(clusters[i].type());
    proposed_clusters.back().set_type_source(clusters[i].type_source());
    proposed_clusters.back().set_score(clusters[i].score());
    if (clusters[i].fen_velocity()) {
      proposed_clusters.back().set_fen_velocity(*(clusters[i].fen_velocity()));
    }
    proposed_clusters.back().set_is_proposed(true);

    if (box) {
      proposed_clusters.back().set_bounding_box(*box);
    }

    if (FLAGS_large_vehicle_proposer_cvs) {
      auto current_box = proposed_clusters.back().bounding_box();
      if (!current_box) {
        current_box = GetMinAreaBox(proposed_clusters.back());
      }
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/vehicle_proposer_cvs");
      vis::Color color = vis::Color::kRed;
      constexpr double kBoundingBoxHeight = 8;
      constexpr double kTimesOfDir = 6;
      canvas.DrawBox(
          {current_box->center(), env_info.pose().z + kBoundingBoxHeight},
          current_box->heading(), {current_box->length(), current_box->width()},
          vis::Color::kYellow);
      const Vec2d dir = Vec2d::FastUnitFromAngle(current_box->heading());
      canvas.DrawLine(
          {current_box->center(), env_info.pose().z + kBoundingBoxHeight},
          {current_box->center() + dir * kTimesOfDir,
           env_info.pose().z + kBoundingBoxHeight},
          vis::Color::kYellow);
    }
  }

  for (int i = 0; i < size; i++) {
    if (!process[i]) {
      proposed_clusters.push_back(clusters[i]);
    }
  }

  QCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  return proposed_clusters;
}
}  // namespace qcraft::segmentation
