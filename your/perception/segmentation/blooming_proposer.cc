#include "onboard/perception/segmentation/blooming_proposer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "offboard/mapping/mapping_core/util/point_cloud_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/utils/map_util.h"
#include "opencv2/core/types.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

DEFINE_bool(blooming_proposer_cvs, false, "Enable blooming proposer cvs.");

namespace qcraft::segmentation {

namespace {

// Sign
constexpr int kMinNumPointsOnSign = 15;
constexpr double kMinSignArea = 0.075;      // m2
constexpr double kMaxDistanceToSign = 0.1;  // m
// Cylinder
constexpr int kMinNumPointsOnCylinder = 10;
constexpr double kMaxCylinderLength = 0.25;                // m
constexpr double kMaxCylinderWidth = 0.25;                 // m
constexpr double kMinCylinderHeight = 0.3;                 // m
constexpr double kMaxCylinderHeight = 1.5;                 // m
constexpr double kMaxCylinderPointToCentroidRadius = 1.2;  // m
constexpr double kMinCylinderRadius = 0.1;                 // m

bool IsClusterBloomingFilterable(const ProposedCluster& cluster) {
  return !(cluster.HasProperty(PP_NOISE) || cluster.type() == MT_VEHICLE ||
           cluster.type() == MT_PEDESTRIAN || cluster.type() == MT_CYCLIST ||
           cluster.type() == MT_MOTORCYCLIST || cluster.type() == MT_CONE);
}

bool IsObstacleBloomingFilterable(const Obstacle& obstacle) {
  if (obstacle.type == ObstacleProto::STATIC &&
      obstacle.type_source == ObstacleProto::DEFAULT) {
    return true;
  }
  if (obstacle.type == ObstacleProto::VEGETATION &&
      obstacle.type_source == ObstacleProto::SEMANTIC_MAP_ZONE) {
    return true;
  }

  return false;
}

bool HasM1PointType(const ObstaclePtr& obstacle) {
  for (const auto& point : obstacle->points) {
    if (IsM1Point(point.lidar_type)) return true;
  }
  return false;
}

Vec3d CalculatePointsCentroid(const std::vector<Vec3d>& points) {
  QCHECK(!points.empty());
  return std::accumulate(points.begin(), points.end(), Vec3d()) / points.size();
}

Vec3d GetRefLidarExtrinsics(const LidarParametersProto& lidar_params) {
  const auto& extrinsics = lidar_params.installation().extrinsics();
  return {extrinsics.x(), extrinsics.y(), extrinsics.z()};
}

void MaybeRenderSign(const RoadSign& sign, const LidarId lidar_id,
                     const VehiclePose& pose, const double angle_diff) {
  if (!FLAGS_blooming_proposer_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/blooming_proposer_cvs");
  vis::Color color = vis::Color::kWhite;
  if (lidar_id == LDR_FRONT_LEFT) {
    color = vis::Color::kGreen;
  } else if (lidar_id == LDR_FRONT_RIGHT) {
    color = vis::Color::kRed;
  }
  for (const auto& point : sign.points) {
    canvas.DrawPoint(point, color, 3);
  }
  canvas.DrawPoint(sign.centroid, vis::Color::kBlue, 5);
  canvas.DrawLine(sign.centroid, sign.centroid + sign.normal,
                  vis::Color::kYellow, 1);
  canvas.DrawBox({sign.bounding_box.center(), pose.z},
                 sign.bounding_box.heading(),
                 {sign.bounding_box.length(), sign.bounding_box.width()},
                 vis::Color::kYellow);
  canvas.DrawText(
      absl::StrFormat("[Sign][%s] %s, angle_diff: %.2f", LidarId_Name(lidar_id),
                      sign.DebugString(), angle_diff),
      {sign.bounding_box.center(), 0}, 0.0, 0.01, vis::Color::kWhite);
}

void MaybeRenderCylinder(const RoadCylinder& cylinder, const LidarId lidar_id,
                         const VehiclePose& pose) {
  if (!FLAGS_blooming_proposer_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/blooming_proposer_cvs");
  vis::Color color = vis::Color::kWhite;
  if (lidar_id == LDR_FRONT_LEFT) {
    color = vis::Color::kGreen;
  } else if (lidar_id == LDR_FRONT_RIGHT) {
    color = vis::Color::kRed;
  }
  for (const auto& point : cylinder.points) {
    canvas.DrawPoint(point, color, 3);
  }
  canvas.DrawPoint(cylinder.centroid, vis::Color::kBlue, 5);
  canvas.DrawLine(cylinder.centroid, cylinder.centroid + cylinder.axis,
                  vis::Color::kYellow, 1);
  canvas.DrawBox(
      {cylinder.bounding_box.center(), pose.z}, cylinder.bounding_box.heading(),
      {cylinder.bounding_box.length(), cylinder.bounding_box.width()},
      vis::Color::kYellow);
  canvas.DrawText(absl::StrFormat("[Cylinder][%s] %s", LidarId_Name(lidar_id),
                                  cylinder.DebugString()),
                  {cylinder.bounding_box.center(), 0}, 0.0, 0.01,
                  vis::Color::kWhite);
}

void MaybeRenderObstacleInBloomingRange(const Obstacle& obstacle,
                                        const double av_height,
                                        const double radius_diff) {
  if (!FLAGS_blooming_proposer_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/blooming_proposer_cvs");
  canvas.DrawBox(Vec3d(obstacle.coord(), obstacle.ground_z), 0.0,
                 {Obstacle::kDiameter, Obstacle::kDiameter},
                 vis::Color(0.0, 0.0, 0.0, 0.0), vis::Color::kDarkGray);
  canvas.DrawText(absl::StrFormat("clearance: %.2f", obstacle.clearance),
                  Vec3d(obstacle.coord(), obstacle.ground_z + 0.05), 0.0, 0.01,
                  vis::Color::kWhite);
  canvas.DrawText(
      absl::StrFormat("radius diff: %.2f", radius_diff),
      Vec3d(obstacle.coord() + Vec2d(0, 0.02), obstacle.ground_z + 0.05), 0.0,
      0.01, vis::Color::kWhite);
  int total_intensity = 0;
  int num_points_above_ground = 0;
  for (const auto& point : obstacle.points) {
    if (!obstacle_util::IsRealObstaclePoint(obstacle, point, av_height)) {
      continue;
    }
    ++num_points_above_ground;
    total_intensity += point.intensity;
  }
  if (num_points_above_ground) {
    canvas.DrawText(
        absl::StrFormat("avg intensity: %.2f",
                        total_intensity / num_points_above_ground),
        Vec3d(obstacle.coord() + Vec2d(0, 0.04), obstacle.ground_z + 0.05), 0.0,
        0.01, vis::Color::kWhite);
  }
}

void MaybeRenderRetroreflectedObstacle(const Obstacle& obstacle,
                                       const LidarId lidar_id,
                                       const bool is_ouster_obstacle,
                                       const bool is_low_intensity_obstacle) {
  if (!FLAGS_blooming_proposer_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/blooming_proposer_cvs");
  vis::Color color = vis::Color::kBlue;
  if (lidar_id == LDR_FRONT_LEFT) {
    color = vis::Color::kDarkGreen;
  } else if (lidar_id == LDR_FRONT_RIGHT) {
    color = vis::Color::kDarkRed;
  }
  color.a() = 0.5;
  canvas.DrawBox(Vec3d(obstacle.coord(), obstacle.ground_z + 0.01), 0.0,
                 {Obstacle::kDiameter, Obstacle::kDiameter},
                 vis::Color(0.0, 0.0, 0.0, 0.0), color);
  canvas.DrawText(
      absl::StrFormat("is ouster: %s, is low intensity: %s",
                      is_ouster_obstacle ? "YES" : "NO",
                      is_low_intensity_obstacle ? "YES" : "NO"),
      Vec3d(obstacle.coord() + Vec2d(0, 0.06), obstacle.ground_z + 0.05), 0.0,
      0.01, vis::Color::kBlue);
}

template <typename Reflector>
void MaybeRenderReflectorCvs(const Reflector& reflector, const LidarId lidar_id,
                             const VehiclePose& pose,
                             const Vec3d& ref_lidar_pos,
                             const int max_blooming_obstacle_range,
                             const double max_blooming_range_radius_diff) {
  if (!FLAGS_blooming_proposer_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/blooming_proposer_cvs");
  vis::Color color = vis::Color::kBlue;
  if (lidar_id == LDR_FRONT_LEFT) {
    color = vis::Color::kDarkGreen;
  } else if (lidar_id == LDR_FRONT_RIGHT) {
    color = vis::Color::kDarkRed;
  }
  const auto centroid_2d = reflector.bounding_box.center();
  canvas.DrawBox(Vec3d(centroid_2d, pose.z), 0.0,
                 {Obstacle::kDiameter * (max_blooming_obstacle_range * 2 + 1),
                  Obstacle::kDiameter * (max_blooming_obstacle_range * 2 + 1)},
                 color);
  const double centroid_dis_to_ref_lidar = Hypot(
      centroid_2d.x() - ref_lidar_pos.x(), centroid_2d.y() - ref_lidar_pos.y(),
      reflector.centroid.z() - ref_lidar_pos.z());
  canvas.DrawCircle(Vec3d(ref_lidar_pos.x(), ref_lidar_pos.y(), pose.z),
                    centroid_dis_to_ref_lidar + max_blooming_range_radius_diff,
                    color);
  canvas.DrawCircle(Vec3d(ref_lidar_pos.x(), ref_lidar_pos.y(), pose.z),
                    centroid_dis_to_ref_lidar - max_blooming_range_radius_diff,
                    color);
}

std::vector<Vec3d> ClusterPoints(const std::vector<Vec3d>& points,
                                 const double threshold) {
  QCHECK(!points.empty());
  std::vector<Vec3d> clustered_points;
  clustered_points.reserve(points.size());
  constexpr int kMaxNumInterations = 10;
  int num_iterations = 0;
  Vec3d prev_centroid = CalculatePointsCentroid(points);
  do {
    clustered_points.clear();
    for (const auto& point : points) {
      if (point.DistanceSquareTo(prev_centroid) < Sqr(threshold)) {
        clustered_points.emplace_back(point);
      }
    }
    if (clustered_points.size() == points.size() || clustered_points.empty()) {
      break;
    }
    const auto& centroid = CalculatePointsCentroid(clustered_points);
    constexpr double kMaxDistanceSquare = 1e-3;
    if (prev_centroid.DistanceSquareTo(centroid) < kMaxDistanceSquare) {
      break;
    }
    prev_centroid = centroid;

    ++num_iterations;
  } while (num_iterations < kMaxNumInterations);

  return clustered_points;
}

// TODO(dong): Introduce a customized ransac implementation and deprecate pcl
// usuage later.
bool FitPlane3DWithRansac(const std::vector<Vec3d>& points,
                          const double threshold, Vec3d* plane_normal,
                          std::vector<int>* inlier_indices) {
  if (points.size() < 3) return false;
  // NOTE(dong): Shift points to nearby the origin. pcl use float in most
  // algorithms and it is not precise enough in the smooth coordinate.
  std::vector<Vec3d> shifted_points;
  shifted_points.reserve(points.size());
  for (const auto& point : points) {
    shifted_points.emplace_back(point.x() - points[0].x(),
                                point.y() - points[0].y(),
                                point.z() - points[0].z());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      mapping::util::ToPclPointCloud(shifted_points);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  pcl::SACSegmentation<pcl::PointXYZ> sac;
  sac.setOptimizeCoefficients(true);
  sac.setModelType(pcl::SACMODEL_PLANE);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setDistanceThreshold(threshold);
  sac.setMaxIterations(100);
  sac.setInputCloud(cloud);
  sac.segment(inliers, coefficients);
  if (0 == inliers.indices.size()) {
    return false;
  }
  // ax + by + cz + d = 0.
  const double a = coefficients.values[0];
  const double b = coefficients.values[1];
  const double c = coefficients.values[2];
  const double d = coefficients.values[3];
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
      !std::isfinite(d)) {
    return false;
  }

  if (a == 0 && b == 0 && c == 0) return false;
  // Normalized normal.
  if (plane_normal) {
    const double x = std::sqrt(Sqr(a) + Sqr(b) + Sqr(c));
    *plane_normal = {a / x, b / x, c / x};
  }
  if (inlier_indices) {
    inlier_indices->resize(inliers.indices.size());
    for (int i = 0; i < inliers.indices.size(); ++i) {
      (*inlier_indices)[i] = inliers.indices[i];
    }
  }

  return true;
}

bool FitAxisWithRansac(const std::vector<Vec3d>& points, const double threshold,
                       Vec3d* axis, std::vector<int>* inlier_indices) {
  if (points.size() < 2) return false;
  // NOTE(dong): Shift points to nearby the origin. pcl use float in most
  // algorithms and it is not precise enough in the smooth coordinate.
  std::vector<Vec3d> shifted_points;
  shifted_points.reserve(points.size());
  for (const auto& point : points) {
    shifted_points.emplace_back(point.x() - points[0].x(),
                                point.y() - points[0].y(),
                                point.z() - points[0].z());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      mapping::util::ToPclPointCloud(shifted_points);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  pcl::SACSegmentation<pcl::PointXYZ> sac;
  sac.setOptimizeCoefficients(true);
  sac.setModelType(pcl::SACMODEL_LINE);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setDistanceThreshold(threshold);
  sac.setMaxIterations(100);
  sac.setInputCloud(cloud);
  sac.segment(inliers, coefficients);
  if (0 == inliers.indices.size()) {
    return false;
  }
  // (coefficients.values[0], coefficients.values[1], coefficients.values[2])
  // indicates a point on the line.
  Vec3d line_direction(coefficients.values[3], coefficients.values[4],
                       coefficients.values[5]);
  if (line_direction.squaredNorm() == 0.0) return false;
  // Always point to the positive direction of the z-axis.
  const Vec3d view_point(0.0, 0.0, 1e10);
  if (line_direction.Dot(view_point) < 0.0) {
    line_direction = -line_direction;
  }
  // Normalized normal.
  if (axis) {
    const double norm = line_direction.norm();
    *axis = {line_direction.x() / norm, line_direction.y() / norm,
             line_direction.z() / norm};
  }
  if (inlier_indices) {
    inlier_indices->resize(inliers.indices.size());
    for (int i = 0; i < inliers.indices.size(); ++i) {
      (*inlier_indices)[i] = inliers.indices[i];
    }
  }

  return true;
}

// TODO(dong): There may be more than one sign in a retroreflector. Need to
// split them later.
std::pair<std::vector<Vec3d>, Vec3d> GetPointsAndPlaneNormalFromRetroreflector(
    const Retroreflector& retroreflector, const VehiclePose& pose) {
  std::vector<Vec3d> points = retroreflector.points;
  Vec3d plane_normal;
  std::vector<int> inlier_indices;
  if (!FitPlane3DWithRansac(points, kMaxDistanceToSign, &plane_normal,
                            &inlier_indices)) {
    return {};
  }
  const Vec3d centroid =
      std::accumulate(points.begin(), points.end(), Vec3d()) / points.size();
  const Vec3d vec(centroid.x() - pose.x, centroid.y() - pose.y,
                  centroid.z() - pose.z);
  if (vec.Dot(plane_normal) > 0) {
    plane_normal = -plane_normal;
  }
  QCHECK_LE(inlier_indices.size(), points.size());
  constexpr double kMinInliersRatio = 0.5;
  if (inlier_indices.size() * 1.0 / points.size() < kMinInliersRatio) {
    return {};
  }
  std::vector<bool> is_inliers(points.size(), false);
  for (const auto index : inlier_indices) {
    is_inliers[index] = true;
  }
  int inlier_index = 0;
  for (int i = 0; i < points.size(); ++i) {
    if (is_inliers[i]) {
      points[inlier_index++] = points[i];
    }
  }
  points.resize(inlier_indices.size());

  return {points, plane_normal};
}

std::pair<std::vector<Vec3d>, Vec3d> GetPointsAndAxisFromRetroreflector(
    const Retroreflector& retroreflector) {
  std::vector<Vec3d> points = retroreflector.points;
  Vec3d axis;
  std::vector<int> inlier_indices;
  if (!FitAxisWithRansac(points, kMinCylinderRadius, &axis, &inlier_indices)) {
    return {};
  }
  QCHECK_LE(inlier_indices.size(), points.size());
  constexpr double kMinInliersRatio = 0.8;
  if (inlier_indices.size() * 1.0 / points.size() < kMinInliersRatio) {
    return {};
  }
  std::vector<bool> is_inliers(points.size(), false);
  for (const auto index : inlier_indices) {
    is_inliers[index] = true;
  }
  int inlier_index = 0;
  for (int i = 0; i < points.size(); ++i) {
    if (is_inliers[i]) {
      points[inlier_index++] = points[i];
    }
  }
  points.resize(inlier_indices.size());

  return {points, axis};
}

RoadSigns FindRoadSigns(const LidarId lidar_id,
                        const Retroreflectors& retroreflectors,
                        const VehiclePose& pose) {
  SCOPED_QTRACE("FindRoadSigns");

  std::vector<const Retroreflector*> retroreflectors_for_currend_lidar;
  for (const auto& retroreflector : retroreflectors) {
    if (retroreflector.lidar_id == lidar_id) {
      retroreflectors_for_currend_lidar.push_back(&retroreflector);
    }
  }

  if (retroreflectors_for_currend_lidar.empty()) return {};

  RoadSigns signs;
  signs.reserve(retroreflectors_for_currend_lidar.size());
  // To avoid repeated malloc/free.
  std::vector<Vec2d> points_in_birds_eye_view;
  for (const auto* retroreflector : retroreflectors_for_currend_lidar) {
    RoadSign sign;
    sign.is_overhanging = retroreflector->is_overhanging;
    // Sign points &  normal.
    std::tie(sign.points, sign.normal) =
        GetPointsAndPlaneNormalFromRetroreflector(*retroreflector, pose);
    if (sign.points.size() < kMinNumPointsOnSign) continue;
    const double kMaxSignPointToCentroidRadius =
        retroreflector->is_overhanging ? 10.0 : 2.5;  // m
    auto clustered_points =
        ClusterPoints(sign.points, kMaxSignPointToCentroidRadius);
    const double kMinNumClusteredPointsRatio = 0.80;
    if (clustered_points.size() <
        sign.points.size() * kMinNumClusteredPointsRatio) {
      continue;
    }
    sign.points = std::move(clustered_points);
    // Sign contour.
    points_in_birds_eye_view.clear();
    points_in_birds_eye_view.reserve(sign.points.size());
    for (const auto& point : sign.points) {
      points_in_birds_eye_view.emplace_back(point.x(), point.y());
    }
    if (!Polygon2d::ComputeConvexHull(points_in_birds_eye_view,
                                      &sign.contour)) {
      continue;
    }
    // Sign bounding box.
    const Vec2d heanding_vec =
        Vec2d(sign.normal.x(), sign.normal.y()).FastRotate(M_PI_2);
    sign.bounding_box =
        sign.is_overhanging
            ? sign.contour.MinAreaBoundingBox()
            : sign.contour.BoundingBoxWithHeading(heanding_vec.FastAngle());
    if (sign.bounding_box.width() > sign.bounding_box.length()) {
      sign.bounding_box = Box2d(
          sign.bounding_box.center(), sign.bounding_box.heading() + M_PI_2,
          sign.bounding_box.width(), sign.bounding_box.length());
    }
    const double kMaxSignLength = sign.is_overhanging ? 20.0 : 5.0;  // m
    const double kMaxSignWidth = 0.8;                                // m
    if (sign.bounding_box.length() > kMaxSignLength ||
        sign.bounding_box.width() > kMaxSignWidth) {
      continue;
    }
    // NOTE(dong): Only deal with signs vertical to the ground.
    constexpr double kMaxCosRangeInVertical = 0.5;  // ±30 degree.
    if (!sign.is_overhanging &&
        std::abs(sign.normal.z()) > kMaxCosRangeInVertical) {
      continue;
    }
    // NOTE(dong): Only deal with signs directly facing ego vehicle.
    const Vec2d vec_1 = Vec2d::FastUnitFromAngle(pose.yaw);
    const Vec2d vec_2 =
        sign.is_overhanging
            ? Vec2d::FastUnitFromAngle(sign.bounding_box.heading() + M_PI_2)
            : sign.normal.head<2>();
    if (vec_2.norm() == 0) {
      continue;
    }
    const double angle_diff =
        std::acos(std::abs(vec_1.Dot(vec_2)) / (vec_1.norm() * vec_2.norm()));
    const double max_radian_range_at_yaw =
        sign.is_overhanging ? M_PI_4 / 2 : M_PI_4;  // 22.5 / 45 degree.
    if (angle_diff > max_radian_range_at_yaw) {
      continue;
    }
    // Sign area.
    const AffineTransformation transform =
        AffineTransformation::FromYawPitchRoll(-vec_2.FastAngle(), 0.0, 0.0);
    std::vector<Vec2d> points_on_the_ground;
    points_on_the_ground.reserve(sign.points.size());
    for (const auto& point : sign.points) {
      const auto& transformed_point = transform.TransformPoint(point);
      points_on_the_ground.emplace_back(transformed_point.y(),
                                        transformed_point.z());
    }
    Polygon2d contour_on_the_ground;
    if (!Polygon2d::ComputeConvexHull(points_on_the_ground,
                                      &contour_on_the_ground)) {
      continue;
    }
    sign.area = contour_on_the_ground.area();
    if (!sign.is_overhanging && sign.area < kMinSignArea) {
      continue;
    }
    // Sign z.
    sign.min_z = sign.points[0].z();
    sign.max_z = sign.points[0].z();
    for (const auto& point : sign.points) {
      sign.min_z = std::min(sign.min_z, point.z());
      sign.max_z = std::max(sign.max_z, point.z());
    }
    // Sign centroid.
    sign.centroid = CalculatePointsCentroid(sign.points);

    signs.push_back(std::move(sign));
    // For Canvas Debug
    MaybeRenderSign(signs.back(), retroreflector->lidar_id, pose, angle_diff);
  }

  return signs;
}

RoadCylinders FindRoadCylinders(const LidarId lidar_id,
                                const Retroreflectors& retroreflectors,
                                const VehiclePose& pose) {
  SCOPED_QTRACE("FindRoadCylinders");

  std::vector<const Retroreflector*> retroreflectors_for_currend_lidar;
  for (const auto& retroreflector : retroreflectors) {
    if (retroreflector.lidar_id == lidar_id) {
      retroreflectors_for_currend_lidar.push_back(&retroreflector);
    }
  }

  if (retroreflectors_for_currend_lidar.empty()) return {};
  RoadCylinders cylinders;
  cylinders.reserve(retroreflectors_for_currend_lidar.size());
  for (const auto* retroreflector : retroreflectors_for_currend_lidar) {
    RoadCylinder cylinder;
    std::tie(cylinder.points, cylinder.axis) =
        GetPointsAndAxisFromRetroreflector(*retroreflector);
    if (cylinder.points.size() < kMinNumPointsOnCylinder) continue;
    auto clustered_points =
        ClusterPoints(cylinder.points, kMaxCylinderPointToCentroidRadius);
    const double kMinNumClusteredPointsRatio = 0.90;
    if (clustered_points.size() * 1.0 / cylinder.points.size() <
        kMinNumClusteredPointsRatio) {
      continue;
    }
    cylinder.points = std::move(clustered_points);
    // NOTE(dong): Only deal with cylinder vertical to the ground.
    constexpr double kMaxCosRange = 0.96;  // ±16.3 degree.
    if (cylinder.axis.z() / cylinder.axis.norm() < kMaxCosRange) continue;
    // cylinder contour.
    std::vector<Vec2d> points_in_birds_eye_view;
    points_in_birds_eye_view.reserve(cylinder.points.size());
    for (const auto& point : cylinder.points) {
      points_in_birds_eye_view.emplace_back(point.x(), point.y());
    }
    if (!Polygon2d::ComputeConvexHull(points_in_birds_eye_view,
                                      &cylinder.contour)) {
      continue;
    }
    // cylinder bounding box.
    cylinder.bounding_box = cylinder.contour.MinAreaBoundingBox();
    if (cylinder.bounding_box.length() > kMaxCylinderLength ||
        cylinder.bounding_box.width() > kMaxCylinderWidth) {
      continue;
    }
    // cylinder z.
    cylinder.min_z = cylinder.points[0].z();
    cylinder.max_z = cylinder.points[0].z();
    for (const auto& point : cylinder.points) {
      cylinder.min_z = std::min(cylinder.min_z, point.z());
      cylinder.max_z = std::max(cylinder.max_z, point.z());
    }
    if (cylinder.max_z - pose.z > kMaxCylinderHeight) {
      continue;
    }
    if (cylinder.max_z - cylinder.min_z < kMinCylinderHeight) {
      continue;
    }
    // cylinder centroid.
    cylinder.centroid = CalculatePointsCentroid(cylinder.points);

    cylinders.push_back(std::move(cylinder));
    // For Canvas Debug
    MaybeRenderCylinder(cylinders.back(), retroreflector->lidar_id, pose);
  }

  return cylinders;
}
template <typename Reflector>
ObstaclePtrs FindRetroReflectedObstacles(
    const ProposerEnvInfo& env_info, const std::vector<Reflector>& reflectors,
    const LidarId lidar_id) {
  const auto& obstacle_manager = env_info.obstacle_manager();
  const auto& pose = env_info.pose();
  const auto& local_imagery = env_info.local_imagery();
  const auto& coordinate_converter = env_info.coordinate_converter();
  const auto& av_height = env_info.av_height();
  const auto& ped_boxes = env_info.context().fiery_eye_net_result.ped_boxes;
  const auto& cyc_boxes = env_info.context().fiery_eye_net_result.cyc_boxes;
  const auto* lidar_params = env_info.GetLidarParams(lidar_id);
  if (lidar_params == nullptr) return {};
  const auto ref_lidar_pos =
      pose.ToTransform().TransformPoint(GetRefLidarExtrinsics(*lidar_params));

  ObstaclePtrs retroreflected_obstacles;
  for (const auto& reflector : reflectors) {
    const Vec2d centroid_2d = reflector.bounding_box.center();
    const auto [centroid_row, centroid_col] =
        obstacle_manager.CoordToRC(centroid_2d.cast<float>());
    // Check if reflector is in barrier zone.
    const bool is_in_barrier_zone = env_info.PointInZones(
        centroid_2d, mapping::PerceptionZoneProto::BARRIER);
    // Check if reflector is offroad.
    const Vec2d centroid_2d_global =
        coordinate_converter.SmoothToGlobal(centroid_2d);
    float dist_to_curb = 0.f;
    if (const auto indexer = local_imagery.GetIndexer(
            centroid_2d_global.x(), centroid_2d_global.y(),
            coordinate_converter.GetLevel())) {
      dist_to_curb = local_imagery.DistToCurbAt(*indexer);
    } else {
      // If there is no info in imagery for this obstacle, treat it as
      // offroad with a considerably large value.
      dist_to_curb = 1e4;
    }
    // We only consider reflector offroad, overhanging or in barrier zone.
    if (dist_to_curb < 0.f && !reflector.is_overhanging &&
        !is_in_barrier_zone) {
      continue;
    }

    bool is_in_boxes = false;
    for (const auto& ped_box : ped_boxes) {
      if (ped_box.box.IsPointIn(centroid_2d)) {
        is_in_boxes = true;
        break;
      }
    }
    if (!is_in_boxes) {
      for (const auto& cyc_box : cyc_boxes) {
        if (cyc_box.box.IsPointIn(centroid_2d)) {
          is_in_boxes = true;
          break;
        }
      }
    }
    if (is_in_boxes && !reflector.is_overhanging) continue;

    // Use half length of reflector as a base of search radius. Blooming is
    // usually larger than the original reflector. We use a factor to extend the
    // search radius.
    const double basic_search_radius = reflector.bounding_box.half_length();
    const int max_blooming_obstacle_range =
        basic_search_radius < 2.0
            ? CeilToInt(basic_search_radius * (10.0 / Obstacle::kDiameter))
            : CeilToInt(basic_search_radius * (2.5 / Obstacle::kDiameter));
    const double max_blooming_range_radius_diff =
        reflector.is_overhanging ? 0.8 : 0.3;
    constexpr double kMaxM1RangeRadiusDiff = 1.5;
    QCHECK(!reflector.points.empty());
    KDTree<Vec3d> kdtree(reflector.points);
    const int min_row = std::max(centroid_row - max_blooming_obstacle_range,
                                 obstacle_manager.min_row());
    const int max_row = std::min(centroid_row + max_blooming_obstacle_range,
                                 obstacle_manager.max_row());
    const int min_col = std::max(centroid_col - max_blooming_obstacle_range,
                                 obstacle_manager.min_col());
    const int max_col = std::min(centroid_col + max_blooming_obstacle_range,
                                 obstacle_manager.max_col());
    for (int r = min_row; r <= max_row; ++r) {
      for (int c = min_col; c <= max_col; ++c) {
        const auto* obstacle = obstacle_manager.ObstacleAt(r, c);
        if (obstacle == nullptr || !IsObstacleBloomingFilterable(*obstacle)) {
          continue;
        }
        const float obstacle_mid_z =
            (obstacle->max_z + std::max(obstacle->ground_z, obstacle->min_z)) *
            0.5;
        const double obstacle_dis_to_ref_lidar = Hypot(
            obstacle->x - ref_lidar_pos.x(), obstacle->y - ref_lidar_pos.y(),
            obstacle_mid_z - ref_lidar_pos.z());
        const auto nearest_point =
            kdtree.FindNearest({obstacle->x, obstacle->y, obstacle_mid_z});
        const double reflector_dis_to_ref_lidar =
            (nearest_point - ref_lidar_pos).norm();
        const double radius_diff =
            obstacle_dis_to_ref_lidar - reflector_dis_to_ref_lidar;
        // For Canvas Debug
        MaybeRenderObstacleInBloomingRange(*obstacle, av_height, radius_diff);
        //
        const bool is_m1_obstacle = HasM1PointType(obstacle);
        if (std::abs(radius_diff) > (is_m1_obstacle
                                         ? kMaxM1RangeRadiusDiff
                                         : max_blooming_range_radius_diff)) {
          continue;
        }

        QCHECK(!obstacle->points.empty());
        float min_z_above_ground = obstacle->points[0].z;
        float max_z_above_ground = obstacle->points[0].z;
        int average_intensity = 0;
        int num_points_above_ground = 0;
        int num_ouster_points_above_ground = 0;
        for (const auto& point : obstacle->points) {
          if (!obstacle_util::IsRealObstaclePoint(*obstacle, point,
                                                  av_height)) {
            continue;
          }
          min_z_above_ground = std::min(min_z_above_ground, point.z);
          max_z_above_ground = std::max(max_z_above_ground, point.z);
          ++num_points_above_ground;
          if (IsOusterPoint(point.lidar_type)) {
            ++num_ouster_points_above_ground;
          }
          average_intensity += point.intensity;
        }
        if (num_points_above_ground == 0) continue;
        average_intensity /= num_points_above_ground;
        constexpr double kMinNumOusterPointsRatio = 0.95;
        const bool is_ouster_obstacle =
            num_ouster_points_above_ground * 1.0 / num_points_above_ground >
            kMinNumOusterPointsRatio;
        constexpr float kMaxIntensityOfLowIntensityObstacle = 30.0f;
        const bool is_low_intensity_obstacle =
            average_intensity < kMaxIntensityOfLowIntensityObstacle;
        double factor = is_low_intensity_obstacle ? 6.0 : 3.0;
        if (is_ouster_obstacle) factor = 8.0;
        if (is_m1_obstacle) factor = 10.5;
        const double obstacle_dist2_to_centroid =
            Sqr(obstacle->x - centroid_2d.x()) +
            Sqr(obstacle->y - centroid_2d.y());
        double max_search_radius = reflector.is_overhanging ? 10.0 : 4.0;  // m
        if (is_ouster_obstacle) max_search_radius = 4.5;
        if (is_m1_obstacle) max_search_radius = 6.0;
        const double search_radius =
            std::min(basic_search_radius * factor, max_search_radius);
        if (obstacle_dist2_to_centroid > Sqr(search_radius)) {
          continue;
        }
        // Check obstacle height.
        if (is_ouster_obstacle &&
            max_z_above_ground - reflector.centroid.z() >
                factor * (reflector.max_z - reflector.centroid.z())) {
          continue;
        }
        constexpr float kMaxOverhangingObstacleIntensity = 40.f;
        constexpr float kMaxOverhangingObstacleIntensityM1 = 80.f;
        if (reflector.is_overhanging &&
            (average_intensity > (is_m1_obstacle
                                      ? kMaxOverhangingObstacleIntensityM1
                                      : kMaxOverhangingObstacleIntensity))) {
          continue;
        }
        if ((!is_ouster_obstacle && !reflector.is_overhanging) &&
            (min_z_above_ground - reflector.centroid.z() <
                 factor * (reflector.min_z - reflector.centroid.z()) ||
             max_z_above_ground - reflector.centroid.z() >
                 factor * (reflector.max_z - reflector.centroid.z()))) {
          continue;
        }
        retroreflected_obstacles.emplace_back(obstacle);
        // For Canvas Debug
        MaybeRenderRetroreflectedObstacle(
            *obstacle, lidar_id, is_ouster_obstacle, is_low_intensity_obstacle);
      }
    }
    // For Canvas Debug
    MaybeRenderReflectorCvs(reflector, lidar_id, pose, ref_lidar_pos,
                            max_blooming_obstacle_range,
                            max_blooming_range_radius_diff);
  }

  return retroreflected_obstacles;
}

}  // namespace

ProposedClusters BloomingProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto& range_images = env_info.context().range_images;
  const auto& retroreflectors = env_info.context().retroreflectors;
  const auto& pose = env_info.pose();

  const std::array<LidarId, 4> kLidarsForBloomingDetection = {
      LDR_CENTER, LDR_FRONT, LDR_FRONT_LEFT, LDR_FRONT_RIGHT};

  std::vector<std::pair<LidarId, const RangeImage*>>
      range_images_for_blooming_detection;
  for (const auto& [lidar_id, range_image] : range_images) {
    if (absl::c_count(kLidarsForBloomingDetection, lidar_id) > 0) {
      range_images_for_blooming_detection.emplace_back(lidar_id, &range_image);
    }
  }

  if (range_images_for_blooming_detection.empty()) return clusters;

  const int num_range_images = range_images_for_blooming_detection.size();
  std::vector<std::vector<ObstaclePtr>>
      retroreflected_obstacles_per_range_image(num_range_images);
  ParallelFor(0, num_range_images, thread_pool_, [&](int range_image_index) {
    const auto& [lidar_id, range_image] =
        range_images_for_blooming_detection[range_image_index];
    {
      SCOPED_QTRACE("road_sign");
      const auto signs = FindRoadSigns(lidar_id, retroreflectors, pose);
      const auto obstacles =
          FindRetroReflectedObstacles(env_info, signs, lidar_id);
      for (const auto& obstacle : obstacles) {
        retroreflected_obstacles_per_range_image[range_image_index].push_back(
            obstacle);
      }
    }

    {
      SCOPED_QTRACE("cylinder");
      const auto cylinders = FindRoadCylinders(lidar_id, retroreflectors, pose);
      const auto obstacles =
          FindRetroReflectedObstacles(env_info, cylinders, lidar_id);
      for (const auto& obstacle : obstacles) {
        retroreflected_obstacles_per_range_image[range_image_index].push_back(
            obstacle);
      }
    }
  });

  const int num_obstacles = std::accumulate(
      retroreflected_obstacles_per_range_image.begin(),
      retroreflected_obstacles_per_range_image.end(), 0,
      [](int sum, const auto& obstacles) { return sum + obstacles.size(); });
  absl::flat_hash_set<ObstaclePtr> retroreflected_obstacles;
  retroreflected_obstacles.reserve(num_obstacles);
  for (const auto& obstacles : retroreflected_obstacles_per_range_image) {
    retroreflected_obstacles.insert(obstacles.begin(), obstacles.end());
  }

  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    if (IsClusterBloomingFilterable(cluster)) {
      auto remaining_cluster = TrimNoiseObstaclesAndGetRemainingCluster(
          cluster, retroreflected_obstacles, &proposed_clusters);
      if (remaining_cluster &&
          remaining_cluster->obstacles().size() == cluster.obstacles().size()) {
        proposed_clusters.emplace_back(std::move(*remaining_cluster));
      } else if (remaining_cluster) {
        QCHECK(!remaining_cluster->obstacles().empty());
        const auto& pose = env_info.pose();
        const auto neighbor_range = [&pose](const Obstacle& obstacle) {
          const double range = Hypot(obstacle.x - pose.x, obstacle.y - pose.y);
          const int neighbor_radius = range * (1 / 20.0);
          return std::clamp(1, 5, neighbor_radius);
        };
        auto segmented_clusters =
            SegmentAndProposeClusterWithConnectedComponents(*remaining_cluster,
                                                            neighbor_range);
        proposed_clusters.insert(
            proposed_clusters.end(),
            std::make_move_iterator(segmented_clusters.begin()),
            std::make_move_iterator(segmented_clusters.end()));
      }
    } else {
      proposed_clusters.emplace_back(cluster);
    }
  }

  QCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
