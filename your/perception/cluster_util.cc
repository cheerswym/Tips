#include "onboard/perception/cluster_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/perception/obstacle_util.h"

namespace qcraft::cluster_util {

Polygon2d ComputeContour(const Cluster& cluster) {
  // Extract rows/cols from all obstacles so that we only need to traverse all
  // obstacles once.
  std::vector<std::pair<int, int> > obstacle_rowcols;
  obstacle_rowcols.reserve(cluster.NumObstacles());
  for (const auto& obstacle : cluster.obstacles()) {
    obstacle_rowcols.emplace_back(obstacle->row, obstacle->col);
  }
  int min_row = INT_MAX;
  int max_row = INT_MIN;
  for (const auto& [row, col] : obstacle_rowcols) {
    min_row = std::min(min_row, row);
    max_row = std::max(max_row, row);
  }
  QCHECK_LE(min_row, max_row);

  // For each row, collect the min/max cols.
  const int num_rows = max_row - min_row + 1;
  std::vector<std::pair<int, int> > min_max_cols(num_rows, {INT_MAX, INT_MIN});
  for (const auto& [row, col] : obstacle_rowcols) {
    auto& [min_col, max_col] = min_max_cols[row - min_row];
    if (col > max_col) {
      max_col = col;
    }
    if (col < min_col) {
      min_col = col;
    }
  }

  const auto& ref_obstacle = *cluster.obstacles().front();
  // A lambda that converts row/col into local coordinate.
  const auto rowcol_to_coord = [&ref_obstacle](double row, double col) {
    return Vec2d(
        ref_obstacle.x + (col - ref_obstacle.col) * Obstacle::kDiameter,
        ref_obstacle.y + (row - ref_obstacle.row) * Obstacle::kDiameter);
  };

  // For each row, collect the top-left/bottom-left corners of the leftmost
  // obstacle and top-right/bottom-right corners of the right-most obstacle.
  std::vector<Vec2d> points;
  for (int row = min_row; row <= max_row; ++row) {
    const auto [min_col, max_col] = min_max_cols[row - min_row];
    // Skip rows without any obstacles.
    if (min_col > max_col) continue;
    points.emplace_back(rowcol_to_coord(row - 0.5, min_col - 0.5));
    points.emplace_back(rowcol_to_coord(row - 0.5, max_col + 0.5));
    points.emplace_back(rowcol_to_coord(row + 0.5, min_col - 0.5));
    points.emplace_back(rowcol_to_coord(row + 0.5, max_col + 0.5));
  }

  // TODO(dong): Optimize this function with an O(n) algorithm. Current one is
  // O(nlogn).
  Polygon2d contour;
  Polygon2d::ComputeConvexHull(points, &contour);
  return contour;
}

std::vector<Vec3d> ComputeContourWithZ(const Cluster& cluster, double z) {
  const auto cluster_contour = cluster_util::ComputeContour(cluster);
  std::vector<Vec3d> contour_points;
  contour_points.reserve(cluster_contour.points().size());
  for (const auto& point : cluster_contour.points()) {
    contour_points.emplace_back(point.x(), point.y(), z);
  }
  return contour_points;
}

Polygon2d ComputeContourFromClusterPoints(const Cluster& cluster) {
  // TODO(yu, dong): Consider to improve the performance here by ignoring points
  // that cannot help form the contour.
  std::vector<Vec2d> cluster_points;
  cluster_points.reserve(cluster.NumPoints());
  for (const auto* obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      if (obstacle_util::IsAboveGroundObstaclePoint(*obstacle, point)) {
        cluster_points.emplace_back(point.x, point.y);
      }
    }
  }
  if (cluster_points.size() >= 3) {
    Polygon2d contour;
    if (Polygon2d::ComputeConvexHull(cluster_points, &contour)) {
      return contour;
    }
  }

  return ComputeContour(cluster);
}

Polygon2d ComputeContourWithRefinement(const VehiclePose& pose,
                                       const Cluster& cluster) {
  // For front nearby clusters, compute the contour to tightly bound all laser
  // points as a more precise contour comparing to the contour from obstacles.
  constexpr double kFrontNearbyZoneLength = 40.0;
  constexpr double kFrontNearbyZoneWidth = 8.0;
  Box2d front_nearby_zone({pose.x, pose.y}, pose.yaw, kFrontNearbyZoneLength,
                          kFrontNearbyZoneWidth);
  front_nearby_zone.Shift(Vec2d::FastUnitFromAngle(pose.yaw) *
                          kFrontNearbyZoneLength * 0.5);

  bool nearby_cluster = false;
  for (const auto* obstacle : cluster.obstacles()) {
    if (front_nearby_zone.IsPointIn(obstacle->coord())) {
      nearby_cluster = true;
      break;
    }
  }
  if (nearby_cluster) {
    return ComputeContourFromClusterPoints(cluster);
  }

  return ComputeContour(cluster);
}

std::vector<Vec3d> CollectPoints(const Cluster& cluster) {
  std::vector<Vec3d> points;
  points.reserve(cluster.NumPoints());
  for (const auto& obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      points.push_back(point.coord());
    }
  }
  return points;
}

double ComputeIoU(const Cluster& cluster1, const Cluster& cluster2) {
  const auto contour1 = ComputeContour(cluster1);
  const auto contour2 = ComputeContour(cluster2);
  Polygon2d intersection;
  if (!contour1.ComputeOverlap(contour2, &intersection)) {
    return 0.0;
  }
  const double intersection_area = intersection.area();
  return intersection_area /
         (contour1.area() + contour2.area() - intersection_area);
}

double ComputeGroundZ(const Cluster& cluster) {
  const int point_num = cluster.NumObstacles();
  double ground_z = 0.0;
  for (const auto& obstacle : cluster.obstacles()) {
    ground_z += obstacle->ground_z;
  }
  return point_num > 0 ? ground_z / point_num : DBL_MAX;
}

std::vector<Vec3d> CollectPointsInZRange(const Cluster& cluster, double min_z,
                                         double max_z) {
  std::vector<Vec3d> points;
  points.reserve(cluster.NumPoints());
  for (const auto& obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      if (point.coord().z() > min_z && point.coord().z() < max_z) {
        points.push_back(point.coord());
      }
    }
  }
  return points;
}

}  // namespace qcraft::cluster_util
