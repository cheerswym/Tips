#include "onboard/perception/cluster.h"

#include "onboard/lite/check.h"

namespace qcraft {

Cluster::Cluster(ObstaclePtrs obstacles) : obstacles_(std::move(obstacles)) {
  QCHECK(!obstacles_.empty());
  for (const auto* obstacle : obstacles_) {
    timestamp_ += obstacle->timestamp;
  }
  timestamp_ /= obstacles_.size();
}

Vec2d Cluster::ComputeCentroidFromObstacles() const {
  Vec2d centroid;
  QCHECK(!obstacles_.empty());
  for (const auto* obstacle : obstacles_) {
    centroid += Vec2d{obstacle->x, obstacle->y};
  }
  return centroid / obstacles_.size();
}

Vec3d Cluster::ComputeCentroidFromPoints() const {
  Vec3d centroid;
  int num_points = 0;
  for (const auto* obstacle : obstacles_) {
    for (const auto& point : obstacle->points) {
      centroid += point.coord();
      num_points++;
    }
  }
  QCHECK_NE(num_points, 0);
  return centroid / num_points;
}

bool Cluster::IsStaticType() const {
  switch (type_) {
    case MT_VEGETATION:
    case MT_BARRIER:
    case MT_ROAD:
    case MT_CONE:
    case MT_MIST:
    case MT_STATIC_OBJECT:
    case MT_WARNING_TRIANGLE:
      return true;
    case MT_UNKNOWN:
    case MT_VEHICLE:
    case MT_PEDESTRIAN:
    case MT_CYCLIST:
    case MT_MOTORCYCLIST:
    case MT_FLYING_BIRD:
    case MT_FOD:
      return false;
  }
  return false;
}

}  // namespace qcraft
