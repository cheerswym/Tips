#include "onboard/perception/utils/point_filtering_utils.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

namespace qcraft::point_filtering_util {

std::vector<LaserPointGroundZ> FilterPedPoints(
    const std::vector<LaserPointGroundZ>& raw_points,
    const double max_distance_to_ground_z) {
  const int num_points = raw_points.size();
  if (num_points == 0) {
    return {};
  }

  std::vector<LaserPointGroundZ> points;
  points.reserve(num_points);
  for (const auto& point : raw_points) {
    float height = point.first.coord().z() - point.second;
    if (height < 0 || height > max_distance_to_ground_z) {
      continue;
    }
    points.push_back(point);
  }

  return points;
}

}  // namespace qcraft::point_filtering_util
