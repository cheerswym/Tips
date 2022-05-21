#ifndef ONBOARD_PERCEPTION_UTILS_POINT_FILTERING_UTILS_H_
#define ONBOARD_PERCEPTION_UTILS_POINT_FILTERING_UTILS_H_

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/perception/laser_point.h"

namespace qcraft {
using LaserPointGroundZ = std::pair<LaserPoint, float>;
namespace point_filtering_util {

// Returns filtered points for pedestrain objects given all points falling into
// pedestrain 3D box.
// The logic is:
// - 1) Filtering by distance to ground z.
// - 2) Filtering points below ground z.
std::vector<LaserPointGroundZ> FilterPedPoints(
    const std::vector<LaserPointGroundZ>& raw_points,
    const double max_distance_to_ground_z);

}  // namespace point_filtering_util
}  // namespace qcraft
#endif  // ONBOARD_PERCEPTION_UTILS_POINT_FILTERING_UTILS_H_
