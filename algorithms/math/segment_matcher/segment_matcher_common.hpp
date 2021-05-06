#ifndef ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_COMMON_HPP__
#define ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_COMMON_HPP__

#include <limits>
#include <string>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/aabox_info.h"

namespace qcraft {

static bool NearestSegsIndex(const std::vector<AABoxInfo>& seg_info, double x,
                             double y, int* const index) {
  QCHECK_NOTNULL(index);
  if (seg_info.empty()) {
    return false;
  }
  double nearest_dist = std::numeric_limits<double>::infinity();
  int nearest_index = 0;
  for (int i = 0; i < seg_info.size(); ++i) {
    double dist_square = seg_info[i].DistanceSquareTo({x, y});
    if (dist_square < nearest_dist) {
      nearest_dist = dist_square;
      nearest_index = i;
    }
  }
  *index = nearest_index;
  return true;
}

}  // namespace qcraft

#endif  // ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_COMMON_HPP__
