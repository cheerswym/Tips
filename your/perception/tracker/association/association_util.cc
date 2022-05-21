#include "onboard/perception/tracker/association/association_util.h"

#include <algorithm>

#include "onboard/lite/logging.h"

namespace qcraft::tracker {
namespace association_util {

double IoP(const Polygon2d& lhs, const Polygon2d& rhs) {
  Polygon2d intersection;
  const bool has_overlap = lhs.ComputeOverlap(rhs, &intersection);
  if (!has_overlap) return 0.0;
  const double intersection_area = intersection.area();
  const double lhs_area = lhs.area();
  const double rhs_area = rhs.area();
  return lhs_area < rhs_area ? intersection_area / lhs_area
                             : intersection_area / rhs_area;
}

double IoU(const Polygon2d& lhs, const Polygon2d& rhs) {
  Polygon2d intersection;
  const bool has_overlap = lhs.ComputeOverlap(rhs, &intersection);
  if (!has_overlap) return 0.0;
  const double intersection_area = intersection.area();
  return intersection_area / (lhs.area() + rhs.area() - intersection_area);
}

double IoU(const Box2d& lhs, const Box2d& rhs) {
  return IoU(Polygon2d(lhs.GetCornersCounterClockwise()),
             Polygon2d(rhs.GetCornersCounterClockwise()));
}

double IoU(const BoundingBox2dProto& lhs, const BoundingBox2dProto& rhs) {
  const int tl_x = std::max(lhs.x(), rhs.x());
  const int tl_y = std::max(lhs.y(), rhs.y());
  const int br_x = std::min(lhs.x() + lhs.width(), rhs.x() + rhs.width());
  const int br_y = std::min(lhs.y() + lhs.height(), rhs.y() + rhs.height());
  const double intersection_area = (br_y - tl_y) * (br_x - tl_x);
  if (tl_x >= br_x || tl_y >= br_y) return 0.0;
  return intersection_area / (lhs.width() * lhs.height() +
                              rhs.width() * rhs.height() - intersection_area);
}

}  // namespace association_util
}  // namespace qcraft::tracker
