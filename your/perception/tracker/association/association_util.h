#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_ASSOCIATION_UTIL_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_ASSOCIATION_UTIL_H_

#include <limits>
#include <numeric>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {
namespace association_util {

inline Box2d GetBox2dFromCamera3dMeasurementProto(
    const Camera3dMeasurementProto& m) {
  const Vec3d center = Vec3dFromProto(m.pos());
  return Box2d(center.block<2, 1>(0, 0), m.heading(), m.length(), m.width());
}

double IoP(const Polygon2d& lhs, const Polygon2d& rhs);

double IoU(const Polygon2d& lhs, const Polygon2d& rhs);

double IoU(const Box2d& lhs, const Box2d& rhs);

double IoU(const BoundingBox2dProto& lhs, const BoundingBox2dProto& rhs);

template <class Input1, class Input2>
double CosineSimilarity(const Input1& feat1, const Input2& feat2) {
  const double dot_product =
      std::inner_product(feat1.begin(), feat1.end(), feat2.begin(), 0.0);
  const double norm1 =
      std::inner_product(feat1.begin(), feat1.end(), feat1.begin(), 0.0);
  const double norm2 =
      std::inner_product(feat2.begin(), feat2.end(), feat2.begin(), 0.0);
  return dot_product /
         (sqrt(norm1) * sqrt(norm2) + std::numeric_limits<double>::epsilon());
}

}  // namespace association_util
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACKER_UTIL_H_
