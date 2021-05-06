#ifndef ONBOARD_MATH_GEOMETRY_CIRCLE2D_H_
#define ONBOARD_MATH_GEOMETRY_CIRCLE2D_H_

#include "onboard/math/vec.h"

namespace qcraft {

class Circle2d {
 public:
  Circle2d() = default;
  Circle2d(Vec2d center, double radius) : center_(center), radius_(radius) {}

  Vec2d center() const { return center_; }
  double radius() const { return radius_; }

  bool inside(Vec2d v) const {
    return Sqr(radius_) >= Vec2d(v - center_).Sqr();
  }

 private:
  Vec2d center_;
  double radius_;
};
}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_CIRCLE2D_H_
