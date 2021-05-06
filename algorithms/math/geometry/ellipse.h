#ifndef ONBOARD_MATH_GEOMETRY_ELLIPSE_H_
#define ONBOARD_MATH_GEOMETRY_ELLIPSE_H_

namespace qcraft {

// TODO(weixin): Add unit test.
// Get the parameters for an ellipse from two inputs:
//     back_dis = a - c
//     front_dis = a + c
//     heading = rotation of the semi major axis (+x) with respect to the
//     center.
//
//   Output parameters:
//     a - length of the semi major axis
//     b - length of the semi minor axis
//     c - length of the semi focus length
//     center - center of the ellipse
//
// Assuming the long axis points to +x direction.
class Ellipse {
 public:
  Ellipse(double a_plus_c, double a_minus_c)
      : rotation_(0.0), origin_(Vec2d(0.0, 0.0)) {
    CHECK_GE(a_plus_c, a_minus_c);
    a_ = (a_plus_c + a_minus_c) / 2.0;
    c_ = (a_plus_c - a_minus_c) / 2.0;
    b_ = sqrt(Sqr(a_) - Sqr(c_));
  }

  Ellipse(double a_plus_c, double a_minus_c, double rotation,
          const Vec2d left_focus)
      : rotation_(NormalizeAngle(rotation)) {
    CHECK_GE(a_plus_c, a_minus_c);
    a_ = (a_plus_c + a_minus_c) / 2.0;
    c_ = (a_plus_c - a_minus_c) / 2.0;
    b_ = sqrt(Sqr(a_) - Sqr(c_));
    origin_ = left_focus + c_ * Vec2d::FastUnitFromAngle(rotation_);
  }

  const Vec2d RelativePos(Vec2d x) const {
    return Vec2d(x - origin_).FastRotate(-rotation_);
  }

  double Evaluate(Vec2d x) const {
    const Vec2d x_hat = RelativePos(x);
    return Sqr(x_hat.x() / a_) + Sqr(x_hat.y() / b_);
  }

  double a() const { return a_; }
  double b() const { return b_; }
  double c() const { return c_; }
  const Vec2d origin() const { return origin_; }

 private:
  double a_;
  double b_;
  double c_;
  double rotation_;
  Vec2d origin_;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_ELLIPSE_H_
