#ifndef ONBOARD_MATH_VEC_H_
#define ONBOARD_MATH_VEC_H_

// Vec2d/Vec2i/Vec3d/Vec3i implementation that is derived from Eigen matrix with
// custormized interfaces. They guarantee zero initialization in default c'tor.

#include <cfloat>
#include <string>

#include "Eigen/Core"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/affine_transformation.pb.h"

namespace qcraft {

template <typename T>
class Vec3;

template <typename T>
class Vec2 : public Eigen::Matrix<T, 2, 1> {
 public:
  using Base = Eigen::Matrix<T, 2, 1>;

  Vec2() : Base(0, 0) {}
  Vec2(T x, T y) : Base(x, y) {}
  explicit Vec2(Vec3<T> v) : Base(v.x(), v.y()) {}

  static constexpr Vec2<T> Zero() { return Vec2<T>(T(0), T(0)); }

  template <typename OtherDerived>
  Vec2(const Eigen::EigenBase<OtherDerived> &vec)  // NOLINT
      : Base(vec) {}

  explicit Vec2(const Vec2dProto &proto) : Base(proto.x(), proto.y()) {}

  static Vec2<T> UnitFromAngle(T angle) {
    return {std::cos(angle), std::sin(angle)};
  }
  // Faster version, with maximum error 5e-5.
  static Vec2<T> FastUnitFromAngle(T angle) {
    const T normalized_angle = NormalizeAngle(angle);
    return {fast_math::CosNormalized(normalized_angle),
            fast_math::SinNormalized(normalized_angle)};
  }

  // Overloads of operator+.
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  Vec2<T> operator+(BuiltinT rhs) const {
    return {this->x() + rhs, this->y() + rhs};
  }
  template <typename OtherT,
            typename std::enable_if_t<!std::is_arithmetic_v<OtherT>, bool> = 0>
  Vec2<T> operator+(const OtherT &rhs) const {
    return this->Base::operator+(rhs);
  }
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  friend Vec2<T> operator+(BuiltinT lhs, const Vec2<T> &rhs) {
    return {lhs + rhs.x(), lhs + rhs.y()};
  }

  // Overloads of operator-.
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  Vec2<T> operator-(BuiltinT rhs) const {
    return {this->x() - rhs, this->y() - rhs};
  }
  template <typename OtherT,
            typename std::enable_if_t<!std::is_arithmetic_v<OtherT>, bool> = 0>
  Vec2<T> operator-(const OtherT &rhs) const {
    return this->Base::operator-(rhs);
  }
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  friend Vec2<T> operator-(BuiltinT lhs, const Vec2<T> &rhs) {
    return {lhs - rhs.x(), lhs - rhs.y()};
  }
  friend Vec2<T> operator-(const Vec2<T> &rhs) { return {-rhs.x(), -rhs.y()}; }

  // Overloads of operator*.
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  Vec2<T> operator*(const BuiltinT &rhs) const {
    return {this->x() * rhs, this->y() * rhs};
  }
  template <typename OtherT,
            typename std::enable_if_t<!std::is_arithmetic_v<OtherT>, bool> = 0>
  auto operator*(const OtherT &rhs) const {
    return this->Base::operator*(rhs);
  }
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  friend Vec2<T> operator*(const BuiltinT &lhs, const Vec2<T> &rhs) {
    return {lhs * rhs.x(), lhs * rhs.y()};
  }

  // Overloads of operator/.
  template <typename BuiltinT,
            typename std::enable_if_t<std::is_arithmetic_v<BuiltinT>, bool> = 0>
  Vec2<T> operator/(BuiltinT rhs) const {
    return {this->x() / rhs, this->y() / rhs};
  }

  bool operator<(const Vec2<T> &v) const {
    return std::make_tuple(this->x(), this->y()) <
           std::make_tuple(v.x(), v.y());
  }

  Vec2<T> normalized() const { return this->Base::normalized(); }
  auto DistanceTo(const Vec2<T> &vec) const { return (*this - vec).norm(); }
  auto DistanceSquareTo(const Vec2<T> &vec) const {
    return (*this - vec).squaredNorm();
  }
  T CrossProd(const Vec2<T> &vec) const {
    return this->x() * vec.y() - vec.x() * this->y();
  }
  T Dot(const Vec2<T> &vec) const {
    return this->x() * vec.x() + this->y() * vec.y();
  }
  T Sqr() const { return this->x() * this->x() + this->y() * this->y(); }
  T Angle() const { return std::atan2(this->y(), this->x()); }
  // Faster version, with maximum error 0.012 degree.
  T FastAngle() const { return fast_math::Atan2(this->y(), this->x()); }
  // Represents a counterclockwise rotation of a vector v by an angle yaw
  // For more details: https://en.wikipedia.org/wiki/Rotation_matrix#Ambiguities
  Vec2<T> Rotate(T yaw) const {
    const T sin_yaw = std::sin(yaw);
    const T cos_yaw = std::cos(yaw);
    return Rotate(cos_yaw, sin_yaw);
  }
  // Faster version, with maximum error 5e-5 of sin/cos.
  Vec2<T> FastRotate(T yaw) const {
    const T normalized_yaw = NormalizeAngle(yaw);
    const T sin_yaw = fast_math::SinNormalized(normalized_yaw);
    const T cos_yaw = fast_math::CosNormalized(normalized_yaw);
    return Rotate(cos_yaw, sin_yaw);
  }
  Vec2<T> Rotate(T cos_yaw, T sin_yaw) const {
    return {this->x() * cos_yaw - this->y() * sin_yaw,
            this->x() * sin_yaw + this->y() * cos_yaw};
  }
  Vec2<T> Unit() const {
    const T norm = sqrt(this->x() * this->x() + this->y() * this->y());
    DCHECK_NE(norm, 0.0);
    return {this->x() / norm, this->y() / norm};
  }

  Vec2<T> Perp() const { return {-this->y(), this->x()}; }

  bool ApproximatelyEquals(Vec2<T> p, T max_sqr_dist) const {
    return (*this - p).squaredNorm() <= max_sqr_dist;
  }

  void ToProto(Vec2dProto *proto) const {
    proto->set_x(this->x());
    proto->set_y(this->y());
  }

  void FromProto(const Vec2dProto &proto) { *this = Vec2<T>(proto); }

  std::string DebugString() const {
    return absl::StrFormat("{x: %f, y: %f}", this->x(), this->y());
  }
  std::string DebugStringFullPrecision() const {
    return absl::StrFormat("{x: %.*e, y: %.*e}", DECIMAL_DIG, this->x(),
                           DECIMAL_DIG, this->y());
  }
};

using Vec2d = Vec2<double>;
using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;

// A utility function to print a list of Vec2 points.
template <typename T>
std::string DebugString(absl::Span<const Vec2<T>> vec) {
  std::string out;
  const auto to_str = [](std::string *out, const Vec2<T> v) {
    out->append(v.DebugString());
  };
  return absl::StrJoin(vec, ", ", to_str);
}

// Print a list of Vec2d with full precision.
std::string DebugStringFullPrecision(absl::Span<const Vec2d> vec);

// Compute the normalized angle between two vectors (from v1 to v2) in radius
// into [-pi, pi). Positive => Counter-clockwise.
template <typename T>
T NormalizeAngle2D(const Vec2<T> v1, const Vec2<T> v2) {
  T dot = v1.x() * v2.x() + v1.y() * v2.y();
  T det = v1.x() * v2.y() - v1.y() * v2.x();
  return NormalizeAngle(std::atan2(det, dot));
}

// Compute the normalized angle of the vector in radius into [-pi, pi).
// Positive => Counter-clockwise.
template <typename T>
T NormalizeAngle2D(const Vec2<T> v) {
  return NormalizeAngle(std::atan2(v.y(), v.x()));
}

template <typename T>
class Vec3 : public Eigen::Matrix<T, 3, 1> {
 public:
  using Base = Eigen::Matrix<T, 3, 1>;

  Vec3() : Base(0, 0, 0) {}
  Vec3(T x, T y, T z) : Base(x, y, z) {}
  Vec3(Vec2<T> v, T z) : Base(v.x(), v.y(), z) {}

  template <typename OtherDerived>
  Vec3(const Eigen::EigenBase<OtherDerived> &vec)  // NOLINT
      : Base(vec) {}

  bool operator<(const Vec3<T> &v) const {
    return std::make_tuple(this->x(), this->y(), this->z()) <
           std::make_tuple(v.x(), v.y(), v.z());
  }

  Vec2<T> xy() const { return Vec2<T>(this->x(), this->y()); }

  auto DistanceTo(const Vec3<T> &vec) const { return (*this - vec).norm(); }
  auto DistanceSquareTo(const Vec3<T> &vec) const {
    return (*this - vec).squaredNorm();
  }
  T Dot(const Vec3<T> &vec) const {
    return this->x() * vec.x() + this->y() * vec.y() + this->z() * vec.z();
  }

  std::string DebugString() const {
    return absl::StrCat("<", std::to_string(this->x()), ",",
                        std::to_string(this->y()), ",",
                        std::to_string(this->z()), ">");
  }
};

using Vec3d = Vec3<double>;
using Vec3f = Vec3<float>;
using Vec3i = Vec3<int>;

}  // namespace qcraft

#endif  // ONBOARD_MATH_VEC_H_
