#ifndef ONBOARD_MATH_PIECEWISE_LINEAR_FUNCTION_H_
#define ONBOARD_MATH_PIECEWISE_LINEAR_FUNCTION_H_

#include <utility>
#include <vector>

#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.pb.h"

namespace qcraft {

template <typename T, typename TS>
class Lerper {
 public:
  T operator()(T a, T b, TS alpha) const { return Lerp(a, b, alpha); }
};

template <typename T, typename TS>
class AngleLerper {
 public:
  T operator()(T a, T b, TS alpha) const { return LerpAngle(a, b, alpha); }
};

/*
  A piecewise linear function f(x) defined by a sequence of N 2d points (x_i,
  y_i) for i = 0, ..., N-1 (sorted by x_i in ascending order), such that
  - 1) f(x_i) = y_i;
  - 2) f(x) is linear between any pair of adjacent x_i's;
  - 3) eqqual to constant f(x_0) (x < x_0) or f(x_{N-1}) (x > x_{N-1}).
 */
template <typename TY, typename TX = double, typename LERPER = Lerper<TY, TX>>
class PiecewiseLinearFunction final {
 public:
  PiecewiseLinearFunction() = default;
  PiecewiseLinearFunction(std::vector<TX> x, std::vector<TY> y)
      : x_(std::move(x)), y_(std::move(y)), lerper_(LERPER()) {
    QCHECK_EQ(x_.size(), y_.size());
    QCHECK_GT(x_.size(), 1);
  }

  // Complexity: O(log n).
  // n = x_.size()
  TY operator()(TX x) const { return Evaluate(x); }
  TY Evaluate(TX x) const {
    DCHECK(!x_.empty());
    DCHECK(!y_.empty());
    const int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) return y_.front();
    if (index == x_.size()) return y_.back();
    DCHECK_GT(x_[index], x_[index - 1]);  // Guaranteed by std::upper_bound().
    const TX alpha = (x - x_[index - 1]) / (x_[index] - x_[index - 1]);
    return lerper_(y_[index - 1], y_[index], alpha);
  }

  // x must be ordered.
  // Complexity: O(m + log n)
  // m = x.size()
  // n = x_.size()
  std::vector<TY> Evaluate(const std::vector<TX> &x) const {
    DCHECK(!x_.empty());
    DCHECK(!y_.empty());
    DCHECK(!x.empty());
    DCHECK(std::is_sorted(x.begin(), x.end()));
    std::vector<TY> y;
    y.reserve(x.size());
    int index = std::upper_bound(x_.begin(), x_.end(), x.front()) - x_.begin();
    for (int i = 0; i < x.size(); ++i) {
      while (index < x_.size() && x_[index] <= x[i]) ++index;
      if (index == x_.size()) {
        y.insert(y.end(), x.size() - y.size(), y_.back());
        return y;
      }
      if (index == 0) {
        y.push_back(y_.front());
        continue;
      }
      DCHECK_GT(x_[index], x_[index - 1]);  // Guaranteed by std::upper_bound().
      const TX alpha = (x[i] - x_[index - 1]) / (x_[index] - x_[index - 1]);
      y.push_back(lerper_(y_[index - 1], y_[index], alpha));
    }
    return y;
  }

  // Complexity: O(log n).
  // n = x_.size()
  // The slope is discontinuous when x is exactly on a vertex. This function
  // returns the slope of the piece after that vertex (except when x is on the
  // last vertex where the slope of the last piece would be returned).
  TY EvaluateSlope(TX x) const {
    DCHECK(!x_.empty());
    DCHECK(!y_.empty());
    int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) return TY();
    if (index == x_.size()) {
      if (x > x_.back()) {
        return TY();
      } else {
        index--;
      }
    }
    DCHECK_GT(x_[index], x_[index - 1]);  // Guaranteed by std::upper_bound().
    return (y_[index] - y_[index - 1]) / (x_[index] - x_[index - 1]);
  }

  const std::vector<TX> &x() const { return x_; }
  const std::vector<TY> &y() const { return y_; }

 private:
  std::vector<TX> x_;
  std::vector<TY> y_;
  LERPER lerper_;
};

template <typename LERPER = Lerper<double, double>>
PiecewiseLinearFunction<double, double, LERPER>
PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionDoubleProto &proto) {
  QCHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<double> y(proto.y().begin(), proto.y().end());
  return PiecewiseLinearFunction<double, double, LERPER>(std::move(x),
                                                         std::move(y));
}

template <typename LERPER = Lerper<Vec2d, double>>
PiecewiseLinearFunction<Vec2d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec2dProto &proto) {
  QCHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec2d> y;
  for (const auto &proto_y : proto.y()) y.push_back(Vec2dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec2d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<Vec3d, double>>
PiecewiseLinearFunction<Vec3d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec3dProto &proto) {
  QCHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec3d> y;
  for (const auto &proto_y : proto.y()) y.push_back(Vec3dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec3d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<Vec4d, double>>
PiecewiseLinearFunction<Vec4d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec4dProto &proto) {
  QCHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec4d> y;
  for (const auto &proto_y : proto.y()) y.push_back(Vec4dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec4d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<double, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<double, double, LERPER> &plf,
    PiecewiseLinearFunctionDoubleProto *proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const double y : plf.y()) proto->add_y(y);
}

template <typename LERPER = Lerper<Vec2d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec2d, double, LERPER> &plf,
    PiecewiseLinearFunctionVec2dProto *proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec2d y : plf.y()) Vec2dToProto(y, proto->add_y());
}

template <typename LERPER = Lerper<Vec3d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec3d, double, LERPER> &plf,
    PiecewiseLinearFunctionVec3dProto *proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec3d y : plf.y()) Vec3dToProto(y, proto->add_y());
}

template <typename LERPER = Lerper<Vec4d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec4d, double, LERPER> &plf,
    PiecewiseLinearFunctionVec4dProto *proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec4d y : plf.y()) Vec4dToProto(y, proto->add_y());
}

// Interpolator for sqrt function.
template <typename T, typename TS>
class SqrtInterpolator {
 public:
  T operator()(T a, T b, TS alpha) const {
    return std::sqrt(Lerp(Sqr(a), Sqr(b), alpha));
  }
};

template <typename TY, typename TX = double>
using PiecewiseSqrtFunction =
    PiecewiseLinearFunction<TY, TX, SqrtInterpolator<TY, TX>>;

}  // namespace qcraft

#endif  // ONBOARD_MATH_PIECEWISE_LINEAR_FUNCTION_H_
