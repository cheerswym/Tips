#ifndef ONBOARD_MATH_GEOMETRY_BEZIER_CURVE_H_
#define ONBOARD_MATH_GEOMETRY_BEZIER_CURVE_H_

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace Bezier {

/**
 * @brief Get binomial coefficients from C_n^{0} to C_n^{n}
 */
std::vector<int64_t> GetBinomialCoeff(int n);

/**
 * @brief Sample a point from an n-th order Bezier curve.
 * @param control_points The control points which should be of size n+1
 * @param t The sampling parameter.
 */
Vec2d SampleBezier(absl::Span<const Vec2d> control_points, double t);

/**
 * @brief Sample a point from an n-th order Bezier curve with precomputed
 * binomial coefficients.
 * @param control_points The control points which should be of size n+1
 * @param binomial_coeffs binomial coefficients from C_n^{0} to C_n^{n}
 * @param t The sampling parameter.
 */
Vec2d SampleBezierWithBinomialCoeffs(absl::Span<const Vec2d> control_points,
                                     absl::Span<const int64_t> binomial_coeffs,
                                     double t);

/**
 * @brief Compute the tangent vector of a sampled point from an n-th order
 * Bezier curve.
 * @param control_points The control points which should be of size n+1
 * @param t The sampling parameter.
 */
Vec2d SampleBezierTangent(absl::Span<const Vec2d> control_points, double t);

/**
 * @brief Compute the tangent vector of a sampled point from an n-th order
 * Bezier curve with precomputed binomial coefficients.
 * @param control_points The control points which should be of size n+1
 * @param binomial_coeffs binomial coefficients from C_n^{0} to C_n^{n}
 * @param t The sampling parameter.
 */
Vec2d SampleBezierTangentWithBinomialCoeffs(
    absl::Span<const Vec2d> control_points,
    absl::Span<const int64_t> binomial_coeffs, double t);

/**
 * @brief Sample a point from a third-order Bezier curve.
 * @param control_points The control points which should be of size 4
 * @param t The sampling parameter.
 */
Vec2d SampleThirdOrderBezier(absl::Span<const Vec2d> control_points, double t);

/**
 * @brief Compute the tangent vector of a sampled point from a third-order
 * Bezier curve.
 * @param control_points The control points which should be of size n+1
 * @param t The sampling parameter.
 */
Vec2d SampleThirdOrderBezierTangent(absl::Span<const Vec2d> control_points,
                                    double t);

/**
 * @brief Split up a Bézier curve into two, smaller curves using de Casteljau's
 * algorithm.
 * @param control_points The control points which should be of size n+1
 * @param t The sampling parameter.
 */
std::pair<std::vector<Vec2d>, std::vector<Vec2d>> SplitBezier(
    absl::Span<const Vec2d> control_points, double t);

/**
 * @brief Fit n-1th order bezier curve when. When the data points is n.
 * @param polyline_points Data points of the bezier curve.
 */
absl::Status FitBezier(absl::Span<const Vec2d> polyline_points,
                       std::vector<Vec2d> *control_points);

}  // namespace Bezier
}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_BEZIER_CURVE_H_
