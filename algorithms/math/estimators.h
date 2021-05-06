#ifndef ONBOARD_MATH_ESTIMATORS_H_
#define ONBOARD_MATH_ESTIMATORS_H_

#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"

namespace qcraft {

// Fit a quadratic function over a local neighborhood of 9 pixels (3x3 patch).
// The values at the pixels are given by matrix local_patch.
// For clear parameterization the pixels are embedded in a coordinate frame
// whose origin sits at the center pixel, and whose scale is specified by dx and
// dy. In other words, the 3x3 patch of pixels have the following coordinates:
//  row:  col:   0               1                 2
//   0      (-dx, -dy)        (0, -dy)        (+dx, -dy)
//   1       (-dx, 0)          (0, 0)          (+dx, 0)
//   2      (-dx, +dy)        (0, +dy)        (+dx, +dy)
// The return value r = (rx, ry) is the coordinates of the extremum. Note that
// the extremum could be a maximum, a minimum or a saddle point, depending on
// the eigenvalues of the other return value, the (symmetric) quadratic form A.
// The fitted quadratic function is:
//   f(x) = (x - r)^T A (x - r) + c
Vec2d QuadraticFit2D(const Mat3d &patch, double dx, double dy,
                     Mat2d *A = nullptr, double *c = nullptr);

}  // namespace qcraft

#endif  // ONBOARD_MATH_ESTIMATORS_H_
