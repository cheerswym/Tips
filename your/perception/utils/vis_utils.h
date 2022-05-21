#ifndef ONBOARD_PERCEPTION_UTILS_VIS_UTILS_H_
#define ONBOARD_PERCEPTION_UTILS_VIS_UTILS_H_

#include <optional>
#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft {
// Returns sampled points of the covariance ellipse, the sigma_scale
// is used to determine confidence level.
std::optional<std::vector<Vec2d>> GetCovarianceEllipsePoints(
    const Mat2d& cov, const double sigma_scale);

}  // namespace qcraft
#endif  // ONBOARD_PERCEPTION_UTILS_VIS_UTILS_H_
