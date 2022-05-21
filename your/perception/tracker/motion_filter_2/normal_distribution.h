#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_NORMAL_DISTRIBUTION_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_NORMAL_DISTRIBUTION_H_

#include <cmath>

#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
namespace qcraft::tracker {

template <typename VectorType>
class NormalDistribution {
 public:
  using CovarianceType = Covariance<VectorType>;
  using T = typename VectorType::Scalar;
  static constexpr T N = VectorType::RowsAtCompileTime;
  NormalDistribution() = delete;
  explicit NormalDistribution(const CovarianceType& s) : sigma_(s) {}

  T pdf(const VectorType& diff) const {
    const T sqrt2pi = std::sqrt(2 * M_PI);
    T quadform = 0.0;
    if constexpr (VectorType::RowsAtCompileTime == 1)
      quadform = diff.transpose() * 1.0 / sigma_(0) * diff;
    else
      quadform = diff.transpose() * sigma_.inverse() * diff;
    const T norm = std::pow(sqrt2pi, -N) * std::pow(sigma_.determinant(), -0.5);
    const T result = norm * exp(-0.5 * quadform);
    if (std::isnan(result)) {
      QLOG(WARNING) << "NaN received in motion filter.";
      return 0.;
    }
    return result;
  }
  CovarianceType sigma_;
};
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_NORMAL_DISTRIBUTION_H_
