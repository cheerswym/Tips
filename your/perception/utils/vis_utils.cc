#include "onboard/perception/utils/vis_utils.h"

#include "onboard/lite/logging.h"

namespace qcraft {

std::optional<std::vector<Vec2d>> GetCovarianceEllipsePoints(
    const Mat2d& cov, const double sigma_scale) {
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver;
  eigen_solver.compute(cov, /*computeEigenvectors=*/true);
  if (eigen_solver.info() != Eigen::ComputationInfo::Success) {
    QLOG(INFO) << "Eigen decomposition failed for covariance " << cov;
    return std::nullopt;
  }
  const auto& eig_vals = eigen_solver.eigenvalues();

  const double half_axis1_size = sigma_scale * std::sqrt(eig_vals[0].real());
  const double half_axis2_size = sigma_scale * std::sqrt(eig_vals[1].real());
  const auto& eig_vec_axis1 = eigen_solver.eigenvectors().col(0);
  constexpr int kNumSegments = 180;
  constexpr float kTheta = 2 * M_PI / kNumSegments;
  const float kCosTheta = cos(kTheta);
  const float kSinTheta = sin(kTheta);

  std::vector<Vec2d> ellipse_points;
  ellipse_points.reserve(kNumSegments);
  double x = eig_vec_axis1[0].real(), y = eig_vec_axis1[1].real();
  const double eig_vec_axis1_radius = Hypot(x, y);
  for (int i = 0; i < kNumSegments; ++i) {
    // Radius of the ellipse given a certain theta from the major axis
    // relative to the norm of eigen vector.
    const double radius_scale = half_axis1_size * half_axis2_size /
                                Hypot(half_axis1_size * sinf(kTheta * i),
                                      half_axis2_size * cosf(kTheta * i)) /
                                eig_vec_axis1_radius;
    ellipse_points.emplace_back(x * radius_scale, y * radius_scale);
    double x_prev = x;
    x = kCosTheta * x - kSinTheta * y;
    y = kSinTheta * x_prev + kCosTheta * y;
  }
  return ellipse_points;
}

}  // namespace qcraft
