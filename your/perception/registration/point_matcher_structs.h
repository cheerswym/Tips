#ifndef ONBOARD_PERCEPTION_REGISTRATION_POINT_MATCHER_STRUCTS_H_
#define ONBOARD_PERCEPTION_REGISTRATION_POINT_MATCHER_STRUCTS_H_

#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/math/geometry/affine_transformation.h"

namespace Eigen {
using Matrix6d = ::Eigen::Matrix<double, 6, 6>;
using Vector6d = ::Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace qcraft {

struct PointMatcherOptions {
  std::string DebugString() const {
    return absl::StrFormat(
        "max_mse: %.4f, max_num_iters: %d, max_num_points: %d, "
        "max_matching_dist: %.2f, optimize_convergence: %s, translation_only: "
        "%s",
        max_mse, max_num_iters, max_num_points, max_matching_dist,
        optimize_convergence ? "YES" : "NO", translation_only ? "YES" : "NO");
  }

  // Maximum value of mean square error.
  double max_mse = 0.01;
  // Maximum number of iterations.
  int max_num_iters = 10;
  // Initial hypothesis.
  AffineTransformation hypothesis;
  // Maximum number of points per obstacle for downsampling.
  int max_num_points = 2000;
  // Maximum distance between two matched points.
  double max_matching_dist = 1.0;  // m
  // Optimize convergence by reducing max_matching_dist in the iteration.
  bool optimize_convergence = false;
  // Only estimate translation
  bool translation_only = false;
  // Valid match ratio
  double min_match_ratio = 0.15;
  // Feature icp
  double max_ground_matching_dist = 0.2;        // m
  double max_edge_matching_dist = 0.15;         // m
  double max_plane_matching_dist = 0.15;        // m
  double max_spherical_matching_dist = 0.1;     // m
  double min_diff_angle = 0.1 / 180.0 * M_PI;   // rad
  double min_diff_translation_sqr = Sqr(0.01);  // m^2
};

struct PointNeighborAndNormal {
  Vec3d point;
  Vec3d heading_point;
  std::vector<Vec3d> neighbors;
};

struct PointMatchResult {
  std::string DebugString() const {
    return absl::StrFormat(
        "success: %s, mse: %.4f, num_matched_points: %d, matched_ratio: %.2f, "
        "pca_angle: %.2f, num_iteration: %d, matching_dist: %.2f",
        success ? "YES" : "NO", mse, num_matched_points, matched_ratio,
        pca_angle, num_iteration, matching_dist);
  }

  double mse = 0.0;
  int num_matched_points = 0;
  AffineTransformation transform;
  bool success = false;
  double matched_ratio = 0.0;
  Vec3d diff_cov_diag;
  double pca_angle = 0.0;
  int num_iteration = 0;
  double matching_dist = 0.0;
  // Bias used in registration to avoid "float" overflow.
  Vec3d bias;
  Eigen::Matrix6d cov = Eigen::Matrix6d::Identity();
  Eigen::Matrix6d information = Eigen::Matrix6d::Identity();
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_POINT_MATCHER_STRUCTS_H_
