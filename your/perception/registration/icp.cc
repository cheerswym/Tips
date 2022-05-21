#include "onboard/perception/registration/icp.h"

#include <cfloat>
#include <cmath>
#include <limits>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/registration/registration_common_tool.h"

namespace qcraft {
namespace {}  // namespace

PointMatchResult Icp::MatchCluster(const Cluster& ref_cluster,
                                   const Cluster& src_cluster,
                                   const PointMatcherOptions& options) const {
  const std::vector<Vec3d> ref_points =
      cluster_util::CollectPoints(ref_cluster);
  const std::vector<Vec3d> src_points =
      cluster_util::CollectPoints(src_cluster);
  return MatchPoints(ref_points, src_points, options);
}

PointMatchResult Icp::MatchPoints(const std::vector<Vec3d>& ref_points,
                                  const std::vector<Vec3d>& src_points,
                                  const PointMatcherOptions& options) const {
  std::vector<Vec3d> sampled_ref_points = ref_points;
  // Bias is used to offset pointcloud so that the coordinate value of points
  // will not overflow.
  const auto bias = sampled_ref_points[0];
  std::vector<Vec3d> sampled_src_points = src_points;
  if (ref_points.size() > options.max_num_points) {
    reg_tool::DownSamplePointsUsingFixedStep(&sampled_ref_points,
                                             options.max_num_points);
  }
  if (src_points.size() > options.max_num_points) {
    reg_tool::DownSamplePointsUsingFixedStep(&sampled_src_points,
                                             options.max_num_points);
  }
  const int num_points = sampled_src_points.size();
  QCHECK_GT(num_points, 0);

  double max_matching_dist_sqr = Sqr(options.max_matching_dist);

  const KDTree kd_tree(sampled_ref_points);
  AffineTransformation transform = options.hypothesis;
  double mse = std::numeric_limits<double>::max();
  bool coveraged = false;
  int num_matched_points = 0;
  int num_iteration = 0;
  for (int i = 0; i < options.max_num_iters + 1; ++i) {
    // Transform all points using the latest transformation.
    ++num_iteration;
    Eigen::MatrixXd ref_points_mat(3, num_points);
    Eigen::MatrixXd src_points_mat(3, num_points);
    Eigen::MatrixXd weights(1, num_points);
    mse = 0.0;
    num_matched_points = 0;
    for (int j = 0; j < num_points; ++j) {
      const auto transformed_point =
          transform.TransformPoint(sampled_src_points[j]);
      ref_points_mat.col(j) = kd_tree.FindNearest(transformed_point);
      src_points_mat.col(j) = transformed_point;
      // If the matched points have the distance > max_matching_dist, set the
      // weight for this match to 0.0.
      const double dist2 =
          (ref_points_mat.col(j) - src_points_mat.col(j)).squaredNorm();
      if (dist2 <= max_matching_dist_sqr) {
        weights(j) = 1.0;
        num_matched_points++;
      } else {
        weights(j) = 0.0;
      }
      mse += dist2;
    }
    mse /= num_points;
    // Return success = false if diveraged.
    // iteration termination condition.
    if (coveraged || mse < options.max_mse) {
      coveraged = true;
      break;
    }
    if (i == options.max_num_iters) {
      VLOG_EVERY_N(2, 100) << absl::StrFormat(
          "Point Matcher has reached max_num_iters %d. mse %.3f. num_points "
          "%d. num_matched_points %d. optimize_convergence %d. Maybe not "
          "coveraged.",
          options.max_num_iters, mse, num_points, num_matched_points,
          options.optimize_convergence);
      break;
    }
    if (options.optimize_convergence &&
        num_matched_points > num_points * 2 / 3) {
      double new_dist_sqr = 0.25 * mse;
      if (max_matching_dist_sqr > new_dist_sqr * 2) {
        max_matching_dist_sqr /= 2.0;
      } else {
        max_matching_dist_sqr = new_dist_sqr;
      }
    }

    const double weight_sum = weights.sum();
    if (weight_sum == 0.0) {
      return {.mse = mse,
              .num_matched_points = num_matched_points,
              .transform = transform,
              .success = false,
              .matched_ratio = 0.0,
              .diff_cov_diag = {0, 0, 0},
              .pca_angle = 0,
              .num_iteration = num_iteration,
              .matching_dist = 0.0,
              .bias = {0, 0, 0}};
    }

    const double num_points_inv = 1.0 / weight_sum;
    const Vec3d ref_points_mean =
        (ref_points_mat * weights.transpose()) * num_points_inv;
    const Vec3d src_points_mean =
        (src_points_mat * weights.transpose()) * num_points_inv;
    ref_points_mat.colwise() -= ref_points_mean;
    src_points_mat.colwise() -= src_points_mean;
    Mat3d rotation_mat;
    if (options.translation_only) {
      rotation_mat = Mat3d::Identity();
    } else {
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
          ref_points_mat * weights.asDiagonal() * src_points_mat.transpose(),
          Eigen::ComputeThinU | Eigen::ComputeThinV);
      rotation_mat = svd.matrixU() * svd.matrixV().transpose();
      // It is possible to get a reflection instead of a rotation. In this case,
      // we take the second best solution, guaranteed to be a rotation.
      if (rotation_mat.determinant() < 0.0) {
        Mat3d v_tmp = svd.matrixV().transpose();
        v_tmp.row(2) *= -1.0;
        rotation_mat = svd.matrixU() * v_tmp;
      }
    }

    const Vec3d translation_vec =
        ref_points_mean - rotation_mat * src_points_mean;
    Mat4d transform_mat(Mat4d::Identity(4, 4));
    transform_mat.topLeftCorner(3, 3) = rotation_mat;
    transform_mat.topRightCorner(3, 1) = translation_vec;

    AffineTransformation diff_transform = AffineTransformation(transform_mat);
    if (diff_transform.GetTranslation().norm() < 1e-6 &&
        diff_transform.GetRotation().vec().norm() < 1e-6) {
      coveraged = true;
    }
    transform = diff_transform * transform;
  }
  return {.mse = mse,
          .num_matched_points = num_matched_points,
          .transform = transform,
          .success = true,
          .matched_ratio = 0.0,
          .diff_cov_diag = {0, 0, 0},
          .pca_angle = 0,
          .num_iteration = num_iteration,
          .matching_dist = 0.0,
          .bias = bias};
}

}  // namespace qcraft
