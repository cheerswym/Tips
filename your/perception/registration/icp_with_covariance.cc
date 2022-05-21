#include "onboard/perception/registration/icp_with_covariance.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "onboard/global/trace.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/perception/registration/registration_common_tool.h"

namespace qcraft {

constexpr int kMinPointsForMatching = 5;

inline bool IsTransformValid(const Mat4d& transform) {
  return std::isfinite(transform(0, 0)) && std::isfinite(transform(0, 1)) &&
         std::isfinite(transform(0, 2)) && std::isfinite(transform(0, 3)) &&
         std::isfinite(transform(1, 0)) && std::isfinite(transform(1, 1)) &&
         std::isfinite(transform(1, 2)) && std::isfinite(transform(1, 3)) &&
         std::isfinite(transform(2, 0)) && std::isfinite(transform(2, 1)) &&
         std::isfinite(transform(2, 2)) && std::isfinite(transform(2, 3)) &&
         transform(3, 0) == 0.0 && transform(3, 1) == 0.0 &&
         transform(3, 2) == 0.0 && transform(3, 3) == 1.0;
}

struct Cloud {
  std::vector<Vec3d> points;
  std::vector<Vec3d> normals;

  void reserve(const int capacity) {
    points.reserve(capacity);
    normals.reserve(capacity);
  }
};
using CloudPtr = std::shared_ptr<Cloud>;
using CloudConstPtr = std::shared_ptr<const Cloud>;

CloudPtr ToCloudPtr(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud) {
  if (nullptr == cloud) return nullptr;
  CloudPtr cloud_out = std::make_shared<Cloud>();
  cloud_out->reserve(cloud->points.size());
  for (const auto& pt : cloud->points) {
    cloud_out->points.emplace_back(pt.x, pt.y, pt.z);
    cloud_out->normals.emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
  }
  return cloud_out;
}

/**********************Point to Plane ICP with covariance**********************
>> Registration from src to ref, which means ref_point = T * src_point
>> Paper "Observability, Covariance and Uncertainty of ICP Scan Matching"
>> Loss function: f(x)=sum{(y_i-H_i*x)^2}
in which:
   y_i = [n_i.T*(a_i-b_i)]
   H_i = [-(a_i \cross n_i.T), -n_i.T]
>> Linearized approximation of rotation:
   df/dx = 2Ax-2b
in which:
   A = sum{H_i.T * H_i}
   b = sum{H_i.T * y_i}
So, we get the transform:
   x_hat = A.inverse() * b
And the covariance will be:
   cov(x_hat) = E[error * error.T]
in which:
   error = A.inverse() * sum{H_i.T * r_i}
*/
IcpCovMatchResult IcpWithCovariance::MatchPoints(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr& ref_points,
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr& src_points,
    const PointMatcherOptions& options) const {
  SCOPED_QTRACE("MatchPoints");

  IcpCovMatchResult result;
  if (ref_points->size() < kMinPointsForMatching ||
      src_points->size() < kMinPointsForMatching) {
    QLOG(WARNING) << "[Registration Failed] Too few points.";
    result.exit = FAIL_TOO_FEW_POINTS;
    return result;
  }

  const auto ref_cloud = ToCloudPtr(ref_points);
  const auto src_cloud = ToCloudPtr(src_points);
  const int num_points = src_cloud->points.size();
  QCHECK_GT(num_points, 0);

  const double max_matching_dist_sqr = Sqr(options.max_matching_dist);
  constexpr double kMatchRatioThreshold = 0.2;
  const int min_matched_points = num_points * kMatchRatioThreshold;

  double mse = std::numeric_limits<double>::max();
  double mse_prev = std::numeric_limits<double>::max();
  int num_matched_points = 0;
  int num_iteration = 0;
  const KDTree kd_tree(ref_cloud->points);
  Eigen::Matrix4d transform = options.hypothesis.mat();
  // x_hat(6x1) [rotation_vector, translation_vector]^T
  // Rotation vector is [u1, u2, u3].T
  // Translation vector is [x, y, z].T
  Eigen::Vector6d x_hat;
  // Solve helper matrix
  Eigen::Matrix6d A, A_inverse;
  Eigen::Vector6d B;
  while (true) {
    Eigen::MatrixXd H(num_points, 6);
    H.setZero();
    Eigen::VectorXd Y(num_points);
    Y.setZero();
    num_matched_points = 0;
    mse = 0;
    for (int i = 0; i < num_points; ++i) {
      const auto a_i = transform.block<3, 3>(0, 0) * src_cloud->points[i] +
                       transform.block<3, 1>(0, 3);
      const auto idx =
          kd_tree.GetOriginalIndex(*kd_tree.FindKNearest(a_i, 1).begin());
      const auto& b_i = ref_cloud->points[idx];
      const auto& n_i = ref_cloud->normals[idx];
      const auto delta = a_i - b_i;
      if (delta.squaredNorm() <= max_matching_dist_sqr &&
          std::isfinite(n_i.x()) && std::isfinite(n_i.y()) &&
          std::isfinite(n_i.z())) {
        // Only assign value to the corresponding matrix rows for correct
        // correspondences.
        // The matrix rows corresponding to the false correspondences default to
        // 0, which won't contribute to the result.
        H.block<1, 3>(i, 0) = -a_i.cross(n_i).transpose();
        H.block<1, 3>(i, 3) = (-n_i.transpose()).transpose();
        Y.row(i) = n_i.transpose() * delta;
        num_matched_points++;
        mse += (n_i.transpose() * delta).squaredNorm();
      }
    }

    // No enough correspondences, result is unreliable.
    if (num_matched_points < min_matched_points || num_matched_points <= 0) {
      QLOG(WARNING) << "[Registration Result Unreliable] Not enough "
                       "correspondences, result "
                       "is unreliable. "
                    << num_matched_points << " < " << min_matched_points;
      result.exit = FAIL_NOT_ENOUGH_CORRESPONDENCES;
      break;
    }
    mse /= num_matched_points;

    // Diverge if mse gets bigger.
    constexpr double kMseDiverageRatioThreshold = 1.4;
    if (mse > mse_prev * kMseDiverageRatioThreshold) {
      QLOG(WARNING) << "[Registration Diverge] Mse greater than mse_prev";
      result.exit = FAIL_MSE_DIVERGE;
      break;
    }
    mse_prev = mse;

    // If iteration meets max_iters and mse is still big.
    if (num_iteration >= options.max_num_iters && mse >= options.max_mse) {
      QLOG(WARNING)
          << "[Registration Result Unreliable] Iterations meets limit "
             "but mse is "
             "not small enough.";
      result.exit = FAIL_ITERATION_REACH_LIMIT;
      break;
    }

    // Estimate Transform
    A = H.transpose() * H;
    // NOTION: To accelerate, change ColPivHouseholderQr to LDLT.
    const auto QR = A.colPivHouseholderQr();
    B = H.transpose() * Y;
    x_hat = QR.solve(B);

    const double theta = x_hat.block<3, 1>(0, 0).norm();
    const double diff_trans_sqr = x_hat.block<3, 1>(3, 0).squaredNorm();
    // If diff_transform matrix diff too small, converge
    if (theta < options.min_diff_angle &&
        diff_trans_sqr < options.min_diff_translation_sqr) {
      result.exit = SUCCESS_DIFF_TRANSFORM_CONVERGE;
      result.success = true;
      A_inverse = QR.inverse();
      break;
    }

    /* NOTE(jingwei) we need inverse of jacobian to estimate covariance, which
     * means we need to do transformation estimation at least once before we can
     * get covariance. So we put mse exit check behind transformation estimation
     * in case initial hypothesis is too good and mse exit directly.
     */
    if (mse < options.max_mse) {
      result.exit = SUCCESS_MSE_CONVERGE;
      result.success = true;
      A_inverse = QR.inverse();
      break;
    }

    // Update transform
    const Eigen::Vector3d axis = x_hat.block<3, 1>(0, 0).normalized();
    const Eigen::AngleAxisd angle_axis(theta, axis);
    Eigen::Matrix4d diff_transform = Eigen::Matrix4d::Identity();
    diff_transform.block<3, 3>(0, 0) = angle_axis.toRotationMatrix();
    diff_transform.block<3, 1>(0, 3) = x_hat.block<3, 1>(3, 0);
    if (!IsTransformValid(diff_transform)) {
      QLOG(WARNING)
          << "Transformation matrix meets numerical calculation problem.\n"
          << "transform is \n"
          << transform << "\ndiff_transform is \n"
          << diff_transform;
      result.exit = FAIL_NUMERAL_CALCULATIONS_FAIL;
      break;
    }
    transform = diff_transform * transform;

    num_iteration++;
  }

  result.mse = mse;
  result.pre_mse = mse_prev;
  result.src_num_points = num_points;
  result.num_matched_points = num_matched_points;
  result.num_iteration = num_iteration;
  if (result.success) {
    result.transform.Set(transform);
    // cov(6x6) corresponding to [u1, u2, u3, x, y, z]
    // cov = sigma^2 * (H.T * H)
    // TODO(jingwei) use mse instead of kLaserMeaSigma2?
    constexpr double kLaserMeaSigma2 = Sqr(1.0);
    result.cov = A_inverse * kLaserMeaSigma2;
  }

  return result;
}

}  // namespace qcraft
