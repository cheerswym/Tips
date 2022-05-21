#include "onboard/perception/registration/icp_with_normal_lls.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "absl/time/time.h"
#include "offboard/mapping/mapping_core/util/point_cloud_util.h"
#include "onboard/global/trace.h"
#include "onboard/math/eigen.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/perception/registration/registration_common_tool.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/transformation_estimation_point_to_plane_weighted.h"
#include "pcl/registration/warp_point_rigid.h"

namespace qcraft {
namespace {

void DownSamplePoints(std::vector<Vec3d>* points, int max_num_points) {
  DCHECK_GT(points->size(), max_num_points);
  const float step = points->size() / static_cast<float>(max_num_points);
  int index = 0;
  float index_f = 0.0f;
  const int num_points = points->size();
  while (true) {
    const int i = RoundToInt(index_f);
    if (i >= num_points) break;
    (*points)[index++] = (*points)[i];
    index_f += step;
  }
  DCHECK_LE(index, num_points);
  points->resize(index);
}

}  // namespace

// Warning! Input point cloud may be changed.
template <typename PointSource, typename PointTarget, typename Scalar>
PointMatchResult
IcpWithNormalLls<PointSource, PointTarget, Scalar>::MatchPoints(
    const typename pcl::PointCloud<PointSource>::Ptr src_cloud_ptr,
    const typename pcl::PointCloud<PointTarget>::Ptr ref_cloud_ptr,
    const PointMatcherOptions& options) const {
  SCOPED_QTRACE("MatchPoints");
  PointMatchResult result;

  constexpr int kMinPointNumForRegistration = 10;
  if (src_cloud_ptr->size() < kMinPointNumForRegistration ||
      ref_cloud_ptr->size() < kMinPointNumForRegistration) {
    return {};
  }
  // Only use absl::Now() for latency calculation and logging, use Clock::Now()
  // for other purposes.
  const absl::Time pre_regsitration = absl::Now();
  if (src_cloud_ptr->size() > options.max_num_points) {
    reg_tool::DownSamplePoints<PointSource>(src_cloud_ptr,
                                            options.max_num_points);
  }
  if (ref_cloud_ptr->size() > options.max_num_points) {
    reg_tool::DownSamplePoints<PointTarget>(ref_cloud_ptr,
                                            options.max_num_points);
  }

  // Init convergence criteria
  Eigen::Matrix4d delta_transform = Eigen::Matrix4d::Identity();
  pcl::CorrespondencesPtr correspondences(
      boost::make_shared<pcl::Correspondences>());
  pcl::registration::DefaultConvergenceCriteria<double> convergence_criteria(
      0, delta_transform, *correspondences);
  const int max_iteration = options.max_num_iters;
  convergence_criteria.setMaximumIterations(max_iteration);
  convergence_criteria.setMaximumIterationsSimilarTransforms(2);
  convergence_criteria.setRelativeMSE(1e-6);
  // Init correspondence solver
  // To use euclidean distance as metric, kdtree should be built by xyz, which
  // means <typename PointTarget> of CorrespondenceEstimation should be defined
  // as pcl::PointXYZ.
  boost::shared_ptr<
      pcl::registration::CorrespondenceEstimation<PointSource, pcl::PointXYZ>>
      cor_est_closest_point(
          boost::make_shared<pcl::registration::CorrespondenceEstimation<
              PointSource, pcl::PointXYZ>>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_xyz =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*ref_cloud_ptr, *ref_cloud_xyz);
  cor_est_closest_point->setInputTarget(ref_cloud_xyz);
  // Init transformation
  typename pcl::PointCloud<PointSource>::Ptr src_aligned_cloud_ptr =
      boost::make_shared<typename pcl::PointCloud<PointSource>>();
  Eigen::Matrix4d final_transform = options.hypothesis.mat();
  pcl::transformPointCloud(*src_cloud_ptr, *src_aligned_cloud_ptr,
                           final_transform);
  // Solve iteratively
  double correspondence_threshold = options.max_matching_dist;
  bool first_iteration = true;
  int iteration = 0;
  do {
    SCOPED_QTRACE_ARG1("estimateRigidTransformation", "iteration", iteration);
    // NOTE(dong): We decrease the correspondence threshold by a factor of 1.5
    // in the iteration to reduce outliers while estimating transformation.
    constexpr double kMinCorrespondenceThreshold = 0.15;  // parameter
    if (!first_iteration &&
        correspondence_threshold > kMinCorrespondenceThreshold) {
      correspondence_threshold *= 1.0 / 1.5;
    }
    first_iteration = false;
    cor_est_closest_point->setInputSource(src_aligned_cloud_ptr);
    cor_est_closest_point->determineCorrespondences(
        *correspondences, correspondence_threshold + 0.05);
    delta_transform.setIdentity();
    transformation_estimation_.estimateRigidTransformation(
        *src_aligned_cloud_ptr, *ref_cloud_ptr, *correspondences,
        delta_transform);
    // NOTE(ZHANKUN): now, judge the delta_transform is or not a valid value is
    // by comparing the delta_transform(0, 0) and nan, but this is fragile. This
    // will be completed in the later
    if (std::isnan(delta_transform(0, 0))) {
      break;
    }
    final_transform = delta_transform * final_transform;
    pcl::transformPointCloud(*src_cloud_ptr, *src_aligned_cloud_ptr,
                             final_transform);
    iteration++;
  } while (!convergence_criteria.hasConverged());
  // Manage result.
  const double max_matching_dist_sqr = Sqr(correspondence_threshold);
  int match_count = 0;
  double mse = 0;
  for (const auto& elem : *correspondences) {
    if (elem.index_match == -1) continue;
    const auto& point_target = ref_cloud_ptr->points[elem.index_match];
    const auto& point_source = src_aligned_cloud_ptr->points[elem.index_query];
    const Vec3d vec_diff(point_target.x - point_source.x,
                         point_target.y - point_source.y,
                         point_target.z - point_source.z);
    const double dist = vec_diff.squaredNorm();
    if (dist < max_matching_dist_sqr) {
      ++match_count;
      mse += dist;
    }
  }
  // Log result
  result.num_matched_points = match_count;
  result.matched_ratio =
      1.0 * match_count /
      std::min(src_aligned_cloud_ptr->size(), ref_cloud_ptr->size());
  result.mse =
      match_count > 0 ? mse / match_count : std::numeric_limits<double>::max();
  result.transform = AffineTransformation(final_transform);
  result.success = iteration < max_iteration;
  result.num_iteration = iteration;
  result.matching_dist = correspondence_threshold;

  VLOG(2) << "------------Icp With Normal LLS------------";
  VLOG(2) << "result.num_matched_points " << result.num_matched_points;
  VLOG(2) << "result.matched_ratio " << result.matched_ratio;
  VLOG(2) << "result.mse " << result.mse;
  VLOG(2) << "iteration " << result.num_iteration;
  VLOG(2) << "result.success " << result.success;
  VLOG(2) << "result.matching_dist " << result.matching_dist;

  // Only use absl::Now() for latency calculation and logging, use Clock::Now()
  // for other purposes.
  const auto after_registration = absl::Now();
  VLOG(2) << "[ICP] Point2Plane time_consume is "
          << absl::ToInt64Milliseconds(after_registration - pre_regsitration)
          << " ns. source pts num is " << src_cloud_ptr->size()
          << ", target pts num is " << ref_cloud_ptr->size();

  return result;
}

template <typename PointSource, typename PointTarget, typename Scalar>
PointMatchResult
IcpWithNormalLls<PointSource, PointTarget, Scalar>::MatchPoints(
    const SegmentedCluster& ref_cluster, const std::vector<Vec3d>& src_points,
    const PointMatcherOptions& options) const {
  SCOPED_QTRACE("MatchPoints");
  PointMatchResult result;

  // Get bias ref cloud and bias src cloud.
  auto [ref_cloud_normals, bias] = ref_cluster.GetBiasCloud();
  std::vector<Vec3d> transformed_src_points;
  transformed_src_points.reserve(src_points.size());
  for (const auto& src_point : src_points) {
    transformed_src_points.emplace_back(src_point - bias);
  }
  result.bias = bias;

  constexpr int kMinPointsForMatching = 5;
  if (ref_cloud_normals->size() < kMinPointsForMatching ||
      src_points.size() < kMinPointsForMatching) {
    result.success = false;
    result.num_matched_points = 0;
    result.mse = false;
    return result;
  }
  // Only use absl::Now() for latency calculation and logging, use Clock::Now()
  // for other purposes.
  const absl::Time pre = absl::Now();

  if (src_points.size() > options.max_num_points) {
    DownSamplePoints(&transformed_src_points, options.max_num_points);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud =
      mapping::util::ToPclPointCloud(transformed_src_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
  // const int num_points = transformed_src_points.size();
  const int num_points = ref_cloud_normals->size();
  QCHECK_GT(num_points, 0);

  pcl::CorrespondencesPtr correspondences(
      boost::make_shared<pcl::Correspondences>());
  int iteration = 0;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // Convergence criteria
  const int max_iteration = options.max_num_iters;
  pcl::registration::DefaultConvergenceCriteria<double> convergence_criteria(
      iteration, transform, *correspondences);
  convergence_criteria.setMaximumIterations(max_iteration);
  convergence_criteria.setMaximumIterationsSimilarTransforms(2);
  convergence_criteria.setRelativeMSE(1e-6);
  // Correspondence estimation
  boost::shared_ptr<
      pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>>
      cor_est_closest_point(
          boost::make_shared<pcl::registration::CorrespondenceEstimation<
              pcl::PointXYZ, pcl::PointXYZ>>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_xyz =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*ref_cloud_normals, *ref_cloud_xyz);
  cor_est_closest_point->setInputTarget(ref_cloud_xyz);

  Eigen::Matrix4d initial_estimate = options.hypothesis.mat();
  pcl::transformPointCloud(*src_cloud, *source_aligned, initial_estimate);
  Eigen::Matrix4d final_transform = initial_estimate;
  Eigen::Matrix4d initial_transform = initial_estimate;

  double correspondence_threshold = options.max_matching_dist;
  bool first_iteration = true;
  do {
    {
      SCOPED_QTRACE_ARG1("estimateRigidTransformation", "iteration", iteration);
      // NOTE(dong): We decrease the correspondence threshold by a factor of 1.5
      // in the iteration to reduce outliers while estimating transformation.
      constexpr double kMinCorrespondenceThreshold = 0.15;  // parameter
      if (!first_iteration &&
          correspondence_threshold > kMinCorrespondenceThreshold) {
        correspondence_threshold *= 1.0 / 1.5;
      }
      first_iteration = false;

      cor_est_closest_point->setInputSource(source_aligned);
      cor_est_closest_point->determineCorrespondences(
          *correspondences, correspondence_threshold + 0.05);
      transform.setIdentity();
      transformation_estimation_.estimateRigidTransformation(
          *source_aligned, *ref_cloud_normals, *correspondences, transform);
      // NOTE(ZHANKUN): now, judge the transform is or not a valid value is
      // by comparing the transform(0, 0) and nan, but this is fragile. This
      // will be completed in the later
      if (std::isnan(transform(0, 0))) {
        break;
      }
      final_transform = transform * final_transform;
      pcl::transformPointCloud(*src_cloud, *source_aligned, final_transform);
    }
    iteration++;
  } while (!convergence_criteria.hasConverged());
  // estimate matching points number
  const double max_matching_dist_sqr = Sqr(correspondence_threshold);
  int match_count = 0;
  double mse = 0;
  for (const auto& elem : *correspondences) {
    if (elem.index_match == -1) continue;
    const auto& point_target = ref_cloud_normals->points[elem.index_match];
    const auto& point_source = source_aligned->points[elem.index_query];
    const Vec3d vec_diff(point_target.x - point_source.x,
                         point_target.y - point_source.y,
                         point_target.z - point_source.z);
    const double dist = vec_diff.squaredNorm();
    if (dist < max_matching_dist_sqr) {
      ++match_count;
      mse += dist;
    }
  }

  result.num_matched_points = match_count;
  result.matched_ratio =
      1.0 * match_count /
      std::min(transformed_src_points.size(), ref_cloud_normals->size());
  result.mse =
      match_count > 0 ? mse / match_count : std::numeric_limits<double>::max();
  result.transform = AffineTransformation(final_transform);
  result.success = iteration < max_iteration;
  result.num_iteration = iteration;
  result.matching_dist = correspondence_threshold;

  VLOG(2) << "------------Icp With Normal LLS------------";
  VLOG(2) << "result.num_matched_points " << result.num_matched_points;
  VLOG(2) << "result.matched_ratio " << result.matched_ratio;
  VLOG(2) << "result.mse " << result.mse;
  VLOG(2) << "iteration " << result.num_iteration;
  VLOG(2) << "result.success " << result.success;
  VLOG(2) << "result.matching_dist " << result.matching_dist;
  // Only use absl::Now() for latency calculation and logging, use Clock::Now()
  // for other purposes.
  VLOG(2) << "time cost(ms): " << absl::ToInt64Milliseconds(absl::Now() - pre);

  return result;
}

template class IcpWithNormalLls<pcl::PointXYZ, pcl::PointNormal, double>;

}  // namespace qcraft
