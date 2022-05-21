#include "onboard/perception/registration/icp_with_normal.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "offboard/mapping/mapping_core/util/point_cloud_util.h"
#include "onboard/global/trace.h"
#include "onboard/math/eigen.h"
#include "onboard/math/util.h"
#include "onboard/perception/registration/registration_common_tool.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_types.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/transformation_estimation_point_to_plane_weighted.h"
#include "pcl/registration/warp_point_rigid.h"
namespace pcl {
namespace registration {

template <typename PointSourceT, typename PointTargetT,
          typename Scalar = double>
class WarpPointRigid3DTzRollPitch
    : public WarpPointRigid<PointSourceT, PointTargetT, Scalar> {
 public:
  using Matrix4 =
      typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4;
  using VectorX =
      typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX;

  using Ptr = boost::shared_ptr<
      WarpPointRigid3DTzRollPitch<PointSourceT, PointTargetT, Scalar>>;
  using ConstPtr = boost::shared_ptr<
      const WarpPointRigid3DTzRollPitch<PointSourceT, PointTargetT, Scalar>>;

  WarpPointRigid3DTzRollPitch()
      : WarpPointRigid<PointSourceT, PointTargetT, Scalar>(3) {}

  ~WarpPointRigid3DTzRollPitch() {}

  // p warp parameters (tz, roll, pitch)
  void setParam(const VectorX& p) override {
    assert(p.rows() == this->getDimension());
    Matrix4& trans = this->transform_matrix_;

    trans = Matrix4::Zero();
    trans(3, 3) = 1;

    // Copy the translation components
    trans.block(0, 3, 4, 1) = Eigen::Matrix<Scalar, 4, 1>(0.0, 0.0, p[0], 1.0);

    // Compute rotation from roll & pitch.
    trans.topLeftCorner(3, 3) = Eigen::Matrix<Scalar, 3, 3>(
        qcraft::AngleAxis(p[2], qcraft::Vec3d::UnitY()) *
        qcraft::AngleAxis(p[1], qcraft::Vec3d::UnitX()));
  }
};

constexpr double kMinWeightedRangeSqr = 10 * 10;  // m * m
constexpr double kMaxWeightedRangeSqr = 70 * 70;  // m * m
constexpr double kMaxWeightedFactor = 1.2;
constexpr double kMinWeightedFactor = 1.0;

template <typename PointSource, typename PointTarget, typename Scalar = double>
class TransformationEstimationPointToPlaneCustomized
    : public TransformationEstimationLM<PointSource, PointTarget, Scalar> {
 public:
  using Ptr = boost::shared_ptr<TransformationEstimationPointToPlaneCustomized<
      PointSource, PointTarget, Scalar>>;
  using ConstPtr =
      boost::shared_ptr<const TransformationEstimationPointToPlaneCustomized<
          PointSource, PointTarget, Scalar>>;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

  TransformationEstimationPointToPlaneCustomized() = default;
  ~TransformationEstimationPointToPlaneCustomized() = default;

 protected:
  Scalar computeDistance(const Vector4& p_src,
                         const PointTarget& p_tgt) const override {
    // Compute the point-to-plane distance
    Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
    Vector4 n(p_tgt.normal_x, p_tgt.normal_y, p_tgt.normal_z, 0);

    const double factor =
        p_src.x() > 0
            ? kMaxWeightedFactor +
                  std::clamp((qcraft::Sqr(p_tgt.x) + qcraft::Sqr(p_tgt.y) -
                              kMinWeightedRangeSqr) *
                                 (kMaxWeightedFactor - kMinWeightedFactor) /
                                 (kMinWeightedRangeSqr - kMaxWeightedRangeSqr),
                             kMinWeightedFactor - kMaxWeightedFactor, 0.0)
            : 1.0;

    return ((p_src - t).dot(n)) * factor;
  }
};

}  // namespace registration
}  // namespace pcl

namespace qcraft {

namespace {

constexpr double kMinDistThreshold = 0.3;
constexpr double kDistThresholdReductionFactor = 1.0 / 1.5;
constexpr double kExpectedMaxDistThreshold1 = 0.3;
constexpr double kExpectedMaxDistThreshold2 = 0.6;
constexpr double kExpectedMaxDistThreshold3 = 1.0;
constexpr double kMinMatchCountRatio = 0.35;
constexpr double kExpectedMinMatchCountRatio = 0.5;
constexpr double kMaxConvergedMse = 0.1;
constexpr double kMaxConvergedMseWithMaxIteration = 0.01;

void TransformPoints(std::vector<Vec3d>* dst_points,
                     const std::vector<Vec3d>& src_points,
                     const AffineTransformation& t) {
  dst_points->reserve(src_points.size());
  for (auto& src_point : src_points) {
    dst_points->emplace_back(t.TransformPoint(src_point));
  }
}

}  // namespace

PointMatchResult IcpWithNormal::MatchPoints(
    const std::vector<Vec3d>& ref_points, const std::vector<Vec3d>& src_points,
    const PointMatcherOptions& options,
    const AffineTransformation& transform_inv) const {
  SCOPED_QTRACE("IcpWithNormal::MatchPoints");

  // Only use absl::Now() for latency calculation and logging, use Clock::Now()
  // for other purposes.
  absl::Time pre = absl::Now();
  // Disable pcl console output.
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  std::vector<Vec3d> transformed_ref_points;
  TransformPoints(&transformed_ref_points, ref_points, transform_inv);
  std::vector<Vec3d> transformed_src_points;
  TransformPoints(&transformed_src_points, src_points, transform_inv);

  if (ref_points.size() > options.max_num_points) {
    reg_tool::DownSamplePointsUsingFixedStep(&transformed_ref_points,
                                             options.max_num_points);
  }
  if (src_points.size() > options.max_num_points) {
    reg_tool::DownSamplePointsUsingFixedStep(&transformed_src_points,
                                             options.max_num_points);
  }

  if (transformed_src_points.empty()) {
    QLOG(ERROR)
        << "Transformed src points is empty. Transformed ref points size is "
        << transformed_ref_points.size();
    return {};
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud =
      mapping::util::ToPclPointCloud(transformed_ref_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud =
      mapping::util::ToPclPointCloud(transformed_src_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation;
  normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
      boost::make_shared<pcl::search::KdTree<pcl::PointNormal>>()));
  normal_estimation.setKSearch(7);
  pcl::PointCloud<pcl::PointNormal>::Ptr ref_cloud_normals(
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>());
  pcl::copyPointCloud(*ref_cloud, *ref_cloud_normals);
  normal_estimation.setInputCloud(ref_cloud_normals);
  normal_estimation.setViewPoint(0.0, 0.0, 1000.0);
  {
    SCOPED_QTRACE("IcpWithNormal::MatchPoints_estimate_normal");
    normal_estimation.compute(*ref_cloud_normals);
  }

  pcl::CorrespondencesPtr correspondences(
      boost::make_shared<pcl::Correspondences>());
  int iteration = 0;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // Convergence criteria
  const int max_iteration = options.max_num_iters;
  pcl::registration::DefaultConvergenceCriteria<double> convergence_criteria(
      iteration, transform, *correspondences);
  convergence_criteria.setMaximumIterations(max_iteration);
  convergence_criteria.setMaximumIterationsSimilarTransforms(5);
  convergence_criteria.setRelativeMSE(1e-5);

  // Correspondence estimation
  boost::shared_ptr<
      pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>>
      cor_est_closest_point(
          boost::make_shared<pcl::registration::CorrespondenceEstimation<
              pcl::PointXYZ, pcl::PointXYZ>>());
  cor_est_closest_point->setInputTarget(ref_cloud);
  cor_est_closest_point->setInputSource(source_aligned);

  pcl::registration::WarpPointRigid3DTzRollPitch<pcl::PointXYZ,
                                                 pcl::PointNormal, double>::Ptr
      warp_fcn(
          boost::make_shared<pcl::registration::WarpPointRigid3DTzRollPitch<
              pcl::PointXYZ, pcl::PointNormal, double>>());
  // Transformation estimation
  pcl::registration::TransformationEstimationPointToPlaneCustomized<
      pcl::PointXYZ, pcl::PointNormal, double>
      transformation_estimation;
  transformation_estimation.setWarpFunction(warp_fcn);
  Eigen::Matrix4d initial_estimate = options.hypothesis.mat();
  pcl::transformPointCloud(*src_cloud, *source_aligned, initial_estimate);
  Eigen::Matrix4d final_transform = initial_estimate;

  double correspondence_threshold =
      options.max_matching_dist * (1 / kDistThresholdReductionFactor);
  bool init_with_expected_max_dist_threshold = false;
  auto compute_match_count_ratio = [&](const double dist_threshold) {
    cor_est_closest_point->determineCorrespondences(*correspondences,
                                                    dist_threshold);
    int match_count = 0;
    for (const auto& elem : *correspondences) {
      if (elem.index_match == -1) continue;
      if (elem.distance < dist_threshold) {
        ++match_count;
      }
    }
    return match_count * 1.0 / source_aligned->points.size();
  };
  // Init with expected distance threshold.
  do {
    if (correspondence_threshold <= kExpectedMaxDistThreshold1) {
      break;
    }
    double match_count_ratio =
        compute_match_count_ratio(kExpectedMaxDistThreshold1);
    if (match_count_ratio > kExpectedMinMatchCountRatio) {
      correspondence_threshold = kExpectedMaxDistThreshold1;
      init_with_expected_max_dist_threshold = true;
      break;
    }

    if (correspondence_threshold <= kExpectedMaxDistThreshold2) {
      break;
    }
    match_count_ratio = compute_match_count_ratio(kExpectedMaxDistThreshold2);
    if (match_count_ratio > kExpectedMinMatchCountRatio) {
      correspondence_threshold = kExpectedMaxDistThreshold2;
      init_with_expected_max_dist_threshold = true;
      break;
    }

    if (correspondence_threshold <= kExpectedMaxDistThreshold3) {
      break;
    }
    match_count_ratio = compute_match_count_ratio(kExpectedMaxDistThreshold3);
    if (match_count_ratio > kExpectedMinMatchCountRatio) {
      correspondence_threshold = kExpectedMaxDistThreshold3;
      init_with_expected_max_dist_threshold = true;
      break;
    }

    QLOG(WARNING) << absl::StrFormat(
        "Can't find enough correspondences with dist threshold of %.2f. "
        "Match count ratio is %.2f, source points num is %d",
        kExpectedMaxDistThreshold3, match_count_ratio,
        source_aligned->points.size());
  } while (false);

  do {
    {
      SCOPED_QTRACE_ARG1("EstimateRigidTransformation", "iteration", iteration);
      // NOTE(dong): We decrease the correspondence threshold by a factor of 1.5
      // in the iteration to reduce outliers while estimating transformation.
      if (!init_with_expected_max_dist_threshold) {
        if (correspondence_threshold > kMinDistThreshold) {
          correspondence_threshold *= kDistThresholdReductionFactor;
        }
        cor_est_closest_point->determineCorrespondences(
            *correspondences, correspondence_threshold + 0.05);
      } else {
        init_with_expected_max_dist_threshold = false;
      }

      transform.setIdentity();
      transformation_estimation.estimateRigidTransformation(
          *source_aligned, *ref_cloud_normals, *correspondences, transform);
      final_transform = transform * final_transform;
      pcl::transformPointCloud(*src_cloud, *source_aligned, final_transform);
    }
    iteration++;
  } while (!convergence_criteria.hasConverged());

  // estimate matching points number
  const double max_matching_dist_sqr = qcraft::Sqr(correspondence_threshold);
  int match_count = 0;
  double mse = 0;
  for (const auto& elem : *correspondences) {
    if (elem.index_match == -1) continue;
    const auto& point_target = ref_cloud->points[elem.index_match];
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

  PointMatchResult result;
  result.num_matched_points = match_count;
  result.matched_ratio = 1.0 * match_count / src_cloud->size();
  result.mse =
      match_count > 0 ? mse / match_count : std::numeric_limits<double>::max();
  result.transform = AffineTransformation(
      transform_inv.Inverse() * AffineTransformation(final_transform) *
      transform_inv);
  result.num_iteration = iteration;
  result.matching_dist = correspondence_threshold;
  result.success =
      ((iteration < max_iteration && result.mse < kMaxConvergedMse) ||
       (result.mse < kMaxConvergedMseWithMaxIteration)) &&
      result.matched_ratio > kMinMatchCountRatio;

  VLOG(2) << "------------Icp With Normal------------";
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

}  // namespace qcraft
