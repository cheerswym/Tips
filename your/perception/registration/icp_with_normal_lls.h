#ifndef ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_LLS_H_
#define ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_LLS_H_

#include <type_traits>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/registration/point_matcher_structs.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_types.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/transformation_estimation_point_to_plane_lls.h"
#include "pcl/registration/transformation_estimation_point_to_plane_weighted.h"
#include "pcl/registration/warp_point_rigid.h"

namespace pcl {
namespace registration {

// Used for ICP tracker,
template <typename PointSource, typename PointTarget, typename Scalar = double>
class TransformationEstimationPointToPlaneForIcpLlsTracker
    : public TransformationEstimationPointToPlaneLLS<PointSource, PointTarget,
                                                     Scalar> {
 public:
  using Ptr =
      boost::shared_ptr<TransformationEstimationPointToPlaneForIcpLlsTracker<
          PointSource, PointTarget, Scalar>>;
  using ConstPtr = boost::shared_ptr<
      const TransformationEstimationPointToPlaneForIcpLlsTracker<
          PointSource, PointTarget, Scalar>>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using Matrix4 =
      typename TransformationEstimationPointToPlaneLLS<PointSource, PointTarget,
                                                       Scalar>::Matrix4;
  using TransformationEstimationPointToPlaneLLS<
      PointSource, PointTarget, Scalar>::constructTransformationMatrix;

  TransformationEstimationPointToPlaneForIcpLlsTracker() = default;
  ~TransformationEstimationPointToPlaneForIcpLlsTracker() = default;
  void estimateRigidTransformation(
      const pcl::PointCloud<PointSource>& cloud_src,
      const pcl::PointCloud<PointTarget>& cloud_tgt,
      const pcl::Correspondences& correspondences,
      Matrix4* transformation_matrix) const override {
    if (nullptr == transformation_matrix) return;
    ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
    ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences,
                                              false);
    estimateRigidTransformation(source_it, target_it, transformation_matrix);
  }
  void estimateRigidTransformation(
      const ConstCloudIterator<PointSource>& source_it,
      const ConstCloudIterator<PointTarget>& target_it,
      Matrix4* transformation_matrix) const {
    if (nullptr == transformation_matrix) return;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Matrix3d = Eigen::Matrix<double, 3, 3>;
    Matrix3d ATA;
    Vector3d ATb;
    ATA.setZero();
    ATb.setZero();
    while (source_it.isValid() && target_it.isValid()) {
      if (!std::isfinite(source_it->x) || !std::isfinite(source_it->y) ||
          !std::isfinite(source_it->z) || !std::isfinite(target_it->x) ||
          !std::isfinite(target_it->y) || !std::isfinite(target_it->z) ||
          !std::isfinite(target_it->normal_x) ||
          !std::isfinite(target_it->normal_y) ||
          !std::isfinite(target_it->normal_z)) {
        ++target_it;
        ++source_it;
        continue;
      }

      const double sx = source_it->x;
      const double sy = source_it->y;
      const double sz = source_it->z;
      const double dx = target_it->x;
      const double dy = target_it->y;
      const double dz = target_it->z;
      const double nx = target_it->normal[0];
      const double ny = target_it->normal[1];
      const double nz = target_it->normal[2];

      const double c = ny * sx - nx * sy;

      //    0  1  2
      //    3  4  5
      //    6  7  8

      ATA.coeffRef(0) += c * c;
      ATA.coeffRef(1) += c * nx;
      ATA.coeffRef(2) += c * ny;
      ATA.coeffRef(4) += nx * nx;
      ATA.coeffRef(5) += nx * ny;
      ATA.coeffRef(8) += ny * ny;

      const double d =
          nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;
      ATb.coeffRef(0) += c * d;
      ATb.coeffRef(1) += nx * d;
      ATb.coeffRef(2) += ny * d;

      ++target_it;
      ++source_it;
    }

    ATA.coeffRef(3) = ATA.coeff(1);
    ATA.coeffRef(6) = ATA.coeff(2);
    ATA.coeffRef(7) = ATA.coeff(5);
    const Vector3d x = static_cast<Vector3d>(ATA.inverse() * ATb);
    constructTransformationMatrix(0, 0, x(0), x(1), x(2), 0,
                                  *transformation_matrix);
  }

  void set_ground_z(double ground_z) { ground_z_ = ground_z; }

 private:
  double ground_z_;
};

}  // namespace registration
}  // namespace pcl

namespace qcraft {

// Used for ICP lls

template <typename PointSource, typename PointTarget, typename Scalar = double>
class IcpWithNormalLls {
  static_assert(std::is_same<pcl::PointNormal, PointTarget>::value,
                "PointSource must have normals. For now, use pcl::PointNormal");

 public:
  using Solver = pcl::registration::TransformationEstimationPointToPlaneLLS<
      PointSource, PointTarget, Scalar>;

  IcpWithNormalLls() = default;

  PointMatchResult MatchCluster(const Cluster& ref_cluster,
                                const Cluster& src_cluster,
                                const PointMatcherOptions& options) const;

  PointMatchResult MatchPoints(
      const typename pcl::PointCloud<PointSource>::Ptr src_cloud_ptr,
      const typename pcl::PointCloud<PointTarget>::Ptr ref_cloud_ptr,
      const PointMatcherOptions& options) const;

  // In this MatchPoints function, the normals has been compute before and
  // stored in the ref_cluster
  PointMatchResult MatchPoints(const SegmentedCluster& ref_cluster,
                               const std::vector<Vec3d>& src_points,
                               const PointMatcherOptions& options) const;
  // In this MatchPoints function, the kdtree is used to compute the neighbors
  // and normals
  PointMatchResult MatchPoints(const std::vector<Vec3d>& ref_points,
                               const std::vector<Vec3d>& src_points,
                               const PointMatcherOptions& options) const;

 private:
  Solver transformation_estimation_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_LLS_H_
