#ifndef ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_UTIL_H_
#define ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_UTIL_H_

#include <utility>
#include <vector>

#include "boost/make_shared.hpp"
#include "onboard/math/vec.h"
#include "onboard/perception/cluster.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_cloud.h"

namespace qcraft {

struct GridPoint {
  int upright_index;
  int level_index;
};

template <typename PointType>
void VoxelDownsample(const typename pcl::PointCloud<PointType>::Ptr cloud,
                     float leaf_x, float leaf_y, float leaf_z);

void GetNeighborsIndex(int upright_dim, int level_dim, GridPoint index,
                       std::vector<GridPoint>* neighbor_index);

void CylinderProjuectNeighborSearch(
    const Vec3d& qcraft_pose,
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
    std::vector<std::vector<int>>* neighobrs);

void ComputeNormals(pcl::PointCloud<pcl::PointNormal>::Ptr ref_cloud_normals,
                    const std::vector<std::vector<int>>& neighbor_indexs);

std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, Vec3d>
ComputeNormalsOfCluster(const Vec3d& anchor_pose,
                        const SegmentedCluster& cluster);

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr CollectMainLidarPointsFromCluster(
    const Cluster& cluster);

class CylindricalProjectionNearestNeighborSearcher {
  // Cylindrical projection projects point cloud to range map.
  // The upr_idx is the index in vertical direction and
  // the level_index is the index in the angular direction
  using Voxel = std::vector<int>;
  using VoxelIndex = std::pair<int, int>;
  using NeighborVoxelIndex = std::vector<VoxelIndex>;
  // We store the voxel index of each point in the point clouds
  // in the process of contrusting grid
  using CloudsLocationVoxelIndex = std::vector<VoxelIndex>;
  using GridStorageType = std::vector<std::vector<Voxel>>;
  using CloudsNeighborIndex = std::vector<std::vector<int>>;
  using PointCloud = std::vector<Vec3d>;

 public:
  explicit CylindricalProjectionNearestNeighborSearcher(
      const int upr_dim, const int circle_lev_dim, const double dep_rsl)
      : upr_dim_(upr_dim),
        lec_rsl_(360.0 / circle_lev_dim),
        dep_rsl_(dep_rsl) {}
  // Contrust the Voxel based on the point clouds
  void ContrustVoxels();
  // Set the anchor pose for constructing voxels
  void SetAnchorPoint(const Vec3d qcraft_pose) { anchar_point_ = qcraft_pose; }
  // Set the roof height for the cluster
  void SetRoofHeight(double RoofHeight) { RoofHeight_ = RoofHeight; }
  // Set the raidus for the search
  void SetRadiusSearch(double radius) {
    radius_ = radius;
    radius2_ = radius_ * radius_;
  }
  // Input the point clouds
  void SetInputCluster(SegmentedCluster* cluster) { cluster_ = cluster; }
  // Perform the radius search
  void RadiusSearch(CloudsNeighborIndex* neighbor_indexs);

 private:
  void NeighborVoxelSearch(VoxelIndex index,
                           NeighborVoxelIndex* neighobor_voxel_index) const;
  // const int UPR_DIM_;
  // // NOTE(ZHANKUN)::In practice, In the implementation of Grid,
  // // the LEVEL_DIM_ is not a fixed value,
  // int LEV_DIM_;
  // double UPR_RSL_;
  // const double LEC_RSL_;
  // [[maybe_unused]] const double DEP_RSL_;  // hasn't been used Now
  const int upr_dim_;
  // NOTE(ZHANKUN)::In practice, In the implementation of Grid,
  // the LEVEL_DIM_ is not a fixed value,
  int lev_dim_;
  double upr_rsl_;
  const double lec_rsl_;
  [[maybe_unused]] const double dep_rsl_;  // hasn't been used Now
  GridStorageType grids_;
  CloudsLocationVoxelIndex clouds_voxel_index_;
  double radius_;
  double radius2_;
  PointCloud points_;
  SegmentedCluster* cluster_;
  Vec3d anchar_point_;
  double RoofHeight_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_UTIL_H_
