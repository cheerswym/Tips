#include "onboard/perception/registration/icp_with_normal_util.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "boost/smart_ptr/make_shared_array.hpp"
#include "offboard/mapping/mapping_core/util/point_cloud_util.h"
#include "onboard/global/trace.h"
#include "onboard/perception/cluster_util.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/voxel_grid.h"

namespace qcraft {

template <typename PointType>
void VoxelDownsample(const typename pcl::PointCloud<PointType>::Ptr cloud,
                     float leaf_x, float leaf_y, float leaf_z) {
  typename pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_x, leaf_y, leaf_z);
  sor.filter(*cloud);
}

// the function is used to find the neighbor grids of a grid
void GetNeighborsIndex(int upright_dim, int level_dim, GridPoint index,
                       std::vector<GridPoint>* neighbors_index) {
  neighbors_index->clear();
  // upright_index is the index in the vertical direction
  const int upright_index = index.upright_index;
  // level_index is the index in the horizontal direction
  int level_index = index.level_index;
  if (upright_index == 0) {
    neighbors_index->insert(
        neighbors_index->end(),
        {{upright_index, level_index},
         {upright_index,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1},
         {upright_index,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index + 1, level_index},
         {upright_index + 1,
          level_index = level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index + 1, level_index = level_index - 1 >= 0
                                               ? level_index - 1
                                               : level_dim - 1}});
  } else if (upright_index == upright_dim - 1) {
    neighbors_index->insert(
        neighbors_index->end(),
        {{upright_index, level_index},
         {upright_index,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1},
         {upright_index,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index - 1, level_index},
         {upright_index - 1,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index - 1,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1}});
  } else {
    neighbors_index->insert(
        neighbors_index->end(),
        {{upright_index, level_index},
         {upright_index,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1},
         {upright_index,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index - 1, level_index},
         {upright_index - 1,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index - 1,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1},
         {upright_index + 1, level_index},
         {upright_index + 1,
          level_index + 1 <= level_dim - 1 ? level_index + 1 : 0},
         {upright_index + 1,
          level_index - 1 >= 0 ? level_index - 1 : level_dim - 1}});
  }
}

// the function is used to find the neighbors of every point
void CylinderProjuectNeighborSearch(
    const Vec3d& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
    std::vector<std::vector<int>>* neighobrs) {
  const int size = cloud->points.size();
  float min_z = cloud->points[0].z;
  float max_z = cloud->points[0].z;
  for (auto const& point : cloud->points) {
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  constexpr int kUprightDim = 5;
  // The max_z is increased by 0.0001 for avoiding the upright_index
  // of the point with max_z exceed the kUprightDim - 1
  const double upright_resolution_inv = kUprightDim / (max_z + 0.0001 - min_z);
  constexpr int kLevelDim = 360 * 2;
  constexpr double kLevelResolutionInv = kLevelDim / 360.0;
  std::vector<int> grid[kUprightDim][kLevelDim];
  std::vector<GridPoint> uprights_azimuths_indexs;
  uprights_azimuths_indexs.reserve(size);
  for (int i = 0; i < size; ++i) {
    const int upright_index =
        FloorToInt((cloud->points[i].z - min_z) * upright_resolution_inv);
    const double x = cloud->points[i].x - pose[0];
    const double y = cloud->points[i].y - pose[1];
    double azi = r2d(fast_math::Atan2(y, x));
    if (azi < 0.0) {
      azi += 360.0;
    }
    const int level_index = FloorToInt(azi * kLevelResolutionInv);
    uprights_azimuths_indexs.push_back({upright_index, level_index});
    grid[upright_index][level_index].push_back(i);
  }
  // Reuse this variable to avoid repeated memory allocation/free.
  std::vector<GridPoint> neighbors_index;
  for (int i = 0; i < size; ++i) {
    GetNeighborsIndex(kUprightDim, kLevelDim, uprights_azimuths_indexs[i],
                      &neighbors_index);
    (*neighobrs)[i].reserve(50);
    for (const auto& grid_point : neighbors_index) {
      const Vec3d point(cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z);
      const int upright_index = grid_point.upright_index;
      int level_index = grid_point.level_index;
      for (const auto j : grid[upright_index][level_index]) {
        const double dist2 =
            (point -
             Vec3d(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z))
                .squaredNorm();
        constexpr double kMaxSearchRadiusSquare = Sqr(0.5);
        if (dist2 < kMaxSearchRadiusSquare) {
          (*neighobrs)[i].push_back(j);
        }
      }
    }
  }
}

// NOTE: Set normal to zero for points that we cannot compute their normals.
void ComputeNormals(pcl::PointCloud<pcl::PointNormal>::Ptr ref_cloud_normals,
                    const std::vector<std::vector<int>>& neighbor_indexs) {
  auto& points = ref_cloud_normals->points;
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation;
  const int num_points = ref_cloud_normals->size();
  for (int i = 0; i < num_points; ++i) {
    constexpr int kMinNeighborSize = 6;
    auto& point = points[i];
    if (neighbor_indexs[i].empty() ||
        neighbor_indexs[i].size() < kMinNeighborSize ||
        !normal_estimation.computePointNormal(
            (*ref_cloud_normals), neighbor_indexs[i], point.normal[0],
            point.normal[1], point.normal[2], point.curvature)) {
      point.normal_x = point.normal_y = point.normal_z = 0;
    }
  }
}

// This function is used to compute the normal cloud and bias
// of the cluster
std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, Vec3d>
ComputeNormalsOfCluster(const Vec3d& anchor_pose,
                        const SegmentedCluster& cluster) {
  SCOPED_QTRACE("ComputeNormalsOfCluster");

  if (cluster.HasNormal()) {
    QLOG(WARNING) << "You have already calcualted normal.";
    return cluster.GetBiasCloud();
  }

  const auto cloud_normals =
      CollectMainLidarPointsFromCluster<pcl::PointNormal>(cluster);
  const int orig_num_points = cloud_normals->size();

  // TODO(jingwei): consider to reduce the following size for performance.
  constexpr int kTooDensePointNum = 1000;
  auto& points = cloud_normals->points;
  if (points.size() > kTooDensePointNum) {
    VoxelDownsample<pcl::PointNormal>(cloud_normals, 0.2f, 0.2f, 0.2f);
  }

  SCOPED_QTRACE_ARG2("ComputeNormalsOfCluster_1", "num_points", orig_num_points,
                     "num_filtered_points", cloud_normals->size());

  const int num_points = points.size();
  QCHECK_GT(num_points, 0);

  const Vec3d bias(points[0].x, points[0].y, points[0].z);
  const Vec3f bias_f = bias.cast<float>();
  for (auto& point : points) {
    point.x -= bias_f.x();
    point.y -= bias_f.y();
    point.z -= bias_f.z();
  }

  const Vec3d transform_pose = anchor_pose - bias;
  std::vector<std::vector<int>> neighbor_indexs(num_points);
  CylinderProjuectNeighborSearch(transform_pose, cloud_normals,
                                 &neighbor_indexs);

  // TOOD(jingwei) rewrite
  ComputeNormals(cloud_normals, neighbor_indexs);

  // Delete the point with the normal of nan
  points.erase(std::remove_if(points.begin(), points.end(),
                              [](const auto& p) {
                                return p.normal_x == 0 && p.normal_y == 0 &&
                                       p.normal_z == 0;
                              }),
               points.end());

  return {cloud_normals, bias};
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr CollectMainLidarPointsFromCluster(
    const Cluster& cluster) {
  typename pcl::PointCloud<PointT>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<PointT>>();
  cloud->reserve(cluster.NumPoints());
  for (const auto& obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      switch (point.lidar_id) {
        case LDR_CENTER:
        case LDR_FRONT_LEFT:
        case LDR_FRONT_RIGHT:
        case LDR_FRONT:
        case LDR_REAR:
        case LDR_REAR_LEFT:
        case LDR_REAR_RIGHT: {
          auto& p = cloud->points.emplace_back();
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          break;
        }
        default:
          continue;
      }
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  return cloud;
}
template pcl::PointCloud<pcl::PointNormal>::Ptr
CollectMainLidarPointsFromCluster<pcl::PointNormal>(const Cluster& cluster);

void CylindricalProjectionNearestNeighborSearcher::RadiusSearch(
    CloudsNeighborIndex* neighbor_indexs) {
  int size = points_.size();
  neighbor_indexs->resize(size);
  for (int i = 0; i < size; ++i) {
    NeighborVoxelIndex neighbor_voxels;
    NeighborVoxelSearch(clouds_voxel_index_[i], &neighbor_voxels);
    (*neighbor_indexs)[i].reserve(50);
    for (const auto& voxel : neighbor_voxels) {
      const Vec3d& point = points_[i];
      int upright_index = voxel.first;
      int level_index = voxel.second;
      for (const auto j : grids_[upright_index][level_index]) {
        const double dist2 = (point - points_[j]).squaredNorm();
        if (dist2 < radius_) {
          (*neighbor_indexs)[i].push_back(j);
        }
      }
    }
  }
}

void CylindricalProjectionNearestNeighborSearcher::ContrustVoxels() {
  int size = points_.size();
  double min_z = points_[0][2];
  double max_z = points_[0][2];
  // std::vector<double> dpts;
  // depths.reserve(size);
  // a and b are two vector. If the cross product of a and b, represented by a x
  // b, exceeds 0, a is anti-clockwise to b, otherwise a is clockwise to b. We
  // can judge two endpoints in the angle direction in the process of
  // cylindrical projection based on this.
  Vec3d clockwise_point = points_[0] - anchar_point_;
  Vec3d anti_clockwise_point = points_[0] - anchar_point_;
  for (int i = 0; i < size; ++i) {
    min_z = std::min(min_z, points_[i][2]);
    max_z = std::max(max_z, points_[i][2]);
    // depths.push_back((points[i]-pose).norm());
    Vec2d v1 = (points_[i] - anchar_point_).block<2, 1>(0, 0);
    Vec2d v2 = (anti_clockwise_point - anchar_point_).block<2, 1>(0, 0);
    if (v1.CrossProd(v2) > 0) {
      anti_clockwise_point[0] = points_[i][0];
      anti_clockwise_point[1] = points_[i][1];
      anti_clockwise_point[2] = points_[i][2];
    }
    Vec2d v3 = (clockwise_point - anchar_point_).block<2, 1>(0, 0);
    if (v1.CrossProd(v3) < 0) {
      clockwise_point[0] = points_[i][0];
      clockwise_point[1] = points_[i][1];
      clockwise_point[2] = points_[i][2];
    }
  }
  // The upright resolution is not fixed for different objects and determined
  // there
  // The max_z is increased by 0.0001 for avoiding the upright_index
  // of the point with max_z exceed the kUprightDim - 1
  upr_rsl_ = (max_z + 0.0001 - min_z) / upr_dim_;
  // Compute the instersection angle of the
  double max_intersec_ang = acos(Vec3d(anti_clockwise_point - anchar_point_)
                                     .Dot(clockwise_point - anchar_point_) /
                                 (anti_clockwise_point - anchar_point_).norm() /
                                 (clockwise_point - anchar_point_).norm());
  lev_dim_ = (max_intersec_ang) / lec_rsl_ + 1;
  // Pre allocated memory
  grids_.reserve(upr_dim_);
  for (int i = 0; i < upr_dim_; ++i) {
    grids_[i].reserve(lev_dim_);
  }
  clouds_voxel_index_.reserve(size);
  for (int i = 0; i < size; ++i) {
    double intersec_ang = acos(
        Vec3d(points_[i] - anchar_point_).Dot(clockwise_point - anchar_point_) /
        (points_[i] - anchar_point_).norm() /
        (clockwise_point - anchar_point_).norm());
    int upr_idx = (points_[i].z() - min_z) / upr_rsl_;
    int lel_idx = intersec_ang / lec_rsl_;
    grids_[upr_idx][lel_idx].emplace_back(i);
    clouds_voxel_index_.emplace_back(upr_idx, lel_idx);
  }
}

void CylindricalProjectionNearestNeighborSearcher::NeighborVoxelSearch(
    VoxelIndex index, NeighborVoxelIndex* neighobor_voxel_index) const {
  // upright_index is the index in the vertical direction
  int upr_idx = index.first;
  // level_index is the index in the horizontal direction
  int lel_idx = index.second;
  // A Voxel can has up to nine neighborhood Voxels
  neighobor_voxel_index->reserve(9);
  neighobor_voxel_index->emplace_back(upr_idx, lel_idx);
  if (lel_idx - 1 >= 0)
    neighobor_voxel_index->emplace_back(upr_idx, lel_idx - 1);
  if (upr_idx - 1 >= 0)
    neighobor_voxel_index->emplace_back(upr_idx - 1, lel_idx);
  if (lel_idx + 1 <= lev_dim_ - 1)
    neighobor_voxel_index->emplace_back(upr_idx, lel_idx + 1);
  if (upr_idx + 1 <= upr_dim_ - 1)
    neighobor_voxel_index->emplace_back(upr_idx + 1, lel_idx);
  if (upr_idx - 1 >= 0 && lel_idx - 1 >= 0)
    neighobor_voxel_index->emplace_back(upr_idx - 1, lel_idx - 1);
  if (upr_idx - 1 >= 0 && lel_idx + 1 <= lev_dim_ - 1)
    neighobor_voxel_index->emplace_back(upr_idx - 1, lel_idx + 1);
  if (upr_idx + 1 <= upr_dim_ - 1 && lel_idx - 1 >= 0)
    neighobor_voxel_index->emplace_back(upr_idx + 1, lel_idx - 1);
  if (upr_idx + 1 <= upr_dim_ - 1 && lel_idx + 1 <= lev_dim_ - 1)
    neighobor_voxel_index->emplace_back(upr_idx + 1, lel_idx + 1);
  return;
}

}  // namespace qcraft
