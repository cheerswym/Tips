#include "onboard/perception/cluster_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "absl/synchronization/mutex.h"
#include "fastforest/fastforest.h"
#include "gflags/gflags.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/intensity_util.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/vis/common/colormap.h"
#include "pcl/kdtree/kdtree_flann.h"

DEFINE_bool(enable_cluster_reflection_filter, false,
            "Enable mirror reflection filter");
DEFINE_bool(
    enable_cluster_mist_filter, true,
    "Enable rain filter for segment clusters, only opens in rain weather");
DEFINE_bool(cluster_filtering_cvs, false,
            "Whether to render filtering results.");
DEFINE_bool(enable_blooming_filter, true,
            "Whether to enable the blooming filter.");
DEFINE_bool(blooming_filter_cvs, false,
            "Whether to render blooming filter results.");
DEFINE_bool(enable_small_cluster_filter, true,
            "Whether to enable the small cluster filter.");
DEFINE_bool(collect_cluster_data, false,
            "Collect all cluster information for training filters.");
DEFINE_string(cluster_data_file_name, "", "File name to save cluster data.");

namespace qcraft {
namespace {

// Mirror reflection filter second return ratio setting
constexpr int kMirrorReflectionPointNum = 110;
constexpr double kMirrorReflection2ndReturnRatioMin = 0.425;
constexpr double kMirrorReflection2ndReturnRatio = 0.782;
constexpr double kMirrorReflection2ndReturnRatioMax = 0.9;
constexpr double kMirrorReflectionClusterScore = 0.25;
constexpr double kMirrorReflectionClusterScoreMax = 0.425;
constexpr double kMirrorReflectionAvgClearance = 0.567;
constexpr double kMirrorReflection2ndReturnRatioForOverhangingTrafficSign = 0.3;
constexpr double kMirrorReflectionObstacleZRange = 0.1;
constexpr double kMirrorReflectionAvgClearanceForOverhangingTrafficSign = 1.5;

constexpr int kNonFilterableClusterMinNumOusterPoints = 10;

// Small cluster filter setting.
constexpr int kClusterMinNumPointsAboveGround = 4;

// Blooming cluster filter setting
constexpr double kBloomingMinDistToCurb = 0.6;                        // m
constexpr int kMaxBloomingObstacleRange = 1.6 / Obstacle::kDiameter;  // 1.6m
constexpr int kRetroreflectorMinIntensity = 250;
constexpr float kMinRetroreflectedPointsRatio = 0.2f;
constexpr float kMinMaybeRetroreflectedPointsRatio = 0.1f;
constexpr int kMinMaybeRetroreflectedPointsCount = 3;
constexpr float kMaxBloomingRangeRadiusDiff = 0.4;  // m

// Mist filter setting
constexpr int kMinMaybeMistPointsCount = 10;
constexpr int kMaxMistObstacleRange = 0.4 / Obstacle::kDiameter;  // 0.4m
constexpr int kMaxMistNetRange = 60;                              // m
constexpr int kMinUnfilterableClusterIntensity = 56;

// Bins number of cluster histogram feature.
constexpr int kHistogramBinsNumPoints = 5;
constexpr int kHistogramBinsIntensity = 5;

Vec2d ComputeClusterCentroid(const SegmentedCluster& cluster) {
  return std::accumulate(
             cluster.obstacles().begin(), cluster.obstacles().end(), Vec2d(),
             [](Vec2d init, const auto* obstacle) {
               return Vec2d(init.x() + obstacle->x, init.y() + obstacle->y);
             }) /
         cluster.NumObstacles();
}

// The type of Cluster histogram feature.
enum HistogramType { kNumPointsRatio, kHasReturnBehindRatio, kIntensityMean };
// NOTE(wushuai): Compute the histogram feature of a Segmented Cluster on
// z-axis, that is, count the different type of attributes of point cloud in
// each bin.
template <HistogramType kHistType>
std::vector<float> ComputeClusterHistogramFeature(
    const int bins_num, const std::vector<LaserPoint>& points) {
  std::vector<int> histogram_num_points(bins_num);
  std::vector<int> histogram_feature_sum(bins_num);

  const int num_points = points.size();
  const float dim_bin_width =
      (points.back().coord().z() - points.front().coord().z()) / bins_num;
  const float dim_bin_width_inv =
      dim_bin_width == 0.0f ? 0.0f : 1.0f / dim_bin_width;
  const float min_z = points.front().coord().z();
  for (const auto& point : points) {
    const float delta = point.coord().z() - min_z;
    const int bin_index =
        std::min(bins_num - 1, FloorToInt(delta * dim_bin_width_inv));
    DCHECK_GE(bin_index, 0);

    ++histogram_num_points[bin_index];
    switch (kHistType) {
      case kNumPointsRatio:
        break;
      case kHasReturnBehindRatio:
        histogram_feature_sum[bin_index] += point.has_return_behind;
        break;
      case kIntensityMean:
        histogram_feature_sum[bin_index] += point.intensity;
        break;
    }
  }
  std::vector<float> histogram;
  histogram.reserve(bins_num);
  for (int bin = 0; bin < bins_num; bin++) {
    const int num_points_bin = histogram_num_points[bin];
    float feature_val = 0.0f;
    if (num_points_bin > 0) {
      switch (kHistType) {
        case kNumPointsRatio:
          feature_val = static_cast<float>(num_points_bin) / num_points;
          break;
        case kHasReturnBehindRatio:
          feature_val =
              histogram_feature_sum[bin] / static_cast<float>(num_points_bin);
          break;
        case kIntensityMean:
          feature_val =
              histogram_feature_sum[bin] / static_cast<float>(num_points_bin);
          break;
      }
    }
    histogram.emplace_back(feature_val);
  }
  return histogram;
}

Mat3f ComputeCovariance(const std::vector<Vec3f>& points) {
  DCHECK(!points.empty());
  const Vec3f mean =
      std::accumulate(points.begin(), points.end(), Vec3f()) / points.size();
  Mat3f aa(3, 3);
  aa.setZero();
  for (const auto& point : points) {
    const auto v = point - mean;
    aa += v * v.transpose();
  }
  aa /= points.size();
  return aa;
}

PlaneAndLineFeature ComputePlanePointRatioAndLinePointRatio(
    const std::vector<LaserPoint>& laser_points) {
  SCOPED_QTRACE_ARG1("ComputePlanePointRatioAndLinePointRatio", "num_points",
                     laser_points.size());

  constexpr float kMaxPlaneThreshold = 0.1;
  constexpr float kMaxLineThreshold = 0.1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_ptr->width = laser_points.size();
  cloud_ptr->height = 1;
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);

  for (int i = 0; i < laser_points.size(); i++) {
    cloud_ptr->points[i].x = laser_points[i].x;
    cloud_ptr->points[i].y = laser_points[i].y;
    cloud_ptr->points[i].z = laser_points[i].z;
  }

  constexpr int kMinNumNeighbour = 6;
  if (laser_points.size() < kMinNumNeighbour) {
    return {-1, -1};
  }
  std::pair<int, int> nums_count = {0, 0};
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  kd_tree.setInputCloud(cloud_ptr);

  SCOPED_QTRACE("ComputePlanePointRatioAndLinePointRatio_1");

  std::vector<int> point_indices(kMinNumNeighbour);
  std::vector<float> point_sqr_dis(kMinNumNeighbour);
  std::vector<Vec3f> neighbors;
  for (const auto& point : cloud_ptr->points) {
    if (kd_tree.nearestKSearch(point, kMinNumNeighbour, point_indices,
                               point_sqr_dis) > 0) {
      neighbors.clear();
      for (const int index : point_indices) {
        const auto& p = cloud_ptr->points[index];
        neighbors.emplace_back(p.x, p.y, p.z);
      }
      const Mat3f mat = ComputeCovariance(neighbors);
      const Eigen::SelfAdjointEigenSolver<Mat3f> solver(mat);
      const Vec3f eigen_values = solver.eigenvalues();
      std::array<float, 3> abs_eigenvalues = {std::abs(eigen_values.x()),
                                              std::abs(eigen_values.y()),
                                              std::abs(eigen_values.z())};
      std::sort(abs_eigenvalues.begin(), abs_eigenvalues.end());
      if (abs_eigenvalues[2] < std::numeric_limits<float>::epsilon()) {
        continue;
      }

      if (std::max(abs_eigenvalues[0], abs_eigenvalues[1]) <
          abs_eigenvalues[2] * kMaxLineThreshold) {
        nums_count.second++;
      } else {
        const float sum_of_abs_eigenvalues =
            abs_eigenvalues[0] + abs_eigenvalues[1] + abs_eigenvalues[2];
        if (abs_eigenvalues[0] < sum_of_abs_eigenvalues * kMaxPlaneThreshold) {
          nums_count.first++;
        }
      }
    }
  }

  PlaneAndLineFeature plane_and_line_feature = {0.0, 0.0};
  plane_and_line_feature.first = static_cast<float>(nums_count.first) /
                                 static_cast<float>(laser_points.size());
  plane_and_line_feature.second = static_cast<float>(nums_count.second) /
                                  static_cast<float>(laser_points.size());
  return plane_and_line_feature;
}

ClusterFeature ExtractClusterFeature(
    const VehiclePose& pose, const SegmentedCluster& cluster,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  const int num_points = cluster.NumPoints();
  DCHECK_GT(num_points, 0);

  SCOPED_QTRACE_ARG1("ExtractClusterFeature", "num_points", num_points);

  ClusterFeature cluster_feature;
  cluster_feature.new_feature_flag = true;

  {  // get num points
    cluster_feature.num_points = num_points;
  }

  {  // get range
    const auto& centroid = ComputeClusterCentroid(cluster);
    cluster_feature.range = (centroid - pose.coord2d()).norm();
  }

  {  // intensity mean
    int intensity_sum = 0;
    for (const auto* obstacle : cluster.obstacles()) {
      for (const auto& p : obstacle->points) {
        intensity_sum += p.intensity;
      }
    }
    cluster_feature.intensity_mean = intensity_sum / num_points;
  }

  {  // intensity variance
    float accumulate = 0.0f;
    for (const auto* obstacle : cluster.obstacles()) {
      for (const auto& p : obstacle->points) {
        accumulate += Sqr(p.intensity - cluster_feature.intensity_mean);
      }
    }
    cluster_feature.intensity_dev = std::sqrt(accumulate / num_points);
  }

  {  // clearance mean
    float clearance_sum = 0.0f;
    for (const auto* obstacle : cluster.obstacles()) {
      clearance_sum += obstacle->clearance;
    }
    cluster_feature.clearance_mean = clearance_sum / cluster.NumObstacles();
  }

  {  // clearance variance
    float accumulate = 0.0f;
    for (const auto* obstacle : cluster.obstacles()) {
      accumulate += Sqr(obstacle->clearance - cluster_feature.clearance_mean);
    }
    cluster_feature.clearance_dev =
        std::sqrt(accumulate / cluster.NumObstacles());
  }

  {  // adjacent scan gap mean, adjacent scan gap variance
    SCOPED_QTRACE("adjacent_scan_gap_mean");

    std::array<Vec3d, LidarId_ARRAYSIZE> lidar_poses;
    for (const auto& [lidar_id, lidar_param] : lidar_params) {
      const auto& extrinsics = lidar_param.installation().extrinsics();
      lidar_poses[lidar_id] = {extrinsics.x(), extrinsics.y(), extrinsics.z()};
    }
    const auto& t_inv = pose.ToTransform().Inverse();
    // NOTE(dong): Tuple includes (0: beam index) (1: scan index) (2: point
    // range)
    std::array<std::vector<std::tuple<int, int, float>>, LidarId_ARRAYSIZE>
        beams_of_each_lidar;
    for (const auto* obstacle : cluster.obstacles()) {
      for (const auto& p : obstacle->points) {
        const auto& lidar_pos = lidar_poses[p.lidar_id];
        const float range =
            (t_inv.TransformPoint(p.coord()) - lidar_pos).norm();
        beams_of_each_lidar[p.lidar_id].emplace_back(
            p.beam_index, p.scan_or_point_index, range);
      }
    }
    for (auto& beams : beams_of_each_lidar) {
      // NOTE(dong): Sort beams by beam index in ascending order, and sort
      // scan index in ascending order if beam index is the same.
      std::sort(beams.begin(), beams.end(),
                [](const auto& lhs, const auto& rhs) {
                  if (std::get<0>(lhs) < std::get<0>(rhs)) {
                    return true;
                  } else if (std::get<0>(lhs) > std::get<0>(rhs)) {
                    return false;
                  } else if (std::get<1>(lhs) < std::get<1>(rhs)) {
                    return true;
                  } else {
                    return false;
                  }
                });
    }
    float gap_sum = 0.0f;
    int num_gaps = 0;
    for (int i = 0; i < beams_of_each_lidar.size(); ++i) {
      const auto& beams = beams_of_each_lidar[i];
      if (beams.size() <= 1) continue;
      for (int j = 1; j < beams.size(); ++j) {
        // NOTE(dong): Need to mention that small scan index (such as 0) and
        // large scan index (such as 1800) could also be adjacent scans which
        // is not considered here.
        if (std::get<0>(beams[j]) == std::get<0>(beams[j - 1])) {
          const float cur_range = std::get<2>(beams[j]);
          const float pre_range = std::get<2>(beams[j - 1]);
          gap_sum += std::abs(cur_range - pre_range);
          ++num_gaps;
        }
      }
    }
    if (num_gaps) {
      cluster_feature.adjacent_scan_gap_mean = gap_sum / num_gaps;

      float accumulate = 0.0f;
      for (int i = 0; i < beams_of_each_lidar.size(); ++i) {
        const auto& beams = beams_of_each_lidar[i];
        if (beams.size() <= 1) continue;
        for (int j = 1; j < beams.size(); ++j) {
          if (std::get<0>(beams[j]) == std::get<0>(beams[j - 1])) {
            const float cur_range = std::get<2>(beams[j]);
            const float pre_range = std::get<2>(beams[j - 1]);
            accumulate += Sqr(std::abs(cur_range - pre_range) -
                              cluster_feature.adjacent_scan_gap_mean);
          }
        }
      }
      cluster_feature.adjacent_scan_gap_dev = std::sqrt(accumulate / num_gaps);
    }
  }

  std::vector<LaserPoint> points;
  std::vector<std::pair<float, int>> point_z_and_indices;
  points.reserve(num_points);
  point_z_and_indices.reserve(num_points);
  for (const auto* obstacle : cluster.obstacles()) {
    const int prev_num_points = points.size();
    for (const auto& point : obstacle->points) {
      point_z_and_indices.emplace_back(point.z, points.size());
      points.emplace_back(point);
      auto& new_point = points.back();
      if (new_point.lidar_type == LIDAR_RS_BPEARL) {
        new_point.intensity = intensity_util::RecalibrateBpearlIntensity(
            new_point.intensity, new_point.range, new_point.z,
            obstacle->ground_z);
      } else if (new_point.lidar_type == LIDAR_PANDAR_QT128) {
        new_point.intensity = intensity_util::RecalibrateQT128Intensity(
            new_point.intensity, new_point.range, new_point.z,
            obstacle->ground_z);
      }
    }
    // Merge sort points on z-axis. Points in each obstacle are already sorted
    // on z.
    std::inplace_merge(point_z_and_indices.begin(),
                       point_z_and_indices.begin() + prev_num_points,
                       point_z_and_indices.end());
  }

  std::vector<LaserPoint> sorted_points;
  sorted_points.reserve(num_points);
  for (const auto& [_, i] : point_z_and_indices) {
    sorted_points.push_back(points[i]);
  }
  points = std::move(sorted_points);

  {  // Histogram feature
    cluster_feature.hist_num_points =
        ComputeClusterHistogramFeature<kNumPointsRatio>(kHistogramBinsNumPoints,
                                                        points);
    cluster_feature.hist_intensity =
        ComputeClusterHistogramFeature<kIntensityMean>(kHistogramBinsIntensity,
                                                       points);
  }

  {  // Plane and line point ratio.
    cluster_feature.plane_and_line_feature =
        ComputePlanePointRatioAndLinePointRatio(points);
  }

  return cluster_feature;
}

void AddClusterFilterDebugProto(ClusterFilterDebugProto* debug_proto,
                                std::string debug_string,
                                const Cluster& cluster,
                                ClusterFilterDebugProto::FilterType filter_type,
                                Vec2d media_centroid = {0, 0}) {
  auto* cluster_info_proto = debug_proto->add_cluster_info();
  cluster_info_proto->set_debug_info(std::move(debug_string));
  cluster_info_proto->set_filter_type(filter_type);
  const auto cluster_contour = cluster_util::ComputeContourWithZ(cluster, 0.1);
  for (const auto& point : cluster_contour) {
    auto* contour_proto = cluster_info_proto->add_contour();
    contour_proto->set_x(point.x());
    contour_proto->set_y(point.y());
  }
  Vec2dProto centroid;
  centroid.set_x(media_centroid.x());
  centroid.set_y(media_centroid.y());
  *cluster_info_proto->mutable_media_centroid() = centroid;
}

// Returns if cluster is road agent or cone. We don't filter these clusters in
// blooming filter.
bool IsClusterBloomingFilterable(const Cluster& cluster) {
  return !(cluster.type() == MT_VEHICLE || cluster.type() == MT_PEDESTRIAN ||
           cluster.type() == MT_CYCLIST || cluster.type() == MT_MOTORCYCLIST ||
           cluster.type() == MT_CONE);
}

bool IsClusterFilterable(const Cluster& cluster) {
  const auto& obstacles = cluster.obstacles();
  // Cluster is not filterable in cluster level filtering as long as cluster
  // is classified by semantic map zones.
  if (cluster.type_source() == MTS_SEMANTIC_MAP_ZONE) {
    return false;
  }
  // Do not filter ped and cyclist.
  if (cluster.type() == MT_PEDESTRIAN || cluster.type() == MT_CYCLIST ||
      cluster.type() == MT_MOTORCYCLIST) {
    return false;
  }
  for (const auto& obstacle : obstacles) {
    if (obstacle->type == ObstacleProto::CONE) {
      return false;
    }
  }

  return true;
}

bool IsOversizedCluster(const Cluster& cluster) {
  constexpr int kMinOversizedClusterNumPoints = 5000;
  constexpr double kMinOversizedClusterArea = 100.0;  // m*m
  const auto area = cluster_util::ComputeContour(cluster).area();

  return (cluster.NumPoints() > kMinOversizedClusterNumPoints ||
          area > kMinOversizedClusterArea);
}

std::pair<int, int> CountsOusterNumPointsAboveGroundInObstacle(
    const Obstacle& obstacle) {
  int ouster_num_points_above_ground = 0;
  int total_num_points_above_ground = 0;
  for (const auto& point : obstacle.points) {
    if (!obstacle_util::IsAboveGroundObstaclePoint(obstacle, point)) {
      continue;
    }
    ++total_num_points_above_ground;
    if (IsOusterPoint(point.lidar_type)) {
      ++ouster_num_points_above_ground;
    }
  }
  return {ouster_num_points_above_ground, total_num_points_above_ground};
}

std::pair<int, int> CountsOusterPointsAboveGroundInCluster(
    const SegmentedCluster& segmented_cluster) {
  int num_points_from_ouster = 0;
  int total_num_points = 0;
  for (const auto& obstacle : segmented_cluster.obstacles()) {
    const auto [ouster_num, total_num] =
        CountsOusterNumPointsAboveGroundInObstacle(*obstacle);
    num_points_from_ouster += ouster_num;
    total_num_points += total_num;
  }
  return {num_points_from_ouster, total_num_points};
}

// Return if the given cluster has some non-ground laser points from Ouster
// lidar. In most cases mist/rain won't form points for Ouster lidars.
bool ContainsOusterPoints(const SegmentedCluster& segmented_cluster) {
  // Don't filter cluster with sufficient points from Ouster lidar.
  return CountsOusterPointsAboveGroundInCluster(segmented_cluster).first >=
         kNonFilterableClusterMinNumOusterPoints;
}

void MaybeRenderFilteredClusters(
    const SegmentedClusters& ignored_reflection_clusters,
    const SegmentedClusters& ignored_mist_clusters,
    const SegmentedClusters& ignored_small_clusters) {
  if (!FLAGS_cluster_filtering_cvs) return;
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/filtered_clusters");
  for (const auto& cluster : ignored_reflection_clusters) {
    QCHECK_GT(cluster.NumObstacles(), 0);
    const auto cluster_contour =
        cluster_util::ComputeContourWithZ(cluster, 1.0);
    canvas.DrawPolygon(cluster_contour, vis::Color::kLightYellow, 2,
                       vis::BorderStyleProto::DASHED);
  }
  for (const auto& cluster : ignored_mist_clusters) {
    QCHECK_GT(cluster.NumObstacles(), 0);
    const auto cluster_contour =
        cluster_util::ComputeContourWithZ(cluster, 1.0);
    canvas.DrawPolygon(cluster_contour, vis::Color::kLightMagenta, 2,
                       vis::BorderStyleProto::DASHED);
  }
  for (const auto& cluster : ignored_small_clusters) {
    QCHECK_GT(cluster.NumObstacles(), 0);
    const auto cluster_contour =
        cluster_util::ComputeContourWithZ(cluster, 1.0);
    canvas.DrawPolygon(cluster_contour, vis::Color::kLightGreen, 2,
                       vis::BorderStyleProto::DASHED);
  }
}

LaserPointProto ToLaserPointProto(LaserPoint pt) {
  LaserPointProto laser_pt_proto;
  laser_pt_proto.set_x(pt.x);
  laser_pt_proto.set_y(pt.y);
  laser_pt_proto.set_z(pt.z);
  laser_pt_proto.set_range(pt.range);
  laser_pt_proto.set_intensity(pt.intensity);
  laser_pt_proto.set_return_index(static_cast<int>(pt.return_index));
  laser_pt_proto.set_has_returns_behind(pt.has_return_behind);
  laser_pt_proto.set_beam_index(pt.beam_index);
  laser_pt_proto.set_scan_or_point_index(pt.scan_or_point_index);
  laser_pt_proto.set_lidar_id(pt.lidar_id);
  laser_pt_proto.set_lidar_type(
      static_cast<LidarType>(static_cast<LidarModel>(pt.lidar_type)));

  return laser_pt_proto;
}

ObstacleProto ToObstacleProto(const Obstacle& obstacle) {
  ObstacleProto obstacle_proto;
  // TODO(Jessie): check coordinate
  obstacle_proto.set_x(obstacle.x);
  obstacle_proto.set_y(obstacle.y);
  obstacle_proto.set_min_z(obstacle.min_z);
  obstacle_proto.set_max_z(obstacle.max_z);
  obstacle_proto.set_ground_z(obstacle.ground_z);
  obstacle_proto.set_num_points(obstacle.points.size());
  obstacle_proto.set_type(obstacle.type);
  obstacle_proto.set_clearance(obstacle.clearance);
  obstacle_proto.set_mist_score(obstacle.mist_score);
  obstacle_proto.set_type_source(obstacle.type_source);
  obstacle_proto.set_dist_to_curb(obstacle.dist_to_curb);
  for (const auto& pt : obstacle.points) {
    *obstacle_proto.add_points() = ToLaserPointProto(pt);
  }

  return obstacle_proto;
}

Box2dProto ToBox2dProto(const Box2d& box2d) {
  Box2dProto proto;
  proto.set_x(box2d.center_x());
  proto.set_y(box2d.center_y());
  proto.set_heading(box2d.heading());
  proto.set_length(box2d.length());
  proto.set_width(box2d.width());
  return proto;
}

filtering::ClusterFeatureProto ToClusterFeatureProto(
    const ClusterFeature& feature) {
  filtering::ClusterFeatureProto cluster_feature_proto;
  cluster_feature_proto.set_num_points(feature.num_points);
  cluster_feature_proto.set_range(feature.range);
  cluster_feature_proto.set_has_return_behind_ratio(
      feature.has_return_behind_ratio);
  cluster_feature_proto.set_intensity_mean(feature.intensity_mean);
  cluster_feature_proto.set_intensity_dev(feature.intensity_dev);
  cluster_feature_proto.set_clearance_mean(feature.clearance_mean);
  cluster_feature_proto.set_clearance_dev(feature.clearance_dev);
  cluster_feature_proto.set_adjacent_scan_gap_mean(
      feature.adjacent_scan_gap_mean);
  cluster_feature_proto.set_adjacent_scan_gap_dev(
      feature.adjacent_scan_gap_dev);

  return cluster_feature_proto;
}

filtering::ClusterDataProto ToClusterDataProto(
    const SegmentedCluster& cluster, const VehiclePose& pose,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  filtering::ClusterDataProto cluster_data_proto;
  cluster_data_proto.set_timestamp(cluster.timestamp());
  cluster_data_proto.set_type(cluster.type());
  cluster_data_proto.set_score(cluster.score());
  cluster_data_proto.set_num_obstacles(cluster.NumObstacles());
  cluster_data_proto.set_num_points(cluster.NumPoints());
  for (const auto& obstacle : cluster.obstacles()) {
    *cluster_data_proto.add_obstacles() = ToObstacleProto(*obstacle);
  }
  if (cluster.bounding_box()) {
    *cluster_data_proto.mutable_bounding_box() =
        ToBox2dProto(*cluster.bounding_box());
  }

  const auto cluster_feature =
      ExtractClusterFeature(pose, cluster, lidar_params);
  VLOG(2) << "[ClusterFeature] " << cluster_feature.DebugString();
  *cluster_data_proto.mutable_cluster_feature() =
      ToClusterFeatureProto(cluster_feature);

  *cluster_data_proto.mutable_vehicle_pose() = pose.ToVehiclePoseProto();
  VLOG(2) << "[Vehicle Pose] " << pose.DebugString();
  return cluster_data_proto;
}

// NOTE(wushuai): If current segmented cluster contains few points, recover
// obstacles surrounded that filtered by obstacle rain filter. This is only
// used for mist filter data generation.
SegmentedCluster DilateFewPointsCluster(
    const SegmentedCluster& segmented_cluster,
    const ObstacleManager& obstacle_manager) {
  const int num_points = segmented_cluster.NumPoints();
  if (num_points > kMinMaybeMistPointsCount) {
    return segmented_cluster;
  }
  SegmentedCluster dilated_cluster(segmented_cluster.id(), segmented_cluster);
  ObstaclePtrs* obstacles = dilated_cluster.mutable_obstacles();
  absl::flat_hash_set<ObstaclePtr> visited_obstacles(
      segmented_cluster.obstacles().begin(),
      segmented_cluster.obstacles().end());
  for (const auto& obstacle : segmented_cluster.obstacles()) {
    const int row = obstacle->row;
    const int col = obstacle->col;
    for (int r = row - kMaxMistObstacleRange; r <= row + kMaxMistObstacleRange;
         ++r) {
      for (int c = col - kMaxMistObstacleRange;
           c <= col + kMaxMistObstacleRange; ++c) {
        const ObstaclePtr neighbor = obstacle_manager.ObstacleAt(r, c);
        if (neighbor == nullptr || ContainsKey(visited_obstacles, neighbor)) {
          continue;
        }
        if (neighbor->type_source == ObstacleProto::MIST_OBSTACLE_FILTER_V2 ||
            neighbor->type_source == ObstacleProto::MIST_OBSTACLE_NET_FILTER) {
          obstacles->emplace_back(neighbor);
        }
        visited_obstacles.insert(neighbor);
      }
    }
  }
  return dilated_cluster;
}

absl::StatusOr<filtering::ClusterLabelProto> MatchClusterLabel(
    const SegmentedCluster& cluster,
    const labeling::LabelFrameProto& label_frame,
    const ObstacleManager& obstacle_manager) {
  using namespace filtering;  // NOLINT
  const int total_obstacles_count = cluster.NumObstacles();
  QCHECK_NE(total_obstacles_count, 0);

  if (std::abs(cluster.timestamp() - label_frame.timestamp()) > 0.2) {
    QLOG(WARNING) << "label time: "
                  << absl::StrFormat("%.6f", label_frame.timestamp());
    QLOG(WARNING) << "cluster time: "
                  << absl::StrFormat("%.6f", cluster.timestamp());
    return absl::FailedPreconditionError(
        absl::StrCat("Timestamps do not match"));
  }

  ClusterLabelProto cluster_label_proto;
  const double contour_area = cluster_util::ComputeContour(cluster).area();
  // ambiguous being true means we are not sure what the cluster is.
  bool ambiguous = true;
  // First check if the cluster is in mist zone, then check if it also happens
  // to be in any other bounding box, if so, overwrite its label.
  for (const auto& zone : label_frame.zones()) {
    if (zone.type() != labeling::Zone_ZoneType_MIST) continue;
    // Construct zone polygon
    std::vector<Vec2d> zone_points;
    for (int i = 0; i < zone.xs_size(); ++i) {
      zone_points.emplace_back(zone.xs(i), zone.ys(i));
    }
    if (zone_points.size() < 3) {
      QLOG(ERROR) << labeling::Zone_ZoneType_Name(zone.type())
                  << "zone points number should not be less than 3.";
      continue;
    }

    const Polygon2d zone_polygon(zone_points);

    ObstaclePtrs obstacles_in_zone;
    for (const auto& obstacle : cluster.obstacles()) {
      if (zone_polygon.IsPointIn({obstacle->x, obstacle->y})) {
        obstacles_in_zone.emplace_back(obstacle);
      }
    }

    if (obstacles_in_zone.empty()) continue;

    const double contour_in_zone_area =
        cluster_util::ComputeContour(Cluster(obstacles_in_zone)).area();

    if (obstacles_in_zone.size() * 1.0 / total_obstacles_count > 0.8 &&
        contour_in_zone_area / contour_area > 0.8) {
      ambiguous = false;
      cluster_label_proto.set_cluster_type(ClusterLabelProto::MIST);
      break;
    }
  }
  // check static object zone, vegetation zone and barrier zone
  for (const auto& zone : label_frame.zones()) {
    if (zone.type() != labeling::Zone_ZoneType_STATIC_OBJECT &&
        zone.type() != labeling::Zone_ZoneType_VEGETATION &&
        zone.type() != labeling::Zone_ZoneType_BARRIER)
      continue;
    // Construct zone polygon
    std::vector<Vec2d> zone_points;
    for (int i = 0; i < zone.xs_size(); ++i) {
      zone_points.emplace_back(zone.xs(i), zone.ys(i));
    }
    if (zone_points.size() < 3) {
      QLOG(ERROR) << labeling::Zone_ZoneType_Name(zone.type())
                  << " zone points number should not be less than 3.";
      continue;
    }

    const Polygon2d zone_polygon(zone_points);

    ObstaclePtrs obstacles_in_zone;
    for (const auto& obstacle : cluster.obstacles()) {
      if (zone_polygon.IsPointIn({obstacle->x, obstacle->y})) {
        obstacles_in_zone.emplace_back(obstacle);
      }
    }

    if (obstacles_in_zone.empty()) continue;

    const double contour_in_zone_area =
        cluster_util::ComputeContour(Cluster(obstacles_in_zone)).area();

    constexpr double kMinObstaclesInZoneRatio = 0.8;
    if (obstacles_in_zone.size() * 1.0 / total_obstacles_count >
            kMinObstaclesInZoneRatio &&
        contour_in_zone_area / contour_area > kMinObstaclesInZoneRatio) {
      ambiguous = false;
      cluster_label_proto.set_cluster_type(ClusterLabelProto::OTHER);
      break;
    } else if (obstacles_in_zone.size() > 1) {
      ambiguous = true;
    }
  }
  // check label
  double contour_in_box_area_to_label_area_ratio = 0.0;
  double contour_in_box_area_to_contour_area_ratio = 0.0;
  int obstacles_inside_box_num = -1;
  for (const auto& label : label_frame.labels()) {
    const auto label_bounding_box =
        Polygon2d(Box2d({label.x(), label.y()}, label.heading(), label.length(),
                        label.width()));
    ObstaclePtrs obstacles_inside_box;
    for (const auto* obstacle : cluster.obstacles()) {
      if (label_bounding_box.IsPointIn({obstacle->x, obstacle->y})) {
        obstacles_inside_box.emplace_back(obstacle);
      }
    }

    if (obstacles_inside_box.empty()) continue;

    obstacles_inside_box_num = obstacles_inside_box.size();

    double contour_in_box_area =
        cluster_util::ComputeContour(Cluster(obstacles_inside_box)).area();
    double label_area = label.width() * label.length();
    VLOG(2) << label.x() << " " << label.y() << " " << label.width() << " "
            << label.length();
    VLOG(2) << contour_in_box_area << " / " << label_area << " = "
            << contour_in_box_area / label_area;
    contour_in_box_area_to_label_area_ratio = contour_in_box_area / label_area;
    contour_in_box_area_to_contour_area_ratio =
        contour_in_box_area / contour_area;
    constexpr double kContourInLabelRatio = 0.8;
    if (contour_in_box_area_to_contour_area_ratio > kContourInLabelRatio &&
        obstacles_inside_box_num * 1.0f / total_obstacles_count >
            kContourInLabelRatio) {
      ambiguous = false;
      cluster_label_proto.set_cluster_type(ClusterLabelProto::OTHER);
      break;
    } else if (obstacles_inside_box.size() > 1) {
      ambiguous = true;
    }
  }
  if (!cluster_label_proto.has_cluster_type()) {
    return absl::FailedPreconditionError(absl::StrCat("no cluster type."));
  }
  // Use this visualization to observe and tune visualizations
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/filtered_clusters");
  const Vec2d contour_centroid = cluster.ComputeCentroidFromObstacles();
  if (ambiguous) {
    canvas.DrawPolygon(cluster_util::ComputeContourWithZ(cluster, 0.1),
                       vis::Color::kRed,
                       vis::Color(vis::Color::kRed.r(), vis::Color::kRed.g(),
                                  vis::Color::kRed.b(), 0.3));
    canvas.DrawText(
        absl::StrFormat("contour_in_box_area_to_label_area_ratio %.2f\n"
                        "contour_in_box_area_to_contour_area_ratio %.2f\n"
                        "obstacles_inside_box_num %d total_obstacles_count %d",
                        contour_in_box_area_to_label_area_ratio,
                        contour_in_box_area_to_contour_area_ratio,
                        obstacles_inside_box_num, total_obstacles_count),
        {contour_centroid, 0.3}, 0, 0.2, vis::Color::kGreen);
    return absl::FailedPreconditionError(absl::StrCat("ambiguous"));
  }

  switch (cluster_label_proto.cluster_type()) {
    case ClusterLabelProto::OTHER:
      canvas.DrawPolygon(
          cluster_util::ComputeContourWithZ(cluster, 0.1), vis::Color::kYellow,
          vis::Color(vis::Color::kYellow.r(), vis::Color::kYellow.g(),
                     vis::Color::kYellow.b(), 0.4));
      canvas.DrawText(absl::StrFormat("OTHER"), {contour_centroid, 0.2}, 0, 0.2,
                      vis::Color::kGreen);
      break;
    case ClusterLabelProto::MIST:
      canvas.DrawPolygon(
          cluster_util::ComputeContourWithZ(cluster, 0.1), vis::Color::kWhite,
          vis::Color(vis::Color::kWhite.r(), vis::Color::kWhite.g(),
                     vis::Color::kWhite.b(), 0.3));
      canvas.DrawText(absl::StrFormat("MIST"), {contour_centroid, 0.2}, 0, 0.2,
                      vis::Color::kGreen);
  }

  return cluster_label_proto;
}

// NOTE(dong): Returns two clusters by Cuts through. This is only used for
// mist filter data generation.
SegmentedClusters DivideOversizedClusterIntoTwo(
    const SegmentedCluster& cluster) {
  SegmentedClusters divided_clusters;
  const auto contour = cluster_util::ComputeContour(cluster);
  const auto min_area_box = contour.MinAreaBoundingBox();
  double half_len = min_area_box.half_length();
  Vec2d heading(min_area_box.cos_heading(), min_area_box.sin_heading());
  if (min_area_box.length() > min_area_box.width()) {
    heading = heading.Perp();
    half_len = min_area_box.half_width();
  }
  const Segment2d non_principle_axis{
      min_area_box.center() + heading * half_len,
      min_area_box.center() - heading * half_len};
  ObstaclePtrs obstacles_1, obstacles_2;
  for (const auto* obstacle : cluster.obstacles()) {
    if (non_principle_axis.SignedDistanceTo(obstacle->coord()) >= 0.) {
      obstacles_1.push_back(obstacle);
    } else {
      obstacles_2.push_back(obstacle);
    }
  }

  QCHECK(!obstacles_1.empty() && !obstacles_2.empty());

  static std::default_random_engine e;
  static std::uniform_int_distribution<int> u(0,
                                              std::numeric_limits<int>::max());
  if (!obstacles_1.empty()) {
    // NOTE(dong): Generate a random id for divided segmented cluster. This is
    // acceptable as it's only used for data generation.
    SegmentedCluster cluster_1(u(e), Cluster(obstacles_1));
    cluster_1.set_type(cluster.type());
    cluster_1.set_type_source(cluster.type_source());
    divided_clusters.push_back(cluster_1);
  }
  if (!obstacles_2.empty()) {
    SegmentedCluster cluster_2(u(e), Cluster(obstacles_2));
    cluster_2.set_type(cluster.type());
    cluster_2.set_type_source(cluster.type_source());
    divided_clusters.push_back(cluster_2);
  }

  return divided_clusters;
}

SegmentedClusters DivideEnormousUnknownClusters(
    const SegmentedClusters& clusters) {
  double max_unknown_cluster_area = 40.0;
  int max_unknown_cluster_obstacle_num = 1000;

  if (!FLAGS_enable_obstacle_mist_filter) {
    max_unknown_cluster_area = 10.0;
    max_unknown_cluster_obstacle_num = 100;
  }

  auto is_enormous_unknown_cluster = [=](const auto& cluster) {
    if (cluster.type() == MT_VEHICLE || cluster.type() == MT_PEDESTRIAN ||
        cluster.type() == MT_CYCLIST || cluster.type() == MT_MOTORCYCLIST) {
      return false;
    } else if (cluster_util::ComputeContour(cluster).area() <
                   max_unknown_cluster_area &&
               cluster.NumObstacles() < max_unknown_cluster_obstacle_num) {
      return false;
    } else {
      return true;
    }
  };

  SegmentedClusters new_clusters;
  for (const auto& cluster : clusters) {
    if (!is_enormous_unknown_cluster(cluster)) {
      new_clusters.push_back(cluster);
      continue;
    }
    std::queue<SegmentedCluster> enormous_clusters;
    enormous_clusters.push(cluster);
    while (!enormous_clusters.empty()) {
      auto divided_clusters =
          DivideOversizedClusterIntoTwo(enormous_clusters.front());
      enormous_clusters.pop();
      for (auto& divided_cluster : divided_clusters) {
        if (is_enormous_unknown_cluster(divided_cluster)) {
          enormous_clusters.push(std::move(divided_cluster));
        } else {
          new_clusters.push_back(std::move(divided_cluster));
        }
      }
    }
  }

  return new_clusters;
}

const std::vector<Vec3d> GetBloomingRefLidarPoses(
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  QCHECK(!lidar_params.empty());
  if (const auto* param = FindOrNull(lidar_params, LDR_CENTER)) {
    const auto& lidar_extrinsics = param->installation().extrinsics();
    return {{lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()}};
  } else if (const auto* param = FindOrNull(lidar_params, LDR_FRONT)) {
    const auto& lidar_extrinsics = param->installation().extrinsics();
    return {{lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()}};
  } else if (const auto* fl_param = FindOrNull(lidar_params, LDR_FRONT_LEFT),
             *fr_param = FindOrNull(lidar_params, LDR_FRONT_RIGHT);
             fl_param != nullptr && fr_param != nullptr) {
    const auto& left_lidar_extrinsics = fl_param->installation().extrinsics();
    const Vec3d left_pos = {left_lidar_extrinsics.x(),
                            left_lidar_extrinsics.y(),
                            left_lidar_extrinsics.z()};
    const auto& right_lidar_extrinsics = fr_param->installation().extrinsics();
    const Vec3d right_pos = {right_lidar_extrinsics.x(),
                             right_lidar_extrinsics.y(),
                             right_lidar_extrinsics.z()};
    // NOTE(dong): maybe only use right lidar as ref lidar as most blooming
    // cases are at the front right of av.
    return {left_pos, right_pos};
  } else {
    const auto& lidar_extrinsics =
        lidar_params.begin()->second.installation().extrinsics();
    return {{lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()}};
  }
}

bool CheckIfTwoObstaclesAreInBloomingRadiusRange(
    const std::vector<Vec3d>& ref_lidar_poses, const Vec2d& obstacle_pos,
    const Vec2d& retroreflector_obstacle_pos) {
  for (const auto& ref_lidar_pos : ref_lidar_poses) {
    const float dis0 = Hypot(obstacle_pos.x() - ref_lidar_pos.x(),
                             obstacle_pos.y() - ref_lidar_pos.y());
    const float dis1 =
        Hypot(retroreflector_obstacle_pos.x() - ref_lidar_pos.x(),
              retroreflector_obstacle_pos.y() - ref_lidar_pos.y());
    if (std::abs(dis0 - dis1) < kMaxBloomingRangeRadiusDiff) {
      return true;
    }
  }

  return false;
}

}  // namespace

bool ClusterFilter::IsBloomingSegmentedCluster(
    const VehiclePose& pose, const SegmentedCluster& segmented_cluster,
    const ObstacleManager& obstacle_manager,
    ClusterFilterDebugProto* debug_proto) const {
  if (!IsClusterBloomingFilterable(segmented_cluster)) {
    return false;
  }

  const auto& obstacles = segmented_cluster.obstacles();
  QCHECK_GT(obstacles.size(), 0);

  // Blooming clusters are always close to curbs, small and unknown.
  // Negative distance to curb means on road and positive means offroad.
  ObstaclePtrs near_curb_obstacles;
  for (const auto& obstacle : obstacles) {
    if (obstacle->dist_to_curb > -kBloomingMinDistToCurb) {
      near_curb_obstacles.push_back(obstacle);
    }
  }

  // Not a blooming if it's not near curb.
  if (near_curb_obstacles.empty()) {
    return false;
  }

  constexpr double kBloomingClusterArea = 8.;  // m^2
  const auto contour_area =
      cluster_util::ComputeContour(segmented_cluster).area();
  if (contour_area > kBloomingClusterArea) {
    return false;
  }

  const auto blooming_ref_lidar_poses_in_vehicle =
      GetBloomingRefLidarPoses(lidar_params_);

  QCHECK(!blooming_ref_lidar_poses_in_vehicle.empty());

  std::vector<Vec3d> blooming_ref_lidar_poses;
  for (const auto& pose_in_vehicle : blooming_ref_lidar_poses_in_vehicle) {
    blooming_ref_lidar_poses.emplace_back(
        pose.ToTransform().TransformPoint(pose_in_vehicle));
  }

  std::vector<ObstaclePtr> retroreflector_obstacles;
  std::vector<ObstaclePtr> maybe_retroreflector_obstacles;
  std::unordered_set<ObstaclePtr> visited_obstacles;
  int total_num_retroreflected_points = 0;
  // Going offroad to fetch offroad obstacles and check if there's a large
  // retroreflector (intensity > 250).
  // NOTE(dong): A near curb obstacle needs to find retroreflector obstacles
  // from its offroad neighbors in blooming radius range.
  for (const auto* near_curb_obstacle : near_curb_obstacles) {
    const int row = near_curb_obstacle->row;
    const int col = near_curb_obstacle->col;
    for (int r = row - kMaxBloomingObstacleRange;
         r <= row + kMaxBloomingObstacleRange; ++r) {
      for (int c = col - kMaxBloomingObstacleRange;
           c <= col + kMaxBloomingObstacleRange; ++c) {
        if (r == row && c == col) continue;
        const ObstaclePtr neighbor = obstacle_manager.ObstacleAt(r, c);
        if (neighbor == nullptr || ContainsKey(visited_obstacles, neighbor)) {
          continue;
        }
        visited_obstacles.insert(neighbor);
        if (neighbor->dist_to_curb < -0.2) {
          continue;
        }
        if (!CheckIfTwoObstaclesAreInBloomingRadiusRange(
                blooming_ref_lidar_poses,
                {near_curb_obstacle->x, near_curb_obstacle->y},
                {neighbor->x, neighbor->y})) {
          continue;
        }
        int num_retroreflected_points = 0;
        for (const auto& point : neighbor->points) {
          if (point.intensity >= kRetroreflectorMinIntensity) {
            ++num_retroreflected_points;
          }
        }
        total_num_retroreflected_points += num_retroreflected_points;
        float retroreflected_points_ratio =
            static_cast<float>(num_retroreflected_points) /
            neighbor->points.size();
        if (retroreflected_points_ratio > kMinRetroreflectedPointsRatio) {
          retroreflector_obstacles.emplace_back(neighbor);
        }
        if (retroreflected_points_ratio > kMinMaybeRetroreflectedPointsRatio ||
            num_retroreflected_points >= kMinMaybeRetroreflectedPointsCount) {
          maybe_retroreflector_obstacles.emplace_back(neighbor);
        }
      }
    }
  }
  constexpr int kMinNumOffroadRetroreflectorObstacles = 2;
  const bool is_normal_blooming =
      retroreflector_obstacles.size() >= kMinNumOffroadRetroreflectorObstacles;

  bool is_ouster_blooming = false;
  double ouster_points_ratio = 0.0;
  constexpr double kMinOusterBloomingClusterArea = 1.0;  // m^2
  constexpr int kMinOusterBloomingNeighborTotalRetroreflectedPoints = 5;
  if (!maybe_retroreflector_obstacles.empty() &&
      contour_area <= kMinOusterBloomingClusterArea &&
      total_num_retroreflected_points >
          kMinOusterBloomingNeighborTotalRetroreflectedPoints) {
    const auto [ouster_num_points, total_num_points] =
        CountsOusterPointsAboveGroundInCluster(segmented_cluster);
    constexpr double kMinBloomingOusterPointsRatio = 0.85;
    QCHECK_LE(ouster_num_points, total_num_points);
    ouster_points_ratio =
        static_cast<double>(ouster_num_points) / total_num_points;
    if (ouster_points_ratio > kMinBloomingOusterPointsRatio) {
      is_ouster_blooming = true;
    }
  }
  // NOTE(dong): Do not deal with normal blooming clusters as they should be
  // filtered by blooming proposer. Filtering ouster blooming is temporary a
  // hack here which should also be moved to blooming proposer later.
  const bool is_blooming = is_ouster_blooming;

  std::string debug_string = absl::StrFormat(
      "id: %d, is_normal_blooming? %s, is_ouster_blooming? %s, #near curb "
      "obstacles: %.2f, \n num offroad retroreflector obstacles: %d, contour "
      "area: %.2f, \n num maybe offroad retroreflector obstacles: %d, ouster "
      "points ratio: %.2f, \n total num retroreflected points: %d",
      segmented_cluster.id(), is_normal_blooming ? "YES" : "NO",
      is_ouster_blooming ? "YES" : "NO", near_curb_obstacles.size(),
      retroreflector_obstacles.size(), contour_area,
      maybe_retroreflector_obstacles.size(), ouster_points_ratio,
      total_num_retroreflected_points);

  AddClusterFilterDebugProto(debug_proto, std::move(debug_string),
                             segmented_cluster,
                             ClusterFilterDebugProto::BLOOMING);

  if (FLAGS_blooming_filter_cvs && is_blooming) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/filtered_clusters");
    for (const auto* obstacle_ptr : retroreflector_obstacles) {
      const std::vector<Vec3d> obstacle_contour =
          obstacle_util::ComputeContourWithZ(*obstacle_ptr,
                                             obstacle_ptr->max_z + 1.0);
      canvas.DrawPolygon(obstacle_contour, vis::Color::kLightBlue, 1);
    }
    for (const auto* obstacle_ptr : maybe_retroreflector_obstacles) {
      const std::vector<Vec3d> obstacle_contour =
          obstacle_util::ComputeContourWithZ(*obstacle_ptr,
                                             obstacle_ptr->max_z + 1.0);
      canvas.DrawPolygon(obstacle_contour, vis::Color::kBlue, 1);
    }
    if (!maybe_retroreflector_obstacles.empty()) {
      auto draw_blooming_radius_range = [&](const Vec3d& ref_lidar_pos,
                                            const vis::Color color) {
        float min_radius = std::numeric_limits<float>::max();
        float max_radius = 0;
        for (const auto* obstacle : maybe_retroreflector_obstacles) {
          const float dis = Hypot(obstacle->x - ref_lidar_pos.x(),
                                  obstacle->y - ref_lidar_pos.y());
          min_radius = std::min(min_radius, dis - kMaxBloomingRangeRadiusDiff);
          max_radius = std::max(max_radius, dis + kMaxBloomingRangeRadiusDiff);
        }
        min_radius = std::max(min_radius, 0.0f);
        canvas.DrawCircle(ref_lidar_pos, min_radius, color, 1);
        canvas.DrawCircle(ref_lidar_pos, max_radius, color, 1);
        canvas.DrawPoint(ref_lidar_pos, color, 5);
      };
      draw_blooming_radius_range(blooming_ref_lidar_poses[0],
                                 vis::Color::kGreen);
      if (blooming_ref_lidar_poses.size() == 2) {
        draw_blooming_radius_range(blooming_ref_lidar_poses[1],
                                   vis::Color::kRed);
      }
    }
  }

  return is_blooming;
}

bool ClusterFilter::IsRainSegmentedCluster(
    const VehiclePose& pose, const SegmentedCluster& segmented_cluster,
    ClusterFilterDebugProto* debug_proto) const {
  if (!IsClusterFilterable(segmented_cluster)) {
    return false;
  }
  if (ContainsOusterPoints(segmented_cluster)) {
    return false;
  }
  if (IsOversizedCluster(segmented_cluster)) {
    return false;
  }
  const auto& cluster_feature =
      ExtractClusterFeature(pose, segmented_cluster, lidar_params_);

  if (cluster_feature.range > kMaxMistNetRange ||
      cluster_feature.intensity_mean > kMinUnfilterableClusterIntensity) {
    return false;
  }

  const float mist_net_score =
      mist_net_v1_.ClassifySegmentedCluster(cluster_feature);
  const bool is_rain = RoundToInt(mist_net_score);

  if (FLAGS_cluster_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/filtered_clusters");
    const Vec2d contour_centroid =
        segmented_cluster.ComputeCentroidFromObstacles();
    canvas.DrawText(
        absl::StrFormat("score: %.3f, has_return_behind_ratio: %.3f",
                        segmented_cluster.score(),
                        cluster_feature.has_return_behind_ratio),
        {contour_centroid, 0.01}, 0, 0.2, vis::Color::kLightMagenta);
    canvas.DrawText(absl::StrFormat("intensity_mean: %.3f, intensity_var: %.3f",
                                    cluster_feature.intensity_mean,
                                    cluster_feature.intensity_dev),
                    {contour_centroid - Vec2d(0, 0.3), 0.01}, 0, 0.2,
                    vis::Color::kLightMagenta);
    canvas.DrawText(
        absl::StrFormat("id: %d, is rain? %s ", segmented_cluster.id(),
                        is_rain ? "YES" : "NO"),
        {contour_centroid - Vec2d(0, 0.6), 0.01}, 0, 0.2,
        vis::Color::kLightMagenta);
  }

  std::string debug_string = absl::StrFormat(
      "id: %d, is rain? %s, score: %.3f, \n"
      "has_return_behind_ratio: %.3f,\n"
      "intensity_mean: %.3f, intensity_var: %.3f, mist net score: %.3f",
      segmented_cluster.id(), is_rain ? "YES" : "NO", segmented_cluster.score(),
      cluster_feature.has_return_behind_ratio, cluster_feature.intensity_mean,
      cluster_feature.intensity_dev, mist_net_score);
  {
    absl::MutexLock lock(&mist_mutex_);
    AddClusterFilterDebugProto(debug_proto, std::move(debug_string),
                               segmented_cluster,
                               ClusterFilterDebugProto::RAIN);
  }

  return is_rain;
}

bool ClusterFilter::IsMirrorReflectionSegmentedCluster(
    const SegmentedCluster& segmented_cluster,
    ClusterFilterDebugProto* debug_proto) const {
  if (!IsClusterFilterable(segmented_cluster)) {
    return false;
  }
  const auto& obstacles = segmented_cluster.obstacles();

  // Features: 2nd points ratio; cluster score;
  //           num of points; obstacle clearance.
  const float score = segmented_cluster.score();
  const float num_obstacles = obstacles.size();
  const float num_pts = segmented_cluster.NumPoints();
  float avg_clearance = 0.0f;
  float avg_obstacle_z_range = 0.0f;
  float num_2nd_return_pts = 0.0f;

  for (const auto& obstacle : obstacles) {
    avg_clearance += obstacle->clearance;
    avg_obstacle_z_range += obstacle->max_z - obstacle->min_z;
    const auto& pts = obstacle->points;
    for (const auto& pt : pts) {
      if (pt.return_index == 1) {
        ++num_2nd_return_pts;
      }
    }
  }

  if (num_obstacles > 1) {
    avg_clearance /= num_obstacles;
    avg_obstacle_z_range /= num_obstacles;
  }

  const float ratio_2nd_return_pts = num_2nd_return_pts / num_pts;

  const bool condition_0 = num_pts <= kMirrorReflectionPointNum;

  const bool condition_1 =
      ratio_2nd_return_pts >= kMirrorReflection2ndReturnRatioMin &&
      avg_clearance <= kMirrorReflectionAvgClearance &&
      score <= kMirrorReflectionClusterScore;

  const bool condition_2 =
      ratio_2nd_return_pts >= kMirrorReflection2ndReturnRatio &&
      score <= kMirrorReflectionClusterScore;

  const bool condition_3 =
      ratio_2nd_return_pts >= kMirrorReflection2ndReturnRatioMax &&
      score <= kMirrorReflectionClusterScoreMax;

  // This is solely for reflections from overhanging traffic signs.
  // From observation, reflections from traffic signs are mostly second
  // returns (it has to be all second returns theoritically but some of the
  // first returns are too weak to be detected), with high intensities, with
  // very small obstacle z range (max_z - min_z), small obstacle clearance;
  const bool condition_4 =
      score <= kMirrorReflectionClusterScore &&
      ratio_2nd_return_pts >=
          kMirrorReflection2ndReturnRatioForOverhangingTrafficSign &&
      avg_obstacle_z_range <= kMirrorReflectionObstacleZRange &&
      avg_clearance <= kMirrorReflectionAvgClearanceForOverhangingTrafficSign;

  if (FLAGS_cluster_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/filtered_clusters");
    const Vec2d contour_centroid =
        segmented_cluster.ComputeCentroidFromObstacles();
    canvas.DrawText(
        absl::StrFormat(
            "score: %.2f, avg_clearance: %.2f, avg_ob_z_range: %.2f ", score,
            avg_clearance, avg_obstacle_z_range),
        {contour_centroid + Vec2d(0, 0.6), 0.01}, 0, 0.2,
        vis::Color::kLightYellow);
    canvas.DrawText(absl::StrFormat("ratio_2nd_return_pts: %.2f, num_pts: %.2f",
                                    ratio_2nd_return_pts, num_pts),
                    {contour_centroid + Vec2d(0, 0.4), 0.01}, 0, 0.2,
                    vis::Color::kLightYellow);
    canvas.DrawText(
        absl::StrFormat("reflection conditions: %d & (%d | %d | %d | %d)",
                        condition_0 ? 1 : 0, condition_1 ? 1 : 0,
                        condition_2 ? 1 : 0, condition_3 ? 1 : 0,
                        condition_4 ? 1 : 0),
        {contour_centroid + Vec2d(0, 0.2), 0.01}, 0, 0.2,
        vis::Color::kLightYellow);
  }

  bool is_mirror_reflection =
      condition_0 && (condition_1 || condition_2 || condition_3 || condition_4);

  std::string debug_string = absl::StrFormat(
      "is mirror reflection?: %s, score: %.2f, avg_clearance: %.2f,\n"
      "avg_ob_z_range: %.2f, ratio_2nd_return_pts: %.2f, num_pts: %.2f,\n"
      "reflection conditions: %d & (%d | %d | %d | %d)",
      is_mirror_reflection ? "YES" : "NO", score, avg_clearance,
      avg_obstacle_z_range, ratio_2nd_return_pts, num_pts, condition_0 ? 1 : 0,
      condition_1 ? 1 : 0, condition_2 ? 1 : 0, condition_3 ? 1 : 0,
      condition_4 ? 1 : 0);
  AddClusterFilterDebugProto(debug_proto, std::move(debug_string),
                             segmented_cluster,
                             ClusterFilterDebugProto::MIRROR);

  return is_mirror_reflection;
}

ClusterFilter::FilterOutput ClusterFilter::FilterReflectionClusters(
    const VehiclePose& pose,
    const SegmentedClusters& segmented_clusters) const {
  SCOPED_QTRACE_ARG1("ClusterFilter::FilterReflections", "num_clusters",
                     segmented_clusters.size());

  ClusterFilter::FilterOutput output;
  for (int i = 0; i < segmented_clusters.size(); ++i) {
    if (IsMirrorReflectionSegmentedCluster(segmented_clusters[i],
                                           &output.debug_proto)) {
      output.filtered_cluster_indices.push_back(i);
    }
  }
  return output;
}

ClusterFilter::FilterOutput ClusterFilter::FilterBloomingClusters(
    const VehiclePose& pose, const SegmentedClusters& segmented_clusters,
    const ObstacleManager& obstacle_manager) const {
  SCOPED_QTRACE_ARG1("ClusterFilter::FilterBlooming", "num_clusters",
                     segmented_clusters.size());
  ClusterFilter::FilterOutput output;
  for (int i = 0; i < segmented_clusters.size(); ++i) {
    if (IsBloomingSegmentedCluster(pose, segmented_clusters[i],
                                   obstacle_manager, &output.debug_proto)) {
      output.filtered_cluster_indices.push_back(i);
    }
  }
  return output;
}

ClusterFilter::FilterOutput ClusterFilter::FilterMistClusters(
    const VehiclePose& pose,
    const SegmentedClusters& segmented_clusters) const {
  SCOPED_QTRACE_ARG1("ClusterFilter::FilterMistClusters", "num_clusters",
                     segmented_clusters.size());
  ClusterFilter::FilterOutput output;
  std::vector<int> indices(segmented_clusters.size(), false);
  ParallelFor(0, segmented_clusters.size(), thread_pool_, [&](size_t i) {
    if (IsRainSegmentedCluster(pose, segmented_clusters[i],
                               &output.debug_proto)) {
      indices[i] = true;
    }
  });
  for (int i = 0; i < indices.size(); ++i) {
    if (indices[i]) {
      output.filtered_cluster_indices.push_back(i);
    }
  }

  constexpr int kMinNumMistClustersSuspectedMistScenario = 10;
  if (output.filtered_cluster_indices.size() >=
      kMinNumMistClustersSuspectedMistScenario) {
    QEVENT_EVERY_N_SECONDS("dongchen", "suspected_mist_scenario", /*sec*/ 5.0,
                           [=](QEvent* qevent) {
                             qevent->AddField(
                                 "num_mist_clusters",
                                 output.filtered_cluster_indices.size());
                           });
  }

  return output;
}

ClusterFilter::FilterOutput ClusterFilter::FilterSmallClusters(
    const SegmentedClusters& segmented_clusters) const {
  SCOPED_QTRACE_ARG1("ClusterFilter::FilterSmall", "num_clusters",
                     segmented_clusters.size());
  ClusterFilter::FilterOutput output;
  for (int i = 0; i < segmented_clusters.size(); ++i) {
    const auto& cluster = segmented_clusters[i];
    if (!IsClusterFilterable(cluster)) {
      continue;
    }
    // Don't create measurement for clusters with too few points.
    int num_points = 0;
    for (const auto* obstacle : cluster.obstacles()) {
      for (int j = 0; j < obstacle->points.size(); ++j) {
        if (obstacle_util::IsAboveGroundObstaclePoint(*obstacle,
                                                      obstacle->points[j])) {
          num_points += obstacle->points.size() - j;
          break;
        }
      }
    }
    if (num_points < kClusterMinNumPointsAboveGround) {
      std::string debug_string = absl::StrFormat(
          "id: %d, small cluster with %d points", cluster.id(), num_points);
      AddClusterFilterDebugProto(&output.debug_proto, std::move(debug_string),
                                 cluster, ClusterFilterDebugProto::SMALL);

      output.filtered_cluster_indices.push_back(i);
    }
  }
  return output;
}

void ClusterFilter::AddClusterData(
    const VehiclePose& pose, const SegmentedClusters& clusters,
    filtering::ClustersProto* clusters_proto,
    const labeling::LabelFrameProto* label_frame,
    const ObstacleManager& obstacle_manager) const {
  QCHECK_NOTNULL(clusters_proto);

  if (label_frame == nullptr) {
    QLOG_EVERY_N_SEC(WARNING, 1.0) << "No label frame received.";
    return;
  }

  SegmentedClusters divided_clusters = DivideEnormousUnknownClusters(clusters);

  for (const auto& cluster : divided_clusters) {
    QCHECK_GT(cluster.NumObstacles(), 0);
    const auto contour = cluster_util::ComputeContour(cluster);
    if (contour.IsPointIn({pose.x, pose.y})) continue;

    const auto dilated_cluster =
        DilateFewPointsCluster(cluster, obstacle_manager);
    auto cluster_label_proto_or =
        MatchClusterLabel(dilated_cluster, *label_frame, obstacle_manager);
    if (!cluster_label_proto_or.ok()) {
      VLOG(1) << "no label for current cluster, failed with message: "
              << cluster_label_proto_or.status().message();
      continue;
    }
    const auto cluster_label_proto = cluster_label_proto_or.value();

    // Label might not have a type, will not save this cluster
    if (!cluster_label_proto.has_cluster_type()) continue;

    filtering::ClusterProto* cluster_proto = clusters_proto->add_clusters();

    *cluster_proto->mutable_cluster_label() = cluster_label_proto;

    *cluster_proto->mutable_cluster_data() =
        ToClusterDataProto(dilated_cluster, pose, lidar_params_);
  }

  return;
}

void ClusterFilter::Filter(const VehiclePose& pose,
                           const labeling::LabelFrameProto* latest_label_frame,
                           ObstacleManager* obstacle_manager,
                           SegmentedClusters* segmented_clusters) {
  SCOPED_QTRACE_ARG1("ClusterFilter::Filter", "num_clusters",
                     QCHECK_NOTNULL(segmented_clusters)->size());

  enum ClusterFilterType {
    kReflection = 0,
    kMist,
    kBlooming,
    kSmallCluster,
    kNumFilters,
  };

  if (FLAGS_collect_cluster_data) {
    AddClusterData(pose, *segmented_clusters, &clusters_proto_,
                   latest_label_frame, *obstacle_manager);
  }

  std::vector<std::pair<ClusterFilterType, std::function<FilterOutput()>>>
      filter_funcs;

  if (FLAGS_enable_cluster_reflection_filter) {
    filter_funcs.emplace_back(kReflection, [&] {
      return FilterReflectionClusters(pose, *segmented_clusters);
    });
  }

  if (FLAGS_enable_cluster_mist_filter) {
    filter_funcs.emplace_back(
        kMist, [&] { return FilterMistClusters(pose, *segmented_clusters); });
  }

  if (FLAGS_enable_blooming_filter) {
    filter_funcs.emplace_back(kBlooming, [&] {
      return FilterBloomingClusters(pose, *segmented_clusters,
                                    *obstacle_manager);
    });
  }

  if (FLAGS_enable_small_cluster_filter) {
    filter_funcs.emplace_back(kSmallCluster, [&] {
      return FilterSmallClusters(*segmented_clusters);
    });
  }

  std::vector<FilterOutput> filter_outputs(filter_funcs.size());
  ParallelFor(0, filter_funcs.size(), thread_pool_,
              [&](int i) { filter_outputs[i] = filter_funcs[i].second(); });

  std::vector<SegmentedClusters> ignored_clusters_map(kNumFilters);
  std::vector<bool> ignore_masks(segmented_clusters->size());
  ClusterFilterDebugProto collected_debug_proto;
  for (int i = 0; i < filter_funcs.size(); ++i) {
    const auto& output = filter_outputs[i];
    auto& ignored_clusters = ignored_clusters_map[filter_funcs[i].first];
    for (const int i : output.filtered_cluster_indices) {
      if (!ignore_masks[i]) {
        ignored_clusters.push_back((*segmented_clusters)[i]);
        ignore_masks[i] = true;
      }
    }
    for (const auto& info : output.debug_proto.cluster_info()) {
      *collected_debug_proto.add_cluster_info() = info;
    }
  }

  MaybeRenderFilteredClusters(ignored_clusters_map[kReflection],
                              ignored_clusters_map[kMist],
                              ignored_clusters_map[kSmallCluster]);

  {
    SCOPED_QTRACE("ClusterFilter::Filter_publish_debug_proto");
    QLOG_IF_NOT_OK(WARNING, module_->Publish(collected_debug_proto));
  }

  // NOTE(zheng): Make sure obstacle proto is created after all
  // obstacle whitelist/backlist/filtering logics.
  obstacle_manager->BlacklistObstaclesWithinFilteredClusters(
      ignored_clusters_map[kReflection], ignored_clusters_map[kMist],
      ignored_clusters_map[kBlooming], ignored_clusters_map[kSmallCluster]);

  SegmentedClusters new_segmented_clusters;
  for (int i = 0; i < segmented_clusters->size(); ++i) {
    if (!ignore_masks[i]) {
      new_segmented_clusters.push_back((*segmented_clusters)[i]);
    }
  }
  *segmented_clusters = std::move(new_segmented_clusters);
}

ClusterFilter::~ClusterFilter() {
  if (FLAGS_collect_cluster_data) {
    if (FLAGS_cluster_data_file_name.empty()) {
      QLOG(ERROR) << "Need to set cluster data file name while collecting "
                     "cluster data.";
      return;
    }
    CHECK(file_util::ProtoToTextFile(clusters_proto_,
                                     FLAGS_cluster_data_file_name));
    QLOG(INFO) << "Wrote clusters data :" << FLAGS_cluster_data_file_name;
  }
}

}  // namespace qcraft
