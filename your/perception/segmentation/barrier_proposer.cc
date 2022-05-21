#include "onboard/perception/segmentation/barrier_proposer.h"

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include "offboard/mapping/mapping_core/util/point_cloud_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/perception/obstacle_util.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

DEFINE_bool(barrier_proposer_cvs, false, "Enable barrier proposer cvs.");

namespace qcraft::segmentation {

namespace {
constexpr int kMinFittingObstacleSize = 5;
constexpr double kMinLineSacThreshold = 0.2;  // m
constexpr int kMinObstaclePointsNums = 10;
constexpr float kMinPointsZDiff = 0.2f;  // m
constexpr int kProbNoiseObstacleNums = 4;
constexpr float kMinPointToLineDistance = 0.3f;  // m
}  // namespace

std::optional<Vec3f> FittingLineRansac(const std::vector<Vec3d>& points) {
  if (points.size() < 3) return std::nullopt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      mapping::util::ToPclPointCloud(points);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  pcl::SACSegmentation<pcl::PointXYZ> sac;
  sac.setOptimizeCoefficients(true);
  sac.setModelType(pcl::SACMODEL_LINE);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setDistanceThreshold(kMinLineSacThreshold);
  sac.setMaxIterations(100);
  sac.setInputCloud(cloud);
  sac.segment(inliers, coefficients);
  if (0 == inliers.indices.size()) return std::nullopt;
  const float point_x = coefficients.values[0];
  const float point_y = coefficients.values[1];
  // const float point_z = coefficients.values[2];
  const float direction_x = coefficients.values[3];
  const float direction_y = coefficients.values[4];
  const float direction_z = coefficients.values[5];

  const Vec3f line_direction(direction_x, direction_y, direction_z);
  if (line_direction.squaredNorm() == 0.0) return std::nullopt;
  // Line function Ax + B y + C = 0, and general_coeff means {A, B, C}.
  Vec3f general_coeff;
  if (direction_x == 0.f) {
    general_coeff = {1, 0, -point_x};
  } else {
    const float mid_value = direction_y / direction_x;
    general_coeff = {mid_value, -1, point_y - point_x * mid_value};
  }

  if (FLAGS_barrier_proposer_cvs) {
    const Vec2d start_point(
        points.front().x(),
        (points.front().x() * general_coeff.x() + general_coeff.z()) /
            (-general_coeff.y()));
    const Vec2d end_point(
        points.back().x(),
        (points.back().x() * general_coeff.x() + general_coeff.z()) /
            (-general_coeff.y()));
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/barrier_proposer_cvs");
    canvas.DrawLine({start_point, 0.0}, {end_point, 0.0}, vis::Color::kRed);
  }

  return general_coeff;
}

ProposedClusters FilterBarrierNoise(const ProposedCluster& cluster,
                                    const bool is_unknown_barrier) {
  // Line fitting
  std::vector<Vec3d> points;
  points.reserve(cluster.NumPoints());
  for (const auto& obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      points.emplace_back(point.x, point.y, 0.0);
    }
  }
  std::optional<Vec3f> general_axis;
  if (!is_unknown_barrier && cluster.NumObstacles() > kMinFittingObstacleSize) {
    general_axis = FittingLineRansac(points);
  }

  const auto distance_point_to_line = [&general_axis](const Vec2d& point) {
    if (!general_axis) return 0.f;
    const float numerator =
        std::abs(general_axis->x() * point.x() + general_axis->y() * point.y() +
                 general_axis->z());
    const float denominator = Hypot(general_axis->x(), general_axis->y());
    return numerator / denominator;
  };

  const auto& all_obstacles = cluster.obstacles();
  const auto get_obstacles_info =
      [&](const std::vector<int>& indices) -> std::pair<bool, bool> {
    const bool barrier_exist =
        std::any_of(indices.begin(), indices.end(), [&](const auto index) {
          return all_obstacles[index]->type == ObstacleProto::BARRIER;
        });
    const bool is_noise =
        indices.size() < kProbNoiseObstacleNums &&
        std::all_of(indices.begin(), indices.end(), [&](const auto index) {
          const bool few_points =
              all_obstacles[index]->points.size() < kMinObstaclePointsNums;
          const bool points_piece = all_obstacles[index]->max_z -
                                        all_obstacles[index]->ground_z -
                                        all_obstacles[index]->clearance <
                                    kMinPointsZDiff;
          return few_points && points_piece;
        });
    return {barrier_exist, is_noise};
  };

  absl::flat_hash_set<int> ignore_indices;
  absl::flat_hash_set<int> outlier_indices;
  obstacle_util::LocalObstacleGrid local_grid(all_obstacles);
  std::vector<bool> processed(all_obstacles.size(), false);
  if (!general_axis) {
    for (int i = 0; i < all_obstacles.size(); ++i) {
      if (processed[i]) continue;
      const auto& obstacle = *(all_obstacles[i]);
      const auto raw_indices =
          local_grid.FindConnectedNeighbors(obstacle.row, obstacle.col, 4);
      for (const auto index : raw_indices) {
        processed[index] = true;
      }
      const auto [barrier_exist, is_noise] = get_obstacles_info(raw_indices);
      if (!barrier_exist && (is_noise || raw_indices.size() == 1)) {
        std::copy(raw_indices.begin(), raw_indices.end(),
                  std::inserter(ignore_indices, ignore_indices.end()));
      }
    }
  } else {
    for (int i = 0; i < all_obstacles.size(); ++i) {
      if (processed[i]) continue;
      const auto& obstacle = *(all_obstacles[i]);
      if (distance_point_to_line(obstacle.coord()) < kMinPointToLineDistance) {
        continue;
      }
      const auto raw_indices =
          local_grid.FindConnectedNeighbors(obstacle.row, obstacle.col, 8);
      for (const auto index : raw_indices) {
        processed[index] = true;
      }
      const auto [barrier_exist, is_noise] = get_obstacles_info(raw_indices);
      if (!barrier_exist && is_noise) {
        std::copy(raw_indices.begin(), raw_indices.end(),
                  std::inserter(ignore_indices, ignore_indices.end()));
      } else if (std::all_of(raw_indices.begin(), raw_indices.end(),
                             [&](const auto index) {
                               return distance_point_to_line(
                                          all_obstacles[index]->coord()) >
                                      kMinPointToLineDistance;
                             })) {
        std::copy(raw_indices.begin(), raw_indices.end(),
                  std::inserter(outlier_indices, outlier_indices.end()));
      }
    }
  }

  // Only two types, inlier barrier obstacles and outlier obstacles.
  std::vector<ObstaclePtrs> segment_obstacles(2);
  ObstaclePtrs ignore_obstacles;
  for (int i = 0; i < all_obstacles.size(); ++i) {
    if (ContainsKey(ignore_indices, i)) {
      ignore_obstacles.emplace_back(all_obstacles[i]);
    } else if (ContainsKey(outlier_indices, i)) {
      segment_obstacles[0].emplace_back(all_obstacles[i]);
    } else {
      segment_obstacles[1].emplace_back(all_obstacles[i]);
    }
  }

  ProposedClusters divided_clusters;
  if (!ignore_obstacles.empty()) {
    ProposedCluster cluster_sub =
        ProposedCluster::InheritFrom(cluster).ConstructBase(ignore_obstacles);
    cluster_sub.set_is_proposed(true);
    cluster_sub.set_property(PP_NOISE);
    divided_clusters.push_back(std::move(cluster_sub));
  }
  for (const auto& obstacles : segment_obstacles) {
    if (!obstacles.empty()) {
      ProposedCluster cluster_sub =
          ProposedCluster::InheritFrom(cluster).ConstructBase(obstacles);
      cluster_sub.set_is_proposed(true);
      cluster_sub.set_type(cluster.type());
      cluster_sub.set_type_source(cluster.type_source());
      cluster_sub.set_score(cluster.score());
      if (cluster.bounding_box()) {
        cluster_sub.set_bounding_box(*cluster.bounding_box());
      }
      divided_clusters.push_back(std::move(cluster_sub));
    }
  }

  return divided_clusters;
}

ProposedClusters BarrierProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  ProposedClusters proposed_clusters;
  for (const auto& cluster : clusters) {
    bool is_unknown_barrier = false;
    if (cluster.type() == MT_UNKNOWN) {
      is_unknown_barrier = std::any_of(
          cluster.obstacles().begin(), cluster.obstacles().end(),
          [](const auto& obs) { return obs->type == ObstacleProto::BARRIER; });
    }
    if (cluster.type() != MT_BARRIER && !is_unknown_barrier) {
      proposed_clusters.emplace_back(cluster);
      continue;
    }
    const auto& segmented_clusters =
        FilterBarrierNoise(cluster, is_unknown_barrier);
    proposed_clusters.insert(
        proposed_clusters.end(),
        std::make_move_iterator(segmented_clusters.begin()),
        std::make_move_iterator(segmented_clusters.end()));
  }

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
