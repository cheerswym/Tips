#include "onboard/perception/segmentation/proposer_util.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/math/util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(segmentation_profile, false, "Print function time consuming.");

namespace qcraft::segmentation {

namespace {
template <typename C>
double GetMeanAngle(const C& c) {
  QCHECK(!c.empty());
  auto it = std::cbegin(c);
  auto end = std::cend(c);
  double x = 0.0;
  double y = 0.0;
  double len = 0.0;
  while (it != end) {
    x += cos(*it);
    y += sin(*it);
    len++;
    it = std::next(it);
  }
  return atan2(y / len, x / len);
}

std::vector<std::vector<double>> ComputeKMeanOneDimension(
    const std::vector<double>& angles, const int& num_category) {
  const int len = angles.size();
  const std::set<double> deduplicate_set(angles.begin(), angles.end());
  std::vector<double> mean_angles(num_category, 0);

  int index = 0;
  for (const auto& angle : deduplicate_set) {
    mean_angles[index++] = angle;
    if (index == num_category) {
      break;
    }
  }

  while (index < num_category) {
    mean_angles[index++] = std::numeric_limits<double>::max();
  }

  // Comment(zhangtao): Maximum number of iterations, if the maximum number of
  // iterations is reached and there is no convergence, the original array is
  // returned
  constexpr int kMaxNumIteration = 100;
  int iteration = 0;

  while (iteration < kMaxNumIteration) {
    std::vector<std::vector<double>> clusters(num_category);
    for (int i = 0; i < len; ++i) {
      double min_dis = std::abs(NormalizeAngle(angles[i] - mean_angles[0]));
      int min_dis_index = 0;

      for (int j = 1; j < num_category; ++j) {
        const double dis = std::abs(NormalizeAngle(angles[i] - mean_angles[j]));
        if (dis < min_dis) {
          min_dis = dis;
          min_dis_index = j;
        }
      }

      clusters[min_dis_index].push_back(angles[i]);
    }

    // Comment(zhangtao): The number of records whose mean value has not
    // changed. If it is equal to the number of categories, it means that it has
    // converged.
    int convergence = 0;
    // Comment(zhangtao): Update means
    for (int i = 0; i < num_category; ++i) {
      if (i > len - 1) {
        mean_angles[i] = std::numeric_limits<double>::max();
      }

      const double last_mean_angle = mean_angles[i];

      if (clusters[i].empty()) {
        mean_angles[i] = std::numeric_limits<double>::max();
      } else {
        mean_angles[i] = GetMeanAngle(clusters[i]);
      }

      if (std::abs(NormalizeAngle(last_mean_angle - mean_angles[i])) <
          std::numeric_limits<double>::epsilon())
        convergence++;
    }

    if (convergence == num_category) {
      return clusters;
    }

    iteration++;
  }

  return std::vector<std::vector<double>>();
}

}  // namespace

int CalculateTotalNumObstacles(const ProposedClusters& clusters) {
  return std::accumulate(clusters.begin(), clusters.end(), 0,
                         [](const int init, const ProposedCluster& cluster) {
                           return init + cluster.NumObstacles();
                         });
}

int CalculateTotalNumObstacles(const ProposedClusterPtrs& clusters) {
  return std::accumulate(clusters.begin(), clusters.end(), 0,
                         [](const int init, ProposedClusterPtr cluster) {
                           return init + cluster->NumObstacles();
                         });
}

// Returns two clusters by cuts through along the main axis.
ProposedClusters DivideClusterIntoTwoHalves(const ProposedCluster& cluster) {
  ProposedClusters divided_clusters;
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
  if (!obstacles_1.empty()) {
    ProposedCluster cluster_1 =
        ProposedCluster::InheritFrom(cluster).ConstructBase(obstacles_1);
    cluster_1.set_is_proposed(true);
    cluster_1.set_type(cluster.type());
    cluster_1.set_type_source(cluster.type_source());
    cluster_1.set_score(cluster.score());
    if (cluster.bounding_box()) {
      cluster_1.set_bounding_box(*cluster.bounding_box());
    }
    divided_clusters.push_back(std::move(cluster_1));
  }
  if (!obstacles_2.empty()) {
    ProposedCluster cluster_2 =
        ProposedCluster::InheritFrom(cluster).ConstructBase(obstacles_2);
    cluster_2.set_is_proposed(true);
    cluster_2.set_type(cluster.type());
    cluster_2.set_type_source(cluster.type_source());
    cluster_2.set_score(cluster.score());
    if (cluster.bounding_box()) {
      cluster_2.set_bounding_box(*cluster.bounding_box());
    }
    divided_clusters.push_back(std::move(cluster_2));
  }
  return divided_clusters;
}

ProposedClusters SegmentAndProposeClusterIf(
    const ProposedCluster& cluster,
    const std::function<bool(const ProposedCluster&)>& condition) {
  if (!condition(cluster)) {
    return {cluster};
  }
  ProposedClusters proposed_clusters;
  ProposedClusters unsatisfied_clusters{cluster};
  while (!unsatisfied_clusters.empty()) {
    auto divided_clusters =
        DivideClusterIntoTwoHalves(unsatisfied_clusters.back());
    unsatisfied_clusters.pop_back();

    for (auto& divided_cluster : divided_clusters) {
      if (condition(divided_cluster)) {
        unsatisfied_clusters.push_back(std::move(divided_cluster));
      } else {
        proposed_clusters.push_back(std::move(divided_cluster));
      }
    }
  }

  QCHECK_EQ(CalculateTotalNumObstacles(proposed_clusters),
            cluster.NumObstacles());

  return proposed_clusters;
}

ProposedClusters SegmentAndProposeClusterWithConnectedComponents(
    const ProposedCluster& cluster,
    const obstacle_util::NeighborRange& neighbor_range,
    const obstacle_util::ConnectedCondition& condition) {
  obstacle_util::LocalObstacleGrid local_obstacle_grid(cluster.obstacles());
  std::vector<bool> processed(cluster.NumObstacles(), false);
  std::vector<std::vector<int>> connected_components;
  for (int i = 0; i < cluster.NumObstacles(); ++i) {
    if (processed[i]) continue;
    const auto& obstacle = *(cluster.obstacles()[i]);
    auto raw_indices = local_obstacle_grid.FindConnectedNeighbors(
        obstacle.row, obstacle.col, neighbor_range, condition, processed);
    if (raw_indices.empty()) continue;
    for (const auto index : raw_indices) {
      processed[index] = true;
    }
    connected_components.emplace_back(std::move(raw_indices));
  }

  const int total_num_elems =
      std::accumulate(connected_components.begin(), connected_components.end(),
                      0, [](const int init, const std::vector<int>& component) {
                        return init + component.size();
                      });
  QCHECK_EQ(total_num_elems, cluster.NumObstacles());

  ProposedClusters proposed_clusters;
  for (const auto& component : connected_components) {
    QCHECK(!component.empty());
    ObstaclePtrs obstacles;
    obstacles.reserve(component.size());
    for (const auto& index : component) {
      obstacles.emplace_back(cluster.obstacles()[index]);
    }
    proposed_clusters.emplace_back(
        ProposedCluster::InheritFrom(cluster).ConstructBase(
            std::move(obstacles)));
    proposed_clusters.back().set_is_proposed(true);
    proposed_clusters.back().set_type(cluster.type());
    proposed_clusters.back().set_type_source(cluster.type_source());
    proposed_clusters.back().set_score(cluster.score());
    if (cluster.bounding_box()) {
      proposed_clusters.back().set_bounding_box(*cluster.bounding_box());
    }
  }

  return proposed_clusters;
}

std::optional<ProposedCluster> TrimNoiseObstaclesAndGetRemainingCluster(
    const ProposedCluster& cluster,
    const std::vector<size_t>& noise_obstacle_indices,
    ProposedClusters* proposed_clusters) {
  QCHECK_NOTNULL(proposed_clusters);
  const auto& obstacles = cluster.obstacles();
  std::vector<ObstaclePtr> noise_obstacles, other_obstacles;
  size_t prev_index = 0;
  for (const size_t index : noise_obstacle_indices) {
    noise_obstacles.emplace_back(obstacles[index]);
    other_obstacles.insert(other_obstacles.end(),
                           obstacles.begin() + prev_index,
                           obstacles.begin() + index);
    prev_index = index + 1;
  }
  other_obstacles.insert(other_obstacles.end(), obstacles.begin() + prev_index,
                         obstacles.end());

  if (noise_obstacles.empty()) return cluster;

  QCHECK_EQ(noise_obstacles.size() + other_obstacles.size(), obstacles.size());
  proposed_clusters->emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(noise_obstacles)));
  proposed_clusters->back().set_is_proposed(true);
  proposed_clusters->back().set_property(PP_NOISE);

  if (other_obstacles.empty()) return std::nullopt;

  ProposedCluster proposed_cluster =
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(other_obstacles));
  proposed_cluster.set_is_proposed(true);
  proposed_cluster.set_type(cluster.type());
  proposed_cluster.set_type_source(cluster.type_source());
  proposed_cluster.set_score(cluster.score());
  if (cluster.bounding_box()) {
    proposed_cluster.set_bounding_box(*cluster.bounding_box());
  }

  return proposed_cluster;
}

std::optional<ProposedCluster> TrimNoiseObstaclesAndGetRemainingCluster(
    const ProposedCluster& cluster,
    const absl::flat_hash_set<ObstaclePtr>& noise_obstacle_set,
    ProposedClusters* proposed_clusters) {
  QCHECK_NOTNULL(proposed_clusters);
  const auto& obstacles = cluster.obstacles();
  std::vector<ObstaclePtr> noise_obstacles, other_obstacles;
  for (const auto* obstacle : obstacles) {
    if (ContainsKey(noise_obstacle_set, obstacle)) {
      noise_obstacles.emplace_back(obstacle);
    } else {
      other_obstacles.emplace_back(obstacle);
    }
  }

  if (noise_obstacles.empty()) return cluster;

  QCHECK_EQ(noise_obstacles.size() + other_obstacles.size(), obstacles.size());
  proposed_clusters->emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(noise_obstacles)));
  proposed_clusters->back().set_is_proposed(true);
  proposed_clusters->back().set_property(PP_NOISE);

  if (other_obstacles.empty()) return std::nullopt;

  ProposedCluster proposed_cluster =
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(other_obstacles));
  proposed_cluster.set_is_proposed(true);
  proposed_cluster.set_type(cluster.type());
  proposed_cluster.set_type_source(cluster.type_source());
  proposed_cluster.set_score(cluster.score());
  if (cluster.bounding_box()) {
    proposed_cluster.set_bounding_box(*cluster.bounding_box());
  }

  return proposed_cluster;
}

double ComputeClusterBoundingBoxMeanHeading(const std::vector<double>& angles) {
  QCHECK(!angles.empty());
  if (angles.size() == 1) {
    return angles[0];
  }
  std::vector<std::vector<double>> clusters =
      ComputeKMeanOneDimension(angles, 2);

  if (clusters.empty()) {
    return angles[0];
  } else {
    QCHECK_EQ(clusters.size(), 2);
    QCHECK(!clusters[0].empty() || !clusters[1].empty());
    if (clusters[0].empty()) {
      return GetMeanAngle(clusters[1]);
    }
    if (clusters[1].empty()) {
      return GetMeanAngle(clusters[0]);
    }
    const double mean_angle_1 = GetMeanAngle(clusters[0]);
    const double mean_angle_2 = GetMeanAngle(clusters[1]);
    if (std::abs(NormalizeAngle(mean_angle_1 - mean_angle_2)) < M_PI_2) {
      return GetMeanAngle(angles);
    } else {
      const int size_0 = clusters[0].size();
      const int size_1 = clusters[1].size();
      int min_index = 0;
      if (size_0 > size_1) {
        min_index = 1;
      }

      std::vector<double> new_angles;
      new_angles.reserve(angles.size());
      for (const auto& angle : clusters[min_index]) {
        new_angles.push_back(angle + M_PI);
      }

      // Comment(zhangtao): min_index = 0 or 1
      for (const auto& angle : clusters[1 - min_index]) {
        new_angles.push_back(angle);
      }

      return GetMeanAngle(new_angles);
    }
  }
}

}  // namespace qcraft::segmentation
