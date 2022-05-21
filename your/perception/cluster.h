#ifndef ONBOARD_PERCEPTION_CLUSTER_H_
#define ONBOARD_PERCEPTION_CLUSTER_H_

#include <algorithm>
#include <cfloat>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/perception/obstacle.h"
#include "onboard/proto/perception.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace qcraft {

class Cluster {
 public:
  Cluster() = default;
  explicit Cluster(ObstaclePtrs obstacles);

  void set_type(MeasurementType type) { type_ = type; }
  MeasurementType type() const { return type_; }

  void set_type_source(MeasurementTypeSource type_source) {
    type_source_ = type_source;
  }
  MeasurementTypeSource type_source() const { return type_source_; }

  void set_score(float score) { score_ = score; }
  float score() const { return score_; }

  double timestamp() const { return timestamp_; }
  void set_timestamp(double timestamp) { timestamp_ = timestamp; }

  Vec3d ComputeCentroidFromPoints() const;
  Vec2d ComputeCentroidFromObstacles() const;

  const ObstaclePtrs& obstacles() const { return obstacles_; }
  ObstaclePtrs* mutable_obstacles() { return &obstacles_; }
  int NumObstacles() const { return obstacles_.size(); }

  bool HasObstacle(const Obstacle* obstacle) {
    return std::find(obstacles_.begin(), obstacles_.end(), obstacle) !=
           obstacles_.end();
  }

  // For iterating obstacles.
  ObstaclePtrs::const_iterator begin() const { return obstacles_.begin(); }
  ObstaclePtrs::const_iterator end() const { return obstacles_.end(); }

  int NumPoints() const {
    int num_points = 0;
    for (const auto* obstacle : obstacles_) {
      num_points += obstacle->points.size();
    }
    return num_points;
  }

  int MainLidarNumPoints() const {
    int num_points = 0;
    for (const auto* obstacle : obstacles_) {
      for (const auto& point : obstacle->points) {
        switch (point.lidar_id) {
          case LDR_CENTER:
          case LDR_FRONT_LEFT:
          case LDR_FRONT_RIGHT:
          case LDR_FRONT:
          case LDR_REAR:
          case LDR_REAR_LEFT:
          case LDR_REAR_RIGHT:
            num_points++;
            break;
          default:
            continue;
        }
      }
    }
    return num_points;
  }

  int NumPointsAboveGround() const {
    int num_points_above_ground = 0;
    for (const auto* obstacle : obstacles_) {
      num_points_above_ground += obstacle->num_points_above_ground;
    }
    return num_points_above_ground;
  }

  float ComputeHeight() const {
    float height = 0.0f;
    for (const auto* obstacle : obstacles_) {
      height = std::max(height, obstacle->height());
    }
    return height;
  }

  float ComputeClearance() const {
    QCHECK(!obstacles_.empty());
    return (*std::min_element(obstacles_.begin(), obstacles_.end(),
                              [](const auto* a, const auto* b) {
                                return a->clearance < b->clearance;
                              }))
        ->clearance;
  }

  float ComputeGroundZ() const {
    QCHECK(!obstacles_.empty());
    const float sum =
        std::accumulate(obstacles_.begin(), obstacles_.end(), 0.f,
                        [](const float init, const auto* obstacle) {
                          return init + obstacle->ground_z;
                        });
    return sum / obstacles_.size();
  }

  float ComputeMaxZ() const {
    QCHECK(!obstacles_.empty());
    return (*std::max_element(obstacles_.begin(), obstacles_.end(),
                              [](const auto* lhs, const auto* rhs) {
                                return lhs->max_z < rhs->max_z;
                              }))
        ->max_z;
  }

  float ComputeMinZ() const {
    QCHECK(!obstacles_.empty());
    return (*std::min_element(obstacles_.begin(), obstacles_.end(),
                              [](const auto* lhs, const auto* rhs) {
                                return lhs->min_z < rhs->min_z;
                              }))
        ->min_z;
  }

  Vec2d icp_velocity() const { return icp_velocity_; }

  const Eigen::Matrix2d& icp_velocity_covariance() const {
    return icp_velocity_covariance_;
  }

  void set_icp_velocity(Vec2d velocity) {
    icp_matched_ = true;
    icp_velocity_ = velocity;
  }

  void set_icp_velocity_and_covariance(const Vec2d& velocity,
                                       const Eigen::Matrix2d& covariance) {
    icp_matched_ = true;
    icp_velocity_ = velocity;
    icp_velocity_covariance_ = covariance;
  }

  bool icp_matched() const { return icp_matched_; }

  double icp_mse() const { return icp_mse_; }
  void set_icp_mse(double icp_mse) { icp_mse_ = icp_mse; }

  double matched_prev_cluster_id() const { return matched_prev_cluster_id_; }
  void set_matched_prev_cluster_id(double matched_prev_cluster_id) {
    matched_prev_cluster_id_ = matched_prev_cluster_id;
  }

  double matched_prev_cluster_timestamp() const {
    return matched_prev_cluster_timestamp_;
  }
  void set_matched_prev_cluster_timestamp(
      double matched_prev_cluster_timestamp) {
    matched_prev_cluster_timestamp_ = matched_prev_cluster_timestamp;
  }

  const std::optional<Box2d>& bounding_box() const { return bounding_box_; }
  void set_bounding_box(const Box2d& bounding_box) {
    bounding_box_ = bounding_box;
  }

  const std::optional<Vec2d>& fen_velocity() const { return fen_velocity_; }
  void set_fen_velocity(const Vec2d& fen_velocity) {
    fen_velocity_ = std::make_optional(fen_velocity);
  }

  bool IsStaticType() const;

  ObservationState observation_state() const { return observation_state_; }
  void set_observation_state(ObservationState observation_state) {
    observation_state_ = observation_state;
  }

 protected:
  // Initialized by average obstacle timestamp in constructor.
  double timestamp_ = 0.0;
  MeasurementType type_ = MT_UNKNOWN;
  MeasurementTypeSource type_source_ = MTS_DEFAULT;
  ObstaclePtrs obstacles_;
  std::optional<Box2d> bounding_box_;  // Optional bounding box
  // TODO(zheng): Modify it to icp_velocity_.
  // Velocity from ICP. Will move it to measurement in the future.
  Vec2d icp_velocity_;
  Eigen::Matrix2d icp_velocity_covariance_ = Eigen::Matrix2d::Identity() * 1e8;
  // from FEN.
  float score_ = -1.0;
  bool icp_matched_ = false;
  double icp_mse_ = std::numeric_limits<double>::max();
  int matched_prev_cluster_id_ = -1;
  double matched_prev_cluster_timestamp_ = 0.0;
  std::optional<Vec2d> fen_velocity_;
  ObservationState observation_state_ = OS_UNKNOWN;
};

class SegmentedCluster : public Cluster {
 public:
  // Bias cloud consists of a biased cloud and a bias to avoid 'float' overflow
  // in registraion. You can get original cloud, which is under world
  // coordination system by adding bias back to points in biased cloud.
  using BiasCloud = std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, Vec3d>;

  SegmentedCluster() = default;
  SegmentedCluster(int id, const Cluster& cluster)
      : Cluster(cluster), id_(id) {}

  int id() const { return id_; }

  SegmentedCluster Clone() const { return *this; }

  void SaveBiasCloud(const BiasCloud& bias_cloud, const bool has_normal) {
    bias_cloud_ = bias_cloud;
    has_normal_ = has_normal;
  }

  const BiasCloud& GetBiasCloud() const { return bias_cloud_; }

  // Origin_cloud = bias_cloud + bias
  pcl::PointCloud<pcl::PointNormal>::Ptr GetOriginCloud() const {
    if (!HasCloud()) return nullptr;
    pcl::PointCloud<pcl::PointNormal>::Ptr origin_cloud(
        new pcl::PointCloud<pcl::PointNormal>(*(bias_cloud_.first)));
    const auto& bias = bias_cloud_.second;
    for (auto& pt : origin_cloud->points) {
      pt.x += bias.x();
      pt.y += bias.y();
      pt.z += bias.z();
    }
    return origin_cloud;
  }

  bool HasCloud() const { return bias_cloud_.first != nullptr; }

  bool HasNormal() const { return has_normal_; }

 private:
  int id_ = -1;

  bool has_normal_ = false;
  BiasCloud bias_cloud_ = {nullptr, {0, 0, 0}};
};

// Unlike Cluster, this class contains copies of obstacles.
class ClusterWithObstacles final : public SegmentedCluster {
 public:
  ClusterWithObstacles() {}
  explicit ClusterWithObstacles(const SegmentedCluster& cluster)
      : SegmentedCluster(cluster) {
    // Deep copy of obstacles and bbox.
    obstacles_.clear();
    const auto& obstacle_ptrs = cluster.obstacles();
    obstacles_.reserve(obstacle_ptrs.size());
    obstacle_copies_.reserve(obstacle_ptrs.size());
    for (const auto* obstacle : obstacle_ptrs) {
      obstacle_copies_.push_back(*obstacle);
      obstacles_.push_back(&obstacle_copies_.back());
    }
    if (cluster.bounding_box()) {
      this->set_bounding_box(*cluster.bounding_box());
    }
  }

 private:
  std::vector<Obstacle> obstacle_copies_;
};

using SegmentedClusters = std::vector<SegmentedCluster>;
using ClusterRef = std::shared_ptr<Cluster>;
using ClusterVector = std::vector<Cluster>;

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_CLUSTER_H_
