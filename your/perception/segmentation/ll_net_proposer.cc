#include "onboard/perception/segmentation/ll_net_proposer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/util.h"
#include "onboard/nets/panonet_config.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/vis/common/color.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

DEFINE_bool(ll_net_proposer_cvs, false, "Render ll net proposer results");

namespace qcraft::segmentation {

using namespace std::placeholders;  // NOLINT

namespace {

constexpr float kMinConePointsRatio = 0.35f;
constexpr float kMinBarrierPointsRatio = 0.80f;
constexpr float kMinObjectPointsRatio = 0.70f;
constexpr float kMinConeUncertainty = 0.5f;
constexpr float kMinBarrierUncertainty = 0.5f;
constexpr float kMinObjectUncertainty = 0.5f;

struct PointSemanticInfo {
  SegmentationType type = ST_DONTCARE;
  float uncertainty = 0.f;  // [0.0, 1.0]
};
using PointSemanticInfos = std::vector<PointSemanticInfo>;

struct SegmentationInfo {
  int num_points = 0;
  float uncertainty = 0.f;  // Average value. [0.0, 1.0]
};
bool operator>(const SegmentationInfo& lhs, const SegmentationInfo& rhs) {
  return std::make_pair(lhs.num_points, lhs.uncertainty) >
         std::make_pair(rhs.num_points, rhs.uncertainty);
}

using SegmentationInfoArray =
    std::array<SegmentationInfo, SegmentationType_ARRAYSIZE>;
using SortedSegmentationInfoArray =
    std::array<std::pair<SegmentationType, SegmentationInfo>,
               SegmentationType_ARRAYSIZE>;

struct ObstacleSemanticInfo {
  SegmentationType type = ST_DONTCARE;
  SegmentationInfoArray segmentation_info_array = {};
};
using ObstacleSemanticInfoPtrs = std::vector<const ObstacleSemanticInfo*>;
using ObstacleSemanticInfoMap =
    absl::flat_hash_map<ObstaclePtr, ObstacleSemanticInfo>;

std::string DebugString(const ObstacleSemanticInfo& semantic_info) {
  std::string str = "Type: ";
  str += SegmentationType_Name(semantic_info.type);
  const auto info_array = semantic_info.segmentation_info_array;
  std::array<std::pair<SegmentationType, SegmentationInfo>,
             SegmentationType_ARRAYSIZE>
      sorted_infos;
  for (int i = 0; i < info_array.size(); ++i) {
    sorted_infos[i] = {static_cast<SegmentationType>(i), info_array[i]};
  }
  std::sort(
      sorted_infos.begin(), sorted_infos.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; });
  for (const auto& [type, info] : sorted_infos) {
    if (info.num_points == 0) break;
    str += absl::StrFormat(" %s: %d %.2f", SegmentationType_Name(type),
                           info.num_points, info.uncertainty);
  }
  return str;
}

const std::unordered_set<CameraId>& GetLlnProposerRefCameras() {
  static const std::unordered_set<CameraId> kLlnProposerRefCameras = {
      CAM_R_FRONT, CAM_FRONT};
  return kLlnProposerRefCameras;
}

bool IsClusterProposable(const ProposedCluster& cluster) {
  return !cluster.IsProposedBy(PT_FIERY_EYE_NET) ||
         !cluster.HasProperty(PP_INBOX);
}

bool IsTargetSegmentationType(const SegmentationType type) {
  return type == ST_TRAFFIC_CONE || type == ST_VEGETATION ||
         type == ST_BARRIER || type == ST_OBJECT;
}

// Returns if the cluster is over-sized. Cluster is over-sized when a cluster is
// too hollow around edges in terms of the whole cluster contour and occupied
// obstacles inside, i.e. not enough obstacles to support a certain edge of the
// cluster. Specifically, we tell if it's oversized by check if the edge centers
// of cluster is too far away from obstacles.
// Currently, we only divide oversized vegetation and barrier clusters promoted
// by llnet.
bool IsOversizedCluster(const ProposedCluster& cluster) {
  const auto cluster_contour = cluster_util::ComputeContour(cluster);
  const auto cluster_edges = cluster_contour.line_segments();
  for (const auto& edge : cluster_edges) {
    float edge_distance_sqr = std::numeric_limits<float>::max();
    for (const auto& obstacle : cluster.obstacles()) {
      edge_distance_sqr =
          std::min(edge_distance_sqr,
                   static_cast<float>(
                       obstacle->coord().DistanceSquareTo(edge.center())));
    }
    // If the center of edges is kMinDistanceSqrToRealObstacle squared distance
    // away from any real obstacles, it's a oversized cluster.
    constexpr float kMinDistanceSqrToRealObstacle =
        Sqr(2.5 * Obstacle::kDiameter);
    if (edge_distance_sqr > kMinDistanceSqrToRealObstacle) {
      return true;
    }
  }
  return false;
}

int CountTotalNumPoints(const SegmentationInfoArray& segmentation_info_array) {
  return std::accumulate(
      segmentation_info_array.begin(), segmentation_info_array.end(), 0,
      [](const int init, const auto& info) { return init + info.num_points; });
}

bool IsParticipant(const SegmentationType type) {
  switch (type) {
    case ST_CAR:
    case ST_XCYCLIST:
    case ST_HUMAN:
      return true;
    case ST_SURFACE:
    case ST_VEGETATION:
    case ST_OBJECT:
    case ST_TRAFFIC_CONE:
    case ST_TRAFFIC_LIGHT:
    case ST_SKY:
    case ST_BARRIER:
    case ST_TRAFFIC_SIGN:
    case ST_TRAFFIC_LANE:
    case ST_SELF:
    case ST_DONTCARE:
      return false;
  }
}

MeasurementType SegmentationTypeToMeasurementType(const SegmentationType type) {
  switch (type) {
    case ST_SURFACE:
      return MT_ROAD;
    case ST_VEGETATION:
      return MT_VEGETATION;
    case ST_OBJECT:
      return MT_STATIC_OBJECT;
    case ST_CAR:
      return MT_VEHICLE;
    case ST_XCYCLIST:
      return MT_CYCLIST;
    case ST_HUMAN:
      return MT_PEDESTRIAN;
    case ST_BARRIER:
      return MT_BARRIER;
    case ST_TRAFFIC_CONE:
      return MT_CONE;
    case ST_TRAFFIC_LIGHT:
    case ST_SKY:
    case ST_TRAFFIC_SIGN:
    case ST_TRAFFIC_LANE:
    case ST_SELF:
    case ST_DONTCARE:
      return MT_UNKNOWN;
  }
}

vis::Color ClusterTypeToColor(const MeasurementType type) {
  switch (type) {
    case MT_UNKNOWN:
      return vis::Color(0.5, 0.5, 0.5, 1.0);  // Dark Gray
    case MT_VEHICLE:
      return vis::Color(1.0, 0.0, 1.0, 1.0);  // Magenta
    case MT_PEDESTRIAN:
      return vis::Color(0.0, 1.0, 1.0, 1.0);  // Cyan
    case MT_CYCLIST:
      return vis::Color(0.0, 0.0, 0.55, 1.0);  // Navy
    case MT_VEGETATION:
      return vis::Color(0.0, 0.5, 0.0, 1.0);  // Green
    case MT_STATIC_OBJECT:
      return vis::Color(1.0, 1.0, 0.0, 1.0);  // Yellow
    case MT_ROAD:
      return vis::Color(0.75, 0.75, 0.75, 1.0);  // Gray
    case MT_CONE:
      return vis::Color(1.0, 0.3, 0.3, 1.0);  // Orange
    case MT_BARRIER:
      return vis::Color(0.0, 0.8, 0.8, 1.0);
    default:
      return vis::Color(0.5, 0.5, 0.5, 1.0);
  }
}

// Render on vantage
void MaybeRenderPointsCvs(
    const std::vector<std::pair<LaserPoint, SegmentationType>>&
        points_with_type) {
  if (FLAGS_ll_net_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/ll_net_proposer_cvs");
    for (const auto& [point, type] : points_with_type) {
      canvas.DrawPoint(
          point.coord(),
          ClusterTypeToColor(SegmentationTypeToMeasurementType(type)), 3);
    }
  }
}
void MaybeRenderObstaclesCvs(
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map,
    const double ground_z) {
  if (FLAGS_ll_net_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/ll_net_proposer_cvs");
    for (const auto& [obstacle, semantic_info] : obstacle_semantic_info_map) {
      vis::Color color = ClusterTypeToColor(
          SegmentationTypeToMeasurementType(semantic_info.type));
      color.a() = 0.5;
      canvas.DrawBox(Vec3d(obstacle->coord(), ground_z), 0.0,
                     {Obstacle::kDiameter, Obstacle::kDiameter},
                     vis::Color(0.0, 0.0, 0.0, 0.0), color);
      canvas.DrawText(DebugString(semantic_info),
                      Vec3d(obstacle->coord(), ground_z + 0.1), 1.0, 0.01,
                      vis::Color::kWhite);
    }
  }
}
void MaybeRenderClusterCvs(const ProposedCluster& cluster,
                           const SegmentationType type,
                           const float uncertainty) {
  if (FLAGS_ll_net_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/ll_net_proposer_cvs");
    const Vec2d centroid = cluster_util::ComputeContour(cluster).centroid();
    canvas.DrawText(
        absl::StrFormat("%s %.2f", SegmentationType_Name(type), uncertainty),
        {centroid, cluster.obstacles()[0]->ground_z}, 1.0, 0.1,
        vis::Color::kWhite);
  }
}

PointSemanticInfo GetPointSemanticInfo(
    const LaserPoint& point,
    const absl::flat_hash_map<LidarId, RangeImage>& range_images) {
  if (range_images.empty()) return {ST_DONTCARE, 0.f};
  const auto* range_image = FindOrNull(range_images, point.lidar_id);
  if (range_image == nullptr) return {ST_DONTCARE, 0.f};
  const auto& semantic_image = range_image->semantic_image();
  const auto [row, col] =
      range_image->ImagePosAt(point.scan_or_point_index, point.beam_index);
  const auto vec2 = semantic_image.at<cv::Vec2b>(row, col);
  return {static_cast<SegmentationType>(vec2[0]), vec2[1] * (1.f / 255.f)};
}

SegmentationInfoArray ComputeObstacleSegmentationInfoArray(
    const PointSemanticInfos& point_semantic_infos) {
  SegmentationInfoArray segmentation_info_array = {};
  for (const auto [type, uncertainty] : point_semantic_infos) {
    segmentation_info_array[static_cast<int>(type)].num_points++;
    segmentation_info_array[static_cast<int>(type)].uncertainty += uncertainty;
  }
  for (auto& info : segmentation_info_array) {
    if (info.num_points) {
      info.uncertainty /= info.num_points;
    }
  }
  return segmentation_info_array;
}

ObstacleSemanticInfo GetObstacleSemanticInfo(
    const PointSemanticInfos& point_semantic_infos) {
  const SegmentationInfoArray segmentation_info_array =
      ComputeObstacleSegmentationInfoArray(point_semantic_infos);
  std::pair<int, SegmentationType> max_num_points_with_type{0, ST_DONTCARE};
  for (int i = 0; i < segmentation_info_array.size(); ++i) {
    const auto& segmentation_info = segmentation_info_array[i];
    if (segmentation_info.num_points > max_num_points_with_type.first) {
      max_num_points_with_type.first = segmentation_info.num_points;
      max_num_points_with_type.second = static_cast<SegmentationType>(i);
    }
  }
  std::pair<int, SegmentationType> second_num_points_with_type{0, ST_DONTCARE};
  for (int i = 0; i < segmentation_info_array.size(); ++i) {
    if (static_cast<SegmentationType>(i) == max_num_points_with_type.second) {
      continue;
    }
    const auto& segmentation_info = segmentation_info_array[i];
    if (segmentation_info.num_points > second_num_points_with_type.first) {
      second_num_points_with_type.first = segmentation_info.num_points;
      second_num_points_with_type.second = static_cast<SegmentationType>(i);
    }
  }
  // NOTE(dong): Semantic results of cone are likely to be polluted by surface.
  if (max_num_points_with_type.second == ST_SURFACE &&
      second_num_points_with_type.second == ST_TRAFFIC_CONE) {
    if (second_num_points_with_type.first * 1.0 >
        point_semantic_infos.size() * kMinConePointsRatio) {
      return {ST_TRAFFIC_CONE, segmentation_info_array};
    }
  }

  return {max_num_points_with_type.second, segmentation_info_array};
}

ObstacleSemanticInfoMap GenerateObstacleSemanticInfoMap(
    const ProposedClusters& clusters, const VehiclePose& pose,
    const absl::flat_hash_map<LidarId, RangeImage>& range_images) {
  const int total_num_obstacles = CalculateTotalNumObstacles(clusters);
  ObstacleSemanticInfoMap obstacle_semantic_info_map;
  obstacle_semantic_info_map.reserve(total_num_obstacles);

  for (const auto& cluster : clusters) {
    for (const auto* obstacle : cluster.obstacles()) {
      PointSemanticInfos point_semantic_infos;
      point_semantic_infos.reserve(obstacle->points.size());
      std::vector<std::pair<LaserPoint, SegmentationType>> points_with_type;
      points_with_type.reserve(obstacle->points.size());
      for (int i = obstacle->above_ground_points_start_index();
           i < obstacle->points.size(); ++i) {
        const auto& point = obstacle->points[i];
        // Only statistic points above ground.
        const auto info = GetPointSemanticInfo(point, range_images);
        point_semantic_infos.emplace_back(info);
        points_with_type.emplace_back(point, info.type);
      }
      const auto semantic_info = GetObstacleSemanticInfo(point_semantic_infos);
      InsertOrDie(&obstacle_semantic_info_map, obstacle, semantic_info);

      MaybeRenderPointsCvs(points_with_type);
    }
  }
  QCHECK_EQ(obstacle_semantic_info_map.size(), total_num_obstacles);
  return obstacle_semantic_info_map;
}

SegmentationInfoArray ComputeClusterSegmentationInfoArray(
    const ProposedCluster& cluster,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map) {
  SegmentationInfoArray segmentation_info_array = {};
  for (const auto* obstacle : cluster.obstacles()) {
    const auto& osi = FindOrDie(obstacle_semantic_info_map, obstacle);
    for (int i = 0; i < osi.segmentation_info_array.size(); ++i) {
      segmentation_info_array[i].num_points +=
          osi.segmentation_info_array[i].num_points;
      segmentation_info_array[i].uncertainty +=
          osi.segmentation_info_array[i].uncertainty *
          osi.segmentation_info_array[i].num_points;
    }
  }
  for (auto& info : segmentation_info_array) {
    if (info.num_points) {
      info.uncertainty /= info.num_points;
    }
  }
  return segmentation_info_array;
}

SortedSegmentationInfoArray ComputeSortedSegmentationInfoArray(
    const SegmentationInfoArray& segmentation_info_array) {
  SortedSegmentationInfoArray sorted_infos;
  for (int i = 0; i < segmentation_info_array.size(); ++i) {
    sorted_infos[i] = {static_cast<SegmentationType>(i),
                       segmentation_info_array[i]};
  }
  std::sort(
      sorted_infos.begin(), sorted_infos.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; });
  return sorted_infos;
}

std::pair<SegmentationType, float> GetClusterSegmentationType(
    const ProposedCluster& cluster,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map) {
  const SegmentationInfoArray segmentation_info_array =
      ComputeClusterSegmentationInfoArray(cluster, obstacle_semantic_info_map);
  const int num_points_above_ground = cluster.NumPointsAboveGround();
  // Total classified points number except ST_DONTCARE. Need to mention that
  // ST_DONTCARE should always be the last enum in SegmentationType.
  const int num_points_on_image = std::accumulate(
      segmentation_info_array.begin(), segmentation_info_array.end() - 1, 0,
      [](const int init, const auto& info) { return init + info.num_points; });
  QCHECK_LE(CountTotalNumPoints(segmentation_info_array),
            num_points_above_ground);
  constexpr float kMinOnImagePointsRatio = 0.5f;
  const bool is_seen_mostly = num_points_on_image * 1.f >
                              num_points_above_ground * kMinOnImagePointsRatio;
  const SortedSegmentationInfoArray sorted_infos =
      ComputeSortedSegmentationInfoArray(segmentation_info_array);
  // NOTE(dong): Semantic results of cone are likely to be polluted by surface.
  if (sorted_infos[0].first == ST_TRAFFIC_CONE ||
      (sorted_infos[0].first == ST_SURFACE &&
       sorted_infos[1].first == ST_TRAFFIC_CONE)) {
    if (sorted_infos[1].second.num_points >
            num_points_on_image * kMinConePointsRatio &&
        sorted_infos[1].second.uncertainty > kMinConeUncertainty) {
      if (is_seen_mostly) {
        return {ST_TRAFFIC_CONE, sorted_infos[1].second.uncertainty};
      }
    }
  }
  if (sorted_infos[0].first == ST_BARRIER) {
    if (sorted_infos[0].second.num_points >
            num_points_on_image * kMinBarrierPointsRatio &&
        sorted_infos[0].second.uncertainty > kMinBarrierUncertainty) {
      if (is_seen_mostly) {
        return {ST_BARRIER, sorted_infos[0].second.uncertainty};
      }
    }
  }
  if (sorted_infos[0].first == ST_OBJECT) {
    if (sorted_infos[0].second.num_points >
            num_points_on_image * kMinObjectPointsRatio &&
        sorted_infos[0].second.uncertainty > kMinObjectUncertainty) {
      if (is_seen_mostly) {
        return {ST_OBJECT, sorted_infos[0].second.uncertainty};
      }
    }
  }

  constexpr float kMinClassifiedPointsRatio = 0.5f;
  constexpr float kMinClassifiedUncertainty = 0.6f;
  const bool is_classified =
      sorted_infos[0].second.num_points >
          num_points_on_image * kMinClassifiedPointsRatio &&
      sorted_infos[0].second.uncertainty > kMinClassifiedUncertainty;
  return is_classified && is_seen_mostly
             ? std::make_pair(sorted_infos[0].first,
                              sorted_infos[0].second.uncertainty)
             : std::make_pair(ST_DONTCARE, 1.f);
}
// TODO(dong): find obstacles that occluded by others which may have a
// wrong obstacle type.
void ResetOccludedObstaclesType() {}
// NOTO(dong): Define tolerance between two obstacles while doing euclidean
// cluster. The uint of tolerance is currently defined in obstacle grid
// coordinate. By now, range and type are two features used to calculate a
// relevant tolerance.
// This function has the most important influence in segmentation result. Need
// to continuously improve in the follow-up process.
bool SatisfyEuclideanClusterTolerance(
    const Obstacle& obstacle_1, const Obstacle& obstacle_2,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map) {
  const float range = Hypot<float>(obstacle_1.x, obstacle_1.y);
  const int dist_of_grid = std::max(std::abs(obstacle_1.col - obstacle_2.col),
                                    std::abs(obstacle_1.row - obstacle_2.row));

  const auto info_a = FindOrDie(obstacle_semantic_info_map, &obstacle_1);
  const auto info_b = FindOrDie(obstacle_semantic_info_map, &obstacle_2);

  const float uncertainty_a =
      info_a.segmentation_info_array[static_cast<int>(info_a.type)].uncertainty;
  const float uncertainty_b =
      info_b.segmentation_info_array[static_cast<int>(info_b.type)].uncertainty;

  constexpr float kMinObstacleUncertainty = 0.5;
  if (info_a.type == info_b.type && (uncertainty_a > kMinObstacleUncertainty &&
                                     uncertainty_b > kMinObstacleUncertainty)) {
    const int max_grid_dist = std::clamp(CeilToInt(range * (1.f / 15.f)), 1, 5);
    return dist_of_grid <= max_grid_dist;
  }

  if (info_a.type == ST_TRAFFIC_CONE || info_b.type == ST_TRAFFIC_CONE) {
    return false;
  }

  return dist_of_grid <= 1;
}
// Build proposed cluster from input cluster & clustered obstacle indices.
ProposedCluster GenerateProposedClusterFromObstacleIndices(
    const ProposedCluster& cluster, const std::vector<int>& obstacle_indices) {
  ObstaclePtrs current_cluster_obstacles;
  current_cluster_obstacles.reserve(obstacle_indices.size());
  for (const int index : obstacle_indices) {
    current_cluster_obstacles.emplace_back(cluster.obstacles()[index]);
  }
  ProposedCluster proposed_cluster =
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(current_cluster_obstacles));
  proposed_cluster.set_type(cluster.type());
  proposed_cluster.set_type_source(cluster.type_source());
  proposed_cluster.set_score(cluster.score());
  QCHECK(!cluster.bounding_box());
  return proposed_cluster;
}

std::vector<bool> SegmentOutConeClusters(
    const ProposedCluster& cluster,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map,
    const obstacle_util::LocalObstacleGrid& local_obstacle_grid,
    const std::vector<bool>& have_processed,
    ProposedClusters* proposed_clusters) {
  QCHECK_NOTNULL(proposed_clusters);
  const auto& obstacles = cluster.obstacles();
  std::vector<bool> processed = have_processed;
  for (int i = 0; i < obstacles.size(); ++i) {
    if (processed[i]) continue;
    const auto& info = FindOrDie(obstacle_semantic_info_map, obstacles[i]);
    if (info.type != ST_TRAFFIC_CONE) continue;
    std::vector<int> obstacle_indices;
    obstacle_indices.push_back(i);
    processed[i] = true;
    int coi = 0;  // current obstacle index
    while (coi < obstacle_indices.size()) {
      const auto* obstacle = obstacles[obstacle_indices[coi]];
      const auto neighbor_indices = local_obstacle_grid.FindNearestInRadius(
          obstacle->row, obstacle->col, 1);
      for (const auto neighbor_index : neighbor_indices) {
        if (processed[neighbor_index]) continue;
        const auto* neighbor_obstacle = obstacles[neighbor_index];
        const auto& obs_info = FindOrDie(obstacle_semantic_info_map, obstacle);
        const auto& neighbor_info =
            FindOrDie(obstacle_semantic_info_map, neighbor_obstacle);
        if (obs_info.type == ST_TRAFFIC_CONE ||
            neighbor_info.type == ST_TRAFFIC_CONE) {
          obstacle_indices.push_back(neighbor_index);
          processed[neighbor_index] = true;
        }
      }
      ++coi;
    }
    proposed_clusters->emplace_back(
        GenerateProposedClusterFromObstacleIndices(cluster, obstacle_indices));
  }
  return processed;
}

std::vector<bool> SegmentOutOtherClusters(
    const ProposedCluster& cluster, const VehiclePose& pose,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map,
    const obstacle_util::LocalObstacleGrid& local_obstacle_grid,
    const std::vector<bool>& have_processed,
    ProposedClusters* proposed_clusters) {
  QCHECK_NOTNULL(proposed_clusters);
  const auto& obstacles = cluster.obstacles();
  std::vector<bool> processed = have_processed;
  auto get_max_neighbor_radius = [&pose](const Obstacle& obstacle) {
    const float range = Hypot(obstacle.x - pose.x, obstacle.y - pose.y);
    const int radius = std::clamp(CeilToInt(range * (1.f / 10.0f)), 1, 5);
    return radius;
  };
  for (int i = 0; i < obstacles.size(); ++i) {
    if (processed[i]) continue;
    std::vector<int> obstacle_indices;
    obstacle_indices.push_back(i);
    processed[i] = true;
    int coi = 0;  // current obstacle index
    while (coi < obstacle_indices.size()) {
      const auto* obstacle = obstacles[obstacle_indices[coi]];
      const auto neighbor_indices = local_obstacle_grid.FindNearestInRadius(
          obstacle->row, obstacle->col, get_max_neighbor_radius(*obstacle));
      for (const auto neighbor_index : neighbor_indices) {
        if (processed[neighbor_index]) continue;
        const auto* neighbor_obstacle = obstacles[neighbor_index];
        if (SatisfyEuclideanClusterTolerance(*obstacle, *neighbor_obstacle,
                                             obstacle_semantic_info_map)) {
          obstacle_indices.push_back(neighbor_index);
          processed[neighbor_index] = true;
        }
      }
      ++coi;
    }
    proposed_clusters->emplace_back(
        GenerateProposedClusterFromObstacleIndices(cluster, obstacle_indices));
  }
  return processed;
}
// NOTO(dong): Use a conditional euclidean cluster method to segment
// original cluster.
ProposedClusters SegmentClusterWithEuclidean(
    const ProposedCluster& cluster, const VehiclePose& pose,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map) {
  const auto& obstacles = cluster.obstacles();
  obstacle_util::LocalObstacleGrid local_obstacle_grid(obstacles);
  ProposedClusters proposed_clusters;
  std::vector<bool> processed(obstacles.size(), false);
  // Deal with cones.
  processed = SegmentOutConeClusters(cluster, obstacle_semantic_info_map,
                                     local_obstacle_grid, processed,
                                     &proposed_clusters);
  // Deal with other objects.
  processed = SegmentOutOtherClusters(cluster, pose, obstacle_semantic_info_map,
                                      local_obstacle_grid, processed,
                                      &proposed_clusters);
  // All obstacles should be processed.
  QCHECK(std::all_of(processed.begin(), processed.end(),
                     [](const auto flag) { return flag == true; }));

  return proposed_clusters;
}

ProposedClusters SegmentOversizedVegetationAndBarrierClusters(
    const ProposedClusters& clusters) {
  ProposedClusters proposed_clusters;
  for (const auto& cluster : clusters) {
    if (!cluster.is_proposed()) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    if (cluster.type() != MT_VEGETATION && cluster.type() != MT_BARRIER) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    auto segmented_clusters =
        SegmentAndProposeClusterIf(cluster, std::bind(IsOversizedCluster, _1));
    for (auto& segmented_cluster : segmented_clusters) {
      proposed_clusters.push_back(std::move(segmented_cluster));
    }
  }
  return proposed_clusters;
}

ProposedClusters DivideClusterByLLNet(
    const ProposedCluster& cluster, const VehiclePose& pose,
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map) {
  int num_classified_obstacles = 0;
  for (const auto* obstacle : cluster.obstacles()) {
    const auto type = FindOrDie(obstacle_semantic_info_map, obstacle).type;
    if (IsTargetSegmentationType(type)) {
      ++num_classified_obstacles;
    }
  }
  constexpr double kMinTargetCategoryObstaclesRatio = 0.25;
  const bool has_enough_classified_obstacles =
      num_classified_obstacles * 1.0 >=
      cluster.obstacles().size() * kMinTargetCategoryObstaclesRatio;

  if (!has_enough_classified_obstacles) return {};

  ProposedClusters proposed_clusters =
      SegmentClusterWithEuclidean(cluster, pose, obstacle_semantic_info_map);
  bool clusters_are_proposed = false;
  for (auto& cluster : proposed_clusters) {
    const auto [type, uncertainty] =
        GetClusterSegmentationType(cluster, obstacle_semantic_info_map);
    if (IsTargetSegmentationType(type)) {
      cluster.set_type(SegmentationTypeToMeasurementType(type));
      cluster.set_type_source(MTS_LL_NET);
      clusters_are_proposed = true;
    }
    MaybeRenderClusterCvs(cluster, type, uncertainty);
  }
  if (clusters_are_proposed) {
    for (auto& cluster : proposed_clusters) {
      cluster.set_is_proposed(true);
    }
  }

  return SegmentOversizedVegetationAndBarrierClusters(proposed_clusters);
}
// NOTE(dong): Unknown objects located in the vegetation zone will be classified
// as vegetation. This may result in security risk because that planner may
// regard vegetation objects as soft objects and ignore them. Here we recovey
// these wrong classified vegetation objects using llnet results.
void RecoverClusterType(
    const ObstacleSemanticInfoMap& obstacle_semantic_info_map,
    ProposedCluster* cluster) {
  QCHECK_NOTNULL(cluster);
  const SegmentationInfoArray segmentation_info_array =
      ComputeClusterSegmentationInfoArray(*cluster, obstacle_semantic_info_map);
  const int num_points_above_ground = cluster->NumPointsAboveGround();
  // Total classified points number except ST_DONTCARE. Need to mention that
  // ST_DONTCARE should always be the last enum in SegmentationType.
  const int num_points_on_image = std::accumulate(
      segmentation_info_array.begin(), segmentation_info_array.end() - 1, 0,
      [](const int init, const auto& info) { return init + info.num_points; });
  QCHECK_LE(CountTotalNumPoints(segmentation_info_array),
            num_points_above_ground);
  constexpr float kMinOnImagePointsRatio = 0.5f;
  const bool is_seen_mostly = num_points_on_image * 1.f >
                              num_points_above_ground * kMinOnImagePointsRatio;

  const SortedSegmentationInfoArray sorted_infos =
      ComputeSortedSegmentationInfoArray(segmentation_info_array);

  constexpr float kMinRecoverVegetationNumPointsRatio = 0.5f;
  constexpr float kMinRecoverVegetationUncertainty = 0.5f;
  // Recover Cluster In VegetationZone
  if (cluster->IsProposedBy(PT_SEMANTIC_MAP) &&
      cluster->type() == MT_VEGETATION) {
    const auto& [first_type, segmentation_info] = sorted_infos[0];
    if (first_type != ST_VEGETATION && is_seen_mostly &&
        segmentation_info.num_points >
            num_points_on_image * kMinRecoverVegetationNumPointsRatio &&
        segmentation_info.uncertainty > kMinRecoverVegetationUncertainty) {
      cluster->set_type(MT_UNKNOWN);
      cluster->set_type_source(MTS_LL_NET);
      cluster->set_is_proposed(true);
    }
  }
}

}  // namespace

ProposedClusters LLNetProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  SCOPED_QTRACE("LLNetProposer::Propose");
  if (env_info.context().ll_net_result.empty()) return clusters;

  const auto& pose = env_info.pose();
  const auto& range_images = env_info.context().range_images;

  ProposedClusters proposed_clusters;

  const auto& obstacle_semantic_info_map =
      GenerateObstacleSemanticInfoMap(clusters, pose, range_images);

  for (const auto& cluster : clusters) {
    if (!IsClusterProposable(cluster)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    auto divided_clusters =
        DivideClusterByLLNet(cluster, pose, obstacle_semantic_info_map);
    if (!divided_clusters.empty()) {
      for (auto& divided_cluster : divided_clusters) {
        proposed_clusters.push_back(std::move(divided_cluster));
      }
    } else {
      proposed_clusters.push_back(cluster);
    }
  }

  for (auto& cluster : proposed_clusters) {
    RecoverClusterType(obstacle_semantic_info_map, &cluster);
  }

  // No obstacle is trimed.
  QCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  MaybeRenderObstaclesCvs(obstacle_semantic_info_map, pose.z);

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
