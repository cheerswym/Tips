#include "onboard/perception/segmentation/cateye_proposer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/segmentation/proposer_util.h"

DEFINE_bool(cateye_proposer_cvs, false, "Enable cateye proposer cvs.");
DEFINE_bool(generate_stop_line_from_lane_boundaries, true,
            "Generate stop line from end points of left and right boundary. "
            "This may cause some fps about stop lines.");

constexpr double kMaxClusterArea = 2.0;
constexpr int kRetroreflectorMinIntensity = 250;
constexpr int kMinNumRetroreflectedPoints = 2;
constexpr int kMaybeRetroreflectorMinIntensity = 180;
constexpr int kMinNumMaybeRetroreflectedPoints = 5;
constexpr double kMaxDisCateyePointToGround = 0.2;
constexpr double kMaxDistanceToBoundary = 0.3;
constexpr double kMaxDistanceToStopLine = 0.8;
constexpr double kStopLineWidth = 1.2;
constexpr double kFindLaneBoundariesRadius = 2.5;
constexpr double kLaneWidth = 3.5;
constexpr double kLaneWidthOffset = 0.5;

namespace qcraft::segmentation {
namespace {

using namespace mapping;  // NOLINT

bool MaybeCateyeNoiseObstacle(const Obstacle& obstacle,
                              const ObstacleManager& obstacle_manager) {
  int num_retroreflected_points = 0;
  int num_maybe_retroreflected_points = 0;
  for (int row_offset = -2; row_offset <= 2; ++row_offset) {
    for (int col_offset = -2; col_offset <= 2; ++col_offset) {
      const auto* obs = obstacle_manager.ObstacleAt(obstacle.row + row_offset,
                                                    obstacle.col + col_offset);
      if (nullptr == obs) continue;
      for (const auto& point : obs->points) {
        if (std::abs(point.z - obs->ground_z) < kMaxDisCateyePointToGround) {
          if (point.intensity >= kRetroreflectorMinIntensity) {
            if (++num_retroreflected_points >= kMinNumRetroreflectedPoints) {
              return true;
            }
          }
          if (point.intensity >= kMaybeRetroreflectorMinIntensity) {
            if (++num_maybe_retroreflected_points >=
                kMinNumMaybeRetroreflectedPoints) {
              return true;
            }
          }
        }
      }
    }
  }

  return false;
}

std::vector<Segment2d> GetSegmentsFromPoints(const std::vector<Vec2d>& points) {
  if (points.size() < 2) return {};
  std::vector<Segment2d> segments;
  segments.reserve(points.size() - 1);
  for (int i = 1; i < points.size(); ++i) {
    segments.emplace_back(points[i - 1], points[i]);
  }
  return segments;
}

std::vector<double> CalculateSignedDistancesToLaneBoundary(
    const std::vector<Vec2d>& points,
    const LaneBoundaryInfo& lane_boundary_info) {
  std::vector<Segment2d> segments =
      GetSegmentsFromPoints(lane_boundary_info.points_smooth);
  std::vector<double> distances;
  distances.reserve(points.size());
  for (const auto& point : points) {
    double min_dis = std::numeric_limits<double>::max();
    for (const auto& segment : segments) {
      double signed_dis = segment.SignedDistanceTo(point);
      if (std::abs(signed_dis) < std::abs(min_dis)) {
        min_dis = signed_dis;
      }
    }
    distances.emplace_back(min_dis);
  }

  return distances;
}

double CalculateSignedDistanceToLaneBoundary(
    const Vec2d& point, const LaneBoundaryInfo& lane_boundary_info) {
  return CalculateSignedDistancesToLaneBoundary({point}, lane_boundary_info)[0];
}

std::vector<double> CalculateDistancesToStopLine(
    const std::vector<Vec2d>& points,
    const SemanticMapManager& semantic_map_manager, const VehiclePose& pose) {
  const auto* lane_info = semantic_map_manager.GetNearestLaneInfoAtLevel(
      semantic_map_manager.GetLevel(), pose.coord2d());
  if (!lane_info) return {};
  std::optional<LaneInfo> next_lane_info;
  for (const auto outgoing_lane_index : lane_info->outgoing_lane_indices) {
    const auto& outgoing_lane_info =
        semantic_map_manager.lane_info()[outgoing_lane_index];
    if (!outgoing_lane_info.proto->startpoint_associated_traffic_lights()
             .empty()) {
      next_lane_info = outgoing_lane_info;
      break;
    }
  }
  if (!next_lane_info || next_lane_info->points_smooth.size() < 2) return {};
  // Next lane norm to stop line.
  const Vec2d& p0 = next_lane_info->points_smooth[0];
  const Vec2d& p1 = next_lane_info->points_smooth[1];
  const Vec2d& diff = p1 - p0;
  if (diff.norm() == 0) return {};
  const Vec2d& norm = diff / diff.norm();
  Segment2d stop_line(norm.FastRotate(M_PI_2) * (kStopLineWidth / 2) + p0,
                      norm.FastRotate(-M_PI_2) * (kStopLineWidth / 2) + p0);
  std::vector<double> distances;
  distances.reserve(points.size());
  for (const auto& point : points) {
    distances.emplace_back(stop_line.DistanceTo(point));
  }
  if (FLAGS_cateye_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/cateye_proposer_cvs");
    canvas.DrawLine(Vec3d(stop_line.start(), pose.z),
                    Vec3d(stop_line.end(), pose.z), vis::Color::kYellow, 5);
    std::vector<Vec3d> vertices;
    vertices.reserve(next_lane_info->points_smooth.size());
    for (const auto& p : next_lane_info->points_smooth) {
      vertices.emplace_back(p.x(), p.y(), pose.z);
    }
    canvas.DrawLineStrip(vertices, vis::Color::kYellow, 2);
  }

  return distances;
}

bool GetLeftAndRightLaneBoundaries(
    const SemanticMapManager& semantic_map_manager, const VehiclePose& pose,
    const LaneBoundaryInfo** left_boundary_info,
    const LaneBoundaryInfo** right_boundary_info) {
  const auto lane_boundaries_info =
      semantic_map_manager.GetLaneBoundariesInfoAtLevel(
          semantic_map_manager.GetLevel(), pose.coord2d(),
          kFindLaneBoundariesRadius);
  if (lane_boundaries_info.size() < 2) return false;
  double left_boundary_dis = std::numeric_limits<double>::max();
  double right_boundary_dis = std::numeric_limits<double>::lowest();
  for (const auto* lane_boundary_info : lane_boundaries_info) {
    double signed_dis = CalculateSignedDistanceToLaneBoundary(
        pose.coord2d(), *lane_boundary_info);
    if (signed_dis > 0 && signed_dis < left_boundary_dis) {
      left_boundary_dis = signed_dis;
      *left_boundary_info = lane_boundary_info;
    } else if (signed_dis < 0 && signed_dis > right_boundary_dis) {
      right_boundary_dis = signed_dis;
      *right_boundary_info = lane_boundary_info;
    }
  }
  if (!(*left_boundary_info) || !(*right_boundary_info)) return false;
  if (left_boundary_dis - right_boundary_dis > kLaneWidth + kLaneWidthOffset ||
      left_boundary_dis - right_boundary_dis < kLaneWidth - kLaneWidthOffset) {
    return false;
  }

  if (FLAGS_cateye_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/cateye_proposer_cvs");
    std::vector<Vec3d> left_vertices, right_vertices;
    for (const auto& p : (*left_boundary_info)->points_smooth) {
      left_vertices.emplace_back(p.x(), p.y(), pose.z);
    }
    canvas.DrawLineStrip(left_vertices, vis::Color::kGreen, 2);
    for (const auto& p : (*right_boundary_info)->points_smooth) {
      right_vertices.emplace_back(p.x(), p.y(), pose.z);
    }
    canvas.DrawLineStrip(right_vertices, vis::Color::kGreen, 2);
  }

  return true;
}
// NOTE(dong): We consider a possible stop line could be the segment between two
// end points from nearest left and right boundary.
std::optional<Segment2d> GetStopLineFromLeftAndRightLaneBoundaries(
    const LaneBoundaryInfo& left_boundary_info,
    const LaneBoundaryInfo& right_boundary_info,
    const SemanticMapManager& semantic_map_manager, const VehiclePose& pose) {
  const auto* lane_info = semantic_map_manager.GetNearestLaneInfoAtLevel(
      semantic_map_manager.GetLevel(), pose.coord2d());
  if (!lane_info || lane_info->points_smooth.size() < 2) return std::nullopt;
  if (left_boundary_info.points_smooth.empty() ||
      right_boundary_info.points_smooth.empty()) {
    return std::nullopt;
  }
  const Vec2d& left_end_point = left_boundary_info.points_smooth.back();
  const Vec2d& right_end_point = right_boundary_info.points_smooth.back();
  const Vec2d& direction = right_end_point - left_end_point;
  if (std::abs(direction.norm() - kLaneWidth) > kLaneWidthOffset) {
    return std::nullopt;
  }
  const Vec2d& p0 =
      lane_info->points_smooth[lane_info->points_smooth.size() - 1];
  const Vec2d& p1 =
      lane_info->points_smooth[lane_info->points_smooth.size() - 2];
  const Vec2d& vec = p1 - p0;
  double angle =
      std::acos(direction.dot(vec) / (direction.norm() * vec.norm()));
  if (std::abs(angle - M_PI_2) < 0.2) {  // about 11.5 degree
    const Vec2d normalized_direction = direction / direction.norm();
    const Vec2d mid_point = (right_end_point + left_end_point) / 2;
    const Vec2d right_point =
        (normalized_direction * kStopLineWidth / 2) + mid_point;
    const Vec2d left_point =
        (-normalized_direction * kStopLineWidth / 2) + mid_point;
    return Segment2d(left_point, right_point);
  }

  return std::nullopt;
}

// NOTE(dong): If an obstacle keeps all the following rules, it is regarded as a
// cateye noise obstacle and will be filtered.
// 1. It is not promoted by FEN or LLN.
// 2. It is close to the nearest left/right lane boundaries or a stop line.
// 3. It has enough high reflectivity points very close to the ground.
// PS: A stop line is a segment norm to a lane that has
// startpoint_associated_traffic_lights.
ProposedClusters MaybeContainCateyeNoise(const ProposerEnvInfo& env_info,
                                         const ProposedCluster& cluster) {
  const auto& pose = env_info.pose();
  const auto& obstacle_manager = env_info.obstacle_manager();
  const auto& semantic_map_manager = env_info.semantic_map_manager();

  if (cluster.type_source() == MTS_SEMANTIC_MAP_ZONE) return {};
  if (cluster.type() != MT_UNKNOWN) return {};
  if (cluster.bounding_box()) return {};

  const auto& contour = cluster_util::ComputeContour(cluster);
  if (contour.area() > kMaxClusterArea) return {};
  absl::flat_hash_set<ObstaclePtr> maybe_noise_obstacles;
  maybe_noise_obstacles.reserve(cluster.obstacles().size());
  bool maybe_has_noise_obstacles = false;
  for (const auto* obstacle : cluster.obstacles()) {
    if (MaybeCateyeNoiseObstacle(*obstacle, obstacle_manager)) {
      maybe_noise_obstacles.insert(obstacle);
      maybe_has_noise_obstacles = true;
    }
  }
  if (!maybe_has_noise_obstacles) {
    return {};
  }
  if (FLAGS_cateye_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/cateye_proposer_cvs");
    for (const auto* obstacle : cluster.obstacles()) {
      if (maybe_noise_obstacles.contains(obstacle)) {
        canvas.DrawPoint(Vec3d(obstacle->coord(), obstacle->ground_z),
                         vis::Color::kWhite, 10);
      }
    }
  }
  const LaneBoundaryInfo* left_boundary_info = nullptr;
  const LaneBoundaryInfo* right_boundary_info = nullptr;
  if (!GetLeftAndRightLaneBoundaries(semantic_map_manager, pose,
                                     &left_boundary_info,
                                     &right_boundary_info)) {
    VLOG(2) << "Can't get left and right boundaries.";
    return {};
  }
  QCHECK_NOTNULL(left_boundary_info);
  QCHECK_NOTNULL(right_boundary_info);
  std::vector<size_t> noise_obstacle_indices;
  noise_obstacle_indices.reserve(cluster.obstacles().size());
  ProposedClusters proposed_clusters;
  std::optional<ProposedCluster> remaining_cluster = cluster;
  std::vector<Vec2d> remaining_obstacles_pos;
  remaining_obstacles_pos.reserve(cluster.obstacles().size());
  // left boundary
  for (const auto* obstacle : remaining_cluster->obstacles()) {
    remaining_obstacles_pos.emplace_back(obstacle->coord());
  }
  QCHECK_GT(remaining_obstacles_pos.size(), 0);
  const auto& obstacles_dis_to_left_boundary =
      CalculateSignedDistancesToLaneBoundary(remaining_obstacles_pos,
                                             *left_boundary_info);
  for (size_t i = 0; i < remaining_cluster->obstacles().size(); ++i) {
    if (std::abs(obstacles_dis_to_left_boundary[i]) < kMaxDistanceToBoundary &&
        maybe_noise_obstacles.contains(remaining_cluster->obstacles()[i])) {
      noise_obstacle_indices.emplace_back(i);
    }
  }
  if (!noise_obstacle_indices.empty()) {
    remaining_cluster = TrimNoiseObstaclesAndGetRemainingCluster(
        *remaining_cluster, noise_obstacle_indices, &proposed_clusters);
  }
  if (!remaining_cluster) return proposed_clusters;
  noise_obstacle_indices.clear();
  remaining_obstacles_pos.clear();
  // right boundary
  for (const auto* obstacle : remaining_cluster->obstacles()) {
    remaining_obstacles_pos.emplace_back(obstacle->coord());
  }
  QCHECK_GT(remaining_obstacles_pos.size(), 0);
  const auto& obstacles_dis_to_right_boundary =
      CalculateSignedDistancesToLaneBoundary(remaining_obstacles_pos,
                                             *right_boundary_info);
  for (size_t i = 0; i < remaining_cluster->obstacles().size(); ++i) {
    if (std::abs(obstacles_dis_to_right_boundary[i]) < kMaxDistanceToBoundary &&
        maybe_noise_obstacles.contains(remaining_cluster->obstacles()[i])) {
      noise_obstacle_indices.emplace_back(i);
    }
  }
  if (!noise_obstacle_indices.empty()) {
    remaining_cluster = TrimNoiseObstaclesAndGetRemainingCluster(
        *remaining_cluster, noise_obstacle_indices, &proposed_clusters);
  }
  if (!remaining_cluster) return proposed_clusters;
  noise_obstacle_indices.clear();
  remaining_obstacles_pos.clear();
  // stop line from lane tl info
  for (const auto* obstacle : remaining_cluster->obstacles()) {
    remaining_obstacles_pos.emplace_back(obstacle->coord());
  }
  QCHECK_GT(remaining_obstacles_pos.size(), 0);
  const auto& obstacles_dis_to_stop_line = CalculateDistancesToStopLine(
      remaining_obstacles_pos, semantic_map_manager, pose);
  if (!obstacles_dis_to_stop_line.empty()) {
    QCHECK_EQ(obstacles_dis_to_stop_line.size(),
              remaining_cluster->obstacles().size());
    for (size_t i = 0; i < remaining_cluster->obstacles().size(); ++i) {
      if (obstacles_dis_to_stop_line[i] < kMaxDistanceToStopLine &&
          maybe_noise_obstacles.contains(remaining_cluster->obstacles()[i])) {
        noise_obstacle_indices.emplace_back(i);
      }
    }
    if (!noise_obstacle_indices.empty()) {
      remaining_cluster = TrimNoiseObstaclesAndGetRemainingCluster(
          *remaining_cluster, noise_obstacle_indices, &proposed_clusters);
    }
  }
  if (!remaining_cluster) return proposed_clusters;
  noise_obstacle_indices.clear();
  remaining_obstacles_pos.clear();
  // stop line from lane boundaries
  if (FLAGS_generate_stop_line_from_lane_boundaries) {
    const auto& stop_line = GetStopLineFromLeftAndRightLaneBoundaries(
        *left_boundary_info, *right_boundary_info, semantic_map_manager, pose);
    if (stop_line) {
      std::vector<double> obstacles_dis_to_lane_boundaries_stop_line;
      for (const auto* obstacle : remaining_cluster->obstacles()) {
        obstacles_dis_to_lane_boundaries_stop_line.emplace_back(
            stop_line->DistanceTo(obstacle->coord()));
      }
      if (FLAGS_cateye_proposer_cvs) {
        vis::Canvas& canvas =
            vantage_client_man::GetCanvas("perception/cateye_proposer_cvs");
        canvas.DrawLine(Vec3d(stop_line->start(), pose.z),
                        Vec3d(stop_line->end(), pose.z), vis::Color::kRed, 5);
      }
      if (!obstacles_dis_to_lane_boundaries_stop_line.empty()) {
        QCHECK_EQ(obstacles_dis_to_lane_boundaries_stop_line.size(),
                  remaining_cluster->obstacles().size());
        for (size_t i = 0; i < remaining_cluster->obstacles().size(); ++i) {
          if (obstacles_dis_to_lane_boundaries_stop_line[i] <
                  kMaxDistanceToStopLine &&
              maybe_noise_obstacles.contains(
                  remaining_cluster->obstacles()[i])) {
            noise_obstacle_indices.emplace_back(i);
          }
        }
        if (!noise_obstacle_indices.empty()) {
          remaining_cluster = TrimNoiseObstaclesAndGetRemainingCluster(
              *remaining_cluster, noise_obstacle_indices, &proposed_clusters);
        }
        if (!remaining_cluster) return proposed_clusters;
        noise_obstacle_indices.clear();
      }
    }
  }

  QCHECK(!remaining_cluster->obstacles().empty());
  proposed_clusters.emplace_back(*remaining_cluster);

  return proposed_clusters;
}

}  // namespace

ProposedClusters CateyeProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    auto contain_cateye_clusters = MaybeContainCateyeNoise(env_info, cluster);
    if (!contain_cateye_clusters.empty()) {
      for (auto& contain_cateye_cluster : contain_cateye_clusters) {
        proposed_clusters.emplace_back(std::move(contain_cateye_cluster));
      }
    } else {
      proposed_clusters.emplace_back(cluster);
    }
  }

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
