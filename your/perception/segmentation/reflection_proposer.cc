#include "onboard/perception/segmentation/reflection_proposer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <unordered_map>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/perception/semantic_map_util.h"

DEFINE_bool(reflection_proposer_cvs, false, "Enable reflection proposer cvs.");

namespace qcraft::segmentation {

constexpr float kTopElevationBufferNormal = 0.25f;      // degree.
constexpr float kTopElevationBufferInSignZone = 0.35f;  // degree.
constexpr float kVehicleProtraitHeight = 1.5f;          // m
const std::set<LidarId> kLidarRefList = {LDR_CENTER,      LDR_FRONT_LEFT,
                                         LDR_FRONT_RIGHT, LDR_REAR_BLIND,
                                         LDR_FRONT,       LDR_REAR};

struct SectorInfo {
  // Reflected count in the sector.
  int reflected_cnt = 0;
  // Max elevation_angle in the sector.
  float elevation = 0.0f;
  // Elevate vehicle sector height.
  float protrait_elevation = 0.0f;
  // Min range of points in the sector.
  float range = std::numeric_limits<float>::max();
  // If this sector belongs to vehicle cluster.
  MeasurementType type;
};

// Describe four bounds of each cluster from the perspective view in
// azimuth/elevation angle.
struct ClusterBound {
  float range = std::numeric_limits<float>::max();
  // Origin cluster index.
  int raw_idx = 0;
  // The angle is discretized on the histogram every 0.25 degree.
  // Key means sector remapped from angle.
  absl::flat_hash_map<int, SectorInfo> sectors_info;
  // If the bound in road sign zone.
  bool in_sign_zone = false;
};

bool IsFilterableCluster(const Cluster& cluster) {
  const auto& obstacles = cluster.obstacles();
  // Cluster is not filterable in cluster level filtering as long as cluster
  // is classified by semantic map zones.
  if (cluster.type_source() == MTS_SEMANTIC_MAP_ZONE) {
    return false;
  }
  const int cone_obstacle_cnt = std::count_if(
      obstacles.begin(), obstacles.end(),
      [](const auto& obs) { return obs->type == ObstacleProto::CONE; });
  constexpr float kClusterConeMinRatio = 0.3f;
  if (cone_obstacle_cnt * 1.0f >
      kClusterConeMinRatio * cluster.NumObstacles()) {
    return false;
  }
  return true;
}

RoadSigns FindHangingSigns(const Retroreflectors& retroreflectors,
                           const VehiclePose& pose) {
  if (retroreflectors.empty()) return {};
  RoadSigns signs;
  signs.reserve(retroreflectors.size());
  for (const auto& retroreflector : retroreflectors) {
    if (!retroreflector.is_overhanging) continue;

    RoadSign sign;
    sign.points = retroreflector.points;
    if (sign.points.size() < 15) continue;
    // Sign contour.
    std::vector<Vec2d> points_in_birds_eye_view;
    points_in_birds_eye_view.reserve(sign.points.size());
    for (const auto& point : sign.points) {
      points_in_birds_eye_view.emplace_back(point.x(), point.y());
    }
    if (!Polygon2d::ComputeConvexHull(points_in_birds_eye_view,
                                      &sign.contour)) {
      continue;
    }
    // Sign bounding box.
    sign.bounding_box = sign.contour.MinAreaBoundingBox();
    constexpr double kMinSignRadius = 0.8;  // m
    if (sign.bounding_box.length() < kMinSignRadius &&
        sign.bounding_box.width() < kMinSignRadius) {
      continue;
    }
    if (sign.bounding_box.width() > sign.bounding_box.length()) {
      sign.bounding_box = Box2d(
          sign.bounding_box.center(), sign.bounding_box.heading() + M_PI_2,
          sign.bounding_box.width(), sign.bounding_box.length());
    }
    // NOTE(dong): Only deal with signs directly facing ego vehicle.
    const Vec2d vec_1 = Vec2d::FastUnitFromAngle(pose.yaw);
    const Vec2d vec_2 =
        Vec2d::FastUnitFromAngle(sign.bounding_box.heading() + M_PI_2);
    if (vec_2.norm() == 0) continue;

    const double angle_diff =
        std::acos(std::abs(vec_1.Dot(vec_2)) / (vec_1.norm() * vec_2.norm()));
    if (angle_diff > M_PI_4) continue;
    // Sign z.
    std::vector<Vec3d>::iterator max_z_res = std::max_element(
        sign.points.begin(), sign.points.end(),
        [](const Vec3d& lhs, const Vec3d& rhs) { return lhs.z() < rhs.z(); });
    const Vec3d max_z_in_vehicle_coord =
        pose.ToTransform().Inverse().TransformPoint(*max_z_res);
    constexpr double kMinHangingSignHeight = 3.0;  // m
    if (max_z_in_vehicle_coord.z() < kMinHangingSignHeight) continue;

    signs.push_back(std::move(sign));
  }
  return signs;
}

std::vector<Box2d> CollectSignBoxes(const Retroreflectors& retroreflectors,
                                    const VehiclePose& pose) {
  std::vector<Box2d> res_boxes;
  const auto& signs = FindHangingSigns(retroreflectors, pose);
  for (const auto& sign : signs) {
    res_boxes.push_back(Box2d(
        sign.bounding_box.center(), sign.bounding_box.heading(),
        sign.bounding_box.length() + 16.0, sign.bounding_box.width() + 8.0));
  }
  return res_boxes;
}

std::map<LidarId, Vec3d> GetRefLidarPos(
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  QCHECK(!lidar_params.empty());
  std::map<LidarId, Vec3d> ref_lidar_poses;
  for (const auto& ref_lidar_id : kLidarRefList) {
    if (const auto* param = FindOrNull(lidar_params, ref_lidar_id)) {
      const auto& lidar_extrinsics = param->installation().extrinsics();
      const Vec3d lidar_pos = {lidar_extrinsics.x(), lidar_extrinsics.y(),
                               lidar_extrinsics.z()};
      InsertOrDie(&ref_lidar_poses, ref_lidar_id, lidar_pos);
    }
  }
  return ref_lidar_poses;
}

std::map<LidarId, LidarModel> GetLidarTypes(
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  std::map<LidarId, LidarModel> lidar_types;
  for (const auto& [lidar_id, params] : lidar_params) {
    lidar_types.emplace(lidar_id, params.model());
  }
  return lidar_types;
}

ProposedClusters FilterReflectionsBySensorFov(
    const ProposerEnvInfo& env_info,
    const std::map<LidarId, Vec3d>& ref_lidar_poses,
    const std::map<LidarId, LidarModel>& lidar_types,
    const ProposedClusters& clusters,
    std::vector<std::pair<double, Box2d>>* sign_zones_history,
    ThreadPool* thread_pool) {
  if (clusters.empty()) return {};

  SCOPED_QTRACE_ARG1("FilterReflectionsBySensorFov", "num_clusters",
                     clusters.size());

  const auto& pose = env_info.pose();
  const auto& radar_measurements = env_info.context().front_radar_measurements;
  const auto& retroreflectors = env_info.context().retroreflectors;
  const auto& semantic_map_manager = env_info.semantic_map_manager();
  const auto av_height = env_info.av_height();

  const int num_clusters = clusters.size();

  // Associate radar object to cluster.
  // Use char instead of bool for thread safety.
  std::vector<char> associated_radar_objs(num_clusters, false);
  const auto get_aabb_box = [](const ProposedCluster& cluster) -> Box2d {
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& obstacle : cluster.obstacles()) {
      min_x = std::min(min_x, obstacle->x);
      max_x = std::max(max_x, obstacle->x);
      min_y = std::min(min_y, obstacle->y);
      max_y = std::max(max_y, obstacle->y);
    }
    return Box2d{Vec2d{(min_x + max_x) * 0.5, (min_y + max_y) * 0.5}, 0.0,
                 max_x - min_x + Obstacle::kDiameter,
                 max_y - min_y + Obstacle::kDiameter};
  };

  ParallelFor(
      0, num_clusters, FLAGS_reflection_proposer_cvs ? nullptr : thread_pool,
      [&](int i) {
        const auto& cluster = clusters[i];
        for (const auto& radar_data : radar_measurements) {
          if (associated_radar_objs[i]) continue;

          const auto radar_vel = Vec2dFromProto(radar_data.vel()).squaredNorm();
          // NOTE(zhenye): Radar object velocity valued as 0m/s if it less than
          // 1m/s for now.
          if (radar_vel == 0.0) continue;
          // NOTE(dong): Check radar object heading with lane heading to remove
          // some radar noise.
          const auto lane_heading =
              perception_semantic_map_util::ComputeLaneHeadingAtPos(
                  semantic_map_manager,
                  {radar_data.pos().x(), radar_data.pos().y()});
          if (!lane_heading) continue;

          const auto radar_heading =
              Vec2d(radar_data.vel().x(), radar_data.vel().y()).Angle();
          constexpr double kMaxRadarAngleDiffWithLane = M_PI_4;
          if (std::abs(AngleDifference(radar_heading, *lane_heading)) >
              kMaxRadarAngleDiffWithLane) {
            continue;
          }

          bool is_in_cluster_box = false;
          bool is_in_aabb_box = false;
          if (cluster.bounding_box()) {
            const Box2d& box = *cluster.bounding_box();
            if (box.IsPointIn(Vec2dFromProto(radar_data.pos()))) {
              is_in_cluster_box = true;
            }
          }
          const Box2d box = get_aabb_box(cluster);
          if (box.IsPointIn(Vec2dFromProto(radar_data.pos()))) {
            is_in_aabb_box = true;
          }
          associated_radar_objs[i] = is_in_cluster_box || is_in_aabb_box;

          if (FLAGS_reflection_proposer_cvs && associated_radar_objs[i]) {
            vis::Canvas& canvas = vantage_client_man::GetCanvas(
                "perception/reflection_proposer_cvs");
            canvas.DrawCircle(
                {radar_data.pos().x(), radar_data.pos().y(), pose.z}, 0.4,
                vis::Color::kRed);
          }
        }
      });

  // Get lidar range image high intensity signs.
  const std::vector<Box2d> hanging_sign_zones =
      CollectSignBoxes(retroreflectors, pose);
  // If sign detecting missed, use the last few sign zones data.
  const double timestamp = clusters.front().timestamp();
  std::vector<std::pair<double, Box2d>> updating_sign_zones;
  updating_sign_zones.reserve(15);
  constexpr double kZonesKeepingMaxTime = 1.0;  // s
  for (const auto& box_pair : *sign_zones_history) {
    if (timestamp - box_pair.first < kZonesKeepingMaxTime) {
      updating_sign_zones.emplace_back(box_pair);
    }
  }
  constexpr double kMaxDistanceSqrOfMatchedBoxes = 1.0;  // m
  for (const auto& zone : hanging_sign_zones) {
    const auto& src_center = zone.center();
    bool has_same_box = false;
    for (auto& box_pair : updating_sign_zones) {
      if (src_center.DistanceSquareTo(box_pair.second.center()) <
          kMaxDistanceSqrOfMatchedBoxes) {
        box_pair = {timestamp, zone};
        has_same_box = true;
        break;
      }
    }
    if (!has_same_box) {
      updating_sign_zones.push_back({timestamp, zone});
    }
  }
  *sign_zones_history = updating_sign_zones;

  if (FLAGS_reflection_proposer_cvs) {
    for (const auto& sign_zone : updating_sign_zones) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/reflection_proposer_cvs");
      canvas.DrawBox({sign_zone.second.center(), pose.z},
                     sign_zone.second.heading(),
                     {sign_zone.second.length(), sign_zone.second.width()},
                     vis::Color::kRed);
    }
  }

  const auto pose_inverse_trans = pose.ToTransform().Inverse();

  const auto point_to_deg_elevation = [](const Vec3d& point) {
    // Angle theta from reverse direction of z axis (0, 0, -1) to the point
    // (x, y, z). Result is theta in range [0, 180].
    return r2d(fast_math::Atan2(Hypot(point.x(), point.y()), -point.z()));
  };
  const auto point_to_deg_azimuth = [](const Vec3d& point) {
    // Angle theta from reverse direction of x axis (-1, 0, 0) to the point
    // (x, y, 0), clockwise. Result is theta if point.y >= 0, 2 * PI - theta
    // otherwise in range [0, 360).
    const double theta = fast_math::Atan2(point.y(), -point.x());
    return r2d(theta >= 0.0 ? theta : 2.0 * M_PI + theta);
  };

  const auto compute_azimuth_key = [](const float azimuth) {
    constexpr float kAnglePerKey = 1.f / 0.25f;  // 0.25 degree
    return static_cast<int>(azimuth * kAnglePerKey);
  };

  const auto compute_cluster_height =
      [av_height](const ProposedCluster& cluster) {
        // TODO(zhenye): Cluster height calculation is not solid, it may cause
        // elevation bug.
        constexpr float kMinClusterHeight = 1.7f;  // m
        constexpr float kMaxClusterHeight = 3.0f;  // m
        constexpr float kLengthToHeightScale = 0.5f;
        if (cluster.bounding_box() == std::nullopt) {
          return av_height;
        }
        const auto length = cluster.bounding_box()->length();
        return std::clamp(static_cast<float>(length * kLengthToHeightScale),
                          kMinClusterHeight, kMaxClusterHeight);
      };

  SCOPED_QTRACE("FilterReflectionsBySensorFov_1");

  const int num_workers = thread_pool->NumWorkers() + 1;
  std::vector<std::map<LidarId, std::vector<ClusterBound>>>
      sub_cluster_bounds_per_worker(num_workers);
  // Raw cluster points centroid.
  std::vector<Vec2d> raw_cluster_centroids(num_clusters);

  const auto parallel_func = [&](parallel_for::WorkerIndex worker, int i) {
    const auto& cluster = clusters[i];
    if (cluster.HasProperty(PP_NOISE)) return;

    raw_cluster_centroids[i] = cluster.ComputeCentroidFromObstacles();
    const float hanging_height = compute_cluster_height(cluster);
    const auto cluster_type = cluster.type();
    std::map<LidarId, ClusterBound> cluster_bound_per_lidar;
    for (const auto& obstacle : cluster.obstacles()) {
      const float protrait_height_offset =
          obstacle->height() < kVehicleProtraitHeight
              ? kVehicleProtraitHeight - obstacle->height()
              : 0.0f;
      for (const auto& point : obstacle->points) {
        if (!obstacle_util::IsAboveGroundObstaclePoint(*obstacle, point) ||
            point.z > obstacle->ground_z + hanging_height) {
          continue;
        }
        const auto lidar_id = point.lidar_id;
        const auto* ref_lidar_pose = FindOrNull(ref_lidar_poses, lidar_id);
        if (ref_lidar_pose == nullptr) continue;

        const Vec3d point_in_lidar_coord =
            pose_inverse_trans.TransformPoint(point.coord()) - *ref_lidar_pose;
        // Take azimuth offset of each beam into consideration.
        const float deg_azimuth = point_to_deg_azimuth(point_in_lidar_coord);
        const float deg_elevation =
            point_to_deg_elevation(point_in_lidar_coord);
        const float raised_point_elevation = point_to_deg_elevation(
            {point_in_lidar_coord.x(), point_in_lidar_coord.y(),
             point_in_lidar_coord.z() + protrait_height_offset});
        const float protrait_elevation_deg =
            cluster_type == MT_VEHICLE ? raised_point_elevation : 0.0f;

        auto& cluster_bound = cluster_bound_per_lidar[lidar_id];
        const int azimuth_key = compute_azimuth_key(deg_azimuth);
        SectorInfo sector_info;
        sector_info.elevation = deg_elevation;
        sector_info.protrait_elevation = protrait_elevation_deg;
        sector_info.type = cluster_type;
        sector_info.range = point.range;
        auto& update_value = LookupOrInsert(&cluster_bound.sectors_info,
                                            azimuth_key, sector_info);
        update_value.elevation =
            std::max(update_value.elevation, deg_elevation);
        update_value.range = std::min(update_value.range, point.range);
        if (cluster_type == MT_VEHICLE) {
          update_value.protrait_elevation =
              std::max(update_value.protrait_elevation, protrait_elevation_deg);
        }
        cluster_bound.range = std::min(cluster_bound.range, point.range);
        cluster_bound.raw_idx = i;
      }
    }

    for (const auto& [lidar_id, bound] : cluster_bound_per_lidar) {
      sub_cluster_bounds_per_worker[worker][lidar_id].emplace_back(bound);
    }
  };
  ParallelFor(0, num_clusters, thread_pool, parallel_func);

  std::map<LidarId, std::vector<ClusterBound>> sub_cluster_bounds;
  for (const auto& all_bounds : sub_cluster_bounds_per_worker) {
    for (const auto& [lidar_id, bounds] : all_bounds) {
      auto& bounds_for_lidar = sub_cluster_bounds[lidar_id];
      bounds_for_lidar.insert(bounds_for_lidar.end(), bounds.begin(),
                              bounds.end());
    }
  }
  // Sort bounds per lidar to avoid non-deterministic results from
  // multithreading.
  for (auto& [_, bounds] : sub_cluster_bounds) {
    std::sort(bounds.begin(), bounds.end(), [](const auto& a, const auto& b) {
      return a.raw_idx < b.raw_idx;
    });
  }

  const auto get_horizontal_overlap_sectors =
      [](const ClusterBound& media, const ClusterBound& reflection) {
        const auto& media_hfov = media.sectors_info;
        const auto& reflection_hfov = reflection.sectors_info;
        std::vector<int> overlap_angle_sectors;
        overlap_angle_sectors.reserve(media_hfov.size());
        for (const auto& [sector, _] : media_hfov) {
          const auto* same_sector = FindOrNull(reflection_hfov, sector);
          if (same_sector == nullptr) continue;

          overlap_angle_sectors.push_back(sector);
        }
        return overlap_angle_sectors;
      };

  const auto get_reflection_condition =
      [&](const MeasurementType& media_type,
          const MeasurementType& reflection_type,
          const int reflection_raw_idx) -> std::pair<bool, bool> {
    // TODO(zhenye): Elevate vehicle height for no points left on the top
    // of the vehicle.
    const bool intractable_pair =
        media_type == MT_VEHICLE && reflection_type != MT_PEDESTRIAN &&
        reflection_type != MT_CYCLIST && reflection_type != MT_MOTORCYCLIST;
    const bool suspicious_zone =
        std::any_of(updating_sign_zones.begin(), updating_sign_zones.end(),
                    [&](const auto& box) {
                      return box.second.IsPointIn(
                          raw_cluster_centroids[reflection_raw_idx]);
                    });
    return {intractable_pair, suspicious_zone};
  };

  SCOPED_QTRACE("FilterReflectionsBySensorFov_2");

  // Pair first for reflection sector count, second for total sector count.
  constexpr float kTypeIgnorableMaxDistance = 100.f;  // m
  std::vector<std::map<LidarId, std::pair<int, int>>> cluster_reflection_marker(
      num_clusters);
  for (auto& [lidar_id, bounds] : sub_cluster_bounds) {
    const auto lidar_type = FindOrDie(lidar_types, lidar_id);
    // Here i's cluster stands for media cluster while j's cluster stands for
    // refection cluster.
    for (int i = 0; i < bounds.size(); ++i) {
      for (int j = 0; j < bounds.size(); ++j) {
        if (i == j) continue;

        const auto& media_bound = bounds[i];
        auto& reflection_bound = bounds[j];
        if (media_bound.range >= reflection_bound.range) continue;

        if (!IsFilterableCluster(clusters[reflection_bound.raw_idx])) continue;

        // TODO(zhenye): Only consider vehicle as media cluster here.
        if (clusters[media_bound.raw_idx].type() != MT_VEHICLE &&
            media_bound.range < kTypeIgnorableMaxDistance) {
          continue;
        }
        const std::vector<int> overlap_angle_sectors =
            get_horizontal_overlap_sectors(media_bound, reflection_bound);
        for (const auto& sector : overlap_angle_sectors) {
          const auto media_sector_info =
              FindOrDie(media_bound.sectors_info, sector);
          // Reflection sector will not cause reflection again.
          if (media_sector_info.reflected_cnt > 0) continue;

          auto& reflection_sector_info =
              FindOrDie(reflection_bound.sectors_info, sector);
          if (media_sector_info.range >= reflection_sector_info.range) continue;

          const auto [is_intractable_pair, in_suspicious_zone] =
              get_reflection_condition(media_sector_info.type,
                                       reflection_sector_info.type,
                                       reflection_bound.raw_idx);
          reflection_bound.in_sign_zone = in_suspicious_zone;
          float media_sector_protrait_elevation =
              media_sector_info.protrait_elevation;
          float media_sector_elevation = media_sector_info.elevation;
          // Hacky for M1 lidar.
          if (lidar_type == LIDAR_RS_M1) {
            if (const auto* left_media_sector_info =
                    FindOrNull(media_bound.sectors_info, sector - 1)) {
              media_sector_protrait_elevation =
                  std::max(media_sector_protrait_elevation,
                           left_media_sector_info->protrait_elevation);
              media_sector_elevation = std::max(
                  media_sector_elevation, left_media_sector_info->elevation);
            }
            if (const auto* right_media_sector_info =
                    FindOrNull(media_bound.sectors_info, sector + 1)) {
              media_sector_protrait_elevation =
                  std::max(media_sector_protrait_elevation,
                           right_media_sector_info->protrait_elevation);
              media_sector_elevation = std::max(
                  media_sector_elevation, right_media_sector_info->elevation);
            }
          }
          const float media_elevation_with_buffer =
              is_intractable_pair && in_suspicious_zone
                  ? media_sector_protrait_elevation +
                        kTopElevationBufferInSignZone
                  : media_sector_elevation + kTopElevationBufferNormal;

          if (media_elevation_with_buffer > reflection_sector_info.elevation) {
            reflection_sector_info.reflected_cnt++;
          }
        }
      }
    }

    for (const auto& bound : bounds) {
      int reflect_sector_cnt = 0;
      // TODO(zhenye): Get reflecting sector count and continuous padding sector
      // count.
      for (const auto& [_, info] : bound.sectors_info) {
        if (info.reflected_cnt > 0) reflect_sector_cnt++;
      }
      cluster_reflection_marker[bound.raw_idx][lidar_id] = {
          reflect_sector_cnt, bound.sectors_info.size()};

      if (FLAGS_reflection_proposer_cvs) {
        std::string debug_text;
        const Vec2d contour_centroid =
            clusters[bound.raw_idx].ComputeCentroidFromObstacles();
        std::vector<std::pair<int, SectorInfo>> sector_elevations(
            bound.sectors_info.begin(), bound.sectors_info.end());
        std::sort(sector_elevations.begin(), sector_elevations.end(),
                  [&](const auto& lhs, const auto& rhs) {
                    return lhs.first < rhs.first;
                  });
        debug_text = absl::StrFormat(
            "%s InSignZone:%d Reflect/Total: %d/%d \n", LidarId_Name(lidar_id),
            bound.in_sign_zone, reflect_sector_cnt, bound.sectors_info.size());
        for (const auto& eva : sector_elevations) {
          std::string text = absl::StrFormat("%.2f|%.2f ", eva.first * 0.25,
                                             eva.second.elevation);
          debug_text += text;
        }
        const float scale = lidar_id * 0.2f + 0.2f;
        vis::Canvas& canvas =
            vantage_client_man::GetCanvas("perception/reflection_proposer_cvs");
        canvas.DrawText(debug_text, {contour_centroid + Vec2d(0, scale), 0.01},
                        0, 0.15, vis::Color::kLightYellow);
      }
    }
  }

  // NOTE(zhenye): The cluster is marked as reflection when reflection angle
  // sectors in majority.
  const auto is_reflection_bound = [](const MeasurementType& cluster_type,
                                      const int num_reflection_sectors,
                                      const int num_total_sectors) {
    if (cluster_type == MT_PEDESTRIAN) {
      return num_reflection_sectors == num_total_sectors;
    }
    constexpr float kMiniClusterReflectionSectorRatio = 0.4f;
    constexpr float kVehicleReflectionSectorRatio = 0.8f;
    constexpr float kReflectionSectorRatio = 0.66f;
    constexpr int kMaxMiniClusterTotalSectorCnt = 10;
    const float reflection_ratio_threshold =
        num_total_sectors < kMaxMiniClusterTotalSectorCnt
            ? kMiniClusterReflectionSectorRatio
            : kReflectionSectorRatio;
    return num_reflection_sectors >
           num_total_sectors * (cluster_type == MT_VEHICLE
                                    ? kVehicleReflectionSectorRatio
                                    : reflection_ratio_threshold);
  };

  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(num_clusters);

  for (int i = 0; i < num_clusters; ++i) {
    if (clusters[i].HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(clusters[i]);
      continue;
    }
    if (cluster_reflection_marker[i].empty()) {
      proposed_clusters.push_back(clusters[i]);
      continue;
    }
    // Exclude radar association.
    if (associated_radar_objs[i] > 0) {
      proposed_clusters.push_back(clusters[i]);
      continue;
    }

    int reflection_sector_cnts = 0;
    int total_sector_cnts = 0;
    for (const auto& [_, cnt] : cluster_reflection_marker[i]) {
      reflection_sector_cnts += cnt.first;
      total_sector_cnts += cnt.second;
    }
    QCHECK_NE(total_sector_cnts, 0);

    if (is_reflection_bound(clusters[i].type(), reflection_sector_cnts,
                            total_sector_cnts)) {
      ProposedCluster reflection_cluster = clusters[i];
      reflection_cluster.set_is_proposed(true);
      reflection_cluster.set_property(PP_NOISE);
      proposed_clusters.push_back(std::move(reflection_cluster));
    } else {
      proposed_clusters.push_back(clusters[i]);
    }
  }

  return proposed_clusters;
}

ProposedClusters ReflectionProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  std::unordered_map<LidarId, LidarParametersProto> lidar_params;
  for (const auto& lidar_param :
       env_info.run_params().vehicle_params().lidar_params()) {
    lidar_params[lidar_param.installation().lidar_id()] = lidar_param;
  }

  const std::map<LidarId, Vec3d>& ref_lidar_poses =
      GetRefLidarPos(lidar_params);
  const std::map<LidarId, LidarModel>& lidar_types =
      GetLidarTypes(lidar_params);

  ProposedClusters proposed_clusters = FilterReflectionsBySensorFov(
      env_info, ref_lidar_poses, lidar_types, clusters, &sign_zones_history_,
      thread_pool_);

  DCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
