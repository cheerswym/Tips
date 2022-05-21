#include "onboard/perception/obstacle_manager.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <string>
#include <unordered_map>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/camera/utils/image_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/nets/mof_net_util.h"
#include "onboard/nets/panonet_config.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/semantic_map_util.h"

DEFINE_bool(enable_obstacle_noisy_filter, true,
            "Enable a noisy filter to handle outlier obstacles.");
DEFINE_bool(enable_obstacle_mist_filter, true,
            "Enable a mist filter to handle rainy weather.");
DEFINE_bool(enable_mof_net_filter, false,
            "Enable a mist filter net to handle rainy weather.");
DEFINE_bool(enable_obstacle_mist_propagation, true,
            "Enable mist propagation to handle underfiltered mist.");
DEFINE_bool(
    enable_obstacle_semantic_map_filter, true,
    "Enable a semantic map filter to handle ignorance & reflection obstacles.");
DEFINE_bool(whitelist_nearcurb_vehicle, false,
            "Whether to whitelist nearcurb vehicle.");
DEFINE_bool(obstacle_filtering_cvs, false,
            "Render filtered obstacles that are likely mist");
DEFINE_bool(
    whitelist_with_semantic_segmentation_results, true,
    "Whether to whitelist obstacles using semantic segmentation results.");

namespace qcraft {

using namespace perception_semantic_map_util;  // NOLINT

namespace {
// Noisy filter
// The maximum number of an obstacle that could be treated as noisy. A noisy
// obstacle is formed by fake laser points on air.
constexpr int kFarawayNoisyObstacleMaxNumPoints = 1;
// The maximum range of noisy obstacles.
constexpr float kFarawayNoisyObstacleMaxRange = 50.0f;
// The maximum number of an obstacle that could be treated as noisy. A noisy
// obstacle is formed by fake laser points on air.
constexpr int kNearbyNoisyObstacleMaxNumPoints = 2;
// The maximum range of noisy obstacles.
constexpr float kNearbyNoisyObstacleMaxRange = 10.0f;

// Rain filter
// Enable obstacle rain filter distance threshold.
constexpr float kObstacleRainFilterDistance = 30.0f;

// Rain filter v1.
// 2nd return laser points ratio.
constexpr float kObstacleRainFilterHasReturnBehindRatioOutBox = 0.04f;
constexpr float kObstacleRainFilterHasReturnBehindRatioInBox = 0.4f;
// If an obstacle has at least this ratio of points that have return behind, it
// is likely a mist obstacle.
constexpr float kMistObstacleNumPointsWithReturnBehindMinRatio = 0.6f;
// Max intensity of mist obstacle.
constexpr float kMistObstacleMaxIntensity = 1.0f;

// Rain filter v2
// Actual reflectivity is 1.95.
constexpr float kMistFullScoreMinAveragedIntensity = 34.0f;
constexpr int kMistHighIntensityLevel[3] = {
    80, 178, 250};  //  Actual reflectivity are 10.17/50.25/98.50.
constexpr float kMistFullScoreMinHasReturnBehindRatio = 0.5f;
constexpr float kMinMistScoreToFilterInBox = 0.6f;
constexpr float kMinMistScoreToFilterOutBox = 0.4f;

SegmentationType GetPointSegmentationType(
    const LaserPoint& point, const cv::Mat& mask,
    const AffineTransformation& smooth_to_camera_trans,
    const CameraParams& camera_params, const double sem_seg_output_scale) {
  const auto& img_pos = projection_util::SmoothPointToImagePos(
      point.coord(), smooth_to_camera_trans, camera_params);
  return img_pos ? static_cast<SegmentationType>(
                       mask.at<uchar>(img_pos->y() / sem_seg_output_scale,
                                      img_pos->x() / sem_seg_output_scale))
                 : ST_DONTCARE;
}

bool IsOccludedByReflectionZones(const Vec2d& obstacle_pos,
                                 const Vec2d& ref_pos,
                                 const std::vector<Polygon2d>& zones) {
  const Segment2d segment(ref_pos, obstacle_pos);
  for (const auto& zone : zones) {
    if (zone.HasOverlap(segment)) {
      return true;
    }
  }
  return false;
}

// Find if the obstacle in a detection box.
bool IsPointInBoxes(
    const std::vector<FieryEyeNetClassifier::DetectionBox>& boxes,
    const Obstacle& obstacle) {
  for (const auto& detection_box : boxes) {
    if (detection_box.box.IsPointIn(obstacle.coord())) {
      return true;
    }
  }
  return false;
}

}  // namespace

ObstacleManager::ObstacleManager(
    ObstacleRefVector obstacles,
    const ObstacleRCCoordConverter& rc_coord_converter,
    const CameraParamsMap& camera_params,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    ThreadPool* thread_pool)
    : rc_coord_converter_(rc_coord_converter),
      camera_params_(camera_params),
      lidar_params_(lidar_params),
      thread_pool_(thread_pool) {
  SCOPED_QTRACE_ARG1("ObstacleManager::ObstacleManager", "num_obstacles",
                     obstacles.size());

  // TODO(cong): Sort obstacles to reduce page fault/cache miss on traversing
  // obstacles in a cluster.
  const int num_obstacles = obstacles.size();
  obstacles_.resize(num_obstacles);
  obstacle_ptrs_.resize(num_obstacles);

  for (int i = 0; i < num_obstacles; ++i) {
    const auto* obstacle = obstacles[i].get();
    obstacles_[i] = *obstacle;
    obstacle_ptrs_[i] = &obstacles_[i];
    min_row_ = std::min<int>(min_row_, obstacle->row);
    min_col_ = std::min<int>(min_col_, obstacle->col);
    max_row_ = std::max<int>(max_row_, obstacle->row);
    max_col_ = std::max<int>(max_col_, obstacle->col);
  }

  min_row_ -= kObstacleGridPadding;
  min_col_ -= kObstacleGridPadding;
  max_row_ += kObstacleGridPadding;
  max_col_ += kObstacleGridPadding;
  const int obstacle_grid_height = max_row_ - min_row_ + 1;
  obstacle_grid_width_ = max_col_ - min_col_ + 1;
  obstacle_existence_grid_.resize(obstacle_grid_width_ * obstacle_grid_height);

  rc_to_obstacles_.reserve(num_obstacles);
  obstacles_to_index_.reserve(num_obstacles);
  for (int i = 0; i < num_obstacles; ++i) {
    ObstaclePtr obstacle = &obstacles_[i];
    rc_to_obstacles_[{obstacle->row, obstacle->col}] = obstacle;
    obstacles_to_index_[obstacle] = i;
    obstacle_existence_grid_[(obstacle->row - min_row_) * obstacle_grid_width_ +
                             obstacle->col - min_col_] = true;
  }

  DestroyContainerAsync(std::move(obstacles));
}

void ObstacleManager::WhitelistObstaclesWithDetections(
    const FieryEyeNetClassifier::DetectionResult& fen_result,
    const SemanticSegmentationResults& semantic_results,
    const VehiclePose& pose) {
  SCOPED_QTRACE("ObstacleManager::WhitelistObstaclesWithDetections");
  // Re-promote offroad and ignored ped/cyc detections.
  for (auto& obstacle : obstacles_) {
    if (obstacle.type != ObstacleProto::IGNORED &&
        obstacle.type != ObstacleProto::OFFROAD) {
      continue;
    }
    for (const auto& det_box : fen_result.ped_boxes) {
      if (det_box.box.IsPointIn(obstacle.coord())) {
        obstacle.type = ObstacleProto::DYNAMIC;
        obstacle.type_source = ObstacleProto::FIERY_EYE_NET;
      }
    }
    for (const auto& det_box : fen_result.cyc_boxes) {
      if (det_box.box.IsPointIn(obstacle.coord())) {
        obstacle.type = ObstacleProto::DYNAMIC;
        obstacle.type_source = ObstacleProto::FIERY_EYE_NET;
      }
    }
  }

  // Re-promote near-curb offroad car detections.
  if (FLAGS_whitelist_nearcurb_vehicle) {
    constexpr double kPromotedOffroadVehicleMaxCurbDistance = 4.;  // m
    for (auto& obstacle : obstacles_) {
      if (obstacle.type != ObstacleProto::OFFROAD ||
          obstacle.dist_to_curb > kPromotedOffroadVehicleMaxCurbDistance) {
        continue;
      }
      for (const auto& det_box : fen_result.car_boxes) {
        if (det_box.box.IsPointIn(obstacle.coord())) {
          obstacle.type = ObstacleProto::DYNAMIC;
          obstacle.type_source = ObstacleProto::FIERY_EYE_NET;
        }
      }
    }
  }

  if (FLAGS_whitelist_with_semantic_segmentation_results) {
    SCOPED_QTRACE("ObstacleManager::WhitelistWithSemanticSegmentationResults");
    const int num_obstacles = obstacles_.size();
    for (const auto& semantic_result : semantic_results) {
      const auto& mask = semantic_result->semantic_mat();
      const double sem_seg_output_scale =
          semantic_result->sem_seg_output_scale();
      const auto& camera_params =
          FindOrDie(camera_params_, semantic_result->camera_id());
      const auto smooth_to_camera_trans =
          (semantic_result->pose().ToTransform() *
           camera_params.camera_to_vehicle_extrinsics().ToTransform())
              .Inverse();
      ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
        auto& obstacle = obstacles_[i];
        if (obstacle.type == ObstacleProto::IGNORED ||
            obstacle.type == ObstacleProto::OFFROAD) {
          return;
        }
        // NOTE(dong): A more general approach to convert obstacle pos from
        // smooth to vehicle:
        // AffineTransformation t_inv = pose.ToTransform().Inverse();
        // const Vec2d obstacle_pose_in_vehicle =
        //     t_inv.TransformPoint({obstacle.x, obstacle.y, 0.0}).head<2>();
        const Vec2d obstacle_pose_in_vehicle =
            Vec2d(obstacle.x - pose.x, obstacle.y - pose.y)
                .FastRotate(-pose.yaw);
        // NOTE(dong): Do not process obstacles behind ego vehicle by now.
        if (obstacle_pose_in_vehicle.x() < 0) {
          return;
        }
        std::array<int, SegmentationType_ARRAYSIZE> num_points_of_type = {};
        int num_points_above_ground = 0;
        for (const auto& point : obstacle.points) {
          if (!obstacle_util::IsAboveGroundObstaclePoint(obstacle, point)) {
            continue;
          }
          ++num_points_above_ground;
          const auto type =
              GetPointSegmentationType(point, mask, smooth_to_camera_trans,
                                       camera_params, sem_seg_output_scale);
          ++num_points_of_type[static_cast<int>(type)];
        }
        constexpr double kMinWhitelistedConePointRatio = 0.5;
        const int num_points_of_cone =
            num_points_of_type[static_cast<int>(ST_TRAFFIC_CONE)];
        if (num_points_of_cone >
            num_points_above_ground * kMinWhitelistedConePointRatio) {
          obstacle.type = ObstacleProto::CONE;
          obstacle.type_source = ObstacleProto::LL_NET;
        }
      });
    }
  }
}

void ObstacleManager::BlacklistObstaclesWithinFilteredClusters(
    const SegmentedClusters& ignored_reflection_clusters,
    const SegmentedClusters& ignored_mist_clusters,
    const SegmentedClusters& ignored_blooming_clusters,
    const SegmentedClusters& ignored_small_clusters) {
  const std::string scoped_info_string = absl::StrFormat(
      "reflection %d mist %d small %d", ignored_reflection_clusters.size(),
      ignored_mist_clusters.size(), ignored_small_clusters.size());
  SCOPED_QTRACE_ARG1(
      "ObstacleManager::BlacklistObstaclesWithinFilteredClusters",
      "filter_info", scoped_info_string);

  const auto ignore_obstacles = [this](const SegmentedClusters& clusters,
                                       ObstacleProto::TypeSource type_source) {
    for (const auto& filtered_cluster : clusters) {
      for (const auto* obstacle_ptr : filtered_cluster) {
        const int obstacle_ind = FindOrDie(obstacles_to_index_, obstacle_ptr);
        Obstacle& obstacle = *mutable_obstacle(obstacle_ind);
        obstacle.type = ObstacleProto::IGNORED;
        obstacle.type_source = type_source;
      }
    }
  };

  ignore_obstacles(ignored_reflection_clusters,
                   ObstacleProto::REFLECTION_CLUSTER_FILTER);
  ignore_obstacles(ignored_mist_clusters, ObstacleProto::MIST_CLUSTER_FILTER);
  ignore_obstacles(ignored_blooming_clusters,
                   ObstacleProto::BLOOMING_CLUSTER_FILTER);
  ignore_obstacles(ignored_small_clusters, ObstacleProto::SMALL_CLUSTER_FILTER);
}

void ObstacleManager::FilterObstacleNoise() {
  SCOPED_QTRACE_ARG1("ObstacleManager::ObstacleNoisyFilter", "num_obstacles",
                     obstacles_.size());
  const int num_obstacles = obstacles_.size();

  auto update_and_render_noise_obstacle = [](Obstacle* obstacle) {
    obstacle->type = ObstacleProto::IGNORED;
    obstacle->type_source = ObstacleProto::NOISY_OBSTACLE_FILTER;
    // Render noisy obstacles cvs.
    if (FLAGS_obstacle_filtering_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/obstacle_manager");
      const std::vector<Vec3d> obstacle_contour =
          obstacle_util::ComputeContourWithZ(*obstacle, obstacle->max_z + 1.0);
      canvas.DrawPolygon(obstacle_contour, vis::Color::kLightBlue, 1);
    }
  };

  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    // Ignore this obstacle if there are too few non-ground points,
    // the obstacle is not far from AV and the obstacle does not have
    // neighbors. It is likely that those points are just noises.
    Obstacle& obstacle = obstacles_[i];
    QCHECK(!obstacle.points.empty());
    const float range = obstacle.points[0].range;
    if (range > kFarawayNoisyObstacleMaxRange) {
      return;
    }
    // NOTE(dong): Hack for removing bpearl noise obstacles.
    bool only_contains_bpearl_points = true;
    bool only_contains_left_blind_points = true;
    bool only_contains_right_blind_points = true;
    for (int i = obstacle.above_ground_points_start_index();
         i < obstacle.points.size(); ++i) {
      const auto& point = obstacle.points[i];
      if (point.lidar_type != LIDAR_RS_BPEARL) {
        only_contains_bpearl_points = false;
        break;
      }
      if (point.lidar_id != LDR_FRONT_LEFT_BLIND) {
        only_contains_left_blind_points = false;
      } else if (point.lidar_id != LDR_FRONT_RIGHT_BLIND) {
        only_contains_right_blind_points = false;
      }
    }
    const bool only_contains_single_blind_lidar_points =
        only_contains_left_blind_points || only_contains_right_blind_points;
    constexpr int kBpearlNoiseObstacleMaxNumPoints = 5;
    constexpr int kBpearlNoiseObstacleSingleBlindMaxNumPoints = 12;
    constexpr float kBpearlNoiseObstacleMaxHeight = 0.3f;
    const bool is_bpearl_noise_obstacle =
        only_contains_bpearl_points &&
        obstacle.height() < kBpearlNoiseObstacleMaxHeight &&
        (obstacle.num_points_above_ground < kBpearlNoiseObstacleMaxNumPoints ||
         (only_contains_single_blind_lidar_points &&
          obstacle.num_points_above_ground <
              kBpearlNoiseObstacleSingleBlindMaxNumPoints));
    if (is_bpearl_noise_obstacle) {
      update_and_render_noise_obstacle(&obstacle);
      return;
    }
    // Deal with normal noise obstacles.
    const int valid_obstacle_num_points =
        (range < kNearbyNoisyObstacleMaxRange)
            ? kNearbyNoisyObstacleMaxNumPoints
            : kFarawayNoisyObstacleMaxNumPoints;
    if (obstacle.num_points_above_ground <= valid_obstacle_num_points) {
      // Ignore this obstacle only when it has no obstacle
      // neighbors.
      const auto has_neighbors = [&]() {
        const int row = obstacle.row;
        const int col = obstacle.col;
        for (int r = row - 2; r <= row + 2; ++r) {
          for (int c = col - 2; c <= col + 2; ++c) {
            if (r == row && c == col) continue;
            const ObstaclePtr neighbor = ObstacleAt(r, c);
            if (neighbor != nullptr) {
              return true;
            }
          }
        }
        return false;
      };
      if (!has_neighbors()) {
        update_and_render_noise_obstacle(&obstacle);
      }
    }
  });
}

bool ObstacleManager::ObstacleIsLikelyMist(const Obstacle& obstacle) {
  int num_above_ground_points = 0;
  int num_above_ground_points_with_return_behind = 0;
  for (const auto& point : obstacle.points) {
    if (obstacle_util::IsAboveGroundObstaclePoint(obstacle, point)) {
      num_above_ground_points++;
      if (point.has_return_behind) {
        num_above_ground_points_with_return_behind++;
      }
    }
  }
  return num_above_ground_points_with_return_behind >
         num_above_ground_points *
             kMistObstacleNumPointsWithReturnBehindMinRatio;
}

float ObstacleManager::ComputeObstacleMistScore(const Obstacle& obstacle) {
  // Currently, the obstacle mist score is calculated by taking average between:
  // 1) Transformed has return behind ratio.
  // 2) Transformed averaged intensity.
  int num_above_ground_points = 0;
  int num_points_with_return_behind = 0;
  int num_high_intensity_points[3] = {};
  int intensity_sum = 0;
  for (const auto& point : obstacle.points) {
    if (!obstacle_util::IsAboveGroundObstaclePoint(obstacle, point)) {
      continue;
    }

    // BANDAID(zhangtao): hack the QT128 intensity
    int intensity = point.intensity;
    switch (point.lidar_type) {
      case LIDAR_RS_BPEARL:
        intensity = intensity_util::RecalibrateBpearlIntensity(
            point.intensity, point.range, point.z, obstacle.ground_z);
        break;
      case LIDAR_PANDAR_QT128:
        intensity = intensity_util::RecalibrateQT128Intensity(
            point.intensity, point.range, point.z, obstacle.ground_z);
        break;
      default:
        break;
    }

    intensity_sum += intensity;
    ++num_above_ground_points;
    if (point.has_return_behind) {
      ++num_points_with_return_behind;
    }
    if (intensity < kMistHighIntensityLevel[0]) continue;
    if (intensity < kMistHighIntensityLevel[1]) {
      ++num_high_intensity_points[0];
    } else if (intensity < kMistHighIntensityLevel[2]) {
      ++num_high_intensity_points[1];
    } else {
      ++num_high_intensity_points[2];
    }
  }

  // Regard as mist if no above ground points.
  if (num_above_ground_points == 0) {
    return 1.f;
  }

  const float num_above_ground_points_inv = 1.0f / num_above_ground_points;
  const float averaged_intensity = intensity_sum * num_above_ground_points_inv;
  const float has_return_behind_ratio =
      static_cast<float>(num_points_with_return_behind) *
      num_above_ground_points_inv;
  const float high_intensity_points_ratio[3] = {
      static_cast<float>(num_high_intensity_points[0]) *
          num_above_ground_points_inv,
      static_cast<float>(num_high_intensity_points[1]) *
          num_above_ground_points_inv,
      static_cast<float>(num_high_intensity_points[2]) *
          num_above_ground_points_inv};

  // Compute intensity mist score by clipping intensity to 1.0 and re-scale it
  // to [0, 1].
  const float intensity_mist_score =
      1.f -
      std::clamp(averaged_intensity, 0.f, kMistFullScoreMinAveragedIntensity) *
          (1.f / kMistFullScoreMinAveragedIntensity);

  // Compute has return behind score by clipping has return behind ratio to 0.5
  // and rescale it to [0, 1].
  const float has_return_behind_mist_score =
      std::clamp(has_return_behind_ratio, 0.f,
                 kMistFullScoreMinHasReturnBehindRatio) *
      (1.f / kMistFullScoreMinHasReturnBehindRatio);

  // For now, different scores contribute equally to the final score. However,
  // some small objects, such as cones, may have a high has return behind mist
  // score because of their appearance. These obstacles usually have some high
  // intensity points. Here we use high intensity points ratios to balance mist
  // score and prevent to over-filter these obstacles.
  const float mist_score = std::clamp(
      has_return_behind_mist_score * 0.5f + intensity_mist_score * 0.5f -
          high_intensity_points_ratio[0] - high_intensity_points_ratio[1] * 2 -
          high_intensity_points_ratio[2] * 3,
      0.f, 1.f);

  if (FLAGS_obstacle_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/obstacle_manager");
    canvas.DrawText(
        absl::StrFormat("raw mist score %.2f", mist_score),
        {obstacle.x - 0.1, obstacle.y - 0.02, obstacle.ground_z + 0.01}, 0.0,
        0.015, vis::Color::kWhite);
    canvas.DrawText(
        absl::StrFormat(
            "i %.2f/r %.2f/h0 %.2f/h1 %.2f/h2 %.2f", intensity_mist_score,
            has_return_behind_mist_score, high_intensity_points_ratio[0],
            high_intensity_points_ratio[1], high_intensity_points_ratio[2]),
        {obstacle.x - 0.1, obstacle.y - 0.05, obstacle.ground_z + 0.01}, 0.0,
        0.01, vis::Color::kWhite);
    canvas.DrawText(
        absl::StrFormat("mean inty %.2f/hrbr %.2f", averaged_intensity,
                        has_return_behind_ratio),
        {obstacle.x - 0.1, obstacle.y - 0.07, obstacle.ground_z + 0.01}, 0.0,
        0.01, vis::Color::kWhite);
  }

  return mist_score;
}

void ObstacleManager::ObstacleRainFilter(
    const FieryEyeNetClassifier::DetectionResult& fen_result) {
  SCOPED_QTRACE_ARG1("ObstacleManager::ObstacleRainFilter", "num_obstacles",
                     obstacles_.size());
  const int num_obstacles = obstacles_.size();
  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    Obstacle& obstacle = obstacles_[i];
    QCHECK(!obstacle.points.empty());
    // Computing if an obstacle is mist.
    if (ObstacleIsLikelyMist(obstacle)) {
      obstacle.is_likely_mist = true;
    }
    // Obstacle level rain filter if it's onroad and within
    // kObstacleRainFilterDistance.
    if (obstacle.dist_to_curb < 0.0f &&
        obstacle.points[0].range < kObstacleRainFilterDistance &&
        obstacle.type != ObstacleProto::IGNORED) {
      // If the obstacle is likely mist, set type to ignored
      if (obstacle.is_likely_mist) {
        obstacle.type = ObstacleProto::IGNORED;
        obstacle.type_source = ObstacleProto::MIST_OBSTACLE_FILTER;
        return;
      }
      int sum_points_intens = 0;
      for (const auto& point : obstacle.points) {
        sum_points_intens += point.intensity;
      }
      const auto& pts = obstacle.points;
      const auto& pts_size = pts.size();
      float num_has_return_behind_pts = 0.0f;

      for (const auto& pt : pts) {
        if (pt.has_return_behind) {
          ++num_has_return_behind_pts;
        }
      }
      const auto& ratio_has_behind_pts =
          num_has_return_behind_pts / static_cast<float>(pts_size);

      bool has_found_box = IsPointInBoxes(fen_result.car_boxes, obstacle);
      if (!has_found_box) {
        has_found_box = IsPointInBoxes(fen_result.cyc_boxes, obstacle);
      }
      if (!has_found_box) {
        has_found_box = IsPointInBoxes(fen_result.ped_boxes, obstacle);
      }

      // Choose rain filter ratio by judging if the obstacle is in the det
      // box.
      const double obstacle_rain_filter_has_behind_ratio =
          has_found_box ? kObstacleRainFilterHasReturnBehindRatioInBox
                        : kObstacleRainFilterHasReturnBehindRatioOutBox;

      // Filter obstacles.
      if (ratio_has_behind_pts > obstacle_rain_filter_has_behind_ratio &&
          sum_points_intens <
              obstacle.points.size() * kMistObstacleMaxIntensity) {
        obstacle.type = ObstacleProto::IGNORED;
        obstacle.type_source = ObstacleProto::MIST_OBSTACLE_FILTER;
      }
    }
  });

  // Render mist obstacles cvs.
  if (FLAGS_obstacle_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/obstacle_manager");
    for (const auto& obstacle : obstacles_) {
      if (obstacle.is_likely_mist) {
        const std::vector<Vec3d> obstacle_contour{
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.max_z + 1.0},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.max_z + 1.0},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.max_z + 1.0},
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.max_z + 1.0},
        };
        canvas.DrawPolygon(obstacle_contour, vis::Color::kLightMagenta, 1);
      }
    }
  }
}

void ObstacleManager::MofNetRainFilter(
    const VehiclePose& pose,
    const FieryEyeNetClassifier::DetectionResult& fen_result,
    const std::unique_ptr<MistObstacleNet>& mof_net) {
  SCOPED_QTRACE_ARG1("ObstacleManager::MofNetRainFilter", "num_obstacles",
                     obstacles_.size());

  const int num_obstacles = obstacles_.size();
  std::vector<std::vector<float>> obstacle_train_points(num_obstacles);
  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    const auto& obstacle = obstacles_[i];
    QCHECK(!obstacle.points.empty());
    const bool has_found_box = IsPointInBoxes(fen_result.car_boxes, obstacle) ||
                               IsPointInBoxes(fen_result.cyc_boxes, obstacle) ||
                               IsPointInBoxes(fen_result.ped_boxes, obstacle);
    // Do not filter if it's offroad or out of range or ignored or
    // barrier by obstacle semantic manager.
    if (obstacle.dist_to_curb > 0.0f || has_found_box ||
        obstacle.points[0].range > kObstacleRainFilterDistance ||
        obstacle.type == ObstacleProto::IGNORED ||
        (obstacle.type == ObstacleProto::BARRIER &&
         obstacle.type_source == ObstacleProto::SEMANTIC_MAP_ZONE)) {
      return;
    }
    obstacle_train_points[i] = GetPointsAboveGroundWithNeighbor(pose, obstacle);
  });

  const int max_batch_size = MofNet::kMaxBatchSize;
  std::vector<int> origin_indices;
  origin_indices.reserve(max_batch_size);
  std::vector<std::vector<float>> one_batch_data;
  one_batch_data.reserve(max_batch_size);
  for (int all_index = 0; all_index < num_obstacles;) {
    int batch_index = 0;
    while (batch_index < max_batch_size && all_index < num_obstacles) {
      if (obstacle_train_points[all_index].empty()) {
        all_index++;
        continue;
      }
      origin_indices.push_back(all_index);
      one_batch_data.push_back(obstacle_train_points[all_index]);
      all_index++;
      batch_index++;
    }

    std::vector<float> result = mof_net->ClassifyMistObstacles(one_batch_data);
    for (int i = 0; i < result.size(); ++i) {
      auto& obstacle = obstacles_[origin_indices[i]];
      obstacle.mist_score = result[i];
      if (result[i] > 0.5f) {
        obstacle.is_likely_mist = true;
        obstacle.type = ObstacleProto::IGNORED;
        obstacle.type_source = ObstacleProto::MIST_OBSTACLE_NET_FILTER;
      }
    }
    one_batch_data.clear();
    origin_indices.clear();
  }

  if (FLAGS_obstacle_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/obstacle_manager");
    for (const auto& obstacle : obstacles_) {
      if (obstacle.type_source == ObstacleProto::MIST_OBSTACLE_NET_FILTER) {
        const std::vector<Vec3d> obstacle_contour{
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
        };
        canvas.DrawPolygon(obstacle_contour, vis::Color::kRed, 1);
      }
    }
  }
}

std::vector<float> ObstacleManager::GetPointsAboveGroundWithNeighbor(
    const VehiclePose& pose, const Obstacle& obstacle) {
  const int row = obstacle.row;
  const int col = obstacle.col;
  std::vector<ObstaclePtr> neighbor_obstacles;
  neighbor_obstacles.reserve(9);
  neighbor_obstacles.emplace_back(&obstacle);
  for (int r = row - 1; r <= row + 1; ++r) {
    for (int c = col - 1; c <= col + 1; ++c) {
      const auto neighbor = ObstacleAt(r, c);
      if (neighbor == nullptr || neighbor == &obstacle) continue;
      neighbor_obstacles.emplace_back(neighbor);
    }
  }
  if (neighbor_obstacles.size() == 1 && obstacle.points.size() < 3) {
    return {};
  }
  int all_points_cnt = 0;
  for (const auto& obs : neighbor_obstacles) {
    const float ground_z = obs->ground_z;
    for (const auto& point : obs->points) {
      if (point.z < ground_z) continue;
      all_points_cnt++;
    }
  }
  const int max_points_num = MofNet::kMaxPointsNum;
  std::vector<float> points(max_points_num * MofNet::kPointsDim, 0.f);
  std::vector<int> rand_indices =
      RandomShuffleNums(0, all_points_cnt, max_points_num);
  int rand_indices_idx = 0;
  int all_points_cnt_idx = 0;
  float neighbor_flag = 0.f;  // 0 for current points and 1 for neighbors.
  constexpr float kXNormVal = 0.005f;    // 1 / 200.f
  constexpr float kYNormVal = 0.00625f;  // 1 / 160.f
  constexpr float kZNormVal = 0.025f;    // 1 / 40.f
  constexpr float kIntensityNormVal = 1 / 255.f;
  for (const auto& obs : neighbor_obstacles) {
    const float ground_z = obs->ground_z;
    for (const auto& point : obs->points) {
      if (point.z < ground_z) continue;
      if (rand_indices_idx >= max_points_num) break;
      if (all_points_cnt_idx != rand_indices[rand_indices_idx]) {
        all_points_cnt_idx++;
        continue;
      }
      points[0 * max_points_num + rand_indices_idx] =
          (point.x - pose.x + 80.f) * kXNormVal;
      points[1 * max_points_num + rand_indices_idx] =
          (point.y - pose.y + 80.f) * kYNormVal;
      points[2 * max_points_num + rand_indices_idx] =
          (point.z - pose.z) * kZNormVal;
      points[3 * max_points_num + rand_indices_idx] =
          point.intensity * kIntensityNormVal;
      if (MofNet::kPointsDim == 6) {
        points[4 * max_points_num + rand_indices_idx] = point.return_index;
        points[5 * max_points_num + rand_indices_idx] = neighbor_flag;
      } else {
        points[4 * max_points_num + rand_indices_idx] = neighbor_flag;
      }
      all_points_cnt_idx++;
      rand_indices_idx++;
    }
    neighbor_flag = 1.f;
  }

  return points;
}

void ObstacleManager::FilterRainObstacleV2(
    const FieryEyeNetClassifier::DetectionResult& fen_result) {
  SCOPED_QTRACE_ARG1("ObstacleManager::FilterRainObstacleV2", "num_obstacles",
                     obstacles_.size());
  const int num_obstacles = obstacles_.size();

  // Compute per-obstacle raw mist score.
  std::vector<float> raw_mist_scores(num_obstacles, 0.);
  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    raw_mist_scores[i] = ComputeObstacleMistScore(obstacles_[i]);
  });

  // Compute mist score and filter out mist obstacles.
  std::vector<uint8_t> obstacle_in_bbox(num_obstacles, 0);
  std::vector<uint8_t> is_rain_obstacle(num_obstacles, false);
  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    auto& obstacle = obstacles_[i];
    QCHECK(!obstacle.points.empty());
    // Do not filter if it's offroad or out of range or already been ignored or
    // barrier by obstacle semantic manager.
    if (obstacle.dist_to_curb > 0.0f ||
        obstacle.points[0].range > kObstacleRainFilterDistance ||
        obstacle.type == ObstacleProto::IGNORED ||
        (obstacle.type == ObstacleProto::BARRIER &&
         obstacle.type_source == ObstacleProto::SEMANTIC_MAP_ZONE)) {
      return;
    }

    const auto weighted_combine = [&](const Obstacle& obstacle) {
      const int row = obstacle.row;
      const int col = obstacle.col;
      // A 5x5 guassian blur. μ = 0.0. σ = 1.0.
      // clang-format off
      constexpr float kGaussianBlurWeight[5][5] = {
        {1.00,  4.48,  7.39,  4.48, 1.00},
        {4.48, 20.09, 33.12, 20.09, 4.48},
        {7.39, 33.12, 54.60, 33.12, 7.39},
        {4.48, 20.09, 33.12, 20.09, 4.48},
        {1.00,  4.48,  7.39,  4.48, 1.00},
      };
      // clang-format on

      float total_weight = 0.;
      float total_score = 0.;
      for (int r = -2; r <= 2; ++r) {
        for (int c = -2; c <= 2; ++c) {
          ObstaclePtr obstacle = ObstacleAt(row + r, col + c);
          const float weight = kGaussianBlurWeight[r + 2][c + 2];
          if (obstacle == nullptr) {
            continue;
          }
          total_weight += weight;
          const int obstacle_ind = FindOrDie(obstacles_to_index_, obstacle);
          total_score += weight * raw_mist_scores[obstacle_ind];
        }
      }
      return total_weight > 0.f ? (total_score / total_weight) : 0.f;
    };
    // NOTE(dong): Do not weighted combine obstacles with 0 mist score. Some
    // real obstacles, such cones, may be surrounded by mist obstacles. Weighted
    // combining these obstacles may cause them to be filtered incorrectly. Here
    // we regard that obstacles with 0 mist score are unfilterable and don't
    // need to be weighted combined.
    obstacle.mist_score =
        raw_mist_scores[i] > 0.f ? weighted_combine(obstacle) : 0.f;

    const bool has_found_box = IsPointInBoxes(fen_result.car_boxes, obstacle) ||
                               IsPointInBoxes(fen_result.cyc_boxes, obstacle) ||
                               IsPointInBoxes(fen_result.ped_boxes, obstacle);

    obstacle_in_bbox[i] = has_found_box ? 1 : 0;

    const double mist_score_threshold = has_found_box
                                            ? kMinMistScoreToFilterInBox
                                            : kMinMistScoreToFilterOutBox;
    if (obstacle.mist_score > mist_score_threshold) {
      obstacle.is_likely_mist = true;
      obstacle.type = ObstacleProto::IGNORED;
      obstacle.type_source = ObstacleProto::MIST_OBSTACLE_FILTER_V2;
      is_rain_obstacle[i] = true;
    }
  });
  // Mist obstacle propagation. It may cause obstacle over filter.
  if (FLAGS_enable_obstacle_mist_propagation) {
    PropagateMistObstacles(obstacle_in_bbox);
  }

  if (FLAGS_obstacle_filtering_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/obstacle_manager");
    for (const auto& obstacle : obstacles_) {
      if (obstacle.is_likely_mist) {
        const std::vector<Vec3d> obstacle_contour{
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y - Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x + Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
            {obstacle.x - Obstacle::kDiameter * 0.5,
             obstacle.y + Obstacle::kDiameter * 0.5, obstacle.ground_z + 0.01},
        };
        canvas.DrawPolygon(obstacle_contour, vis::Color::kLightMagenta, 1);
      }
      canvas.DrawText(
          absl::StrFormat("weighted %.2f", obstacle.mist_score),
          {obstacle.x - 0.1, obstacle.y + 0.04, obstacle.ground_z + 0.01}, 0.0,
          0.02, vis::Color::kWhite);
    }
  }

  constexpr int kMinNumRainObstaclesSuspectedMistScenario = 500;
  const int num_rain_obstacles =
      std::count(is_rain_obstacle.begin(), is_rain_obstacle.end(), true);
  if (num_rain_obstacles >= kMinNumRainObstaclesSuspectedMistScenario) {
    QEVENT_EVERY_N_SECONDS("dongchen", "suspected_mist_scenario", /*sec*/ 5.0,
                           [=](QEvent* qevent) {
                             qevent->AddField("num_rain_obstacles",
                                              num_rain_obstacles);
                           });
  }
}

void ObstacleManager::PropagateMistObstacles(
    const std::vector<uint8_t>& obstacle_in_bbox) {
  SCOPED_QTRACE("ObstacleManager::PropagateMistObstacles");

  const int num_obstacles = obstacles_.size();
  ObstaclePtrs obstacle_ptrs;
  obstacle_ptrs.reserve(num_obstacles);
  std::vector<int> raw_obstacle_indices;
  raw_obstacle_indices.reserve(num_obstacles);
  for (int i = 0; i < num_obstacles; ++i) {
    if (obstacles_[i].dist_to_curb > 0.f) continue;
    if (obstacle_in_bbox[i] == 1) continue;
    obstacle_ptrs.emplace_back(&obstacles_[i]);
    raw_obstacle_indices.push_back(i);
  }
  if (obstacle_ptrs.empty()) return;

  const int num_obstacle_ptrs = obstacle_ptrs.size();
  obstacle_util::LocalObstacleGrid local_grid(obstacle_ptrs);
  std::vector<bool> processed(num_obstacle_ptrs, false);
  std::vector<int> all_mist_components;
  all_mist_components.reserve(num_obstacle_ptrs);
  constexpr float kMinPropagateMistScore = 0.28f;
  constexpr float kMaxMistScoreGradientDiff = 0.06f;
  for (int i = 0; i < num_obstacle_ptrs; ++i) {
    if (processed[i]) continue;
    const auto& obstacle = obstacle_ptrs[i];
    // TODO(zhenye): it may cause over filter.
    if (obstacle->mist_score <= 0.f) {
      const auto neighbor_indices =
          local_grid.FindNearestInRadius(obstacle->row, obstacle->col, 1);
      const int likely_mist_cnts = std::count_if(
          neighbor_indices.begin(), neighbor_indices.end(),
          [&](const int index) {
            return obstacles_[raw_obstacle_indices[index]].mist_score > 0.35f;
          });
      if (likely_mist_cnts + 1 >= neighbor_indices.size()) {
        processed[i] = true;
        all_mist_components.push_back(i);
        continue;
      }
    }

    if (!obstacle->is_likely_mist) continue;
    const auto raw_indices = local_grid.FindConnectedNeighbors(
        obstacle->row, obstacle->col, /*num_neighbors*/ 8,
        [&](const Obstacle& init, const Obstacle& seed,
            const Obstacle& neighbor) {
          return neighbor.mist_score > kMinPropagateMistScore &&
                 std::abs(seed.mist_score - neighbor.mist_score) <
                     kMaxMistScoreGradientDiff;
        });
    for (const auto index : raw_indices) {
      processed[index] = true;
    }
    if (raw_indices.size() == 1) continue;
    all_mist_components.insert(all_mist_components.end(), raw_indices.begin(),
                               raw_indices.end());
  }

  SCOPED_QTRACE("ObstacleManager::PropagateMistObstacles_2");

  for (const auto index : all_mist_components) {
    if (obstacles_[raw_obstacle_indices[index]].is_likely_mist) continue;
    const auto neighbor_indices = local_grid.FindNearestInRadius(
        obstacle_ptrs[index]->row, obstacle_ptrs[index]->col, 1);
    const float score_accums = std::accumulate(
        neighbor_indices.begin(), neighbor_indices.end(), 0.f,
        [&](const float init, const int index) {
          const float mist_score =
              obstacles_[raw_obstacle_indices[index]].mist_score;
          return init + std::max(mist_score, 0.0f);
        });

    if (FLAGS_obstacle_filtering_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/obstacle_manager");
      canvas.DrawText(absl::StrFormat("%.2f", score_accums),
                      {obstacles_[raw_obstacle_indices[index]].coord(), 0.0},
                      0.0, 0.03, vis::Color::kWhite);
      canvas.DrawBox({obstacles_[raw_obstacle_indices[index]].coord(), 0.0},
                     0.0, {0.2, 0.2}, vis::Color::kRed);
    }
    constexpr float kAveMistScore = 0.32f;
    if (score_accums < kAveMistScore * neighbor_indices.size()) continue;
    obstacles_[raw_obstacle_indices[index]].is_likely_mist = true;
    obstacles_[raw_obstacle_indices[index]].type = ObstacleProto::IGNORED;
    obstacles_[raw_obstacle_indices[index]].type_source =
        ObstacleProto::MIST_OBSTACLE_FILTER_V2;
  }
}

void ObstacleManager::FilterWithObstacleSemanticMap(
    const VehiclePose& pose,
    const mapping::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter) {
  SCOPED_QTRACE_ARG1("ObstacleManager::ObstacleSemanticMapFilter",
                     "num_obstacles", obstacles_.size());
  const int num_obstacles = obstacles_.size();

  const auto ignorance_zones = CollectNearPerceptionZones(
      semantic_map_manager, mapping::PerceptionZoneProto::IGNORANCE, pose,
      coordinate_converter);
  const auto reflection_zones = CollectNearPerceptionZones(
      semantic_map_manager, mapping::PerceptionZoneProto::REFLECTION, pose,
      coordinate_converter);

  auto update_and_render_ignored_obstacle = [](Obstacle* obstacle) {
    obstacle->type = ObstacleProto::IGNORED;
    obstacle->type_source = ObstacleProto::SEMANTIC_MAP_ZONE;
    // Render noisy obstacles cvs.
    if (FLAGS_obstacle_filtering_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/obstacle_manager");
      const std::vector<Vec3d> obstacle_contour =
          obstacle_util::ComputeContourWithZ(*obstacle, obstacle->max_z + 1.0);
      canvas.DrawPolygon(obstacle_contour, vis::Color::kOrange, 1);
    }
  };

  const auto ref_lidar_pos =
      projection_util::GetLidarViewPointInVehicleCoord(lidar_params_);
  const Vec2d ref_pos =
      pose.ToTransform().TransformPoint(ref_lidar_pos).head<2>();

  ParallelFor(0, num_obstacles, thread_pool_, [&](int i) {
    Obstacle& obstacle = obstacles_[i];
    QCHECK(!obstacle.points.empty());

    if (obstacle.type == ObstacleProto::OFFROAD) return;

    // Ignore obstacles in ignorance zones.
    if (PointInZones({obstacle.x, obstacle.y}, ignorance_zones)) {
      update_and_render_ignored_obstacle(&obstacle);
      return;
    }

    // Ignore obstacles that occluded by reflection zones.
    if (IsOccludedByReflectionZones({obstacle.x, obstacle.y}, ref_pos,
                                    reflection_zones)) {
      update_and_render_ignored_obstacle(&obstacle);
      return;
    }
  });
}

void ObstacleManager::BlacklistObstaclesWithinSegmentationNoiseClusters(
    const std::map<ProposerType, SegmentedClusters>& pt_clusters) {
  for (const auto& [ptype, clusters] : pt_clusters) {
    ObstacleProto::TypeSource type_source;
    switch (ptype) {
      case ProposerType::PT_REFLECTION:
        type_source = ObstacleProto::REFLECTION_PROPOSER;
        break;
      case ProposerType::PT_BLOOMING:
        type_source = ObstacleProto::BLOOMING_PROPOSER;
        break;
      default:
        type_source = ObstacleProto::SEGMENTATION_NOISE;
        break;
    }
    for (const auto& cluster : clusters) {
      for (const auto* obstacle_ptr : cluster.obstacles()) {
        const int obstacle_ind = FindOrDie(obstacles_to_index_, obstacle_ptr);
        Obstacle& obstacle = *mutable_obstacle(obstacle_ind);
        obstacle.type = ObstacleProto::IGNORED;
        obstacle.type_source = type_source;
      }
    }
  }
}

}  // namespace qcraft
