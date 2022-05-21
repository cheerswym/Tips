#include "onboard/perception/pose_correction.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/vehicle_pose_util.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/perception/registration/icp.h"
#include "onboard/perception/registration/icp_with_normal.h"
#include "onboard/perception/registration/registration_common_tool.h"
#include "onboard/perception/semantic_map_util.h"
#include "onboard/utils/time_util.h"

DEFINE_bool(pose_correction_cvs, false,
            "Render related points in pose correction");
DEFINE_bool(use_point_to_plane_icp_for_pose_correction, true,
            "Enable icp with normal for pose correction.");

DECLARE_bool(collect_cluster_data);

namespace qcraft {

using namespace perception_semantic_map_util;  // NOLINT

namespace {

// Maximum tilting angle between sampled points and map ground.
constexpr double kSampledPointMaxAngleDiff = d2r(7.5);

constexpr double kEnhancedSamplingRoiMinRadiusSqr = 20 * 20;
constexpr double kEnhancedSamplingRoiMaxRadiusSqr = 70 * 70;

constexpr double kMaxMatchResultDelay = 1.0;  // s

Vec3d ComputeSurfaceNormal(const std::vector<Vec3d>& points) {
  // Compute covariance.
  Mat3d cov(3, 3);
  cov.setZero();
  const Vec3d mean =
      std::accumulate(points.begin(), points.end(), Vec3d()) / points.size();
  for (const auto& point : points) {
    const auto v = point - mean;
    cov += v * v.transpose();
  }
  cov /= points.size();

  // Solve SVD.
  const Eigen::EigenSolver<Mat3d> eig(cov);
  const std::array<double, 3> svals{eig.eigenvalues()(0).real(),
                                    eig.eigenvalues()(1).real(),
                                    eig.eigenvalues()(2).real()};
  const int col = std::distance(svals.begin(),
                                std::min_element(svals.begin(), svals.end()));
  return {eig.eigenvectors()(0, col).real(), eig.eigenvectors()(1, col).real(),
          eig.eigenvectors()(2, col).real()};
}

std::optional<Vec3d> GenerateElevationNormal(
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter,
    const LocalImagery& local_imagery) {
  // Sample points from elevation map at the surrounding of AV to compute
  // surface normal of the ground.
  const auto pose_transform = pose.ToTransform();
  constexpr double kElevationMapXOffset = 5.;            // m
  constexpr double kElevationMapYOffset = 2.;            // m
  constexpr double kElevationSamplingGranularity = 0.5;  // m
  const int num_elevation_map_points =
      (2 * kElevationMapYOffset / kElevationSamplingGranularity + 1) *
      (2 * kElevationMapXOffset / kElevationSamplingGranularity + 1);
  std::vector<Vec3d> vehicle_elevation_map_points;
  vehicle_elevation_map_points.reserve(num_elevation_map_points);
  for (double x_offset = -kElevationMapXOffset;
       x_offset <= kElevationMapXOffset;
       x_offset += kElevationSamplingGranularity) {
    for (double y_offset = -kElevationMapYOffset;
         y_offset <= kElevationMapYOffset;
         y_offset += kElevationSamplingGranularity) {
      const auto& surrounding_point_local_frame =
          pose_transform.TransformPoint({x_offset, y_offset, 0.});
      const auto& surrounding_point_global_frame =
          coordinate_converter.SmoothToGlobal(surrounding_point_local_frame);
      if (const auto indexer =
              local_imagery.GetIndexer(surrounding_point_global_frame.x(),
                                       surrounding_point_global_frame.y(),
                                       coordinate_converter.GetLevel())) {
        const auto& vehicle_elevation_map_point =
            coordinate_converter.GlobalToSmooth(
                {surrounding_point_global_frame.x(),
                 surrounding_point_global_frame.y(),
                 local_imagery.ElevationAt(*indexer)});
        vehicle_elevation_map_points.push_back(vehicle_elevation_map_point);
      }
    }
  }
  if (vehicle_elevation_map_points.size() <
      static_cast<int>(num_elevation_map_points * 0.5)) {
    return std::nullopt;
  }
  // Compute surface normal given elevation map, use z axis in local frame if we
  // could not obtain elevation map.
  Vec3d elevation_map_normal =
      ComputeSurfaceNormal(vehicle_elevation_map_points);
  if (elevation_map_normal.z() < 0.) {
    elevation_map_normal = -elevation_map_normal;
  }
  elevation_map_normal /= elevation_map_normal.norm();

  if (FLAGS_pose_correction_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/pose_correction");
    canvas.DrawPoints(vehicle_elevation_map_points, vis::Color::kBlue, 2);
    canvas.DrawLine(pose.coord(), pose.coord() + elevation_map_normal,
                    vis::Color::kDarkYellow, 5);
  }

  return elevation_map_normal;
}

std::vector<Vec3d> GenerateSampledGroundPoints(
    const VehiclePose& pose, const std::vector<LaserPoint>& points,
    const ObstacleGrid& obstacle_grid, const Vec3d& elevation_map_normal,
    const std::vector<Polygon2d>& parking_areas, ThreadPool* thread_pool) {
  SCOPED_QTRACE("PoseCorrection::GenerateSampledGroundPoints");
  const int kSamplingStepSize = 3;
  static const double max_angle_diff_sin_sqr =
      Sqr(fast_math::Sin(kSampledPointMaxAngleDiff));

  const int num_workers = thread_pool->NumWorkers() + 1;
  std::vector<std::vector<Vec3d>> sampled_points_all_workers(num_workers);
  // Subsample points from a subregion (150m x 150m with AV in center) of the
  // obstacle detection region.
  constexpr int kObstacleSamplingSubregionSize = 150.0 / Obstacle::kDiameter;
  ParallelFor(
      0, kObstacleSamplingSubregionSize / kSamplingStepSize, thread_pool,
      [&](int worker_index, int i) {
        if (sampled_points_all_workers[worker_index].capacity() == 0) {
          sampled_points_all_workers[worker_index].reserve(points.size() / 16);
        }
        const int r =
            (obstacle_grid.width() - kObstacleSamplingSubregionSize) / 2 +
            i * kSamplingStepSize;
        for (int c =
                 (obstacle_grid.height() - kObstacleSamplingSubregionSize) / 2;
             c < (obstacle_grid.height() + kObstacleSamplingSubregionSize) / 2;
             c += kSamplingStepSize) {
          if (!obstacle_grid.IsValid(r, c)) continue;
          const auto& info = obstacle_grid(r, c);
          // Check if this obstacle is on road or not, or contains any point.
          if (info.point_indices.empty() || info.dist_to_curb_cm > 0) continue;
          const auto [min_index, max_index] = std::minmax_element(
              info.point_indices.begin(), info.point_indices.end(),
              [&points](const auto lhs, const auto rhs) {
                return CompareLaserPointAtZ(points[lhs], points[rhs]);
              });
          // Pick the lowest point in the obstacle.
          const auto& point = points[*min_index];
          // Ignore points in parking areas (maps may not contain correct
          // elevation for parking areas).
          if (PointInZones({point.coord().x(), point.coord().y()},
                           parking_areas)) {
            continue;
          }
          // Only use thin obstacles.
          const float min_z = points[*min_index].z;
          const float max_z = points[*max_index].z;
          constexpr float kMaxGroundObstacleDiffZ = 0.1;
          if (max_z - min_z > kMaxGroundObstacleDiffZ) {
            continue;
          }
          // Only use obstacle not too high above its neighbors.
          float neighbor_min_z = point.z;
          for (int nr = r - 1; nr <= r + 1; ++nr) {
            for (int nc = c - 1; nc <= c + 1; ++nc) {
              if (nr == r && nc == c) {
                continue;
              }

              const auto& info = obstacle_grid(nr, nc);
              if (info.point_indices.empty()) {
                continue;
              }

              const auto neighbor_min_index = std::min_element(
                  info.point_indices.begin(), info.point_indices.end(),
                  [&points](auto lhs, auto rhs) {
                    return points[lhs].z < points[rhs].z;
                  });
              neighbor_min_z =
                  std::min(neighbor_min_z, points[*neighbor_min_index].z);
            }
          }
          constexpr float kMaxNeighborHeightDiff = 0.1;  // m
          if (point.z - neighbor_min_z > kMaxNeighborHeightDiff) {
            continue;
          }
          //
          const Vec3d v2point_vec_in_local_coord = point.coord() - pose.coord();
          // Filter out points with big tilting angle to ground, which are
          // likely non-ground points.
          // The following comparison is equivalent to
          //  v2point_vec_in_local_coord.dot(elevation_map_normal) /
          //  v2point_vec_in_local_coord.norm() <=
          //  fast_math::Sin(kSampledPointMaxAngleDiff)
          if (Sqr(v2point_vec_in_local_coord.dot(elevation_map_normal)) <=
              max_angle_diff_sin_sqr *
                  v2point_vec_in_local_coord.squaredNorm()) {
            sampled_points_all_workers[worker_index].push_back(point.coord());
          }
        }
      });

  // Collect points from all workers.
  int num_points = 0;
  for (const auto& points : sampled_points_all_workers) {
    num_points += points.size();
  }
  std::vector<Vec3d> sampled_ground_points;
  sampled_ground_points.reserve(num_points);
  for (auto& points : sampled_points_all_workers) {
    sampled_ground_points.insert(sampled_ground_points.end(), points.begin(),
                                 points.end());
  }
  // Sort the points to get deterministic results.
  std::sort(sampled_ground_points.begin(), sampled_ground_points.end());

  return sampled_ground_points;
}
// Note(dong): Balance points num at front & at back.
std::vector<Vec3d> BalanceSampledGroundPoints(
    const VehiclePose& pose, const std::vector<Vec3d>& sampled_ground_points) {
  std::vector<Vec3d> points_at_front, points_at_back;
  points_at_front.reserve(sampled_ground_points.size());
  points_at_back.reserve(sampled_ground_points.size());
  for (const auto& point : sampled_ground_points) {
    const Vec2d point_in_vehicle =
        Vec2d(point.x() - pose.x, point.y() - pose.y).FastRotate(-pose.yaw);
    if (point_in_vehicle.x() > 0) {
      points_at_front.emplace_back(point);
    } else {
      points_at_back.emplace_back(point);
    }
  }
  constexpr double kMaxFrontToBackNumRatio = 2.0;
  if (points_at_front.size() <=
          points_at_back.size() * kMaxFrontToBackNumRatio &&
      points_at_back.size() <=
          points_at_front.size() * kMaxFrontToBackNumRatio) {
    return sampled_ground_points;
  }
  constexpr int kMinNumPoints = 100;
  if (points_at_front.size() > points_at_back.size()) {
    const int target_num_points =
        points_at_back.size() * kMaxFrontToBackNumRatio;
    if (target_num_points > kMinNumPoints) {
      reg_tool::DownSamplePointsUsingFisherYatesShuffle(&points_at_front,
                                                        target_num_points);
    }
  } else if (points_at_back.size() > points_at_front.size()) {
    const int target_num_points =
        points_at_front.size() * kMaxFrontToBackNumRatio;
    if (target_num_points > kMinNumPoints) {
      reg_tool::DownSamplePointsUsingFisherYatesShuffle(&points_at_back,
                                                        target_num_points);
    }
  } else {
    QLOG(FATAL) << "Should not reach here."
                << " points_at_back size: " << points_at_back.size()
                << " points_at_front size: " << points_at_front.size();
  }
  std::vector<Vec3d> balanced_points;
  balanced_points.reserve(sampled_ground_points.size());
  for (const auto& point : points_at_front) {
    balanced_points.emplace_back(point);
  }
  for (const auto& point : points_at_back) {
    balanced_points.emplace_back(point);
  }

  return balanced_points;
}

PointMatchResult MatchGroundPoints(
    const VehiclePose& pose, const std::vector<Vec3d>& gt_ground_points,
    const std::vector<Vec3d>& sampled_ground_points) {
  SCOPED_QTRACE_ARG2("MatchGroundPoints", "num_points",
                     sampled_ground_points.size(), "map_points",
                     gt_ground_points.size());
  PointMatcherOptions options;
  options.max_mse = 0.001;
  options.max_num_iters = 20;
  options.max_num_points = 1000;
  options.max_matching_dist = FLAGS_collect_cluster_data ? 2.5 : 2.0;
  PointMatchResult match_result;
  if (LIKELY(FLAGS_use_point_to_plane_icp_for_pose_correction)) {
    const IcpWithNormal point_matcher;
    match_result =
        point_matcher.MatchPoints(gt_ground_points, sampled_ground_points,
                                  options, pose.ToTransform().Inverse());
  } else {
    const Icp point_matcher;
    match_result = point_matcher.MatchPoints(gt_ground_points,
                                             sampled_ground_points, options);
  }
  return match_result;
}

PoseCorrectionDebugProto GeneratePoseCorrectionDebugProto(
    const VehiclePose& pose, const Vec3d& elevation_map_normal,
    const PointMatchResult& match_result,
    const std::vector<Vec3d>& sampled_ground_points,
    const std::vector<Vec3d>& gt_ground_points) {
  PoseCorrectionDebugProto pose_correction_debug_proto;
  *(pose_correction_debug_proto.mutable_origin_pose()) =
      pose.ToVehiclePoseProto();
  Vec3dToProto(elevation_map_normal,
               pose_correction_debug_proto.mutable_elevation_map_normal());
  // Sync sample points to debug proto.
  pose_correction_debug_proto.set_sampled_ground_points_num(
      sampled_ground_points.size());
  pose_correction_debug_proto.set_mean_square_error(match_result.mse);
  pose_correction_debug_proto.set_num_matched_points(
      match_result.num_matched_points);
  pose_correction_debug_proto.set_success(match_result.success);
  pose_correction_debug_proto.set_num_iteration(match_result.num_iteration);
  pose_correction_debug_proto.set_matching_dist(match_result.matching_dist);

  constexpr int kMaxNumDebugPoints = 200;
  std::vector<Vec3d> sampled_ground_points_for_debug = sampled_ground_points;
  if (sampled_ground_points_for_debug.size() > kMaxNumDebugPoints) {
    reg_tool::DownSamplePointsUsingFixedStep(&sampled_ground_points_for_debug,
                                             kMaxNumDebugPoints);
  }
  std::vector<Vec3d> gt_ground_points_for_debug = gt_ground_points;
  if (gt_ground_points_for_debug.size() > kMaxNumDebugPoints) {
    reg_tool::DownSamplePointsUsingFixedStep(&gt_ground_points_for_debug,
                                             kMaxNumDebugPoints);
  }
  QCHECK_EQ(sampled_ground_points_for_debug.size(),
            gt_ground_points_for_debug.size());
  for (const auto& sampled_ground_point : sampled_ground_points_for_debug) {
    auto* sampled_ground_point_mm =
        pose_correction_debug_proto.add_sampled_ground_points_mm();
    sampled_ground_point_mm->set_x_mm(
        RoundToInt(sampled_ground_point.x() * 1000));
    sampled_ground_point_mm->set_y_mm(
        RoundToInt(sampled_ground_point.y() * 1000));
    sampled_ground_point_mm->set_z_mm(
        RoundToInt(sampled_ground_point.z() * 1000));
    const auto& transformed_point =
        match_result.transform.TransformPoint(sampled_ground_point);
    auto* transformed_ground_point_mm =
        pose_correction_debug_proto.add_transformed_ground_points_mm();
    transformed_ground_point_mm->set_x_mm(
        RoundToInt(transformed_point.x() * 1000));
    transformed_ground_point_mm->set_y_mm(
        RoundToInt(transformed_point.y() * 1000));
    transformed_ground_point_mm->set_z_mm(
        RoundToInt(transformed_point.z() * 1000));
  }
  for (const auto& gt_ground_point : gt_ground_points_for_debug) {
    auto* gt_ground_point_mm =
        pose_correction_debug_proto.add_gt_ground_points_mm();
    gt_ground_point_mm->set_x_mm(RoundToInt(gt_ground_point.x() * 1000));
    gt_ground_point_mm->set_y_mm(RoundToInt(gt_ground_point.y() * 1000));
    gt_ground_point_mm->set_z_mm(RoundToInt(gt_ground_point.z() * 1000));
  }

  return pose_correction_debug_proto;
}

bool IsStaleMatchResult(const double timestamp) {
  return ToUnixDoubleSeconds(Clock::Now()) - timestamp > kMaxMatchResultDelay;
}

void RemoveStaleMatchResult(PoseCorrectionResultHistory* result_history) {
  QCHECK_NOTNULL(result_history);
  auto it = result_history->begin();
  while (it != result_history->end()) {
    if (IsStaleMatchResult(it->timestamp)) {
      it = result_history->erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace

std::optional<PointMatchResult> PoseCorrection::CorrectPose(
    const VehiclePose& pose, const std::vector<LaserPoint>& points,
    const mapping::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter,
    const ObstacleGrid& obstacle_grid, const LocalImagery& local_imagery,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("PoseCorrection::CorrectPose");

  PointMatchResult match_result;
  do {
    const auto& elevation_map_normal =
        GenerateElevationNormal(pose, coordinate_converter, local_imagery);
    if (!elevation_map_normal) {
      QLOG(ERROR) << "Couldn't sample enough points from elevation map to "
                     "estimate surface normal.";
      break;
    }

    const auto parking_areas = CollectNearParkingAreas(
        semantic_map_manager, pose, coordinate_converter);
    auto sampled_ground_points = GenerateSampledGroundPoints(
        pose, points, obstacle_grid, *elevation_map_normal, parking_areas,
        thread_pool);

    constexpr int kMinNumGroundPoints = 50;
    if (sampled_ground_points.size() < kMinNumGroundPoints) {
      QLOG(ERROR)
          << "Couldn't sample enough ground points. Need to check if "
             "localization or map is invalid. sampled_ground_points size:"
          << sampled_ground_points.size();
      break;
    }

    sampled_ground_points =
        BalanceSampledGroundPoints(pose, sampled_ground_points);

    std::vector<Vec3d> gt_ground_points(sampled_ground_points.size());
    int counter = 0;
    for (const auto& point : sampled_ground_points) {
      const Vec3d point_global = coordinate_converter.SmoothToGlobal(point);
      if (const auto indexer =
              local_imagery.GetIndexer(point_global.x(), point_global.y(),
                                       coordinate_converter.GetLevel())) {
        const Vec3d map_point_smooth = coordinate_converter.GlobalToSmooth(
            {point_global.x(), point_global.y(),
             local_imagery.ElevationAt(*indexer)});
        gt_ground_points[counter] = map_point_smooth;
        sampled_ground_points[counter] = point;
        ++counter;
      }
    }
    gt_ground_points.resize(counter);
    sampled_ground_points.resize(counter);

    if (gt_ground_points.size() < kMinNumGroundPoints) {
      QLOG(ERROR) << "Couldn't sample enough points from imagery. "
                     "gt_ground_points size: "
                  << gt_ground_points.size();
      break;
    }

    match_result =
        MatchGroundPoints(pose, gt_ground_points, sampled_ground_points);

    pose_correction_debug_proto_ = GeneratePoseCorrectionDebugProto(
        pose, *elevation_map_normal, match_result, sampled_ground_points,
        gt_ground_points);

    if (FLAGS_pose_correction_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/pose_correction");
      canvas.DrawPoints(sampled_ground_points, vis::Color::kRed, 5);
      canvas.DrawPoints(gt_ground_points, vis::Color::kGreen, 5);
      std::vector<Vec3d> transformed;
      transformed.reserve(sampled_ground_points.size());
      for (const auto& point : sampled_ground_points) {
        transformed.emplace_back(match_result.transform.TransformPoint(point));
      }
      canvas.DrawPoints(transformed, vis::Color::kYellow, 5);
      canvas.DrawCircle({pose.x, pose.y, pose.z + 0.05},
                        std::sqrt(kEnhancedSamplingRoiMinRadiusSqr),
                        vis::Color::kGreen);
      canvas.DrawCircle({pose.x, pose.y, pose.z + 0.05},
                        std::sqrt(kEnhancedSamplingRoiMaxRadiusSqr),
                        vis::Color::kGreen);
    }
  } while (false);

  const auto& match_result_from_history = UpdateAndGetMatchResultFromHistory(
      match_result, coordinate_converter.localization_transform());

  if (!match_result_from_history) {
    QLOG(ERROR) << "Can't get valid match result from pose correction history.";
  }

  return match_result_from_history;
}

std::optional<PointMatchResult>
PoseCorrection::UpdateAndGetMatchResultFromHistory(
    const PointMatchResult& match_result,
    const LocalizationTransformProto& localization_transform) {
  RemoveStaleMatchResult(&pose_correction_result_history_);

  if (!match_result.success) {
    QLOG(WARNING) << absl::StrFormat(
        "Pose correction match failed. Match result: %s. Match result history "
        "size: %d.",
        match_result.DebugString(), pose_correction_result_history_.size());
    return pose_correction_result_history_.empty()
               ? std::nullopt
               : std::make_optional(
                     pose_correction_result_history_.back().match_result);
  }

  if (pose_correction_result_history_.empty()) {
    pose_correction_result_history_.push_back(
        {.timestamp = ToUnixDoubleSeconds(Clock::Now()),
         .match_result = match_result,
         .localization_transform = localization_transform});
    return pose_correction_result_history_.back().match_result;
  }
  // Use axis angle to represent the difference between two rotations.
  // Don't interpolate if rotation has a significant change.
  const auto trans_diff =
      pose_correction_result_history_.back().match_result.transform.Inverse() *
      match_result.transform;
  const double rotation_angle = r2d(std::abs(trans_diff.GetRotationAngle()));
  constexpr double kMaxRotationAngle = 1.0;  // degree
  if (rotation_angle > kMaxRotationAngle) {
    QLOG(WARNING) << "Pose Correction Rotation Angle: " << rotation_angle;
    pose_correction_result_history_.push_back(
        {.timestamp = ToUnixDoubleSeconds(Clock::Now()),
         .match_result = match_result,
         .localization_transform = localization_transform});
    return pose_correction_result_history_.back().match_result;
  }
  // Don't interpolate if localization_transform z_diff has a significant
  // change.
  const double prev_z =
      pose_correction_result_history_.back().localization_transform.z_diff();
  const double curr_z = localization_transform.z_diff();
  const double diff_z = std::abs(curr_z - prev_z);
  constexpr double kMaxLocalizationTransformDiffZ = 0.1;  // m
  if (diff_z > kMaxLocalizationTransformDiffZ) {
    QLOG(WARNING) << "Pose Correction Diff Z: " << diff_z
                  << " Curr z_diff: " << curr_z << " Prev z_diff: " << prev_z;
    pose_correction_result_history_.push_back(
        {.timestamp = ToUnixDoubleSeconds(Clock::Now()),
         .match_result = match_result,
         .localization_transform = localization_transform});
    return pose_correction_result_history_.back().match_result;
  }
  // Interpolate smoothness.
  constexpr float kInterpolateRatio = 0.5;
  const auto inter_transform =
      InterpolateVehiclePoseUsingSpherial(
          VehiclePose::FromTransform(
              pose_correction_result_history_.back().match_result.transform),
          VehiclePose::FromTransform(match_result.transform), kInterpolateRatio)
          .ToTransform();

  PointMatchResult inter_match_result = match_result;
  inter_match_result.transform = inter_transform;

  pose_correction_result_history_.push_back(
      {.timestamp = ToUnixDoubleSeconds(Clock::Now()),
       .match_result = inter_match_result,
       .localization_transform = localization_transform});

  return pose_correction_result_history_.back().match_result;
}

}  // namespace qcraft
