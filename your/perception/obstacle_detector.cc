#include "onboard/perception/obstacle_detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/perception/obstacle_constants.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/semantic_map_util.h"
#include "onboard/proto/mapping.pb.h"
#include "onboard/utils/file_util.h"
#include "s2/s2cell.h"
#include "s2/s2latlng.h"

DEFINE_bool(enable_ground_obstacle_propagation, true,
            "Whether to propogate ground obstacle attribute.");
DEFINE_bool(obstacle_detector_cvs, false, "Render obstacle detector cvs");
DEFINE_bool(obstacle_grid_cvs, false, "Render obstacle grid.");
DEFINE_bool(blind_lidar_valid_range_cvs, false,
            "Render blind lidar valid range");
DEFINE_bool(collect_obstacles_data, false,
            "Collect all obstacles information for training filters.");
DEFINE_string(obstacles_data_file_name, "",
              "File name to save obstacles data.");

namespace qcraft {

using namespace perception_semantic_map_util;  // NOLINT

namespace {
// Only keep points that are below ground level + kMaxPointHeight.
constexpr float kMaxPointHeight = 3.2f;  // m

// Use a considerably large value for default offroad value when there's no
// elevation map information for a specific obstacle.
// NOTE(yu): Not to use FLT_MAX / std::numeric_limits<float>::max() to avoid
// undefined behavior when converting it to int value in cm in downstream logic.
constexpr float kDefaultOffroadDistToCurbValue = 1e4;  // m

Box2d ComputeAvBox(const std::vector<LidarFrame>& lidar_frames,
                   const VehicleParamApi& vehicle_params) {
  Box2d av_box;
  const auto& first_pose = lidar_frames[0].StartPose();
  const auto& last_pose = lidar_frames[0].EndPose();
  const Vec3d pose_diff = last_pose.coord() - first_pose.coord();
  // Check if AV is reversing. OK if pose_diff is zero.
  const bool reversing =
      std::abs(NormalizeAngle(Vec2d(pose_diff).FastAngle() - first_pose.yaw)) >
      M_PI_2;
  // Read vehicle_geometry_params to get the AV contour box info. For old runs,
  // this param is not set, and we make an estimation then.
  QCHECK(vehicle_params.has_vehicle_geometry_params());
  const auto& geometry_params = vehicle_params.vehicle_geometry_params();
  // Compensate rotation offset. Assume roll approximately equals to rotation
  // angle.
  const auto& mid_pose = lidar_frames[0].MidPose();
  const double lateral_offset =
      std::sin(-mid_pose.roll) * geometry_params.height();
  const double av_width =
      geometry_params.width() + std::abs(lateral_offset) + 0.1;
  // Compensate the AV motion.
  const double av_length = geometry_params.length() + pose_diff.norm();
  const double center_to_rac_dist =
      av_length * 0.5 - geometry_params.back_edge_to_center();
  const auto& pose = reversing ? last_pose : first_pose;
  av_box = Box2d({pose.x, pose.y}, pose.yaw, av_length, av_width);

  av_box.Shift(
      Vec2d(center_to_rac_dist, lateral_offset * 0.5).FastRotate(pose.yaw));
  return av_box;
}

Box2d ComputeAvBoxForRayTracing(Vec2d lidar_pos,
                                const Box2d& av_box_for_current_lidar) {
  Box2d av_box_for_raytracing(av_box_for_current_lidar);
  // NOTO(dong): Lidar's location could be inside the av box, which make all
  // segment between lidar and point intersect with the av box. In this case,
  // we shrink the av box depending on point distance to box edges.
  if (av_box_for_raytracing.IsPointIn(lidar_pos)) {
    Vec2d rotated_lidar_pos = Vec2d(lidar_pos - av_box_for_raytracing.center())
                                  .FastRotate(-av_box_for_raytracing.heading());
    double length_dis = std::abs(std::abs(rotated_lidar_pos.x()) -
                                 av_box_for_raytracing.half_length());
    double width_dis = std::abs(std::abs(rotated_lidar_pos.y()) -
                                av_box_for_raytracing.half_width());

    QCHECK_LE(length_dis, av_box_for_raytracing.half_length());
    QCHECK_LE(width_dis, av_box_for_raytracing.half_width());

    constexpr double kReservedGapToBoxEdge = 0.05;
    length_dis = std::min(length_dis + kReservedGapToBoxEdge,
                          av_box_for_raytracing.half_length());
    width_dis = std::min(width_dis + kReservedGapToBoxEdge,
                         av_box_for_raytracing.half_width());

    if (length_dis > av_box_for_raytracing.half_length() / 2 &&
        width_dis < length_dis) {
      av_box_for_raytracing.LateralExtend(-2 * width_dis);
    } else if (width_dis > av_box_for_raytracing.half_width() / 2 &&
               length_dis < width_dis) {
      av_box_for_raytracing.LongitudinalExtend(-2 * length_dis);
    } else {
      av_box_for_raytracing.LateralExtend(-2 * width_dis);
      av_box_for_raytracing.LongitudinalExtend(-2 * length_dis);
    }
  }
  return av_box_for_raytracing;
}

Box2d GetBlindLidarRoi(const LidarModel lidar_type, const Box2d& av_box) {
  double blind_lidar_roi_front = 0.0;
  double blind_lidar_roi_behind = 0.0;
  double blind_lidar_roi_lateral = 0.0;
  switch (lidar_type) {
    case LIDAR_OS0_128:
    case LIDAR_OS0_64:
      blind_lidar_roi_front = 12.0;
      blind_lidar_roi_behind = 30.0;
      blind_lidar_roi_lateral = 10.0;
      break;
    case LIDAR_PANDAR_QT:
    case LIDAR_RS_BPEARL:
    case LIDAR_PANDAR_QT128:
      blind_lidar_roi_front = 50.0;
      blind_lidar_roi_behind = 50.0;
      blind_lidar_roi_lateral = 50.0;
      break;
    default:
      QLOG(FATAL) << "should not reach here.";
  }
  const Box2d blind_lidar_roi = obstacle_util::CreateRegionBox(
      av_box.center(), av_box.heading(), blind_lidar_roi_front,
      blind_lidar_roi_behind, blind_lidar_roi_lateral);

  if (UNLIKELY(FLAGS_blind_lidar_valid_range_cvs)) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/blind_lidar_valid_range");
    canvas.DrawBox({blind_lidar_roi.center(), 0}, blind_lidar_roi.heading(),
                   {blind_lidar_roi.length(), blind_lidar_roi.width()},
                   vis::Color::kGreen);
  }

  return blind_lidar_roi;
}

void MaybeRenderSdcContour(const Box2d& av_box, const VehiclePose& pose) {
  if (!FLAGS_obstacle_detector_cvs) return;
  // Draw AV contour.
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_detector");
  const Polygon2d av_polygon(av_box);
  std::vector<Vec3d> av_polygon_corners;
  for (const auto& point : av_polygon.points()) {
    av_polygon_corners.emplace_back(point.x(), point.y(), pose.z);
  }
  canvas.DrawPolygon(av_polygon_corners, vis::Color::kGreen, 1);
}

void MaybeRenderDetectionRegion(const Box2d& detection_region,
                                const VehiclePose& pose) {
  if (!FLAGS_obstacle_detector_cvs) return;
  // Draw AV contour.
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_detector");
  const Polygon2d detection_region_polygon(detection_region);
  std::vector<Vec3d> detection_region_corners;
  for (const auto& point : detection_region_polygon.points()) {
    detection_region_corners.emplace_back(point.x(), point.y(), pose.z);
  }
  canvas.DrawPolygon(detection_region_corners, vis::Color::kBlue, 1);
}

void MaybeRenderObstacleGrid(const ObstacleGrid& obstacle_grid,
                             const std::vector<LaserPoint>& points) {
  if (!FLAGS_obstacle_grid_cvs) return;
  // Draw obstacle grid.
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_detector");
  for (int row = 0; row < obstacle_grid.height(); ++row) {
    for (int col = 0; col < obstacle_grid.width(); ++col) {
      const auto& info = obstacle_grid(row, col);
      if (info.point_indices.empty()) continue;
      const auto coord = obstacle_grid.RCToCoord({row, col});
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
      for (const int index : info.point_indices) {
        min_z = std::min(min_z, points[index].z);
        max_z = std::max(max_z, points[index].z);
      }
      const std::vector<Vec3d> contour = {
          {coord.x() - Obstacle::kDiameter * 0.5,
           coord.y() - Obstacle::kDiameter * 0.5, min_z},
          {coord.x() + Obstacle::kDiameter * 0.5,
           coord.y() - Obstacle::kDiameter * 0.5, min_z},
          {coord.x() + Obstacle::kDiameter * 0.5,
           coord.y() + Obstacle::kDiameter * 0.5, min_z},
          {coord.x() - Obstacle::kDiameter * 0.5,
           coord.y() + Obstacle::kDiameter * 0.5, min_z}};
      const vis::Color color =
          info.dist_to_curb_cm > 0 ? vis::Color::kDarkGray : vis::Color::kCyan;
      canvas.DrawPolygon(contour, color, 2);
      canvas.DrawText(
          absl::StrFormat("num: %d, z(%.2f, %.2f), h: %.2f, T: %s",
                          info.point_indices.size(), min_z, max_z,
                          max_z - min_z, ObstacleInfo::Type_Name(info.type)),
          {coord.x(), coord.y(), min_z}, 1.0, 0.01, vis::Color::kWhite);
    }
  }
}

struct Cuboid {
  Vec3d right_down_point;
  Vec3d left_up_point;
};

std::vector<Cuboid> TransformAABox3dProtosToCuboids(
    const std::vector<qcraft::AABox3dProto>& aabox3ds,
    const VehiclePose& pose) {
  std::vector<Cuboid> cuboids;
  cuboids.reserve(aabox3ds.size());
  const AffineTransformation pose_transform = pose.ToTransform();
  for (const auto& aabox3d : aabox3ds) {
    Cuboid cuboid;
    Vec3d point_center = {aabox3d.x(), aabox3d.y(), aabox3d.z()};
    const auto length = aabox3d.length();
    const auto width = aabox3d.width();
    QCHECK_NEAR(length, width, 1e-6);
    const auto height = aabox3d.height();
    point_center = pose_transform.TransformPoint(point_center);

    cuboid.right_down_point(0) = point_center(0) - length * 0.5;
    cuboid.right_down_point(1) = point_center(1) - width * 0.5;
    cuboid.right_down_point(2) = point_center(2) - height * 0.5;

    cuboid.left_up_point(0) = cuboid.right_down_point(0) + length;
    cuboid.left_up_point(1) = cuboid.right_down_point(1) + width;
    cuboid.left_up_point(2) = cuboid.right_down_point(2) + height;

    cuboids.push_back(cuboid);
  }
  return cuboids;
}

class CuboidFilter {
 public:
  CuboidFilter(const VehicleParamApi& vehicle_params, const VehiclePose& pose) {
    std::vector<AABox3dProto> aabox3ds;

    const bool have_inside_points_ignorance_boxes =
        (vehicle_params.has_lidar_points_ignorance_boxes_usage() &&
         vehicle_params.lidar_points_ignorance_boxes_usage() !=
             FILTER_OCCLUDED_POINTS);

    if (have_inside_points_ignorance_boxes) {
      aabox3ds = vehicle_params.lidar_points_ignorance_boxes();
    }

    cuboids_ = TransformAABox3dProtosToCuboids(aabox3ds, pose);
  }

  bool ShouldIgnore(const Vec3f& point) const {
    for (const auto& cuboid : cuboids_) {
      if (point.x() >= cuboid.right_down_point.x() &&
          point.x() <= cuboid.left_up_point.x() &&
          point.y() >= cuboid.right_down_point.y() &&
          point.y() <= cuboid.left_up_point.y() &&
          point.z() >= cuboid.right_down_point.z() &&
          point.z() <= cuboid.left_up_point.z()) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<Cuboid> cuboids_;
};

}  // namespace

ObstacleDetector::ObstacleDetector(const RunParamsProtoV2& run_params,
                                   ImageryManager* imagery_manager,
                                   ThreadPool* thread_pool)
    : detection_region_width_(kObstacleDetectionFrontDist * 2),
      detection_region_height_(kObstacleDetectionFrontDist * 2),
      obstacle_grid_(
          RoundToInt(detection_region_width_ / Obstacle::kDiameter),
          RoundToInt(detection_region_height_ / Obstacle::kDiameter)),
      imagery_manager_(imagery_manager),
      local_imagery_(imagery_manager),
      thread_pool_(thread_pool) {
  UpdateRunParams(run_params);
}

ObstacleRefVector ObstacleDetector::CreateObstaclesFromGrid(
    const VehiclePose& pose, const std::vector<LaserPoint>& points,
    const Vec2i row_col_offset, const AffineTransformation& pose_correction,
    const mapping::SemanticMapManager& semantic_map_manager) {
  SCOPED_QTRACE("ObstacleDetector::CreateObstaclesFromGrid");

  std::vector<ObstacleRefVector> obstacles_in_rows(obstacle_grid_.height());

  const AffineTransformation pose_correction_inv(pose_correction.Inverse());

  const auto vegetation_zones = CollectNearPerceptionZones(
      semantic_map_manager, mapping::PerceptionZoneProto::VEGETATION, pose,
      *coordinate_converter_);
  const auto barrier_zones = CollectNearPerceptionZones(
      semantic_map_manager, mapping::PerceptionZoneProto::BARRIER, pose,
      *coordinate_converter_);
  const auto overhanging_zones = CollectNearPerceptionZones(
      semantic_map_manager, mapping::PerceptionZoneProto::OVERHANGING, pose,
      *coordinate_converter_);
  const auto driveways =
      CollectNearDriveways(semantic_map_manager, pose, *coordinate_converter_);

  const Box2d offroad_detection_region = obstacle_util::CreateRegionBox(
      {pose.x, pose.y}, pose.yaw, kObstacleOffroadDetectionFrontDist,
      kObstacleOffroadDetectionRearDist, kObstacleOffroadDetectionLateralDist);

  // Create obstacles for each row in obstacle_grid_ in parallel for
  // acceleration.

  // Use a smaller block size for better load balance.
  parallel_for::Options options;
  options.block_size = 8;
  ParallelFor(0, obstacle_grid_.height(), thread_pool_, options, [&](int row) {
    for (int col = 0; col < obstacle_grid_.width(); ++col) {
      if (!obstacle_grid_.IsValid(row, col)) continue;

      auto& info = obstacle_grid_(row, col);

      auto obstacle = std::make_unique<Obstacle>();
      obstacle->x =
          (col + row_col_offset.y() - obstacle_grid_.width() * 0.5f + 0.5f) *
          Obstacle::kDiameter;
      obstacle->y =
          (row + row_col_offset.x() - obstacle_grid_.height() * 0.5f + 0.5f) *
          Obstacle::kDiameter;
      obstacle->row = row;
      obstacle->col = col;
      obstacle->dist_to_curb = info.dist_to_curb_cm * 0.01f;

      const Vec2d obstacle_pos(obstacle->x, obstacle->y);

      // Ignore offroad obstacles (including obstacles that's crossing curbs to
      // address minus localization nuance.).
      constexpr float kObstacleRadius = Obstacle::kDiameter / 2;
      if (obstacle->dist_to_curb > -kObstacleRadius &&
          !PointInZones(obstacle_pos, driveways)) {
        // Don't consider this obstacle anymore if it is too far.
        if (!offroad_detection_region.IsPointIn(obstacle->coord())) {
          continue;
        }
        obstacle->type = ObstacleProto::OFFROAD;
      }

      // Sort points in each obstacle by their z-coordinates.
      std::sort(info.point_indices.begin(), info.point_indices.end(),
                [&points](int lhs, int rhs) {
                  return CompareLaserPointAtZ(points[lhs], points[rhs]);
                });
      obstacle->timestamp = points[info.point_indices[0]].timestamp;

      // Assign ground_z.
      const Vec2d obstacle_pos_global =
          coordinate_converter_->SmoothToGlobal(obstacle_pos);
      if (const auto indexer = local_imagery_.GetIndexer(
              obstacle_pos_global.x(), obstacle_pos_global.y(),
              coordinate_converter_->GetLevel())) {
        // In the current vehicle frame, compute the ground z (not the
        // elevation readings from the elevmap) in body pose.
        // TODO(dong): Simplify the computation of pose-corrected obstacle z.
        const Vec3d obstacle_pos_3d = coordinate_converter_->GlobalToSmooth(
            {obstacle_pos_global.x(), obstacle_pos_global.y(),
             local_imagery_.ElevationAt(*indexer)});
        obstacle->ground_z =
            pose_correction_inv.TransformPoint(obstacle_pos_3d).z();
      } else {
        // If there is no elevation info in elevmap, treat the lowest z as the
        // ground z.
        // Note: the ground z may not be compatible in local and smooth, the
        // body vehicle frame z is more close to the local. In smooth, this
        // assumption may not exist.
        obstacle->ground_z = points[info.point_indices[0]].z;
      }

      obstacle->points.reserve(info.point_indices.size());
      for (const int i : info.point_indices) {
        // Remove too high points. Break once the z is above the threshold since
        // point_indices is ordered from lowest to highest.
        if (points[i].z > obstacle->ground_z + kMaxPointHeight) {
          break;
        }
        obstacle->points.push_back(points[i]);
      }
      if (obstacle->points.empty()) {
        continue;
      }

      // Ignore obstacles with only ground and underground points or overhang
      // points.
      const float near_ground_max_z =
          obstacle_util::GetObstacleNearGroundMaxZ(*obstacle);
      const float overhang_min_z_from_av_height =
          obstacle_util::GetObstacleOverhangMinZ(*obstacle, av_height_);
      float overhang_min_z = overhang_min_z_from_av_height;
      if (obstacle->type != ObstacleProto::OFFROAD &&
          PointInZones(obstacle_pos, vegetation_zones)) {
        // Lower the overhang min height for obstacles in vegetation zone.
        overhang_min_z = overhang_min_z_from_av_height - 0.8;
        obstacle->type = ObstacleProto::VEGETATION;
        obstacle->type_source = ObstacleProto::SEMANTIC_MAP_ZONE;
      }
      // NOTE(dong): We introduce overhanging zone to deal with overhanging
      // obstacles that no need to be detected. overhang_min_z is subtracted to
      // prevent these obstacles being promoted.
      if (obstacle->type != ObstacleProto::OFFROAD &&
          PointInZones(obstacle_pos, overhanging_zones)) {
        // Lower the overhang min height for obstacles in overhanging zone.
        overhang_min_z = overhang_min_z_from_av_height - 1.2;
      }
      bool has_valid_points = false;
      // NOTE(dong): Points in open interval (near_ground_max_z, overhang_min_z)
      // are regarded as valid. Do not use closed interval [near_ground_max_z,
      // overhang_min_z] at other places which may cause some weired problems.
      for (int i = 0; i < obstacle->points.size(); ++i) {
        if (obstacle->points[i].z <= near_ground_max_z) continue;
        has_valid_points = obstacle->points[i].z < overhang_min_z;
        obstacle->clearance = obstacle->points[i].z - obstacle->ground_z;
        obstacle->num_points_above_ground = obstacle->points.size() - i;
        break;
      }

      if (obstacle->num_points_above_ground == 0 &&
          obstacle->type != ObstacleProto::OFFROAD) {
        info.type = ObstacleInfo::kGround;
      }

      if (!has_valid_points) {
        continue;
      }

      if (obstacle->type != ObstacleProto::OFFROAD) {
        if (PointInZones(obstacle_pos, barrier_zones)) {
          obstacle->type = ObstacleProto::BARRIER;
          obstacle->type_source = ObstacleProto::SEMANTIC_MAP_ZONE;
        }
      }

      obstacle->min_z = obstacle->points.front().z;
      obstacle->max_z = obstacle->points.back().z;

      QCHECK(!obstacle->points.empty());
      info.type = ObstacleInfo::kObstacle;
      obstacles_in_rows[row].push_back(std::move(obstacle));
    }
  });

  ObstacleRefVector obstacles;
  for (auto& obstacles_in_row : obstacles_in_rows) {
    std::move(obstacles_in_row.begin(), obstacles_in_row.end(),
              std::back_inserter(obstacles));
  }
  return obstacles;
}

std::optional<filtering::ObstacleDataProto::Type>
ObstacleDetector::MatchObstacleLabel(
    const Obstacle* obstacle, const labeling::LabelFrameProto& label_frame) {
  filtering::ObstacleDataProto::Type obstacle_type;
  bool is_training_obstacle = false;
  if (std::abs(obstacle->timestamp - label_frame.timestamp()) > 0.2) {
    return std::nullopt;
  }
  // First check if the obstacle group is in mist zone, then check if it also
  // happens to be in any other bounding box, if so, overwrite its label.
  for (const auto& zone : label_frame.zones()) {
    if (zone.type() != labeling::Zone_ZoneType_MIST &&
        zone.type() != labeling::Zone_ZoneType_STATIC_OBJECT &&
        zone.type() != labeling::Zone_ZoneType_VEGETATION &&
        zone.type() != labeling::Zone_ZoneType_BARRIER) {
      continue;
    }
    // Construct zone polygon
    std::vector<Vec2d> zone_points;
    zone_points.reserve(zone.xs_size());
    for (int i = 0; i < zone.xs_size(); ++i) {
      zone_points.emplace_back(zone.xs(i), zone.ys(i));
    }
    if (zone_points.size() < 3) {
      QLOG(ERROR) << labeling::Zone_ZoneType_Name(zone.type())
                  << "zone points number should not be less than 3.";
      continue;
    }
    const Polygon2d zone_polygon(zone_points);
    // canvas.DrawPolygon(zone_points, 0, vis::Color(1.0, 0.0, 0.0));
    if (zone_polygon.IsPointIn({obstacle->x, obstacle->y})) {
      obstacle_type = zone.type() == labeling::Zone_ZoneType_MIST
                          ? filtering::ObstacleDataProto::MIST
                          : filtering::ObstacleDataProto::OTHER;
      is_training_obstacle = true;
    }
  }
  // Check label, maybe not need it.
  for (const auto& label : label_frame.labels()) {
    const auto label_bounding_box =
        Polygon2d(Box2d({label.x(), label.y()}, label.heading(), label.length(),
                        label.width()));
    if (label_bounding_box.IsPointIn({obstacle->x, obstacle->y})) {
      obstacle_type = filtering::ObstacleDataProto::OTHER;
      is_training_obstacle = true;
    }
  }
  if (!is_training_obstacle) {
    return std::nullopt;
  }

  // Use this visualization to observe and tune visualizations
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_data_generating");
  const Vec2d obstacle_coord = obstacle->coord();
  switch (obstacle_type) {
    case filtering::ObstacleDataProto::OTHER:
      canvas.DrawBox({obstacle_coord, 0.0}, 0.0,
                     {Obstacle::kDiameter, Obstacle::kDiameter},
                     vis::Color::kGreen);
      break;
    case filtering::ObstacleDataProto::MIST:
      canvas.DrawBox({obstacle_coord, 0.0}, 0.0,
                     {Obstacle::kDiameter, Obstacle::kDiameter},
                     vis::Color::kRed);
  }

  return obstacle_type;
}

filtering::PointDataProto ObstacleDetector::ToPointDataProto(
    const VehiclePose& pose, const LaserPoint& point) {
  filtering::PointDataProto point_proto;
  // Only apply trans, do not need rotation for net.
  point_proto.set_x(point.x - pose.x);
  point_proto.set_y(point.y - pose.y);
  point_proto.set_z(point.z - pose.z);
  point_proto.set_intensity(point.intensity);
  point_proto.set_return_index(point.return_index);

  return point_proto;
}

void ObstacleDetector::GenerateObstacleTrainingData(
    const VehiclePose& pose, const ObstaclePtrs& obstacles,
    const labeling::LabelFrameProto* label_frame,
    filtering::ObstaclesDataProto* obstacles_proto) {
  if (label_frame == nullptr) {
    QLOG_EVERY_N_SEC(WARNING, 1.0) << "No label frame received.";
    return;
  }
  // filter frame
  if (label_frame->timestamp() - pre_label_time_ < 0.5) {
    return;
  } else {
    pre_label_time_ = label_frame->timestamp();
  }

  obstacle_util::LocalObstacleGrid local_grid(obstacles);
  for (const auto* obstacle : obstacles) {
    QCHECK_GT(obstacle->points.size(), 0);
    if (obstacle->dist_to_curb > 0.f) continue;
    auto type_proto = MatchObstacleLabel(obstacle, *label_frame);
    if (type_proto == std::nullopt) continue;

    const std::vector<int> grown_obstacles_indices =
        local_grid.FindNearestInRadius(obstacle->row, obstacle->col,
                                       /*radius*/ 1);
    auto obstacle_proto = obstacles_proto->add_obstacles();
    obstacle_proto->set_type(type_proto.value());
    const float ground_z = obstacle->ground_z;
    for (const auto& point : obstacle->points) {
      if (point.z < ground_z) continue;
      *obstacle_proto->add_points() = ToPointDataProto(pose, point);
    }
    for (const int neighbor_indice : grown_obstacles_indices) {
      if (obstacle == obstacles[neighbor_indice]) continue;
      for (const auto& point : obstacles[neighbor_indice]->points) {
        if (point.z < ground_z) continue;
        *obstacle_proto->add_neighbor_points() = ToPointDataProto(pose, point);
      }
    }
  }
}

ObstacleRefVector ObstacleDetector::PropagateGroundObstacles(
    const std::vector<LaserPoint>& points, ObstacleRefVector obstacles) {
  SCOPED_QTRACE_ARG1("ObstacleDetector::PropagateGroundObstacles",
                     "num_obstacles", obstacles.size());

  const auto get_12_neighbors = [](const int row, const int col) {
    return std::array<std::pair<int, int>, 12>{{{row - 1, col - 1},
                                                {row - 1, col},
                                                {row - 1, col + 1},
                                                {row, col - 1},
                                                {row, col + 1},
                                                {row + 1, col - 1},
                                                {row + 1, col},
                                                {row + 1, col + 1},
                                                {row - 2, col},
                                                {row + 2, col},
                                                {row, col - 2},
                                                {row, col + 2}}};
  };
  const auto compute_above_ground_max_z = [&](const auto& info,
                                              const float overhang_min_z) {
    DCHECK(!info.point_indices.empty());
    std::optional<float> no_overhang_max_z =
        std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto index : info.point_indices) {
      const float z = points[index].z;
      max_z = std::max(max_z, z);
      if (z < overhang_min_z) {
        no_overhang_max_z = std::max(*no_overhang_max_z, z);
      }
    }
    return no_overhang_max_z ? *no_overhang_max_z : max_z;
  };
  // Eight-neighborhood propagation.
  constexpr float kMaxNearbyObstacleHeightDiff = 0.03;  // m
  for (const auto& obstacle : obstacles) {
    const int row = obstacle->row;
    const int col = obstacle->col;

    auto& info = obstacle_grid_(row, col);
    DCHECK(!info.point_indices.empty());
    if (info.type == ObstacleInfo::kGround) {
      continue;
    }
    const float overhang_min_z =
        obstacle_util::GetObstacleOverhangMinZ(*obstacle, av_height_);
    // Check if current obstacle is ground.
    bool is_ground_obstacle = false;
    const auto neighbors = get_12_neighbors(row, col);
    for (const auto& [nr, nc] : neighbors) {
      if (nr < 0 || nr >= obstacle_grid_.height() || nc < 0 ||
          nc >= obstacle_grid_.width()) {
        continue;
      }
      const auto& ninfo = obstacle_grid_(nr, nc);
      if (ninfo.type != ObstacleInfo::kGround) {
        continue;
      }
      DCHECK(!ninfo.point_indices.empty());
      const float neighbor_max_z =
          compute_above_ground_max_z(ninfo, overhang_min_z);
      const float z_diff = obstacle->points.back().z - neighbor_max_z;
      if (z_diff < kMaxNearbyObstacleHeightDiff) {
        is_ground_obstacle = true;
        break;
      }
    }
    if (!is_ground_obstacle) {
      continue;
    }
    info.type = ObstacleInfo::kGround;
    // Propagate
    std::queue<std::pair<int, int>> queue;
    queue.emplace(obstacle->row, obstacle->col);
    while (!queue.empty()) {
      const auto [r, c] = queue.front();
      queue.pop();
      const auto& info = obstacle_grid_(r, c);
      QCHECK_EQ(info.type, ObstacleInfo::kGround);
      const auto neighbors = get_12_neighbors(r, c);
      for (const auto& [nr, nc] : neighbors) {
        if (nr < 0 || nr >= obstacle_grid_.height() || nc < 0 ||
            nc >= obstacle_grid_.width()) {
          continue;
        }
        auto& ninfo = obstacle_grid_(nr, nc);
        if (ninfo.type != ObstacleInfo::kObstacle) {
          continue;
        }
        DCHECK(!ninfo.point_indices.empty());
        const float z_diff = compute_above_ground_max_z(ninfo, overhang_min_z) -
                             compute_above_ground_max_z(info, overhang_min_z);
        if (z_diff < kMaxNearbyObstacleHeightDiff) {
          ninfo.type = ObstacleInfo::kGround;
          queue.emplace(nr, nc);
        }
      }
    }
  }

  ObstacleRefVector new_obstacles;
  new_obstacles.reserve(obstacles.size());
  for (auto& obstacle : obstacles) {
    const auto& info = obstacle_grid_(obstacle->row, obstacle->col);
    QCHECK_NE(info.type, ObstacleInfo::kUnknown);
    if (info.type == ObstacleInfo::kObstacle) {
      new_obstacles.emplace_back(std::move(obstacle));
    } else if (info.type == ObstacleInfo::kGround) {
      obstacle->type = ObstacleProto::IGNORED;
      obstacle->type_source = ObstacleProto::GROUND_PROPAGATION;
      new_obstacles.emplace_back(std::move(obstacle));
    }
  }
  return new_obstacles;
}

float ObstacleDetector::ComputeDistToCurbForObstacleAt(int row, int col) const {
  const Vec2d obstacle_pos = obstacle_grid_.RCToCoord({row, col});
  const Vec2d obstacle_pos_global =
      coordinate_converter_->SmoothToGlobal(obstacle_pos);
  if (const auto indexer = local_imagery_.GetIndexer(
          obstacle_pos_global.x(), obstacle_pos_global.y(),
          coordinate_converter_->GetLevel())) {
    return local_imagery_.DistToCurbAt(*indexer);
  } else {
    // If there is no info in imagery for this obstacle, treat it as
    // offroad with a considerably large value.
    return kDefaultOffroadDistToCurbValue;
  }
}

bool ObstacleDetector::UpdateObstacleGrid(const std::pair<int, int>& rc,
                                          const int point_index) {
  const auto [row, col] = rc;
  auto& obstacle_info = obstacle_grid_(row, col);
  bool point_added = false;

  // Acquire the spin lock.
  obstacle_info.spin_lock.Lock();

  if (!obstacle_grid_.IsValid(row, col)) {
    obstacle_grid_.SetIsValid(row, col);
    obstacle_info.dist_to_curb_cm = std::clamp<int>(
        RoundToInt(ComputeDistToCurbForObstacleAt(row, col) * 100),
        std::numeric_limits<int16_t>::min(),
        std::numeric_limits<int16_t>::max());
  }
  if (obstacle_info.point_indices.size() < kMaxNumPointsPerObstacle) {
    obstacle_info.point_indices.push_back(point_index);
    point_added = true;
  }

  // Release the spin lock.
  obstacle_info.spin_lock.Unlock();

  return point_added;
}

std::optional<std::pair<int, int>> ObstacleDetector::ComputeObstacleRC(
    const Box2d& av_box, const Box2d& detection_region,
    const Vec2d obstacle_pos, const float range) const {
  // Ignore self returned points.
  constexpr float kRangeOutAvBox = 15.f;  // m
  if (range < kRangeOutAvBox && av_box.IsPointIn(obstacle_pos)) {
    return std::nullopt;
  }
  // Ignore points outside of detection region.
  constexpr float kRangeInDetectionRegion = 50.f;  // m
  if (range > kRangeInDetectionRegion &&
      !detection_region.IsPointIn(obstacle_pos)) {
    return std::nullopt;
  }
  const auto [row, col] =
      obstacle_grid_.CoordToRC(obstacle_pos.template cast<float>());
  if (col < 0 || row < 0 || col >= obstacle_grid_.width() ||
      row >= obstacle_grid_.height()) {
    return std::nullopt;
  }
  return std::make_optional<std::pair<int, int>>(row, col);
}

std::unique_ptr<ObstacleManager> ObstacleDetector::DetectObstacles(
    const std::vector<LidarFrame>& lidar_frames, const VehiclePose& pose,
    const CoordinateConverter& coordinate_converter,
    const labeling::LabelFrameProto* latest_label_frame,
    const mapping::SemanticMapManager& semantic_map_manager,
    const VehicleParamApi& vehicle_params) {
  SCOPED_QTRACE("ObstacleDetector::DetectObstacles");

  QCHECK(!lidar_frames.empty());
  obstacle_grid_.InitializeWithPose(pose);
  coordinate_converter_ = coordinate_converter;

  const Vec2d pose_global =
      coordinate_converter_->SmoothToGlobal({pose.x, pose.y});
  local_imagery_.Update(*imagery_manager_, pose_global.x(), pose_global.y());

  const Box2d av_box = ComputeAvBox(lidar_frames, vehicle_params);

  // The detection region: only generate obstacles in this region.
  const Box2d detection_region = obstacle_util::CreateRegionBox(
      {pose.x, pose.y}, pose.yaw, kObstacleDetectionFrontDist,
      kObstacleDetectionRearDist, kObstacleDetectionLateralDist);

  MaybeRenderSdcContour(av_box, pose);
  MaybeRenderDetectionRegion(detection_region, pose);

  std::vector<int> accumulated_num_points_per_frame = {0};
  for (const auto& lidar_frame : lidar_frames) {
    accumulated_num_points_per_frame.push_back(
        lidar_frame.max_num_points() + accumulated_num_points_per_frame.back());
  }
  points_.resize(accumulated_num_points_per_frame.back());

  const CuboidFilter av_cuboid_filter(vehicle_params, pose);

  for (int frame_index = 0; frame_index < lidar_frames.size(); ++frame_index) {
    const auto& lidar_frame = lidar_frames[frame_index];
    const auto lidar_id = lidar_frame.lidar_id();

    SCOPED_QTRACE_ARG1("ObstacleDetector::UpdateObstacleInfo", "lidar_id",
                       LidarId_Name(lidar_id));

    // The start index in `points' of all points for the current spin.
    const int start_point_index = accumulated_num_points_per_frame[frame_index];

    const auto& pose_for_current_lidar = lidar_frame.MidPose();
    Box2d av_box_for_current_lidar(av_box);
    av_box_for_current_lidar.Shift(
        {pose_for_current_lidar.x - pose.x, pose_for_current_lidar.y - pose.y});

    Box2d lidar_roi;
    if (IsBlindLidar(lidar_id)) {
      // NOTO(dong): We remove blind lidar points far away from ego vehicle, as
      // these points are mostly useless and may cause some issues (e.g.
      // blooming).
      lidar_roi =
          GetBlindLidarRoi(lidar_frame.lidar_type(), av_box_for_current_lidar);
    } else {
      lidar_roi = detection_region;
    }

    const auto& ignore_table = FindOrDie(lidar_ignore_tables_, lidar_id);

    if (lidar_frame.is_spin()) {
      const auto& spin = *QCHECK_NOTNULL(lidar_frame.spin());
      ParallelFor(0, spin.num_scans(), thread_pool_, [&](int scan_index) {
        const auto& scan = spin.scan(scan_index);
        // Correct the scan index for laser point ignorance. This correction is
        // done because the scan index may not be corresponded to the azimuth
        // due to packet loss.
        // TODO(dong): Use ShouldIgnoreByIndex() once we fix the scan index
        // assignment in lidar drivers.
        const int corrected_scan_index =
            ignore_table.AzimuthToScanIndex(scan.azimuth_in_degree);
        for (int beam_index = 0; beam_index < spin.num_beams(); ++beam_index) {
          if (ignore_table.ShouldIgnoreByIndex(beam_index,
                                               corrected_scan_index)) {
            continue;
          }
          const auto& shot = scan.shots[beam_index];
          for (int return_index = 0; return_index < shot.num_returns;
               ++return_index) {
            const auto& ret = shot.calibrated_returns[return_index];
            const Vec2d position_2d(ret.x, ret.y);
            const auto obstacle_rc = ComputeObstacleRC(
                av_box_for_current_lidar, lidar_roi, position_2d, ret.range);
            if (!obstacle_rc) continue;

            // Filter self returns.
            if (av_cuboid_filter.ShouldIgnore({ret.x, ret.y, ret.z})) {
              continue;
            }

            const int accumulated_point_index =
                start_point_index +
                scan_index * spin.num_beams() * LaserShot::kMaxNumReturns +
                beam_index * LaserShot::kMaxNumReturns + return_index;
            if (UpdateObstacleGrid(*obstacle_rc, accumulated_point_index)) {
              points_[accumulated_point_index] = {
                  .timestamp = scan.timestamp,
                  .x = ret.x,
                  .y = ret.y,
                  .z = ret.z,
                  .range = ret.range,
                  .normal_x = ret.nx,
                  .normal_y = ret.ny,
                  .normal_z = ret.nz,
                  .has_return_behind = return_index < shot.num_returns - 1,
                  .intensity = ret.intensity,
                  .planarity = ret.planarity,
                  .beam_index = static_cast<uint8_t>(beam_index),
                  .return_index = static_cast<uint8_t>(return_index),
                  .scan_or_point_index = static_cast<uint32_t>(scan_index),
                  .lidar_id = lidar_id,
                  .lidar_type = spin.lidar_type()};
            }
          }
        }
      });
    } else if (lidar_frame.is_point_cloud()) {
      const auto& point_cloud = *QCHECK_NOTNULL(lidar_frame.point_cloud());
      const int num_points = point_cloud.num_points();
      ParallelFor(0, num_points, thread_pool_, [&](int point_index) {
        const auto& point = point_cloud.point(point_index);
        const Vec2d position_2d(point.x, point.y);
        const auto obstacle_rc = ComputeObstacleRC(
            av_box_for_current_lidar, lidar_roi, position_2d, point.range());
        if (!obstacle_rc) return;
        // NOTE(dong): Temporary disable CuboidFilter for point cloud as
        // it's never used by now.
        const int accumulated_point_index = start_point_index + point_index;
        if (UpdateObstacleGrid(*obstacle_rc, accumulated_point_index)) {
          points_[accumulated_point_index] = {
              .timestamp = point.timestamp(point_cloud.timestamp()),
              .x = point.x,
              .y = point.y,
              .z = point.z,
              .range = point.range(),
              .normal_x = 0,
              .normal_y = 0,
              .normal_z = 0,
              .has_return_behind = point.has_return_behind,
              .intensity = point.intensity,
              .planarity = 0,
              .beam_index = 0,
              .return_index = point.return_index,
              .scan_or_point_index = static_cast<uint32_t>(point_index),
              .lidar_id = lidar_id,
              .lidar_type = point_cloud.lidar_type()};
        }
      });
    } else {
      QLOG(FATAL) << "Should not reach here.";
    }
  }

  const auto match_result = pose_correction_.CorrectPose(
      pose, points_, semantic_map_manager, *coordinate_converter_,
      obstacle_grid_, local_imagery_, thread_pool_);

  pose_difference_ = std::nullopt;
  pose_correction_result_ = std::nullopt;
  if (match_result) {
    const auto pose_transform = pose.ToTransform();
    pose_difference_ = VehiclePose::FromTransform(
        pose_transform.Inverse() * (match_result->transform) * pose_transform);
    pose_correction_result_ =
        VehiclePose::FromTransform(match_result->transform);
    QLOG(INFO) << absl::StrFormat(
        "Corrected pose (vehicle frame): %s num_matched_points: %d mse: %.4f "
        "matched_ratio: %.2f num_iteration: %d matching_dist: %.2f",
        pose_difference_->DebugString(), match_result->num_matched_points,
        match_result->mse, match_result->matched_ratio,
        match_result->num_iteration, match_result->matching_dist);
  } else {
    QLOG(ERROR) << "Invalid pose correction.";
  }

  auto obstacles = CreateObstaclesFromGrid(
      pose, points_,
      Vec2i(obstacle_grid_.rc_coord_converter().row_offset(),
            obstacle_grid_.rc_coord_converter().col_offset()),
      match_result ? match_result->transform : AffineTransformation(),
      semantic_map_manager);

  if (FLAGS_enable_ground_obstacle_propagation) {
    obstacles = PropagateGroundObstacles(points_, std::move(obstacles));
  }

  MaybeRenderObstacleGrid(obstacle_grid_, points_);

  if (FLAGS_collect_obstacles_data) {
    ObstaclePtrs obstacle_ptrs;
    for (const auto& obs : obstacles) {
      obstacle_ptrs.push_back(obs.get());
    }
    GenerateObstacleTrainingData(pose, obstacle_ptrs, latest_label_frame,
                                 &obstacles_proto_);
  }

  auto obstacle_manager = std::make_unique<ObstacleManager>(
      std::move(obstacles), obstacle_grid_.rc_coord_converter(), camera_params_,
      lidar_params_, thread_pool_);

  // Clear point info in obstacle grid asynchronously.
  obstacle_grid_.ResetAsync();

  return obstacle_manager;
}

void ObstacleDetector::UpdateRunParams(const RunParamsProtoV2& run_params) {
  camera_params_ = ComputeAllCameraParams(run_params.vehicle_params());

  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }
  av_height_ = obstacle_util::ComputeAvHeight(run_params);

  lidar_ignore_tables_ =
      lidar_util::ComputeLidarIgnoreTables(run_params.vehicle_params());
}

ObstacleDetector::~ObstacleDetector() {
  if (FLAGS_collect_obstacles_data) {
    if (FLAGS_obstacles_data_file_name.empty()) {
      QLOG(ERROR) << "Need to set obstacle data file name while collecting "
                     "obstacle data.";
      return;
    }
    CHECK(file_util::ProtoToTextFile(obstacles_proto_,
                                     FLAGS_obstacles_data_file_name));
    QLOG(INFO) << "Write obstacles data :" << FLAGS_obstacles_data_file_name;
  }
}
}  // namespace qcraft
