#include "onboard/perception/tracker/tracker_util.h"

#include <algorithm>
#include <cfloat>
#include <limits>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/nets/fiery_eye_net_classifier_config.h"
#include "onboard/perception/sensor_fov/sensor_fov.h"
#include "onboard/perception/tracker/tracker_constant.h"

namespace qcraft::tracker::tracker_util {

bool IsInCautiousRegion(const VehiclePose& vehicle_pose, const Vec2d& vec) {
  // We leave a cautious region where objects are close to ego.
  constexpr double kCautiousRegionLength = 20;      // m
  constexpr double kCautiousRegionWidth = 3.7 * 3;  // m
  constexpr double kOffsetToBackOfVehicle = 7;      // m
  const Vec2d ego_vehicle_bev_pos = {
      vehicle_pose.x - kOffsetToBackOfVehicle * std::cos(vehicle_pose.yaw),
      vehicle_pose.y - kOffsetToBackOfVehicle * std::sin(vehicle_pose.yaw)};
  const Box2d cautious_region(ego_vehicle_bev_pos, vehicle_pose.yaw,
                              kCautiousRegionLength, kCautiousRegionWidth);
  return cautious_region.IsPointIn(vec);
}

bool IsCornerRadar(qcraft::RadarId radar_id) {
  return (radar_id == qcraft::RadarId::RAD_FRONT_LEFT ||
          radar_id == qcraft::RadarId::RAD_FRONT_RIGHT ||
          radar_id == qcraft::RadarId::RAD_REAR_LEFT ||
          radar_id == qcraft::RadarId::RAD_REAR_RIGHT);
}

Vec2d ComputeCentroid(const std::vector<Vec2d>& points) {
  QCHECK(!points.empty());
  return std::accumulate(points.begin(), points.end(), Vec2d()) / points.size();
}

Vec2d ComputeWeightedObstacleCentroid(
    const std::vector<TrackState::ObstacleInfo>& obstacle_infos) {
  Vec2d weighted_obstacle_centroid = {0., 0.};
  int num_point = 0;
  for (const auto& info : obstacle_infos) {
    weighted_obstacle_centroid +=
        Vec2d(info.center.x(), info.center.y()) * info.num_points;
    num_point += info.num_points;
  }
  return weighted_obstacle_centroid /
         (num_point + std::numeric_limits<double>::epsilon());
}

Vec2d ComputeWeightedObstacleCentroid(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m) {
  Vec2d weighted_obstacle_centroid = {0., 0.};
  int num_point = 0;
  if (cluster_m.obstacle_info_size() > 0) {
    for (const auto& info : cluster_m.obstacle_info()) {
      weighted_obstacle_centroid +=
          Vec2d(info.center().x(), info.center().y()) * info.num_points();
      num_point += info.num_points();
    }
  } else if (cluster_m.obstacle_centers_deprecated_size() > 0) {
    for (const auto& info : cluster_m.obstacle_centers_deprecated()) {
      weighted_obstacle_centroid += Vec2d(info.x(), info.y());
      num_point += 1;
    }
  } else {
    QLOG(FATAL) << "No obstacle info or obstacle centers";
  }
  return weighted_obstacle_centroid /
         (num_point + std::numeric_limits<double>::epsilon());
}

Vec2d ComputeCentroid(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m) {
  QCHECK(cluster_m.obstacle_info_size() > 0 ||
         cluster_m.obstacle_centers_deprecated_size() > 0);
  Vec2d centroid = {0, 0};
  // Compatible with the old proto
  if (cluster_m.obstacle_info_size() > 0) {
    for (const auto& info : cluster_m.obstacle_info()) {
      centroid += Vec2d{info.center().x(), info.center().y()};
    }
    return centroid / cluster_m.obstacle_info_size();
  } else {
    for (const auto& point : cluster_m.obstacle_centers_deprecated()) {
      centroid += Vec2d{point.x(), point.y()};
    }
    return centroid / cluster_m.obstacle_centers_deprecated_size();
  }
}

std::vector<int> ComputeMatches(const Eigen::MatrixXd& weight_matrix) {
  SCOPED_QTRACE("ComputeMatches");

  const int rows = weight_matrix.rows();
  const int cols = weight_matrix.cols();
  std::vector<bool> matched_rows(rows);  // measurements
  std::vector<bool> matched_cols(cols);  // tracks
  std::vector<int> match_result(rows, -1);
  while (true) {
    double max_val = -std::numeric_limits<double>::max();
    int max_val_col, max_val_row;
    for (int i = 0; i < cols; ++i) {
      if (matched_cols[i]) continue;
      for (int j = 0; j < rows; ++j) {
        if (matched_rows[j]) continue;
        if (weight_matrix(j, i) > max_val) {
          max_val = weight_matrix(j, i);
          max_val_col = i;
          max_val_row = j;
        }
      }
    }
    if (max_val > 0.0) {
      matched_cols[max_val_col] = true;
      matched_rows[max_val_row] = true;
      match_result[max_val_row] = max_val_col;
    } else {
      break;
    }
  }
  return match_result;
}

bool IsInSpecialZone(const SemanticMapManager& semantic_map_manager,
                     const Polygon2d& contour,
                     const CoordinateConverter& coordinate_converter,
                     const mapping::PerceptionZoneProto::Type zone_type,
                     double min_overlap_requirement) {
  const double object_area = contour.area();
  const auto& semantic_map = semantic_map_manager.semantic_map();
  for (const auto& perception_zone : semantic_map.perception_zones()) {
    if (perception_zone.type() != zone_type) {
      continue;
    }
    if (!mapping::Contains(perception_zone.belonging_levels(),
                           coordinate_converter.GetLevel())) {
      continue;
    }
    const Polygon2d& special_zone_polygon = SmoothPolygon2dFromGeoPolygonProto(
        perception_zone.polygon(), coordinate_converter);
    Polygon2d overlap;
    QCHECK_NE(special_zone_polygon.points().size(), 0);
    special_zone_polygon.ComputeOverlap(contour, &overlap);
    if (overlap.area() > object_area * min_overlap_requirement) {
      return true;
    }
  }
  return false;
}

bool IsInIgnoranceZone(const SemanticMapManager& semantic_map_manager,
                       const Polygon2d& contour,
                       const CoordinateConverter& coordinate_converter) {
  return IsInSpecialZone(semantic_map_manager, contour, coordinate_converter,
                         mapping::PerceptionZoneProto::IGNORANCE,
                         kIgnoranceZoneOverlapMinRatio);
}

bool IsInStaticObjectZone(const SemanticMapManager& semantic_map_manager,
                          const Polygon2d& contour,
                          const CoordinateConverter& coordinate_converter) {
  return IsInSpecialZone(semantic_map_manager, contour, coordinate_converter,
                         mapping::PerceptionZoneProto::STATIC_OBJECT,
                         kStaticObjectZoneOverlapMinRatio);
}

bool IsInCrosswalk(const SemanticMapManager& semantic_map_manager,
                   const Track<TrackState>& track,
                   const CoordinateConverter& coordinate_converter) {
  const Polygon2d& track_contour = GetContour(track);
  const auto& semantic_map = semantic_map_manager.semantic_map();
  // TODO(tao): Do not check through all crosswalks in semantic map. Build
  // spatial index for semantic map for quick neighboring search
  for (const auto& crosswalk : semantic_map.crosswalks()) {
    if (!mapping::Contains(crosswalk.belonging_levels(),
                           coordinate_converter.GetLevel())) {
      continue;
    }
    const Polygon2d& crosswalk_polygon = SmoothPolygon2dFromGeoPolygonProto(
        crosswalk.polygon(), coordinate_converter);
    if (track_contour.HasOverlap(crosswalk_polygon)) {
      return true;
    }
  }
  return false;
}

bool IsInParkingArea(const SemanticMapManager& semantic_map_manager,
                     const Track<TrackState>& track,
                     const CoordinateConverter& coordinate_converter) {
  const Polygon2d& track_contour = GetContour(track);
  const double object_area = track_contour.area();
  const auto& semantic_map = semantic_map_manager.semantic_map();
  for (const auto& parking_area : semantic_map.parking_areas()) {
    if (!mapping::Contains(parking_area.belonging_levels(),
                           coordinate_converter.GetLevel())) {
      continue;
    }
    const Polygon2d& parking_area_polygon = SmoothPolygon2dFromGeoPolygonProto(
        parking_area.polygon(), coordinate_converter);
    QCHECK_GT(parking_area_polygon.points().size(), 2)
        << parking_area.DebugString();
    Polygon2d overlap;
    parking_area_polygon.ComputeOverlap(track_contour, &overlap);
    if (overlap.area() > object_area * kParkingAreaOverlapMinRatio) {
      return true;
    }
  }
  return false;
}

// Check if the given track is in a vegetation zone.
bool IsInVegetationZone(const SemanticMapManager& semantic_map_manager,
                        const Track<TrackState>& track,
                        const CoordinateConverter& coordinate_converter) {
  const Polygon2d& track_contour = GetContour(track);
  const double object_area = track_contour.area();
  const auto& semantic_map = semantic_map_manager.semantic_map();
  std::vector<Polygon2d> all_overlaps;
  for (const auto& perception_zone : semantic_map.perception_zones()) {
    if (perception_zone.type() != mapping::PerceptionZoneProto::VEGETATION) {
      continue;
    }
    if (!mapping::Contains(perception_zone.belonging_levels(),
                           coordinate_converter.GetLevel())) {
      continue;
    }
    const Polygon2d& vegetation_zone_polygon =
        SmoothPolygon2dFromGeoPolygonProto(perception_zone.polygon(),
                                           coordinate_converter);
    Polygon2d overlap;
    vegetation_zone_polygon.ComputeOverlap(track_contour, &overlap);
    if (overlap.area() > DBL_EPSILON) {
      all_overlaps.emplace_back(overlap);
    }
  }

  // Compute the sum of overlaps, should subtract intersection area.
  double overlaps_overlab_area_sum = 0.0;
  double overlaps_area_sum = 0.0;
  for (int i = 0; i < all_overlaps.size(); ++i) {
    overlaps_area_sum += all_overlaps[i].area();
    for (int j = i + 1; j < all_overlaps.size(); ++j) {
      Polygon2d overlap;
      all_overlaps[i].ComputeOverlap(all_overlaps[j], &overlap);
      overlaps_overlab_area_sum += overlap.area();
    }
  }

  overlaps_area_sum -= overlaps_overlab_area_sum;

  if (overlaps_area_sum > object_area * kVegetationZoneOverlapMinRatio) {
    return true;
  }

  return false;
}

bool IsInFenDetectionRange(const Vec3d& point, const VehiclePose& pose) {
  // Judge if the object is in the detection range.
  const auto range = fen::GetDetectionRange();
  const float front_dist = range.front;
  const float rear_dist = range.behind;
  const float left_dist = range.left;
  const float right_dist = range.right;
  const auto pose_transform_inv = pose.ToTransform().Inverse();
  const Vec3d point_local{point.x(), point.y(), point.z()};
  const auto point_vehicle = pose_transform_inv.TransformPoint(point_local);
  if (point_vehicle.x() > front_dist || point_vehicle.x() < -rear_dist ||
      point_vehicle.y() > left_dist || point_vehicle.y() < -right_dist) {
    return false;
  }
  return true;
}

bool IsInLane(const SemanticMapManager& semantic_map_manager,
              const Vec2d& centroid, double threshold) {
  const auto ego_level = semantic_map_manager.GetLevel();
  mapping::ElementId lane_id;
  double fraction;
  Vec2d proj_point;
  double min_dist;
  semantic_map_manager.GetNearestLaneProjectionAtLevel(
      ego_level, centroid, &lane_id, &fraction, &proj_point, &min_dist);
  if (min_dist > threshold) {
    return false;
  }
  return true;
}

bool IsInLane(const SemanticMapManager& semantic_map_manager,
              const Track<TrackState>& track, double threshold) {
  const Vec2d centroid =
      tracker_util::ComputeCentroid(track.track_state.contour.points());
  return IsInLane(semantic_map_manager, centroid, threshold);
}

// Compute the distance between the given coordinate to the nearest curb.
// Return a negative value if onroad, and a positive one if offroad.
float DistanceToCurb(const SemanticMapManager& semantic_map_manager,
                     const Vec2d& SmoothPos) {
  const auto ego_level = semantic_map_manager.GetLevel();
  return semantic_map_manager.ComputeDistanceToCurbAtLevel(ego_level,
                                                           SmoothPos);
}

Polygon2d GetContour(const Track<TrackState>& track) {
  Polygon2d track_contour;
  if (track.track_state.bounding_box) {
    track_contour = Polygon2d(*track.track_state.bounding_box);
  } else {
    track_contour = track.track_state.contour;
  }
  return track_contour;
}

void RenderContourCvs(const Polygon2d& contour, double ground_z,
                      vis::Color color, int size, vis::Canvas* canvas,
                      vis::BorderStyleProto::LineStyle border_line_style) {
  std::vector<Vec3d> contour_points;
  for (const auto& point : contour.points()) {
    contour_points.emplace_back(point.x(), point.y(), ground_z);
  }
  canvas->DrawPolygon(contour_points, color, size, border_line_style);
}

std::optional<Vec2d> GetBboxCenter(
    const LaserMeasurementProto& laser_measurement, bool use_min_area_bbox) {
  if (!use_min_area_bbox && !laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  } else if (use_min_area_bbox &&
             !laser_measurement.has_min_area_bounding_box()) {
    return std::nullopt;
  }
  const auto& box2d = use_min_area_bbox
                          ? laser_measurement.min_area_bounding_box()
                          : laser_measurement.detection_bounding_box();
  return Vec2d(box2d.x(), box2d.y());
}

void RefineMotion(const VehiclePose& pose, Track<TrackState>* track,
                  Vec2d* vel) {
  // We can not judge the moving state by by using track sequence info
  // at the beginning of the track, so we set the moving state to unknown.
  constexpr int kMinCheckPointsNumForMovingStateJudgement = 3;
  if (track->checkpoints.size() < kMinCheckPointsNumForMovingStateJudgement &&
      track->track_state.moving_state == TrackState::MovingState::kUnKnown) {
    track->track_state.moving_state = TrackState::MovingState::kUnKnown;
    return;
  }
  // If the vel of car model is less than kMaxStationaryVehicleSpeed, or the vel
  // of point model is less than kMaxStationaryNonVehicleSpeed, we cap it to
  // zero, and set the moving state to static.
  constexpr double kMaxStationaryVehicleSpeed = 0.25;     // m/s
  constexpr double kMaxStationaryNonVehicleSpeed = 0.15;  // m/s
  const auto s = track->track_state.estimator_3d.GetStateData();
  if (track->track_state.estimator_3d.IsCarModel()) {
    if (std::fabs(s.GetVel()) < kMaxStationaryVehicleSpeed) {
      *vel = Vec2d(0.0, 0.0);
      track->track_state.moving_state = TrackState::MovingState::kStatic;
      return;
    }
  } else {
    if (s.GetVel() < kMaxStationaryNonVehicleSpeed) {
      *vel = Vec2d(0.0, 0.0);
      track->track_state.moving_state = TrackState::MovingState::kStatic;
      return;
    }
  }

  // If the the track has reliable radar measurement velocity,
  // we promote the velocity immediately.
  const auto centroid = track->track_state.contour.centroid();
  const auto pose_transform_inv = pose.ToTransform().Inverse();
  const double kMinDistX = 5.0;  // m
  const auto obj_pos =
      pose_transform_inv.TransformPoint(Vec3d(centroid.x(), centroid.y(), 0.0));
  const auto& m_history = track->measurement_history;
  const int num_m = m_history.size();
  const double last_m_timestamp = m_history.back_time();
  const int checkpoints_num = track->checkpoints.size();

  if (track->track_state.type == MT_VEHICLE &&
      std::abs(obj_pos(0)) > kMinDistX) {
    constexpr double kDurationAsValidMeasurement = 0.3;  // s
    constexpr double kOneQuartersPi = 0.25 * M_PI;
    constexpr double kRadarReliableAreaAngle = 0.5 * kOneQuartersPi;

    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto& m = m_history.value(j);
      if (m->has_radar_measurement()) {
        // NOTE(zheng): If the radar measurement is right side or left side of
        // the av, the radar measurement velocity is not accurate.
        const double obj_body_pos_angle =
            Vec2dFromProto(m->radar_measurement().radar_coord_pos())
                .FastAngle();
        const auto radar_vel =
            Vec2dFromProto(m->radar_measurement().vel()).squaredNorm();
        if (radar_vel > kMaxStationaryVehicleSpeed &&
            std::abs(obj_body_pos_angle) > kRadarReliableAreaAngle &&
            std::abs(obj_body_pos_angle) < (M_PI - kRadarReliableAreaAngle)) {
          track->track_state.moving_state = TrackState::MovingState::kMoving;
          return;
        }
      }
    }
  }

  // NOTE(zheng): We use fen velocity as a prior to do moving state
  // classification to supress shooters caused by bbox jumpping issues,
  // fen velocity is accurate when its value is zero except
  // slowing moving vehicles. To avoid we classify the slow moving vehicle to
  // static, we should judge if the vehicle is slow moving.
  if (!IsSlowMovingVehicle(*track, last_m_timestamp)) {
    constexpr double kMaxStaticObjectFenSpeenNoiseSqr =
        Sqr(kMaxStationaryVehicleSpeed);                      // m/s
    constexpr double kValidFenVelMeasurementDuration = 0.30;  // s
    for (int j = m_history.GetIndexWithTimeAtLeast(
             last_m_timestamp - kValidFenVelMeasurementDuration);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement() &&
          m->laser_measurement().has_fen_velocity()) {
        const Vec2d fen_vel =
            Vec2dFromProto(m->laser_measurement().fen_velocity());
        if (track->track_state.moving_state ==
                TrackState::MovingState::kStatic &&
            fen_vel.squaredNorm() < kMaxStaticObjectFenSpeenNoiseSqr) {
          *vel = Vec2d(0.0, 0.0);
          track->track_state.moving_state = TrackState::MovingState::kStatic;
          return;
        }
      }
    }
  }

  // NOTE(zheng): 1. We only process the vehicle/cyc shooters, the unknown
  // object's shooter case are handled by const position motion filter,
  // the pedestrian's shooter case is a todo list.
  // 2. Based on observation, most shooters' moving state is static in history,
  // so to avoid classify the true moving object to static state, we only
  // process the objects which are static in history.
  // TODO(zheng): Tackle ped shooter cases by moving state classification.

  if (track->track_state.moving_state != TrackState::MovingState::kStatic ||
      !track->track_state.estimator_3d.IsCarModel()) {
    track->track_state.moving_state = TrackState::MovingState::kMoving;
    return;
  }

  // We use some sanity check to judge if the speed is a shooter.
  // 1. Max acc sanity check: we suppose the true moving vechile acc is below
  // a threshold, change moving state directly to static if acc is invalid.
  constexpr double kMaxAccAllowedForVehicle = 5.0;  // m/s^2
  // Use filtered speed difference to compute acc, and get the max acc in a
  // window.
  double max_acc = 0.0;
  // Window size for getting acc statistics.
  constexpr int kWindowSize = 5;  // 5 frames.
  // We skip first kMinCheckPointsNumForMovingStateJudgement checkpoints,
  // because the speed is not stable at first.
  const int checkpoints_begin_index = std::max(
      kMinCheckPointsNumForMovingStateJudgement, checkpoints_num - kWindowSize);
  for (int j = checkpoints_begin_index; j < checkpoints_num - 1; ++j) {
    const TrackState& t0 = track->checkpoints.value(j);
    for (int k = j + 1; k < checkpoints_num; ++k) {
      const TrackState& t1 = track->checkpoints.value(k);
      const double time_diff = t0.state_timestamp - t1.state_timestamp;
      // A bigger time interval is more robust.
      if (fabs(time_diff) < 0.2) {
        continue;
      }
      if (t0.estimator_3d.IsCarModel() && t1.estimator_3d.IsCarModel()) {
        const auto s0 = t0.estimator_3d.GetStateData();
        const auto s1 = t1.estimator_3d.GetStateData();
        const double acc =
            (s0.GetVel() - s1.GetVel()) /
            (std::fabs(t0.state_timestamp - t1.state_timestamp) + DBL_EPSILON);
        max_acc = std::max(std::fabs(acc), max_acc);
        break;
      }
    }
  }
  if (max_acc > kMaxAccAllowedForVehicle) {
    *vel = Vec2d(0.0, 0.0);
    track->track_state.moving_state = TrackState::MovingState::kStatic;
    VLOG(2)
        << "Refine motion judges the track to static, use max acc, track id: "
        << track->track_state.id << "  " << max_acc;
    return;
  }

  // 2. BBox center average moving speed santity check: we suppose the
  // true moving object's average speed in a window is greater than a
  // threshold.
  if (num_m == 0) {
    return;
  }
  constexpr double kDurationAsValidMeasurement = 1.0;  // s
  int oldest_laser_measurement_index = 0;
  int latest_laser_measurement_index = num_m - 1;
  // Get oldest laser measurement index.
  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidMeasurement);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      oldest_laser_measurement_index = j;
      break;
    }
  }

  // Get latest laser measurement index.
  for (int j = num_m - 1;
       j >= m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                              kDurationAsValidMeasurement);
       --j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      latest_laser_measurement_index = j;
      break;
    }
  }

  // Get oldest laser measurement detection bbox center and min area bbox
  // center.
  const auto& oldest_laser_m =
      m_history.value(oldest_laser_measurement_index)->laser_measurement();
  const auto& oldest_laser_m_timestamp =
      m_history.time(oldest_laser_measurement_index);
  const auto oldest_min_area_ref_point_or =
      GetBboxCenter(oldest_laser_m,
                    /*use_min_area_bbox=*/true);
  const auto oldest_detection_ref_point_or =
      GetBboxCenter(oldest_laser_m,
                    /*use_min_area_bbox*/ false);

  // Get latest laser measurement detection bbox center and min area bbox
  // center.
  const auto& latest_laser_m =
      m_history.value(latest_laser_measurement_index)->laser_measurement();
  const auto& latest_laser_m_timestamp =
      m_history.time(latest_laser_measurement_index);
  const auto latest_min_area_ref_point_or =
      GetBboxCenter(latest_laser_m,
                    /*use_min_area_bbox=*/true);
  const auto latest_detection_ref_point_or =
      GetBboxCenter(latest_laser_m,
                    /*use_min_area_bbox*/ false);

  constexpr double kMaxSpeedMeasurementNoise = 0.125;  // m/s
  // Compute detection bbox center moving average speed by using the movement.
  if (latest_detection_ref_point_or && oldest_detection_ref_point_or) {
    const auto detection_ref_point_velocity_measurement =
        (*latest_detection_ref_point_or - *oldest_detection_ref_point_or) /
        (latest_laser_m_timestamp - oldest_laser_m_timestamp + DBL_EPSILON);

    // If the speed is below than threshold, we cap the
    // velocity to zero.
    if (detection_ref_point_velocity_measurement.norm() <
        kMaxSpeedMeasurementNoise) {
      *vel = Vec2d(0.0, 0.0);
      VLOG(2)
          << "Refine motion judges the track to static, detection bbox moving "
             "average speed, track id: "
          << track->track_state.id << "  "
          << detection_ref_point_velocity_measurement.norm();
      track->track_state.moving_state = TrackState::MovingState::kStatic;
      return;
    }
  }
  // Compute min area bbox center moving average speed by using the movement.
  if (latest_min_area_ref_point_or && oldest_min_area_ref_point_or) {
    const auto min_area_ref_point_velocity_measurement =
        (*latest_min_area_ref_point_or - *oldest_min_area_ref_point_or) /
        (latest_laser_m_timestamp - oldest_laser_m_timestamp + DBL_EPSILON);

    // If the speed is below than kMaxStationaryVehicleSpeed, we cap the
    // velocity to zero.
    if (min_area_ref_point_velocity_measurement.norm() <
        kMaxSpeedMeasurementNoise) {
      *vel = Vec2d(0.0, 0.0);
      VLOG(2)
          << "Refine motion judges the track to static, min area bbox moving "
             "average speed, track id: "
          << track->track_state.id << "  "
          << min_area_ref_point_velocity_measurement.norm();

      track->track_state.moving_state = TrackState::MovingState::kStatic;
      return;
    }
  }

  // 3. Detection bbox center moving consistency check: we suppose the
  // detection bbox center movement is consistent when the object is
  // true moving.
  constexpr double kMinTimeInterval = 0.5;
  // TODO(zheng): Handle the detection lost situation.
  if (latest_detection_ref_point_or && oldest_detection_ref_point_or) {
    // The center movement from oldest laser measurement to latest laser
    // measuremnet.
    const Vec2d oldest2latest_center_movement =
        *latest_detection_ref_point_or - *oldest_detection_ref_point_or;
    const double oldest2latest_center_moving_dist =
        oldest2latest_center_movement.norm();

    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement() &&
          m->laser_measurement().has_detection_bounding_box()) {
        const auto laser_m = m->laser_measurement();
        // Compute current frame laser measurement detection bbox center.
        const auto curr_detection_ref_point_or =
            GetBboxCenter(laser_m,
                          /*use_min_area_bbox*/ false);
        if (curr_detection_ref_point_or) {
          // Compute current frame detection bbox center movement.
          const Vec2d latest2current_center_movement =
              *curr_detection_ref_point_or - *oldest_detection_ref_point_or;
          const double latest2current_center_moving_dist =
              latest2current_center_movement.norm();
          // Only the two measurements have enough time interval, we perform
          // measurement moving coincident sanity check.
          const double time_diff =
              m->timestamp() - m_history.time(oldest_laser_measurement_index);
          if (time_diff > kMinTimeInterval) {
            if (oldest2latest_center_moving_dist < DBL_EPSILON ||
                latest2current_center_moving_dist < DBL_EPSILON) {
              *vel = Vec2d(0.0, 0.0);
              track->track_state.moving_state =
                  TrackState::MovingState::kStatic;
              VLOG(2)
                  << "Refine motion judges the track to static, moving dist "
                     "consitince check, track id: "
                  << track->track_state.id << " "
                  << latest2current_center_moving_dist;

              return;
            }

            // Compute the angle between the center movement, if the angle
            // greater than a threshold, we suppose the speed is a shooter,
            // because for vehicle/cyc the movement should not change so
            // large.
            constexpr double kMaxAngleChange = M_PI / 2.0;
            const auto angle = std::acos(oldest2latest_center_movement.dot(
                                             latest2current_center_movement) /
                                         (oldest2latest_center_moving_dist *
                                          latest2current_center_moving_dist));

            if (angle > kMaxAngleChange) {
              *vel = Vec2d(0.0, 0.0);
              VLOG(2) << "Refine motion judges the track to static, angle "
                         "consitince "
                         "check, track id: "
                      << track->track_state.id << " " << angle;
              track->track_state.moving_state =
                  TrackState::MovingState::kStatic;
              return;
            }
          }
        }
      }
    }
  }

  // If the velocity pass through all the sanity check, we suppose it's a true
  // moving object.
  track->track_state.moving_state = TrackState::MovingState::kMoving;
}

ObjectProto::MovingStateProto MovingStateToProto(
    const TrackState::MovingState& moving_state) {
  switch (moving_state) {
    case TrackState::MovingState::kUnKnown:
      return ObjectProto::MS_UNKNOWN;
    case TrackState::MovingState::kStatic:
      return ObjectProto::MS_STATIC;
    case TrackState::MovingState::kMoving:
      return ObjectProto::MS_MOVING;
  }
}

double ComputeCarAndCycHeadingMeasurementNoise(
    const Track<TrackState>& track, const MeasurementProto& curr_measurement,
    double init_heading_noise) {
  // NOTE(zheng): To handle the white noise of the heading measurement,
  // we use max_yawd and min_yawd in a latest time window to model
  // the heading measurement noise, this only suitable for vehicle/cyclist
  // whose motion filer is car model.
  double max_yawd = -DBL_MAX;
  double min_yawd = DBL_MAX;
  // NOTE(zheng): The param is same with heading measurement noise in
  // CTRA motion filter.
  // TODO(zheng): Load he param form param file after we refactor the
  // motion filter code, in case we forget to modify this param when
  // we modify heading measurement noise in CTRA motion filter.
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  std::optional<double> curr_heading_measurement;
  std::optional<Vec2d> curr_bbox_center;

  if (curr_measurement.has_laser_measurement() &&
      curr_measurement.laser_measurement().has_detection_bounding_box()) {
    const auto& bbox =
        curr_measurement.laser_measurement().detection_bounding_box();
    curr_heading_measurement = std::make_optional(bbox.heading());
    curr_bbox_center = std::make_optional(Vec2d(bbox.x(), bbox.y()));
  }
  // If the measurement history is less than 2 measurements, there is no
  // enough history info to compute heading noise, so we return a default value.
  // And if the curr measurement has no detection bbox, there is no need to
  // compute measurement noise, so we alse return a default value.
  if (num_m < 2 || !curr_heading_measurement) {
    return init_heading_noise;
  }
  const double last_m_timestamp = m_history.back_time();
  constexpr double kDurationAsValidMeasurement = 0.6;  // s
  int valid_count = 0;
  // Use measurement bbox heading to compute yawd, and get the min/max yawd in a
  // window.
  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidMeasurement);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    const auto* next_m =
        (j == num_m - 1) ? &curr_measurement : m_history.value(j + 1);
    if (m->has_laser_measurement() && next_m->has_laser_measurement() &&
        m->laser_measurement().has_detection_bounding_box() &&
        next_m->laser_measurement().has_detection_bounding_box()) {
      const double time_diff = next_m->timestamp() - m->timestamp();
      if (std::fabs(time_diff) < DBL_EPSILON) {
        continue;
      }
      ++valid_count;
      const double m_next_heading =
          next_m->laser_measurement().detection_bounding_box().heading();
      const double m_heading =
          m->laser_measurement().detection_bounding_box().heading();
      const double yawd =
          NormalizeAngle(m_next_heading - m_heading) / time_diff;
      max_yawd = std::max(max_yawd, yawd);
      min_yawd = std::min(min_yawd, yawd);
    }
  }

  // If the heading is different with motion direction, we enlarge
  // heading measurement noise.
  double noise_by_motion_direction = 0.0;
  if (track.track_state.moving_state == TrackState::MovingState::kMoving) {
    std::optional<Vec2d> oldest_bbox_center;
    constexpr double kDurationUsedForComputingMovingVec = 0.3;  // s
    // Find oldest bbox center.
    for (int j = m_history.GetIndexWithTimeAtLeast(
             last_m_timestamp - kDurationUsedForComputingMovingVec);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement() &&
          m->laser_measurement().has_detection_bounding_box()) {
        const auto& bbox = m->laser_measurement().detection_bounding_box();
        oldest_bbox_center = std::make_optional(Vec2d(bbox.x(), bbox.y()));
        break;
      }
    }
    // Compute moving direction and heading angle diff.
    if (oldest_bbox_center && curr_bbox_center && curr_heading_measurement) {
      const Vec2d moving_vec = (*curr_bbox_center - *oldest_bbox_center);
      const double moving_vec_angle = moving_vec.FastAngle();
      const double movint_dist = moving_vec.norm();
      // If the moving dist is very is very small, the movement direction is
      // not reliable.
      constexpr double kMinMovingDistThreshold = 0.1;  // m;
      if (movint_dist > kMinMovingDistThreshold) {
        double angle_diff_with_movement = std::fabs(
            NormalizeAngle(moving_vec_angle - *curr_heading_measurement));
        // In case of the car/cyc is reversing, we ignore the heading flip case.
        if (angle_diff_with_movement > M_PI_2) {
          angle_diff_with_movement = M_PI - angle_diff_with_movement;
        }
        // Considering of U turn case, we should subtract the effect of
        // yaw rate.
        angle_diff_with_movement -=
            min_yawd * kDurationUsedForComputingMovingVec;
        angle_diff_with_movement = std::max(0.0, angle_diff_with_movement);
        noise_by_motion_direction =
            angle_diff_with_movement * angle_diff_with_movement;
      }
    }
  }
  // TODO(zheng): Put the params to proto file.
  constexpr double kSecondPerFrame = 0.1;  // s
  // Hyper param to zoom in/out measurement noise.
  constexpr double kNoiseScale = 3.0;
  constexpr double kMotionTermScale = 0.3;
  if (valid_count >= 2) {
    const double noise_per_frame = (max_yawd - min_yawd) * kSecondPerFrame;
    // We add an extra term in case of the noise is very small.
    const double heading_variance =
        noise_per_frame * noise_per_frame * kNoiseScale +
        noise_by_motion_direction * kMotionTermScale + init_heading_noise;
    return heading_variance;
  } else {
    return init_heading_noise;
  }
}

bool IsCertainStaticMeasurement(const MeasurementProto& measurement) {
  if (measurement.type_source() != MTS_SEMANTIC_MAP_ZONE) {
    return false;
  }
  // If the barrier or vegetation measurement comes from semantic map zone,
  // should jump association and promote it immediately.
  if (measurement.type() == MT_BARRIER || measurement.type() == MT_VEGETATION) {
    return true;
  }
  return false;
}

bool IsSuddenBreakHappening(const Track<TrackState>& track) {
  const auto& m_history = track.measurement_history;
  const auto& track_history = track.checkpoints;
  const int num_m = m_history.size();
  const int num_track = track_history.size();
  constexpr int kMinHistorySizeForHardBreakJudging = 10;
  // If the measurement history or track history is less than 10 , there is no
  // enough info to judge if is brake hard.
  if (num_track < kMinHistorySizeForHardBreakJudging ||
      num_m < kMinHistorySizeForHardBreakJudging) {
    return false;
  }
  constexpr double kDurationAsValidVel = 0.5;
  std::vector<double> vel_history;
  std::vector<double> acc_history;
  const double last_m_timestamp = m_history.back_time();
  const auto t_type = track.track_state.type;
  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidVel);
       j < num_m - 1; ++j) {
    const auto* m = m_history.value(j);
    const int t_idx = track_history.GetIndexWithTimeAtLeast(m->timestamp());
    // If the t_idx is bigger than num_track, we consider it unreliable.
    if (t_idx >= num_track) {
      return false;
    }
    const auto t = track_history.value(t_idx);
    // If the motion filter of the object is changing, we consider it
    // unreliable.
    if (!t.estimator_3d.IsCarModel()) {
      return false;
    }
    // If the type of the track is changing, we consider it
    // unreliable.
    if (t.type != t_type) {
      return false;
    }
    // If the time diff between the measurement and track is too big, we
    // consider it unreliable.
    if (std::fabs(m->timestamp() - t.state_timestamp) > 0.1) {
      return false;
    }

    const auto s = t.estimator_3d.GetStateData();
    vel_history.emplace_back(s.GetVel());
    acc_history.emplace_back(s.GetAcc());
  }
  // Consider the brake hard scenairo: The vel should be greater than 0.0 for
  // the past while the vel should be smaller than 0.0 currently and its mean
  // should be greater than a threshold. The acc should be smaller than 0.0 for
  // the past and its mean should be smaller than a threshold. The threshold
  // should be chosen when we have more info.
  constexpr double kMaxStationaryVehicleSpeed = 0.25;
  constexpr double kThresholdscaleMaxVel = 0.3;
  double MinBrakeHardVel = 0.0;
  double mean_vel = 0.0;
  int vel_greater_than_zero = 0;
  for (int i = 0; i < vel_history.size(); ++i) {
    mean_vel += vel_history[i];
    MinBrakeHardVel =
        vel_history[i] > MinBrakeHardVel ? vel_history[i] : MinBrakeHardVel;
    if (vel_history[i] > 0.0) {
      vel_greater_than_zero++;
    }
  }
  mean_vel /= static_cast<double>(vel_history.size());
  MinBrakeHardVel *= kThresholdscaleMaxVel;

  // The vel should be greater than 0.0 for
  // the past while the vel should be smaller than 0.0 currently and its mean
  // should be greater than a threshold.
  if (mean_vel <= MinBrakeHardVel || mean_vel <= kMaxStationaryVehicleSpeed ||
      vel_greater_than_zero < vel_history.size() - 1) {
    return false;
  }

  constexpr double MinBrakeHardAcc = -0.8;
  constexpr double kMaxAccAllowedForBrake = 15.0;
  double mean_acc = 0.0;
  int acc_smaller_than_zero = 0;
  for (int i = 0; i < acc_history.size(); ++i) {
    // If the absolute value of acc is too big, we consider it unreliable.
    if (std::fabs(acc_history[i]) >= kMaxAccAllowedForBrake) {
      return false;
    }
    mean_acc += acc_history[i];
    if (acc_history[i] < -DBL_EPSILON) {
      acc_smaller_than_zero++;
    }
  }
  mean_acc /= static_cast<double>(acc_history.size());
  // The acc should be smaller than 0.0 for
  // the past and its mean should be smaller than a threshold.
  if (mean_acc >= MinBrakeHardAcc ||
      acc_smaller_than_zero < acc_history.size()) {
    return false;
  }

  return true;
}

bool IsSlowMovingVehicle(const Track<TrackState>& track, double timestamp) {
  if (track.track_state.type != MT_VEHICLE) {
    return false;
  }
  // Use icp velocity to check slow moving.
  const auto& m_history = track.measurement_history;
  const double last_m_timestamp = m_history.back_time();
  for (int j = m_history.size() - 1;
       j > m_history.GetIndexWithTimeAtLeast(last_m_timestamp - 0.3); --j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_icp_measurement() &&
        m->laser_measurement().icp_measurement().has_vel()) {
      const Vec2d icp_vel =
          Vec2dFromProto(m->laser_measurement().icp_measurement().vel());
      constexpr double kMaxStaticObjectIcpSpeenNoiseSqr = Sqr(0.9);
      constexpr double kMaxSlowMovingIcpSpeedSqr = Sqr(1.5);
      if (icp_vel.squaredNorm() > kMaxStaticObjectIcpSpeenNoiseSqr &&
          icp_vel.squaredNorm() < kMaxSlowMovingIcpSpeedSqr) {
        return true;
      }
    }
  }

  // We can use the ref point measurement velocity to judge if the vehicle is
  // slow moving.
  double max_ref_point_speed = std::numeric_limits<double>::lowest();
  double min_ref_point_speed = std::numeric_limits<double>::max();
  const int checkpoints_num = track.checkpoints.size();
  int valid_num = 0;
  constexpr double kValidCheckPointsDuration = 0.5;  // s
  for (int j = track.checkpoints.GetIndexWithTimeAtLeast(
           timestamp - kValidCheckPointsDuration);
       j < checkpoints_num; ++j) {
    const TrackState& t0 = track.checkpoints.value(j);
    if (t0.ref_point_vel) {
      const auto speed = t0.ref_point_vel->dot(
          Vec2d::FastUnitFromAngle(track.track_state.heading));
      max_ref_point_speed = std::max(max_ref_point_speed, speed);
      min_ref_point_speed = std::min(min_ref_point_speed, speed);
      ++valid_num;
    }
  }
  // If the vehicle is slow moving, it has some features as follows:
  // 1. The speed should be in one direction in the duration.
  // 2. All speed should  greater than the noise threshold.
  // 3. All speed should less than a threshold.
  constexpr double kMaxStationaryVehicleSpeedNoise = 0.25;  // m/s
  constexpr double kMaxSpeedForSlowMovingVehicle = 1.5;     // m/s
  const bool is_in_one_direction =
      max_ref_point_speed * min_ref_point_speed > 0.0;
  const bool is_true_moving =
      std::fabs(max_ref_point_speed) > kMaxStationaryVehicleSpeedNoise &&
      std::fabs(min_ref_point_speed) > kMaxStationaryVehicleSpeedNoise;
  const bool is_slowing_moving =
      std::fabs(max_ref_point_speed) < kMaxSpeedForSlowMovingVehicle &&
      std::fabs(min_ref_point_speed) < kMaxSpeedForSlowMovingVehicle;
  if (valid_num > 1 && is_in_one_direction && is_true_moving &&
      is_slowing_moving) {
    return true;
  }
  return false;
}

Polygon2d ShiftPoints(const Vec2d& shift,
                      const std::vector<Vec2d>& contour_points) {
  std::vector<Vec2d> shifted_contour_points(contour_points);
  std::for_each(shifted_contour_points.begin(), shifted_contour_points.end(),
                [&](Vec2d& ele) { ele += shift; });
  return Polygon2d(std::move(shifted_contour_points));
}

Polygon2d PredictContour(const double timestamp,
                         const TrackState& track_state) {
  // TODO(zheng): Consider heading shift as well.
  const auto s = track_state.estimator_3d.ComputePrediction(timestamp);
  const Vec2d pos_shift =
      s.GetStatePos() - track_state.estimator_3d.GetStatePos();
  auto contour_points = track_state.contour.points();
  std::for_each(contour_points.begin(), contour_points.end(),
                [&](Vec2d& ele) { ele += pos_shift; });
  return Polygon2d(std::move(contour_points));
}

std::optional<qcraft::sensor_fov::SensorFov> ComputeLidarSensorFov(
    std::shared_ptr<const SensorFovsProto> sensor_fovs_proto) {
  if (sensor_fovs_proto) {
    return qcraft::sensor_fov::BuildLidarViewSensorFov(*sensor_fovs_proto);
  } else {
    return std::nullopt;
  }
}

bool IsTrackInLidarOccludedArea(
    const std::optional<qcraft::sensor_fov::SensorFov>& lidar_sensor_fov,
    const Track<TrackState>& track, const double timestamp) {
  if (lidar_sensor_fov.has_value()) {
    const auto predicted_contour = PredictContour(timestamp, track.track_state);
    const float height = track.track_state.max_z - track.track_state.min_z;
    const auto is_occluded =
        (*lidar_sensor_fov).IsOccluded(predicted_contour, height);
    if (is_occluded.ok() && *is_occluded) {
      return true;
    }
  }
  return false;
}

double ComputePValueFromChiSquareDistr(const double value, const int dof) {
  // To catch non-positive definite covariance matrix.
  QCHECK_GE(value, 0.0) << absl::StrFormat("Illegal chi square value: %f",
                                           value);
  QCHECK(dof == 4 || dof == 2);
  // Construct chi square table of dof 2 and dof 4.
  std::unordered_map<int, std::vector<double>> chi_square_table;
  const std::vector<double> cdf_list = {1.0,   0.995, 0.99,  0.975, 0.95,
                                        0.90,  0.10,  0.05,  0.025, 0.01,
                                        0.005, 0.001, 0.0001};
  chi_square_table[2] =
      std::vector<double>{0.0,   0.010, 0.020, 0.051,  0.103, 0.211, 4.605,
                          5.991, 7.378, 9.210, 10.597, 13.8,  18.5};
  chi_square_table[4] =
      std::vector<double>{0.0,   0.207,  0.297,  0.484,  0.711, 1.064, 7.779,
                          9.488, 11.143, 13.277, 14.860, 18.5,  23.5};
  const PiecewiseLinearFunction plf(chi_square_table[dof], cdf_list);
  return plf.Evaluate(value);
}
bool IsMeasurementGroupContainSpecificMeasurementType(
    const MeasurementsProto_GroupType group_type,
    const MeasurementProto::MeasurementCase measurement_type) {
  switch (group_type) {
    case MeasurementsProto::LIDAR:
      return measurement_type ==
             MeasurementProto::MeasurementCase::kLaserMeasurement;

    case MeasurementsProto::CAMERA:
      return measurement_type ==
                 MeasurementProto::MeasurementCase::kCamera3DMeasurement ||
             measurement_type ==
                 MeasurementProto::MeasurementCase::kCameraMeasurement;

    case MeasurementsProto::RADAR:
      return measurement_type ==
             MeasurementProto::MeasurementCase::kRadarMeasurement;

    case MeasurementsProto::ALL:
      return true;
    default:
      LOG(FATAL) << "Unknown measurement group type: " << group_type;
  }
}

}  // namespace qcraft::tracker::tracker_util
