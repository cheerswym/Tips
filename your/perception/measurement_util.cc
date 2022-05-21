#include "onboard/perception/measurement_util.h"

#include <limits>
#include <set>

#include "onboard/global/trace.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace measurement_util {

std::vector<bool> GetOccludedMeasurementIndex(
    const VehiclePose& pose,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    const MeasurementsProto& measurements) {
  // For the measurements group, we compute which mesurement is occluded, and
  // return the occluded index mask.
  SCOPED_QTRACE("MeasurementUtil::GetOccludedMeasurementIndex");

  const int num_measurements = measurements.measurements_size();

  std::vector<bool> occluded_list(num_measurements, false);
  // Describe two bounds of each coutour from the perspective view in
  // azimuth angle.
  struct ContourBound {
    float left = std::numeric_limits<float>::max();
    float right = std::numeric_limits<float>::lowest();
    float range = 0.0;
    MeasurementType type = MT_STATIC_OBJECT;
  };

  const auto lidar_view_point_in_vehicle_coord =
      projection_util::GetLidarViewPointInVehicleCoord(lidar_params);

  const auto convert_point_to_azimuth_angle_in_degree = [](const Vec3d& point) {
    // Angle theta from reverse direction of x axis (-1, 0, 0) to the point (x,
    // y, 0), clockwise. Result is theta if point.y >= 0, 2 * PI - theta
    // otherwise in range [0, 360).
    const float theta = std::acos(-point.x() / Hypot(point.x(), point.y()));
    return r2d(point.y() >= 0 ? theta : 2 * M_PI - theta);
  };

  std::vector<ContourBound> contour_bounds(num_measurements);
  const auto smooth2vehicle_transform = pose.ToTransform().Inverse();
  for (int i = 0; i < num_measurements; ++i) {
    const auto& m = measurements.measurements(i);
    if (!m.has_laser_measurement()) {
      continue;
    }
    const auto& laser_m = m.laser_measurement();
    const auto measurement_countour = laser_m.contour();
    auto& contour_bound = contour_bounds[i];
    for (const auto& point : measurement_countour) {
      const Vec3d point_ref_lidar_coord =
          smooth2vehicle_transform.TransformPoint(
              Vec3d(point.x(), point.y(), pose.z)) -
          lidar_view_point_in_vehicle_coord;
      // Point is not valid when both x and y is 0.
      if (point_ref_lidar_coord.x() == 0 && point_ref_lidar_coord.y() == 0) {
        continue;
      }
      const float point_range =
          Vec2d(point_ref_lidar_coord.x(), point_ref_lidar_coord.y())
              .squaredNorm();
      const float azimuth =
          convert_point_to_azimuth_angle_in_degree(point_ref_lidar_coord);
      contour_bound.left = std::min(contour_bound.left, azimuth);
      contour_bound.right = std::max(contour_bound.right, azimuth);
      contour_bound.range = std::max(contour_bound.range, point_range);
      contour_bound.type = m.type();
    }
    // Handling those rear objects that are at 0/360 boundary.
    // Keep the definition of contiur left and right.
    constexpr double kMaxObjectAzimuthCoverageInDegree = 180.;  // degree.
    if (contour_bound.left != std::numeric_limits<float>::max() &&
        (contour_bound.right - contour_bound.left >
         kMaxObjectAzimuthCoverageInDegree)) {
      contour_bound.left = std::numeric_limits<float>::max();
      contour_bound.right = std::numeric_limits<float>::lowest();
      for (const auto& point : measurement_countour) {
        const Vec3d point_ref_lidar_coord =
            smooth2vehicle_transform.TransformPoint(
                Vec3d(point.x(), point.y(), pose.z)) -
            lidar_view_point_in_vehicle_coord;
        const float azimuth =
            convert_point_to_azimuth_angle_in_degree(point_ref_lidar_coord);
        if (azimuth >= 180.) {
          contour_bound.left = std::min(contour_bound.left, azimuth);
        } else {
          contour_bound.right = std::max(contour_bound.right, azimuth);
        }
      }
    }
  }

  const auto is_occluded_by_vehicle = [](const ContourBound& media,
                                         const ContourBound& occluded) {
    // Only judge if the contour is occluded by a vehicle.
    if (media.type != MT_VEHICLE) {
      return false;
    }
    if (occluded.left < occluded.right && media.left < media.right) {
      // If neither of them is at 0/360 boundary.
      return !(occluded.left > media.right || occluded.right < media.left);
    } else if (occluded.left > occluded.right && media.left > media.right) {
      // If both of them is at 0/360 boundary, there is overlapped area.
      return true;
    } else if (occluded.left < occluded.right && media.left > media.right) {
      // If media is at 0/360 boundary.
      return (occluded.left < media.right || occluded.right > media.left);
    } else if (occluded.left > occluded.right && media.left < media.right) {
      // If occluded is at 0/360 boundary.
      return (media.left < occluded.right || media.right > occluded.left);
    }
    return true;
  };

  for (int i = 0; i < num_measurements; ++i) {
    for (int j = 0; j < num_measurements; ++j) {
      if (i == j) continue;
      const auto& media_contour_bound = contour_bounds[i];
      const auto& occluded_contour_bound = contour_bounds[j];
      if (media_contour_bound.range > occluded_contour_bound.range) continue;

      if (is_occluded_by_vehicle(/*media=*/media_contour_bound,
                                 /*occluded=*/occluded_contour_bound)) {
        occluded_list[j] = true;
        break;
      }
    }
  }

  return occluded_list;
}

bool ShouldRefineLaserDetectionBBox(
    const MeasurementProto& measurement,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    const VehiclePose& pose, bool is_occluded) {
  if (is_occluded || !measurement.has_laser_measurement()) {
    return false;
  }
  const auto& laser_measurement = measurement.laser_measurement();
  if (!laser_measurement.has_detection_bounding_box()) {
    return false;
  }

  const auto lidar_view_point_in_vehicle_coord =
      projection_util::GetLidarViewPointInVehicleCoord(lidar_params);
  const auto& box2d_proto = laser_measurement.detection_bounding_box();
  Box2d det_bbox =
      Box2d({box2d_proto.x(), box2d_proto.y()}, box2d_proto.heading(),
            box2d_proto.length(), box2d_proto.width());

  // Compute nearest corner.
  const auto corners = det_bbox.GetCornersCounterClockwise();
  double min_dist2 = std::numeric_limits<double>::max();
  Vec3d nearest_corner_ref_lidar_coord;
  const auto smooth2vehicle_transform = pose.ToTransform().Inverse();

  for (int i = 0; i < corners.size(); ++i) {
    const Vec3d corner_ref_lidar_coord =
        smooth2vehicle_transform.TransformPoint(
            Vec3d(corners[i].x(), corners[i].y(), pose.z)) -
        lidar_view_point_in_vehicle_coord;
    const double dist2 = corner_ref_lidar_coord.squaredNorm();
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      nearest_corner_ref_lidar_coord = corner_ref_lidar_coord;
    }
  }

  const double length = det_bbox.length();
  constexpr double kMinLengthForBigVehicle = 6.0;
  constexpr double kMaxYDist = 5.0;
  constexpr double kMinX = -30.0;
  constexpr double kMaxX = 80.0;
  // Condition 1: the object should be a large vehicle.
  const bool is_large_vehicle = length > kMinLengthForBigVehicle;
  // Condition 2: the object distance to ego car should be in a threshold.
  const bool is_near_ego_car = nearest_corner_ref_lidar_coord.x() > kMinX &&
                               nearest_corner_ref_lidar_coord.x() < kMaxX;
  // Condition 3: the object should be in ego car's lane or neighbor lanes.
  const bool is_in_nearby_lanes =
      std::abs(nearest_corner_ref_lidar_coord.y()) < kMaxYDist;
  return is_large_vehicle && is_near_ego_car && is_in_nearby_lanes;
}

void RefineLaserMeasurementBoundingBox(
    const VehiclePose& pose,
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
    MeasurementProto* measurement) {
  SCOPED_QTRACE("MeasurementUtil::RefineLaserMeasurementBoundingBox");
  // We can refine the bbox by using contour to solve bbox jump issue.
  if (!measurement->has_laser_measurement()) {
    return;
  }
  auto* laser_measurement = measurement->mutable_laser_measurement();
  if (!laser_measurement->has_detection_bounding_box()) {
    return;
  }
  const double measurement_heading =
      NormalizeAngle(laser_measurement->detection_bounding_box().heading());
  const auto measurement_contour =
      geometry_util::ToPolygon2d(laser_measurement->contour());

  const int contour_point_num = measurement_contour.num_points();
  const auto contour_bbox = Box2d(laser_measurement->min_area_bounding_box());
  const auto& detection_bbox = laser_measurement->detection_bounding_box();
  const Vec2d det_bbox_center = Vec2d(detection_bbox.x(), detection_bbox.y());
  // Transformation contour points to object coord.
  const Eigen::Vector2d smooth2object_trans = contour_bbox.center();
  const auto transformed_polygon = measurement_contour.Transform(
      smooth2object_trans, std::cos(-measurement_heading),
      std::sin(-measurement_heading), -smooth2object_trans);

  // Convert ego car pos and detection bbox center to object coord.
  const auto lidar_view_point_in_vehicle_coord =
      projection_util::GetLidarViewPointInVehicleCoord(lidar_params);
  const Vec3d ref_lidar_smooth_pos =
      pose.ToTransform().TransformPoint(lidar_view_point_in_vehicle_coord);
  const Vec2d lidar_view_point_in_vehicle_coord_in_object_pos =
      (ref_lidar_smooth_pos.xy() - smooth2object_trans)
          .Rotate(-measurement_heading);
  const Vec2d det_bbox_pos_in_object_coord =
      (det_bbox_center - smooth2object_trans).Rotate(-measurement_heading);

  // Get vehicle front/rear border pos by using bbox length.
  const double min_area_front_border_pos = contour_bbox.half_length();
  const double min_area_rear_border_pos = -contour_bbox.half_length();

  const double det_front_border_pos =
      det_bbox_pos_in_object_coord.x() + 0.5 * detection_bbox.length();
  const double det_rear_border_pos =
      det_bbox_pos_in_object_coord.x() - 0.5 * detection_bbox.length();

  // NOTE(zheng): The big vehicle may undersegmented with
  // cone/vegetation/pedestrian, we can extrate contour points which is close
  // to the border, and compoute the points range in y axis, if the range is
  // greater than a threshold, it has a big probalility to be a true vhicle's
  // border, otherwise it may be undersegmented with cone/vegetation/pedestrian.
  const auto contour_is_true_border = [&](double border_pos) {
    constexpr double kBorderThickness = 0.2;  // m
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (int i = 0; i < contour_point_num; ++i) {
      const auto& point = transformed_polygon.points()[i];
      const double to_border_dist = std::abs(point(0) - border_pos);
      if (to_border_dist < kBorderThickness) {
        min_y = std::min(min_y, point(1));
        max_y = std::max(max_y, point(1));
      }
    }
    const double border_width = max_y - min_y;
    constexpr double kIsTrueBorderWidthThreshold = 1.2;  // m
    return border_width > kIsTrueBorderWidthThreshold;
  };
  // Judge if we can refine bbox and compute center offset.
  double center_offset_x = 0.0;
  bool bbox_should_refine = false;
  if (lidar_view_point_in_vehicle_coord_in_object_pos.x() < 0.0 &&
      contour_is_true_border(min_area_rear_border_pos)) {
    // Refine bbox when the vehicle is in front or beside of the ego car.
    center_offset_x = min_area_rear_border_pos - det_rear_border_pos;
    bbox_should_refine = true;
  } else if (lidar_view_point_in_vehicle_coord_in_object_pos.x() >= 0.0 &&
             contour_is_true_border(min_area_front_border_pos)) {
    // Refine bbox when the vehicle is behind or beside of the ego car.
    center_offset_x = min_area_front_border_pos - det_front_border_pos;
    bbox_should_refine = true;
  }

  if (bbox_should_refine) {
    const Vec2d det_bbox_center_in_object_coord =
        (det_bbox_center - smooth2object_trans).Rotate(-measurement_heading);

    const Vec2d new_center =
        Vec2d(det_bbox_center_in_object_coord.x() + center_offset_x,
              det_bbox_center_in_object_coord.y())
            .Rotate(measurement_heading) +
        smooth2object_trans;
    auto* bb = laser_measurement->mutable_detection_bounding_box();
    bb->set_x(new_center.x());
    bb->set_y(new_center.y());
  }
}
}  // namespace measurement_util
}  // namespace qcraft
