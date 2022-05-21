#ifndef ONBOARD_PERCEPTION_PILLAR_RC_COORD_CONVERTER_H_
#define ONBOARD_PERCEPTION_PILLAR_RC_COORD_CONVERTER_H_

#include <utility>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft {

class PillarRCCoordConverter {
 public:
  PillarRCCoordConverter(const double range_front, const double range_left,
                         const double diameter);

  PillarRCCoordConverter() = delete;

  void InitializeWithPose(const VehiclePose &pose);
  // Should be called after InitializeWithPose.
  std::pair<int, int> SmoothCoordToRC(const Vec2d &coord) const {
    const Vec3d point_3d =
        smooth_to_vehicle_transform_.TransformPoint({coord, 0});
    return VehicleCoordToRC({point_3d.x(), point_3d.y()});
  }
  // Should be called after InitializeWithPose.
  Vec2d RCToSmoothCoord(const std::pair<int, int> &rc) const {
    const Vec2d vehicle_coord = RCToVehicleCoord(rc);
    const Vec3d smooth_coord_3d =
        vehicle_to_smooth_transform_.TransformPoint({vehicle_coord, 0});
    return {smooth_coord_3d.x(), smooth_coord_3d.y()};
  }
  // Convert the point of the vehicle coordinate system to the index in the
  // pillar semantic segmentation result, with the upper left vertex as the
  // origin.
  std::pair<int, int> VehicleCoordToRC(const Vec2d &coord) const {
    const int row = row_offset_ - FloorToInt(coord.x() * diameter_inv_);
    const int col = col_offset_ - FloorToInt(coord.y() * diameter_inv_);
    return {row, col};
  }
  // Convert index in pillar semantic segmentation result to vehicle coordinate
  // system.
  Vec2d RCToVehicleCoord(const std::pair<int, int> &rc) const {
    const double x = (row_offset_ - rc.first + 0.5) * diameter_;
    const double y = (col_offset_ - rc.second + 0.5) * diameter_;
    return {x, y};
  }

  // Convert the point of the vehicle coordinate system to the smooth coordinate
  // system
  Vec2d VehicleCoordToSmoothCoord(const Vec2d &point) const {
    const Vec3d smooth_coord_3d =
        vehicle_to_smooth_transform_.TransformPoint({point.x(), point.y(), 0});
    return {smooth_coord_3d.x(), smooth_coord_3d.y()};
  }

  int row_offset() const { return row_offset_; }
  int col_offset() const { return col_offset_; }

 private:
  const double diameter_;
  const double diameter_inv_;
  // Take the top left vertex as the origin， row_offset_ represents the row
  // where the origin is located, col_offset_ represents the row where the
  // origin is located
  const int row_offset_;
  const int col_offset_;
  AffineTransformation smooth_to_vehicle_transform_;
  AffineTransformation vehicle_to_smooth_transform_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_PILLAR_RC_COORD_CONVERTER_H_
