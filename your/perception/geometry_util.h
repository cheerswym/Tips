#ifndef ONBOARD_PERCEPTION_GEOMETRY_UTIL_H_
#define ONBOARD_PERCEPTION_GEOMETRY_UTIL_H_

#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"

namespace qcraft {
namespace geometry_util {
template <typename PointsT>
Polygon2d ToPolygon2d(const PointsT& points_proto) {
  std::vector<Vec2d> points;
  points.reserve(points_proto.size());
  for (const auto& p : points_proto) {
    points.push_back(Vec2d(p.x(), p.y()));
  }
  return Polygon2d(points);
}

Box2d ConvertBox2dSmoothToVehicle(
    const Box2d& box, const VehiclePose& pose,
    const AffineTransformation& smooth_to_vehicle);

inline Box2d ConvertBox2dSmoothToVehicle(const Box2d& box,
                                         const VehiclePose& pose) {
  const auto smooth_to_vehicle = pose.ToTransform().Inverse();
  return ConvertBox2dSmoothToVehicle(box, pose, smooth_to_vehicle);
}

Box2d ConvertBox2dVehicleToSmooth(
    const Box2d& box, const VehiclePose& pose,
    const AffineTransformation& vehicle_to_smooth);

inline Box2d ConvertBox2dVehicleToSmooth(const Box2d& box,
                                         const VehiclePose& pose) {
  const auto vehicle_to_smooth = pose.ToTransform();
  return ConvertBox2dVehicleToSmooth(box, pose, vehicle_to_smooth);
}

Polygon2d ConvertPolygon2dSmoothToVehicle(
    const Polygon2d& polygon, const AffineTransformation& smooth_to_vehicle);

inline Polygon2d ConvertPolygon2dSmoothToVehicle(const Polygon2d& polygon,
                                                 const VehiclePose& pose) {
  const auto smooth_to_vehicle = pose.ToTransform().Inverse();
  return ConvertPolygon2dSmoothToVehicle(polygon, smooth_to_vehicle);
}

Polygon2d ConvertPolygon2dVehicleToSmooth(
    const Polygon2d& polygon, const AffineTransformation& vehicle_to_smooth);

inline Polygon2d ConvertPolygon2dVehicleToSmooth(const Polygon2d& polygon,
                                                 const VehiclePose& pose) {
  const auto vehicle_to_smooth = pose.ToTransform();
  return ConvertPolygon2dVehicleToSmooth(polygon, vehicle_to_smooth);
}

}  // namespace geometry_util
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_GEOMETRY_UTIL_H_
