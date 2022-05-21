#include "onboard/perception/geometry_util.h"

#include <utility>
#include <vector>

#include "onboard/math/util.h"

namespace qcraft::geometry_util {

Box2d ConvertBox2dSmoothToVehicle(
    const Box2d& box, const VehiclePose& pose,
    const AffineTransformation& smooth_to_vehicle) {
  const Vec2d center_in_vehicle =
      smooth_to_vehicle.TransformPoint(Vec3d(box.center(), 0)).head<2>();
  return Box2d(center_in_vehicle, NormalizeAngle(box.heading() - pose.yaw),
               box.length(), box.width());
}

Box2d ConvertBox2dVehicleToSmooth(
    const Box2d& box, const VehiclePose& pose,
    const AffineTransformation& vehicle_to_smooth) {
  const Vec2d center_in_smooth =
      vehicle_to_smooth.TransformPoint(Vec3d(box.center(), 0)).head<2>();
  return Box2d(center_in_smooth, NormalizeAngle(box.heading() + pose.yaw),
               box.length(), box.width());
}

Polygon2d ConvertPolygon2dSmoothToVehicle(
    const Polygon2d& polygon, const AffineTransformation& smooth_to_vehicle) {
  std::vector<Vec2d> points_in_vehicle;
  points_in_vehicle.reserve(polygon.num_points());
  for (const auto& point : polygon.points()) {
    points_in_vehicle.emplace_back(
        smooth_to_vehicle.TransformPoint(Vec3d(point, 0)).head<2>());
  }
  return Polygon2d(std::move(points_in_vehicle), polygon.is_convex());
}

Polygon2d ConvertPolygon2dVehicleToSmooth(
    const Polygon2d& polygon, const AffineTransformation& vehicle_to_smooth) {
  std::vector<Vec2d> points_in_smooth;
  points_in_smooth.reserve(polygon.num_points());
  for (const auto& point : polygon.points()) {
    points_in_smooth.emplace_back(
        vehicle_to_smooth.TransformPoint(Vec3d(point, 0)).head<2>());
  }
  return Polygon2d(std::move(points_in_smooth), polygon.is_convex());
}

}  // namespace qcraft::geometry_util
