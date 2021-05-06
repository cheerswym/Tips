#include "onboard/math/geometry/util.h"
#include <vector>
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"

namespace qcraft {

Polygon2d SmoothPolygon2dFromGeoPolygonProto(
    const mapping::GeoPolygonProto& geo_polygon_proto,
    const CoordinateConverter& coordinate_converter) {
  std::vector<Vec2d> smooth_points;
  for (const auto& point : geo_polygon_proto.points()) {
    const Vec3d smooth_point = coordinate_converter.GlobalToSmooth(
        {point.longitude(), point.latitude(), point.altitude()});
    smooth_points.emplace_back(smooth_point.x(), smooth_point.y());
  }
  if (smooth_points.size() < 3) return Polygon2d();
  return Polygon2d(smooth_points);
}

AffineTransformation Mat4fToAffineTransformation(const Mat4f& mat) {
  return AffineTransformation::FromTranslation(
             mat.col(3).segment<3>(0).cast<double>())
      .ApplyRotation(Eigen::Quaternionf(mat.block<3, 3>(0, 0))
                         .cast<double>()
                         .normalized());
}

std::pair<double, double> ProjectBoxToRay(Vec2d ray_center, Vec2d ray_dir,
                                          const Box2d& box) {
  const double center = ray_dir.Dot(box.center() - ray_center);
  const Vec2d box_tangent = box.tangent();
  const double offset =
      0.5 * (std::abs(ray_dir.Dot(box_tangent * box.length())) +
             std::abs(ray_dir.Dot(box_tangent.Perp() * box.width())));
  return {center - offset, center + offset};
}
}  // namespace qcraft
