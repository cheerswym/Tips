#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {

double GetCenterMaxCurvature(
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  return std::tan(vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()) /
         vehicle_geometry_params.wheel_base();
}

double GetRelaxedCenterMaxCurvature(
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  constexpr double kMaxCurvatureRelaxFactor = 1.13;
  return GetCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params) *
         kMaxCurvatureRelaxFactor;
}

Vec2d GetAvFrontLeftCorner(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const Vec2d normal = tangent.Perp();
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center() +
         normal * vehicle_geometry_params.width() * 0.5;
}

Vec2d GetAvFrontRightCorner(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const Vec2d normal = tangent.Perp();
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center() -
         normal * vehicle_geometry_params.width() * 0.5;
}

Vec2d GetAvFrontCenter(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  return av_pos + tangent * vehicle_geometry_params.front_edge_to_center();
}

Vec2d GetAvGeometryCenter(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  return av_pos + tangent * 0.5 *
                      (vehicle_geometry_params.front_edge_to_center() -
                       vehicle_geometry_params.back_edge_to_center());
}

Vec2d GetAvGeometryCenter(
    Vec2d av_pos, Vec2d av_dir,
    const VehicleGeometryParamsProto &vehicle_geometry_params) {
  return av_pos + av_dir * 0.5 *
                      (vehicle_geometry_params.front_edge_to_center() -
                       vehicle_geometry_params.back_edge_to_center());
}

Box2d GetAvBox(Vec2d av_pos, double av_theta,
               const VehicleGeometryParamsProto &vehicle_geometry_params) {
  return GetAvBoxWithBuffer(av_pos, av_theta, vehicle_geometry_params,
                            /*length_buffer=*/0.0, /*width_buffer=*/0.0);
}

Box2d GetAvBoxWithBuffer(
    Vec2d av_xy, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    double length_buffer, double width_buffer) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_theta);
  const double half_length =
      vehicle_geometry_params.length() * 0.5 + length_buffer;
  const double half_width =
      vehicle_geometry_params.width() * 0.5 + width_buffer;
  const double rac_to_center =
      half_length -
      (length_buffer + vehicle_geometry_params.back_edge_to_center());
  const Vec2d center = av_xy + tangent * rac_to_center;
  return Box2d(half_length, half_width, center, av_theta, tangent);
}
}  // namespace planner
}  // namespace qcraft
