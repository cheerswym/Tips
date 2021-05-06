#ifndef ONBOARD_PLANNER_UTIL_VEHICLE_GEOMETRY_UTIL_H_
#define ONBOARD_PLANNER_UTIL_VEHICLE_GEOMETRY_UTIL_H_

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/offset_rect.h"
#include "onboard/math/vec.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

inline OffsetRect CreateOffsetRectFromVehicleGeometry(
    const VehicleGeometryParamsProto &vehicle_geom) {
  return OffsetRect(
      0.5 * vehicle_geom.length(), 0.5 * vehicle_geom.width(),
      0.5 * vehicle_geom.length() - vehicle_geom.front_edge_to_center());
}

double GetCenterMaxCurvature(
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params);
double GetRelaxedCenterMaxCurvature(
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params);
Vec2d GetAvFrontLeftCorner(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params);
Vec2d GetAvFrontRightCorner(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params);
Vec2d GetAvFrontCenter(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params);
Vec2d GetAvGeometryCenter(
    Vec2d av_pos, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params);
Vec2d GetAvGeometryCenter(
    Vec2d av_pos, Vec2d av_dir,
    const VehicleGeometryParamsProto &vehicle_geometry_params);

// av_xy is the rear axle center
Box2d GetAvBox(Vec2d av_pos, double av_theta,
               const VehicleGeometryParamsProto &vehicle_geometry_params);
// av_xy is the rear axle center
Box2d GetAvBoxWithBuffer(
    Vec2d av_xy, double av_theta,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    double length_buffer, double width_buffer);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_VEHICLE_GEOMETRY_UTIL_H_
