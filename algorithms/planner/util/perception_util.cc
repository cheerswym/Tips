#include "onboard/planner/util/perception_util.h"

#include <limits>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {

Polygon2d ComputeObjectContour(const ObjectProto &object_proto) {
  int contour_size = object_proto.contour_size();
  std::vector<Vec2d> vertices;
  vertices.reserve(contour_size);
  for (int j = 0; j < contour_size; ++j) {
    vertices.push_back(Vec2dFromProto(object_proto.contour(j)));
  }
  QCHECK_GT(vertices.size(), 2);
  return Polygon2d(std::move(vertices), /*is_convex=*/true);
}

ObjectProto AvPoseProtoToObjectProto(
    const std::string &object_id,
    const VehicleGeometryParamsProto &vehicle_geom, const PoseProto &pose,
    bool offroad) {
  ObjectProto object;
  object.set_id(object_id);
  object.set_type(ObjectType::OT_VEHICLE);

  const Box2d box =
      GetAvBox(/*av_pos=*/Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
               pose.yaw(), vehicle_geom);
  object.mutable_pos()->set_x(box.center().x());
  object.mutable_pos()->set_y(box.center().y());

  object.set_yaw(pose.yaw());
  // Estimate yaw_rate by curvature * speed.
  object.set_yaw_rate(pose.curvature() * pose.speed());

  object.mutable_vel()->set_x(pose.vel_smooth().x());
  object.mutable_vel()->set_y(pose.vel_smooth().y());

  object.mutable_accel()->set_x(pose.accel_smooth().x());
  object.mutable_accel()->set_y(pose.accel_smooth().y());

  for (const auto &pt : box.GetCornersCounterClockwise()) {
    pt.ToProto(object.add_contour());
  }

  box.ToProto(object.mutable_bounding_box());

  object.set_parked(false);
  object.set_offroad(offroad);

  // Set a very long life time.
  constexpr double kAvDefaultLifeTime =
      std::numeric_limits<double>::max();  // Seconds.
  object.set_life_time(kAvDefaultLifeTime);
  constexpr double kMinSpeed = 0.2;  // Meters per second.
  object.set_moving_state(std::abs(Vec2d(object.vel()).norm()) < kMinSpeed
                              ? ObjectProto::MS_STATIC
                              : ObjectProto::MS_MOVING);
  object.set_min_z(pose.pos_smooth().z());
  object.set_max_z(pose.pos_smooth().z() + vehicle_geom.height());

  if (pose.has_timestamp()) {
    object.set_timestamp(pose.timestamp());
  } else {
    QLOG(INFO) << absl::StrFormat("I do not find any timestamp for the av!");
  }
  return object;
}

}  // namespace planner
}  // namespace qcraft
