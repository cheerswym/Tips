#include "onboard/planner/test_util/perception_object_builder.h"

#include <string>

namespace qcraft {
namespace planner {

PerceptionObjectBuilder::PerceptionObjectBuilder() {
  object_.set_id("object");
  object_.set_type(OT_VEHICLE);

  Vec2d::Zero().ToProto(object_.mutable_pos());

  object_.set_yaw(0.0);
  object_.set_yaw_rate(0.0);

  object_.mutable_vel()->set_x(5.0);
  object_.mutable_vel()->set_y(0.0);

  object_.mutable_accel()->set_x(1.0);
  object_.mutable_accel()->set_y(0.0);

  object_.set_timestamp(1.0);

  // Setup the default bounding box.
  object_.mutable_bounding_box()->set_x(0.0);
  object_.mutable_bounding_box()->set_y(0.0);
  object_.mutable_bounding_box()->set_heading(0.0);
  object_.mutable_bounding_box()->set_length(4.0);
  object_.mutable_bounding_box()->set_width(2.0);

  object_.set_bounding_box_source(ObjectProto::FIERY_EYE_NET);
  object_.set_parked(false);
  object_.set_offroad(false);

  object_.mutable_pos_cov()->add_m(1.0);
  object_.mutable_pos_cov()->add_m(0.0);
  object_.mutable_pos_cov()->add_m(0.0);
  object_.mutable_pos_cov()->add_m(1.0);
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_id(absl::string_view id) {
  object_.set_id(std::string(id));
  return *this;
}
PerceptionObjectBuilder& PerceptionObjectBuilder::set_type(ObjectType type) {
  object_.set_type(type);
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_pos(Vec2d pos) {
  pos.ToProto(object_.mutable_pos());
  object_.mutable_bounding_box()->set_x(pos.x());
  object_.mutable_bounding_box()->set_y(pos.y());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_timestamp(double time) {
  object_.set_timestamp(time);
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_box_center(Vec2d pos) {
  object_.mutable_bounding_box()->set_x(pos.x());
  object_.mutable_bounding_box()->set_y(pos.y());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_length_width(
    double length, double width) {
  object_.mutable_bounding_box()->set_length(length);
  object_.mutable_bounding_box()->set_width(width);
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_yaw(double yaw) {
  object_.set_yaw(yaw);
  object_.mutable_bounding_box()->set_heading(yaw);
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_yaw_rate(
    double yaw_rate) {
  object_.set_yaw_rate(yaw_rate);
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_velocity(
    double velocity) {
  const Vec2d v = Vec2d::FastUnitFromAngle(object_.yaw()) * velocity;
  v.ToProto(object_.mutable_vel());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_speed(Vec2d speed) {
  speed.ToProto(object_.mutable_vel());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_accel(Vec2d accel) {
  accel.ToProto(object_.mutable_accel());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_accel(double accel) {
  const Vec2d acc = Vec2d::FastUnitFromAngle(object_.yaw()) * accel;
  acc.ToProto(object_.mutable_accel());
  return *this;
}

PerceptionObjectBuilder& PerceptionObjectBuilder::set_contour(
    const Polygon2d& contour) {
  for (const auto& pt : contour.points()) {
    pt.ToProto(object_.add_contour());
  }
  return *this;
}

ObjectProto PerceptionObjectBuilder::Build() {
  // When contour is empty, reconstruct contour from box info.
  if (object_.contour_size() == 0) {
    Box2d box(object_.bounding_box());
    for (const auto& pt : box.GetCornersCounterClockwise()) {
      pt.ToProto(object_.add_contour());
    }
  }
  return object_;
}

}  // namespace planner
}  // namespace qcraft
