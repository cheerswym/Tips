#include "onboard/planner/test_util/planner_object_builder.h"

#include <string>
#include <utility>
#include <vector>

namespace qcraft {
namespace planner {

void PlannerObjectBuilder::FromObjectProto(const ObjectProto& object_proto) {
  {
    object_.pose_.set_pos(Vec2dFromProto(object_proto.pos()));
    object_.pose_.set_s(0.0);
    // NOTE(lidong): Why not set time to `object_proto`'s time?
    object_.pose_.set_t(0.0);
  }

  const Vec2d v = Vec2dFromProto(object_proto.vel());
  object_.velocity_ = v;

  const auto norm_v = v.norm();
  object_.pose_.set_v(norm_v);

  constexpr double epsilon = 1e-10;
  auto theta = norm_v < epsilon ? object_proto.yaw() : v.FastAngle();
  object_.pose_.set_theta(theta);
  object_.pose_.set_a(Vec2dFromProto(object_proto.accel())
                          .dot(Vec2d::FastUnitFromAngle(theta)));

  {
    auto kappa = norm_v < epsilon ? 0.0 : object_proto.yaw_rate() / norm_v;
    object_.pose_.set_kappa(kappa);
  }

  {
    int contour_size = object_proto.contour_size();
    std::vector<Vec2d> vertices;
    vertices.reserve(contour_size);
    for (int j = 0; j < contour_size; ++j) {
      vertices.push_back(Vec2dFromProto(object_proto.contour(j)));
    }
    QCHECK_GT(vertices.size(), 2);
    object_.contour_ = Polygon2d(std::move(vertices), /*is_convex=*/true);
  }

  // Set the bounding box of the object.
  if (object_proto.has_bounding_box()) {
    const auto& bbox = object_proto.bounding_box();
    object_.bounding_box_ = Box2d({bbox.x(), bbox.y()}, bbox.heading(),
                                  bbox.length(), bbox.width());
  } else {
    object_.bounding_box_ = object_.contour_.BoundingBoxWithHeading(theta);
  }

  object_.aabox_ = object_.contour_.AABoundingBox();

  object_.object_proto_ = object_proto;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_id(std::string id) {
  object_.object_proto_.set_id(std::move(id));
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_type(ObjectType type) {
  object_.set_type(type);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_object(
    const ObjectProto& object) {
  FromObjectProto(object);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_pose(
    const SecondOrderTrajectoryPoint& pose) {
  object_.pose_ = pose;
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_pos(Vec2d pos) {
  object_.pose_.set_pos(pos);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_v(double v) {
  object_.pose_.set_v(v);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_theta(double theta) {
  object_.pose_.set_theta(theta);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_a(double a) {
  object_.pose_.set_a(a);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_t(double t) {
  object_.pose_.set_t(t);
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_stationary(bool stationary) {
  object_.is_stationary_ = stationary;
  return *this;
}

PlannerObjectBuilder& PlannerObjectBuilder::set_contour(Polygon2d contour) {
  object_.contour_ = std::move(contour);
  return *this;
}

ObjectPredictionBuilder* PlannerObjectBuilder::get_object_prediction_builder() {
  return &object_prediction_builder_;
}

PlannerObject PlannerObjectBuilder::Build() {
  object_prediction_builder_.set_object(object_.object_proto_);
  set_object(object_.object_proto_);
  *object_.mutable_prediction() = object_prediction_builder_.Build();
  return object_;
}

}  // namespace planner
}  // namespace qcraft
