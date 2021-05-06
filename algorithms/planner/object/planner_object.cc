#include "onboard/planner/object/planner_object.h"

#include "onboard/math/geometry/util.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {

namespace {
Box2d BuildPerceptionBoundingBox(const ObjectProto& object,
                                 const Box2d& bounding_box) {
  if (object.has_bounding_box()) {
    return Box2d(object.bounding_box());
  } else {
    return bounding_box;
  }
}
}  // namespace

PlannerObject::PlannerObject(prediction::ObjectPrediction object_prediction)
    : prediction_(std::move(object_prediction)) {
  FromPrediction(prediction_);
}

void PlannerObject::FromPrediction(
    const prediction::ObjectPrediction& prediction) {
  object_proto_ = prediction.perception_object();

  pose_.set_s(0.0);
  // NOTE(lidong): Why not set time to `object_proto`'s time?
  pose_.set_t(0.0);

  constexpr double epsilon = 1e-10;
  const double theta = object_proto_.yaw();
  const double accel = Vec2dFromProto(object_proto_.accel()).norm();
  const Vec2d v = Vec2dFromProto(object_proto_.vel());
  const double v_norm = v.norm();
  const double kappa =
      v_norm < epsilon ? 0.0 : object_proto_.yaw_rate() / v_norm;
  const Vec2d pos = Vec2dFromProto(object_proto_.pos());

  pose_.set_pos(pos);
  pose_.set_theta(theta);
  pose_.set_kappa(kappa);
  pose_.set_a(accel);
  pose_.set_v(v_norm);
  velocity_ = v;

  contour_ = ComputeObjectContour(object_proto_);

  // NOTE(all): Perception's bounding box is not reliable. Therefore, we rebuild
  // a bounding box here.
  bounding_box_ = contour_.BoundingBoxWithHeading(theta);

  perception_bbox_ = BuildPerceptionBoundingBox(object_proto_, bounding_box_);

  aabox_ = contour_.AABoundingBox();

  is_stationary_ = prediction_.trajectories().empty() ||
                   prediction::IsStationaryPrediction(prediction_);
}

std::optional<int> PlannerObject::MostLikelyTrajectory() const {
  const int n = num_trajs();
  if (n == 0) return std::nullopt;
  double max_prob = 0.0;
  int traj_index = 0;
  for (int i = 0; i < n; ++i) {
    if (traj(i).probability() > max_prob) {
      max_prob = traj(i).probability();
      traj_index = i;
    }
  }
  return traj_index;
}

void PlannerObject::ToPlannerObjectProto(
    PlannerObjectProto* planner_object_proto) const {
  planner_object_proto->mutable_object()->CopyFrom(object_proto_);
  planner_object_proto->set_is_stationary(is_stationary_);
  const auto& trajs = prediction_.trajectories();
  const int num_trajs = trajs.size();

  for (int i = 0; i < num_trajs; ++i) {
    const auto& traj = trajs[i];

    auto prediction_info = planner_object_proto->add_prediction();
    prediction_info->set_probability(traj.probability());

    const auto& points = traj.points();
    const int num_points = points.size();
    for (int j = 0; j < num_points; ++j) {
      const auto& pt = points[j];
      auto pose = prediction_info->add_points();
      Vec2dToProto(pt.pos(), pose->mutable_pos());
      pose->set_s(pt.s());
      pose->set_theta(pt.theta());
      pose->set_kappa(pt.kappa());
      pose->set_t(pt.t());
      pose->set_v(pt.v());
      pose->set_a(pt.a());
    }
  }
}

}  // namespace planner
}  // namespace qcraft
