#include "onboard/prediction/predicted_trajectory.h"

#include <utility>
#include <vector>

#include "onboard/lite/logging.h"

namespace qcraft {
namespace prediction {

void PredictedTrajectoryPoint::FromProto(
    const PredictedTrajectoryPointProto &proto) {
  set_pos(Vec2dFromProto(proto.pos()));
  set_s(proto.s());
  set_theta(proto.theta());
  set_kappa(proto.kappa());

  set_t(proto.t());
  set_v(proto.v());
  set_a(proto.a());
}

void PredictedTrajectoryPoint::ToProto(
    PredictedTrajectoryPointProto *proto) const {
  Vec2dToProto(pos(), proto->mutable_pos());
  proto->set_s(s());
  proto->set_theta(theta());
  proto->set_kappa(kappa());
  proto->set_t(t());
  proto->set_v(v());
  proto->set_a(a());
}

Polygon2d PredictedTrajectory::CreateContourForPoint(
    const Polygon2d &obj_contour, int i) const {
  QCHECK(!points().empty());
  std::vector<Vec2d> new_contour_points = obj_contour.points();
  const PredictedTrajectoryPoint &pred_point0 = points()[0];
  const PredictedTrajectoryPoint &pred_point = points()[i];
  for (int j = 0; j < new_contour_points.size(); ++j) {
    new_contour_points[j] =
        Vec2d(new_contour_points[j] - pred_point0.pos())
            .FastRotate(pred_point.theta() - pred_point0.theta()) +
        pred_point.pos();
  }
  return Polygon2d(std::move(new_contour_points));
}

void PredictedTrajectory::PrintDebugInfo() const {
  QLOG(INFO) << absl::StrFormat(
      "\ttype = %s (%d), probability = %7.6f, priority = %s (%d), annotation = "
      "%s, "
      "size = %d, "
      "is_reversed = %d",
      PredictionType_Name(type_), type_, probability_,
      ObjectPredictionPriority_Name(priority_), priority_, annotation_,
      points_.size(), is_reversed_);
}

void PredictedTrajectory::FromProto(const PredictedTrajectoryProto &proto) {
  probability_ = proto.probability();
  priority_ = proto.priority();
  type_ = proto.type();
  annotation_ = proto.annotation();
  index_ = proto.index();
  is_reversed_ = proto.is_reversed();
  points_.reserve(proto.points_size());
  for (const auto &point_proto : proto.points()) {
    points_.emplace_back();
    points_.back().FromProto(point_proto);
  }
}

void PredictedTrajectory::ToProto(PredictedTrajectoryProto *proto) const {
  proto->Clear();
  proto->set_probability(probability_);
  proto->set_priority(priority_);
  proto->set_type(type_);
  proto->set_annotation(annotation_);
  proto->set_index(index_);
  proto->set_is_reversed(is_reversed_);
  for (const auto &point : points_) {
    point.ToProto(proto->add_points());
  }
}

}  // namespace prediction
}  // namespace qcraft
