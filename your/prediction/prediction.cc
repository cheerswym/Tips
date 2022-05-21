#include "onboard/prediction/prediction.h"

#include "onboard/math/geometry/util.h"

namespace qcraft {
namespace prediction {

namespace {

Polygon2d GetPolygonFromPerception(const ObjectProto &object) {
  std::optional<Polygon2d> maybe_contour =
      Polygon2d::FromPoints(object.contour(),
                            /*is_convex=*/true);
  QCHECK(maybe_contour.has_value());
  return std::move(maybe_contour.value());
}

}  // namespace

ObjectPrediction::ObjectPrediction(
    std::vector<PredictedTrajectory> trajectories, const ObjectProto &object)
    : trajectories_(std::move(trajectories)), perception_object_(object) {
  contour_ = GetPolygonFromPerception(perception_object_);
}

ObjectPrediction::ObjectPrediction(const ObjectPredictionProto &proto) {
  FromProto(proto);
  contour_ = GetPolygonFromPerception(perception_object_);
}

void ObjectPrediction::CheckTrajectoryProb(double sum, double max,
                                           double min) const {
  const double prob_sum = trajectory_prob_sum();
  const double max_prob = trajectory_min_prob();
  const double min_prob = trajectory_max_prob();
  bool pass = true;
  if (prob_sum > (sum + kEpsilon)) {
    QLOG(WARNING) << absl::StrFormat(
        "I found trajs prob sum %7.6f (of %d traj(s)) larger than %7.6f for "
        "%s!",
        prob_sum, trajectories_.size(), sum, perception_object_.id());
  }
  if (max_prob > max + kEpsilon) {
    QLOG(WARNING) << absl::StrFormat(
        "I found the max traj prob %7.6f (of %d traj(s)) larger than %7.6f for "
        "%s!",
        max_prob, trajectories_.size(), max, perception_object_.id());
    pass = false;
  }
  if (min_prob < min - kEpsilon) {
    QLOG(WARNING) << absl::StrFormat(
        "I found the min traj prob %7.6f (of %d traj(s)) less than %7.6f for "
        "%s!",
        min_prob, trajectories_.size(), min, perception_object_.id());
    pass = false;
  }
  if (!pass) PrintDebugInfo();

  CHECK_LE(max_prob, max);
  CHECK_GE(min_prob, min);
}

void ObjectPrediction::PrintDebugInfo() const {
  QLOG(INFO) << absl::StrFormat(
      "object %s (%7.3f) prediction result (total prob = %6.5f, max prob = "
      "%7.6f, min prob = %7.6f):",
      perception_object_.id(), timestamp(), trajectory_prob_sum(),
      trajectory_max_prob(), trajectory_min_prob());
  for (int i = 0; i < trajectories_.size(); ++i) {
    QLOG(INFO) << absl::StrFormat("\ttraj %d / %d:", i + 1,
                                  trajectories_.size());
    trajectories_.at(i).PrintDebugInfo();
  }
}

Polygon2d ObjectPrediction::CreateContourForPoint(
    int predicted_trajectory_index, int predicted_point_index) const {
  return trajectories()[predicted_trajectory_index].CreateContourForPoint(
      contour(), predicted_point_index);
}

void ObjectPrediction::FromProto(const ObjectPredictionProto &proto) {
  perception_object_ = proto.perception_object();
  // trajectories
  trajectories_.reserve(proto.trajectories_size());
  for (const auto &trajectory_proto : proto.trajectories()) {
    trajectories_.emplace_back().FromProto(trajectory_proto);
  }

  // stop time
  stop_time_ = proto.stop_time();

  // Long-term behavior.
  long_term_behavior_.FromProto(proto.long_term_behavior());
}

void ObjectPrediction::ToProto(ObjectPredictionProto *proto) const {
  proto->Clear();
  proto->set_id(perception_object_.id());
  // perception object
  proto->mutable_perception_object()->CopyFrom(perception_object_);

  // trajectories
  for (const auto &trajectory : trajectories_) {
    trajectory.ToProto(proto->add_trajectories());
  }

  proto->mutable_stop_time()->CopyFrom(stop_time_);

  // Long-term behavior.
  long_term_behavior_.ToProto(proto->mutable_long_term_behavior());
}

}  // namespace prediction
}  // namespace qcraft
