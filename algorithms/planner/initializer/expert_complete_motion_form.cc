#include "onboard/planner/initializer/expert_complete_motion_form.h"

#include <utility>

#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto_converter.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

ExpertCompleteMotion::ExpertCompleteMotion(
    const GeometryFormBuilder* form_builder,
    const HistoryBuffer<planning_dataset::ObjectState>& expert_traj) {
  expert_traj_ = expert_traj;
  duration_ = expert_traj_.duration();
  form_builder_ = form_builder;
  int traj_size = expert_traj_.size();
  relative_stations_.reserve(traj_size);
  relative_stations_.push_back(0.0);
  for (int i = 1; i < traj_size; ++i) {
    relative_stations_[i] =
        relative_stations_[i - 1] +
        std::hypot(expert_traj_[i].second.pos().x() -
                       expert_traj_[i - 1].second.pos().x(),
                   expert_traj_[i].second.pos().y() -
                       expert_traj_[i - 1].second.pos().y());
  }
}

std::vector<MotionState> ExpertCompleteMotion::FastSample(double d_t) const {
  return Sample(d_t);
}

std::vector<MotionState> ExpertCompleteMotion::Sample(double d_t) const {
  const int num_samples = CeilToInt(duration_ / d_t) + 1;
  std::vector<MotionState> samples;
  samples.reserve(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    const auto interpolated_state = State(i * d_t);
    samples.push_back(std::move(interpolated_state));
  }
  return samples;
}

MotionState ExpertCompleteMotion::GetStartMotionState() const {
  const auto& start_object = expert_traj_.begin()->second;
  const auto xy_pos = Vec2d(start_object.pos().x(), start_object.pos().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = start_object.yaw(),
      .k = start_object.curvature(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = 0.0,
      .v = start_object.pose_proto_speed(),
      .a = start_object.body_x_acc(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.front(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState ExpertCompleteMotion::GetEndMotionState() const {
  const auto& end_object = expert_traj_.rbegin()->second;
  const auto xy_pos = Vec2d(end_object.pos().x(), end_object.pos().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = end_object.yaw(),
      .k = end_object.curvature(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = expert_traj_.rbegin()->first - expert_traj_.begin()->first,
      .v = end_object.pose_proto_speed(),
      .a = end_object.body_x_acc(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.back(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState ExpertCompleteMotion::State(double t) const {
  planning_dataset::ObjectState object_state;
  const double start_time = expert_traj_.front_time();
  const double absl_t = start_time + t;
  int least_idx = expert_traj_.GetIndexWithTimeAtLeast(absl_t);
  int most_idx = expert_traj_.GetIndexWithTimeAtMost(absl_t);
  if (least_idx == expert_traj_.size() || most_idx == -1) {
    QCHECK(false) << absl::StrFormat(
        "Interpolation t %f outside of expert_traj_ time range", t);
  }
  ObjectStateLinearInterpolation(
      expert_traj_[most_idx].second, expert_traj_[least_idx].second,
      expert_traj_[most_idx].first, expert_traj_[least_idx].first, absl_t,
      &object_state);
  const auto xy_pos = Vec2d(object_state.pos().x(), object_state.pos().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = object_state.yaw(),
      .k = object_state.curvature(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = t,
      .v = object_state.pose_proto_speed(),
      .a = object_state.body_x_acc(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_[most_idx] +
           std::hypot(xy_pos.x() - expert_traj_[most_idx].second.pos().x(),
                      xy_pos.y() - expert_traj_[most_idx].second.pos().y()),
      .l = sl_pos.l,
  };
  return motion_state;
}

}  // namespace qcraft::planner
