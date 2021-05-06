#include "onboard/planner/initializer/candidate_complete_motion_form.h"

#include <utility>

#include "onboard/math/util.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

CandidateCompleteMotion::CandidateCompleteMotion(
    const GeometryFormBuilder* form_builder,
    const std::vector<ApolloTrajectoryPointProto>& candidate_traj) {
  candidate_traj_ = candidate_traj;
  duration_ = candidate_traj_.back().relative_time() -
              candidate_traj_.front().relative_time();
  form_builder_ = form_builder;
  int traj_size = candidate_traj_.size();
  relative_stations_.reserve(traj_size);
  relative_stations_.push_back(0.0);
  for (int i = 1; i < traj_size; ++i) {
    relative_stations_[i] =
        relative_stations_[i - 1] +
        std::hypot(candidate_traj_[i].path_point().x() -
                       candidate_traj_[i - 1].path_point().x(),
                   candidate_traj_[i].path_point().y() -
                       candidate_traj_[i - 1].path_point().y());
  }
}

std::vector<MotionState> CandidateCompleteMotion::FastSample(double d_t) const {
  return Sample(d_t);
}

std::vector<MotionState> CandidateCompleteMotion::Sample(double d_t) const {
  const int num_samples = CeilToInt(duration_ / d_t) + 1;
  std::vector<MotionState> samples;
  samples.reserve(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    const auto interpolated_state = State(i * d_t);
    samples.push_back(std::move(interpolated_state));
  }
  return samples;
}

MotionState CandidateCompleteMotion::GetStartMotionState() const {
  const auto& start_object = candidate_traj_.front();
  const auto xy_pos =
      Vec2d(start_object.path_point().x(), start_object.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = start_object.path_point().theta(),
      .k = start_object.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = start_object.relative_time(),
      .v = start_object.v(),
      .a = start_object.a(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.front(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState CandidateCompleteMotion::GetEndMotionState() const {
  const auto& end_object = candidate_traj_.back();
  const auto xy_pos =
      Vec2d(end_object.path_point().x(), end_object.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = end_object.path_point().theta(),
      .k = end_object.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = end_object.relative_time(),
      .v = end_object.v(),
      .a = end_object.a(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.back(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState CandidateCompleteMotion::State(double t) const {
  const double start_time = candidate_traj_.front().relative_time();
  const double absl_t = start_time + t;
  auto it_lower =
      std::lower_bound(candidate_traj_.begin(), candidate_traj_.end(), t,
                       [](const ApolloTrajectoryPointProto& tp,
                          const double t) { return tp.relative_time() < t; });
  ApolloTrajectoryPointProto object;
  int object_idx;
  if (it_lower == candidate_traj_.begin()) {
    object = candidate_traj_.front();
    object_idx = 0;
  } else if (it_lower == candidate_traj_.end()) {
    object = candidate_traj_.back();
    object_idx = candidate_traj_.size() - 1;
  } else {
    object = InterpolateTrajectoryPoint(*(it_lower - 1), *it_lower, absl_t);
    object_idx = std::distance(candidate_traj_.begin(), it_lower - 1);
  }
  const auto xy_pos = Vec2d(object.path_point().x(), object.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = object.path_point().theta(),
      .k = object.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = t,
      .v = object.v(),
      .a = object.a(),
      .accumulated_s = sl_pos.s,
      .s =
          relative_stations_[object_idx] +
          std::hypot(xy_pos.x() - candidate_traj_[object_idx].path_point().x(),
                     xy_pos.y() - candidate_traj_[object_idx].path_point().y()),
      .l = sl_pos.l,
  };
  return motion_state;
}

}  // namespace qcraft::planner
