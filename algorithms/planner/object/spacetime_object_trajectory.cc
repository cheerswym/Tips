#include "onboard/planner/object/spacetime_object_trajectory.h"

#include <vector>

#include "onboard/planner/object/planner_object.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject* planner_object, int traj_index,
    double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object->id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap) {
  QCHECK_GE(traj_index_, 0);
  QCHECK_LT(traj_index_, planner_object->num_trajs());
  trajectory_ = std::make_unique<const prediction::PredictedTrajectory>(
      planner_object->traj(traj_index_));
  states_ = SampleTrajectoryStates(
      *trajectory_, planner_object_->pose().pos(), planner_object_->contour(),
      planner_object_->bounding_box(), planner_object_->perception_bbox());
  is_stationary_ = prediction::IsStationaryTrajectory(*trajectory_);
}

absl::StatusOr<SpacetimeObjectTrajectory>
SpacetimeObjectTrajectory::CreateTruncatedCopy(double start_offset,
                                               double horizon) const {
  if (states_.empty()) {
    return absl::FailedPreconditionError(
        absl::StrCat("The trajectory ", traj_id_, " has no states."));
  }
  std::vector<SpacetimeObjectState> truncated_states;
  truncated_states.reserve(states_.size());

  constexpr double kEps = 1e-6;
  const double start_t = states_[0].traj_point->t() + start_offset - kEps;
  for (int i = 0, n = states_.size(); i < n; ++i) {
    if (states_[i].traj_point->t() - start_t > horizon) {
      break;
    }
    if (states_[i].traj_point->t() >= start_t) {
      truncated_states.push_back(states_[i]);
    }
  }

  if (truncated_states.empty()) {
    return absl::NotFoundError(
        absl::StrCat("No trajectory left after truncating ", traj_id_,
                     " with horizon ", horizon));
  }
  return SpacetimeObjectTrajectory(planner_object_, std::move(truncated_states),
                                   traj_index_, required_lateral_gap_);
}

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject* planner_object,
    std::vector<SpacetimeObjectState> states, int traj_index,
    double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object->id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap),
      states_(std::move(states)) {
  QCHECK_GE(traj_index_, 0);
  QCHECK_LT(traj_index_, planner_object->num_trajs());
  trajectory_ = std::make_unique<const prediction::PredictedTrajectory>(
      planner_object->traj(traj_index_));
  is_stationary_ = prediction::IsStationaryTrajectory(*trajectory_);
}

SpacetimeObjectTrajectory::SpacetimeObjectTrajectory(
    const PlannerObject* planner_object,
    std::unique_ptr<const prediction::PredictedTrajectory> traj, int traj_index,
    double required_lateral_gap)
    : traj_index_(traj_index),
      traj_id_(MakeTrajectoryId(planner_object->id(), traj_index)),
      planner_object_(planner_object),
      required_lateral_gap_(required_lateral_gap) {
  QCHECK_GE(traj_index_, 0);
  QCHECK_LT(traj_index_, planner_object->num_trajs());
  trajectory_ = std::move(traj);
  states_ = SampleTrajectoryStates(
      *trajectory_, planner_object_->pose().pos(), planner_object_->contour(),
      planner_object_->bounding_box(), planner_object_->perception_bbox());
  is_stationary_ = prediction::IsStationaryTrajectory(*trajectory_);
}

}  // namespace planner
}  // namespace qcraft
