#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

// A trajectory of a spacetime object.
class SpacetimeObjectTrajectory {
 public:
  SpacetimeObjectTrajectory(const PlannerObject* planner_object, int traj_index,
                            double required_lateral_gap);
  SpacetimeObjectTrajectory(const PlannerObject* planner_object,
                            std::vector<SpacetimeObjectState> states,
                            int traj_index, double required_lateral_gap);
  SpacetimeObjectTrajectory(
      const PlannerObject* planner_object,
      std::unique_ptr<const prediction::PredictedTrajectory> traj,
      int traj_index, double required_lateral_gap);

  SpacetimeObjectTrajectory CreateCopy() const {
    return SpacetimeObjectTrajectory(planner_object_, traj_index_,
                                     required_lateral_gap_);
  }

  absl::StatusOr<SpacetimeObjectTrajectory> CreateTruncatedCopy(
      double start_offset, double end_t) const;

  SpacetimeObjectTrajectory CreateTrajectoryMutatedInstance(
      std::unique_ptr<const prediction::PredictedTrajectory> traj) const {
    return SpacetimeObjectTrajectory(planner_object_, std::move(traj),
                                     traj_index_, required_lateral_gap_);
  }

  SpacetimeObjectTrajectory CreateLateralGapMutatedInstance(
      double lateral_gap) const {
    return SpacetimeObjectTrajectory(planner_object_, traj_index_, lateral_gap);
  }

  absl::Span<const SpacetimeObjectState> states() const { return states_; }

  absl::string_view traj_id() const { return traj_id_; }

  int traj_index() const { return traj_index_; }

  double required_lateral_gap() const { return required_lateral_gap_; }

  bool is_stationary() const { return is_stationary_; }

  absl::string_view object_id() const { return planner_object_->id(); }

  ObjectType object_type() const { return planner_object_->type(); }

  const prediction::ObjectLongTermBehavior long_term_behavior() const {
    return planner_object_->long_term_behavior();
  }

  const SecondOrderTrajectoryPoint& pose() const {
    return *states_[0].traj_point;
  }

  const Polygon2d& contour() const { return states_[0].contour; }

  const Box2d& bounding_box() const { return states_[0].box; }
  const Box2d& perception_box() const { return states_[0].perception_box; }

  // TODO(renjie): Remove it.
  const PlannerObject* planner_object() const { return planner_object_; }

  const prediction::PredictedTrajectory* trajectory() const {
    return trajectory_.get();
  }

  // A utility function to generate trajectory's id.
  static std::string MakeTrajectoryId(absl::string_view obj_id,
                                      int traj_index) {
    return absl::StrFormat("%s-idx%d", obj_id, traj_index);
  }

  // A utility function to extract object's id from trajectory id.
  static std::string GetObjectIdFromTrajectoryId(absl::string_view traj_id) {
    const std::vector<std::string> tokens = absl::StrSplit(traj_id, "-");
    QCHECK(!tokens.empty());
    return tokens.front();
  }

 private:
  int traj_index_;
  // The id of the spacetime trajectory.
  std::string traj_id_;
  bool is_stationary_;
  const PlannerObject* planner_object_;
  double required_lateral_gap_;
  std::vector<SpacetimeObjectState> states_;
  std::unique_ptr<const prediction::PredictedTrajectory> trajectory_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_TRAJECTORY_H_
