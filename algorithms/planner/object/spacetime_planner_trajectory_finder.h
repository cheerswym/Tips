#ifndef ONBOARD_PLANNER_SPACETIME_PLANNER_TRAJECTORY_FINDER_H_
#define ONBOARD_PLANNER_SPACETIME_PLANNER_TRAJECTORY_FINDER_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/planner.pb.h"
namespace qcraft {
namespace planner {

class SpacetimePlannerTrajectoryFinder {
 public:
  virtual SpacetimePlannerTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerTrajectoryFinder() = default;
};

// Find stationary trajs for st planner.
class StationarySpacetimePlannerTrajectoryFinder
    : public SpacetimePlannerTrajectoryFinder {
 public:
  StationarySpacetimePlannerTrajectoryFinder() = default;
  SpacetimePlannerTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;
};

// Find side trajs (trajs that move along the direction of sdc) for st planner.
class FrontSideMovingSpacetimePlannerTrajectoryFinder
    : public SpacetimePlannerTrajectoryFinder {
 public:
  explicit FrontSideMovingSpacetimePlannerTrajectoryFinder(
      const DrivePassage* drive_passage, const PathSlBoundary* sl_boundary,
      const ApolloTrajectoryPointProto* plan_start_point,
      const SpacetimePlannerTrajectories* prev_st_trajs, double av_length,
      double av_width);
  SpacetimePlannerTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;                   // Not owned.
  const PathSlBoundary* path_sl_boundary_;              // Not owned.
  const ApolloTrajectoryPointProto* plan_start_point_;  // Not owned.
  absl::flat_hash_map<std::string, SpacetimePlannerTrajectoryReason::Type>
      prev_st_planner_obj_decisions_;
  double av_length_;  // AV length.
  double av_width_;   // AV width.
  FrenetCoordinate av_sl_;
};

// Find very close side obj for st planner.
class DangerousSideMovingSpacetimePlannerTrajectoryFinder
    : public SpacetimePlannerTrajectoryFinder {
 public:
  explicit DangerousSideMovingSpacetimePlannerTrajectoryFinder(
      const Box2d& av_box, const DrivePassage* drive_passage);
  SpacetimePlannerTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  Box2d av_box_;
  const DrivePassage* drive_passage_;  // Not owned.
  std::optional<FrenetBox> av_sl_box_;
};

// Find all trajs (in front of sdc) for st planner.
// Use with caution, not fully tested yet.
class FrontMovingSpacetimePlannerTrajectoryFinder
    : public SpacetimePlannerTrajectoryFinder {
 public:
  explicit FrontMovingSpacetimePlannerTrajectoryFinder(
      const DrivePassage* drive_passage,
      const ApolloTrajectoryPointProto* plan_start_point, double av_length);
  SpacetimePlannerTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;                   // Not owned.
  const ApolloTrajectoryPointProto* plan_start_point_;  // Not owned.
  FrenetCoordinate av_sl_;
  double av_length_;  // AV length.
};
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPACETIME_PLANNER_TRAJECTORY_FINDER_H_
