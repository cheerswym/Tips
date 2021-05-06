#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

class TrajectoryFilter;
class SpacetimePlannerTrajectoryFinder;
// The manager of all the spacetime object trajectories. Each prediction or
// stationary object is modeled as a `SpacetimeObjectTrajectory`.
class SpacetimeTrajectoryManager {
 public:
  SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter* const> filters,
      absl::Span<const std::unique_ptr<SpacetimePlannerTrajectoryFinder>>
          finders,
      absl::Span<const PlannerObject> planner_objects,
      double st_planner_trajectories_start_offset, ThreadPool* thread_pool);
  explicit SpacetimeTrajectoryManager(
      absl::Span<const PlannerObject> planner_objects,
      ThreadPool* thread_pool = nullptr)
      : SpacetimeTrajectoryManager(
            /*filter=*/{}, /*finder=*/{}, planner_objects,
            /*st_planner_trajectories_start_offset=*/0.0, thread_pool) {}

  // All the considered stationary object 'trajectories'.
  absl::Span<const SpacetimeObjectTrajectory* const> stationary_object_trajs()
      const {
    return considered_stationary_trajs_;
  }

  // All the considered moving trajectories.
  absl::Span<const SpacetimeObjectTrajectory* const> moving_object_trajs()
      const {
    return considered_moving_trajs_;
  }

  // Spacetime planner trajectory and the selected reason.
  struct SpaceTimePlannerTrajectoryInfo {
    int traj_index;
    std::string object_id;
    SpacetimePlannerTrajectoryReason::Type reason;
  };
  absl::Span<const SpaceTimePlannerTrajectoryInfo>
  spacetime_planner_trajs_info() const {
    return spacetime_planner_trajs_info_;
  }
  // Trajectories that will be considered by space time planner (initializer &
  // optimizer).
  absl::Span<const SpacetimeObjectTrajectory> spacetime_planner_trajs() const {
    return spacetime_planner_trajs_;
  }

  absl::Status AddSpaceTimePlannerTrajectory(
      const SpacetimeObjectTrajectory& traj,
      SpacetimePlannerTrajectoryReason::Type reason) {
    if (!spacetime_planner_traj_ids_.contains(traj.traj_id())) {
      ASSIGN_OR_RETURN(
          auto truncated_traj,
          traj.CreateTruncatedCopy(st_planner_trajectories_start_offset_,
                                   kSpacetimePlannerTrajectoryHorizon));
      spacetime_planner_trajs_.push_back(std::move(truncated_traj));
      spacetime_planner_traj_ids_.insert(std::string(traj.traj_id()));
      spacetime_planner_trajs_info_.push_back(
          {.traj_index = traj.traj_index(),
           .object_id = traj.planner_object()->id(),
           .reason = reason});
    }
    return absl::OkStatus();
  }

  absl::Status AddSpaceTimePlannerTrajectoryById(
      absl::string_view traj_id,
      SpacetimePlannerTrajectoryReason::Type reason) {
    const auto* traj = FindTrajectoryById(traj_id);
    if (traj != nullptr) {
      RETURN_IF_ERROR(AddSpaceTimePlannerTrajectory(*traj, reason));
    } else {
      return absl::NotFoundError(
          absl::StrCat("Could not find trajectory ", traj_id));
    }
    return absl::OkStatus();
  }

  // All the considered trajectories.
  absl::Span<const SpacetimeObjectTrajectory* const> trajectories() const {
    return considered_trajs_;
  }

  // Returns all the considered trajectories of an object id.
  absl::Span<const SpacetimeObjectTrajectory* const> FindTrajectoriesByObjectId(
      std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return {};
    return iter->second;
  }

  // Returns the planner object of an object id.
  const PlannerObject* FindObjectByObjectId(std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return nullptr;
    return iter->second.front()->planner_object();
  }

  // Returns the considered trajectory of traj id.
  const SpacetimeObjectTrajectory* FindTrajectoryById(
      std::string_view traj_id) const {
    return FindPtrOrNull(trajectories_id_map_, traj_id);
  }

  // An Ignored trajectory and the ignore reason.
  struct IgnoredTrajectory {
    const prediction::PredictedTrajectory* traj;
    std::string object_id;
    FilterReason::Type reason;
  };
  // All the ignored trajectories.
  absl::Span<const IgnoredTrajectory> ignored_trajectories() const {
    return ignored_trajs_;
  }

 private:
  absl::flat_hash_map<std::string_view,
                      std::vector<const SpacetimeObjectTrajectory*>>
      objects_id_map_;
  absl::flat_hash_map<std::string_view, const SpacetimeObjectTrajectory*>
      trajectories_id_map_;

  std::vector<SpacetimeObjectTrajectory> all_trajs_;
  // TODO(lidong): Remove this one later. It is the same as all_trajs_.
  std::vector<const SpacetimeObjectTrajectory*> considered_trajs_;
  std::vector<IgnoredTrajectory> ignored_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
  std::vector<SpacetimeObjectTrajectory> spacetime_planner_trajs_;
  absl::flat_hash_set<std::string> spacetime_planner_traj_ids_;
  std::vector<SpaceTimePlannerTrajectoryInfo> spacetime_planner_trajs_info_;

  double st_planner_trajectories_start_offset_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
