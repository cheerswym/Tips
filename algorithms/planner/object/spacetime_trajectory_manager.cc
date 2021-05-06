#include "onboard/planner/object/spacetime_trajectory_manager.h"

#include <algorithm>
#include <utility>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/planner/object/spacetime_planner_trajectory_finder.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/prediction_util.h"
namespace qcraft {
namespace planner {
namespace {
constexpr double kEmergencyAvoidanceObjHorizon = 1.5;
template <typename T>
using NestedVector = std::vector<std::vector<T>>;

// This is the object buffer that AV should never enter.
double ComputeRequiredLateralGap(const PlannerObject& object) {
  switch (object.type()) {
    case OT_FOD:
      return 0.0;
    case OT_UNKNOWN_STATIC:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.15;
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
      return 0.2;
  }
}
}  // namespace

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    absl::Span<const TrajectoryFilter* const> filters,
    absl::Span<const std::unique_ptr<SpacetimePlannerTrajectoryFinder>> finders,
    absl::Span<const PlannerObject> planner_objects,
    double st_planner_trajectories_start_offset, ThreadPool* thread_pool)
    : st_planner_trajectories_start_offset_(
          st_planner_trajectories_start_offset) {
  SCOPED_QTRACE("SpacetimeTrajectoryManager");

  const int num_objects = planner_objects.size();
  NestedVector<SpacetimeObjectTrajectory> all_trajs_per_object(num_objects);
  NestedVector<IgnoredTrajectory> ignored_trajs_per_object(num_objects);

  ParallelFor(0, num_objects, thread_pool, [&](int i) {
    const auto& planner_object = planner_objects[i];
    const auto& trajectories = planner_object.prediction().trajectories();
    QCHECK(!trajectories.empty())
        << planner_object.id() << " has no trajectory.";
    for (int traj_index = 0, s = trajectories.size(); traj_index < s;
         ++traj_index) {
      const auto& pred_traj = trajectories[traj_index];
      bool filtered = false;
      for (const auto* filter : filters) {
        const auto filter_reason = filter->Filter(planner_object, pred_traj);
        if (filter_reason != FilterReason::NONE) {
          ignored_trajs_per_object[i].push_back(
              {.traj = &pred_traj,
               .object_id = planner_object.id(),
               .reason = filter_reason});
          filtered = true;
          break;
        }
      }
      if (filtered) continue;

      const double required_lateral_gap =
          ComputeRequiredLateralGap(planner_object);
      all_trajs_per_object[i].emplace_back(&planner_object, traj_index,
                                           required_lateral_gap);
    }
  });

  int trajectories_size = 0;
  int stationary_count = 0;
  for (const auto& planner_object : planner_objects) {
    trajectories_size += planner_object.prediction().trajectories().size();
    if (planner_object.is_stationary()) ++stationary_count;
  }
  all_trajs_.reserve(trajectories_size);
  ignored_trajs_.reserve(trajectories_size);

  // Collect results from parallel for.
  for (auto& trajs : all_trajs_per_object) {
    std::move(trajs.begin(), trajs.end(), std::back_inserter(all_trajs_));
  }
  for (auto& trajs : ignored_trajs_per_object) {
    std::move(trajs.begin(), trajs.end(), std::back_inserter(ignored_trajs_));
  }

  // Pick trajectories for spacetime planner.
  spacetime_planner_trajs_.reserve(all_trajs_.size());
  SCOPED_QTRACE("SpacetimeTrajectoryManager_finders");
  for (const auto& traj : all_trajs_) {
    for (const auto& finder : finders) {
      const auto selected_reason = finder->Find(traj);
      if (selected_reason != SpacetimePlannerTrajectoryReason::NONE) {
        double st_planner_traj_horizon = kSpacetimePlannerTrajectoryHorizon;
        if (selected_reason ==
            SpacetimePlannerTrajectoryReason::EMERGENCY_AVOIDANCE) {
          st_planner_traj_horizon = kEmergencyAvoidanceObjHorizon;
        }
        ASSIGN_OR_CONTINUE(
            auto truncated_traj,
            traj.CreateTruncatedCopy(st_planner_trajectories_start_offset_,
                                     st_planner_traj_horizon));

        spacetime_planner_trajs_.push_back(std::move(truncated_traj));
        spacetime_planner_traj_ids_.insert(std::string(traj.traj_id()));
        spacetime_planner_trajs_info_.push_back(
            {.traj_index = traj.traj_index(),
             .object_id = traj.planner_object()->id(),
             .reason = selected_reason});
        break;
      }
    }
  }
  // Classify trajectories.
  considered_stationary_trajs_.reserve(stationary_count);
  considered_moving_trajs_.reserve(
      std::max(0, trajectories_size - stationary_count));
  considered_trajs_.reserve(trajectories_size);
  objects_id_map_.reserve(all_trajs_.size());
  trajectories_id_map_.reserve(all_trajs_.size());
  for (const auto& traj : all_trajs_) {
    considered_trajs_.push_back(&traj);
    if (traj.is_stationary()) {
      considered_stationary_trajs_.push_back(&traj);
    } else {
      considered_moving_trajs_.push_back(&traj);
    }
    objects_id_map_[traj.planner_object()->id()].push_back(&traj);
    trajectories_id_map_[traj.traj_id()] = &traj;
  }
}

}  // namespace planner
}  // namespace qcraft
