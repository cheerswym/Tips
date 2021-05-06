#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "onboard/planner/object/drive_passage_filter.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/spacetime_planner_trajectory_finder.h"
#include "onboard/planner/planner_flags.h"

DEFINE_bool(planner_st_traj_mgr_use_side_finder, true,
            "Spacetime trajectory manager use side finder");
DEFINE_bool(planner_st_traj_mgr_use_front_finder, false,
            "Spacetime trajectory manager use front finder");
DEFINE_bool(planner_st_traj_mgr_use_emergency_finder, true,
            "Spacetime trajectory manager use emergency finder");

namespace qcraft {
namespace planner {

std::unique_ptr<SpacetimeTrajectoryManager> BuildSpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    ThreadPool* thread_pool) {
  QCHECK_NOTNULL(input.passage);
  QCHECK_NOTNULL(input.sl_boundary);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.veh_geom);
  QCHECK_NOTNULL(input.plan_start_point);
  QCHECK_NOTNULL(input.lane_change_state);
  QCHECK_NOTNULL(input.prev_st_trajs);

  // Build spacetime object manager.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(input.passage,
                                                input.sl_boundary);
  std::vector<const TrajectoryFilter*> filters;
  filters.push_back(&low_likelihood_filter);
  filters.push_back(&drive_passage_filter);

  // Consider all objects in spacetime planner.
  if (FLAGS_planner_st_traj_mgr_use_all) {
    std::vector<std::unique_ptr<SpacetimePlannerTrajectoryFinder> > finders;
    return std::make_unique<SpacetimeTrajectoryManager>(
        filters, finders, input.obj_mgr->planner_objects(),
        input.st_planner_trajectories_start_index * kTrajectoryTimeStep,
        thread_pool);
  }

  // Use customized finders for spacetime objects.
  std::vector<std::unique_ptr<SpacetimePlannerTrajectoryFinder> > finders;
  finders.push_back(
      std::make_unique<StationarySpacetimePlannerTrajectoryFinder>());
  if (FLAGS_planner_st_traj_mgr_use_side_finder) {
    finders.push_back(
        std::make_unique<FrontSideMovingSpacetimePlannerTrajectoryFinder>(
            input.passage, input.sl_boundary, input.plan_start_point,
            input.prev_st_trajs, input.veh_geom->length(),
            input.veh_geom->width()));
  }
  if (FLAGS_planner_st_traj_mgr_use_emergency_finder) {
    const double ra_to_center = input.veh_geom->front_edge_to_center();
    const double av_length = input.veh_geom->length();
    const double av_width = input.veh_geom->width();
    const Vec2d xy(input.plan_start_point->path_point().x(),
                   input.plan_start_point->path_point().y());
    const Vec2d av_tangent =
        Vec2d::UnitFromAngle(input.plan_start_point->path_point().theta());
    const Vec2d av_center = xy + av_tangent * ra_to_center;
    const auto av_box = Box2d(av_center, av_tangent, av_length, av_width);
    finders.push_back(
        std::make_unique<DangerousSideMovingSpacetimePlannerTrajectoryFinder>(
            av_box, input.passage));
  }
  // Activate with caution, all front trajectories will be considered by
  // spacetime planner.
  if (FLAGS_planner_st_traj_mgr_use_front_finder) {
    finders.push_back(
        std::make_unique<FrontMovingSpacetimePlannerTrajectoryFinder>(
            input.passage, input.plan_start_point, input.veh_geom->length()));
  }
  return std::make_unique<SpacetimeTrajectoryManager>(
      filters, finders, input.obj_mgr->planner_objects(),
      input.st_planner_trajectories_start_index * kTrajectoryTimeStep,
      thread_pool);
}

}  // namespace planner
}  // namespace qcraft
