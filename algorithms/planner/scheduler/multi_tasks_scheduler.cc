#include "onboard/planner/scheduler/multi_tasks_scheduler.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/target_lane_clearance.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

absl::StatusOr<LanePathInfo> FindNeighbor(
    const RouteSectionsInfo &route_sections_info,
    const std::vector<LanePathInfo> &lp_infos, bool lc_left,
    mapping::ElementId start_id) {
  const int cur_idx =
      FindOrDie(route_sections_info.front().id_idx_map, start_id);
  const int neighbor_idx = cur_idx + (lc_left ? 1 : -1);
  const auto &lane_ids = route_sections_info.front().lane_ids;
  if (neighbor_idx < 0 || neighbor_idx >= lane_ids.size()) {
    return absl::NotFoundError("Already leftmost/rightmost.");
  }
  const auto neighbor_id = lane_ids.at(neighbor_idx);

  for (const auto &lp_info : lp_infos) {
    if (lp_info.start_lane_id() == neighbor_id) return lp_info;
  }
  return absl::NotFoundError("Neighbor lane not viable.");
}

mapping::ElementId FindWaitingZoneAhead(const TrafficLightInfoMap &tl_info_map,
                                        const mapping::LanePath &lane_path) {
  for (const auto &lane_id : lane_path.lane_ids()) {
    if (tl_info_map.contains(lane_id) &&
        tl_info_map.at(lane_id).tl_control_type() ==
            TrafficLightControlType::LEFT_WAITING_AREA) {
      return lane_id;
    }
  }
  return mapping::kInvalidElementId;
}

bool IsContinuousLaneChange(const LanePathInfo &target_lp_info,
                            const LanePathInfo &lp_info_before_lc,
                            const Vec2d &ego_pos) {
  const double target_lat_offset =
      target_lp_info
          .ProjectionSLInRange(ego_pos, /*start_s=*/0.0,
                               /*end_s=*/kMaxTravelDistanceBetweenFrames)
          .l;
  const double prev_lat_offset =
      lp_info_before_lc
          .ProjectionSLInRange(ego_pos, /*start_s=*/0.0,
                               /*end_s=*/kMaxTravelDistanceBetweenFrames)
          .l;
  return target_lat_offset * prev_lat_offset > 0.0 &&
         std::abs(prev_lat_offset) > kMaxLaneKeepLateralOffset;
}

}  // namespace

bool ShouldSmoothRefLane(const TrafficLightInfoMap &tl_info_map,
                         const DrivePassage &dp, bool prev_smooth_state) {
  const auto &ego_station = dp.FindNearestStationAtS(0.0);
  if (ego_station.is_in_intersection()) {
    // Keep the previous choice once entered intersection.
    return prev_smooth_state;
  }

  const auto waiting_zone_lane_id =
      FindWaitingZoneAhead(tl_info_map, dp.lane_path());
  if (waiting_zone_lane_id == mapping::kInvalidElementId) {
    // No waiting zone, apply smooth regardless of ego pose or traffic light.
    return true;
  }
  // Has waiting zone, decide from traffic light state.
  return tl_info_map.at(waiting_zone_lane_id)
             .tls()
             .at(TrafficLightDirection::LEFT)
             .tl_state == TrafficLightState::TL_STATE_GREEN;
}

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &route_sections_info,
    const std::vector<LanePathInfo> &lp_infos, DrivePassage drive_passage,
    const LanePathInfo &lp_info, const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerParamsProto &planner_params,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &prev_lane_path_from_current,
    const SmoothedReferenceLineResultMap &smooth_result_map, bool borrow,
    bool should_smooth, ThreadPool *thread_pool) {
  const auto &target_lane_path = lp_info.lane_path();

  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);

  ASSIGN_OR_RETURN(
      auto ego_frenet_box,
      drive_passage.QueryFrenetBoxAt(GetAvBox(
          ego_pos, plan_start_point.path_point().theta(), vehicle_geom)),
      _ << "Ego box is out of drive passage!");

  auto lc_state = MakeLaneChangeState(drive_passage, ego_frenet_box);

  if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
    ASSIGN_OR_RETURN(
        const auto path_boundary,
        BuildPathBoundaryFromPose(psmm, drive_passage, plan_start_point,
                                  vehicle_geom, st_traj_mgr, lc_state,
                                  smooth_result_map, borrow, should_smooth),
        _ << "Fail to build path boundary.");
    return SchedulerOutput{
        .drive_passage = std::move(drive_passage),
        .sl_boundary = std::move(path_boundary),
        .lane_change_state = std::move(lc_state),
        .lane_path_before_lc = target_lane_path,
        .length_along_route = lp_info.length_along_route(),
        .should_smooth = should_smooth,
        .borrow_lane = borrow,
        .av_frenet_box_on_drive_passage = std::move(ego_frenet_box),
        .clearance_output = std::nullopt};
  }

  const auto neighbor_lp_info_or =
      FindNeighbor(route_sections_info, lp_infos, lc_state.lc_left(),
                   lp_info.start_lane_id());
  const auto &neighbor_lp_info =
      neighbor_lp_info_or.ok() ? *neighbor_lp_info_or : lp_info;

  std::optional<ClearanceCheckOutput> clearance_output = std::nullopt;
  std::vector<std::string> reasons;

  if (!neighbor_lp_info_or.ok()) {
    // No lane path before lc can be found, have to force merge.
    QLOG(WARNING) << "Applying force merge, no safety check in scheduler!";
    reasons.emplace_back("Force merge");
  } else if (lc_state.entered_target_lane()) {
    // No safety check if already entered target lane.
    reasons.emplace_back("Entered target lane");
  } else {
    if (neighbor_lp_info.length_along_route() > 1.5 * kMinLcLaneLength &&
        IsContinuousLaneChange(lp_info, *neighbor_lp_info_or, ego_pos)) {
      // Forbids continuous lane change if there's still enough space.
      QLOG(INFO) << absl::StrFormat(
          "Scheduler branch deleted: Continuous lane change from %d to %d",
          neighbor_lp_info.start_lane_id(), lp_info.start_lane_id());
      return absl::NotFoundError("Continuous lane change is not allowed.");
    }
    const bool target_lane_changed =
        prev_lane_path_from_current.front().lane_id() !=
        lp_info.start_lane_id();
    // Calculate aggressiveness according to lateral offset to the target lane.
    const double aggressiveness =
        target_lane_changed
            ? ComputeLCPreparingAggressiveness(
                  neighbor_lp_info.length_along_route() + kMinLcLaneLength)
            : ComputeLCExecutingAggressiveness(
                  neighbor_lp_info.length_along_route() + kMinLcLaneLength,
                  ego_frenet_box, lc_state.lc_left());

    const auto clearance_output = CheckTargetLaneClearance(
        psmm, target_lane_path, plan_start_point, st_traj_mgr, stalled_objects,
        vehicle_geom, planner_params, aggressiveness, thread_pool);
    if (!clearance_output.ok()) {
      lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      reasons.emplace_back(absl::StrCat("Cannot continue lane change: ",
                                        clearance_output.status().message()));
      if (target_lane_changed) {
        QLOG(INFO) << absl::StrFormat(
            "Scheduler branch deleted: Target lane: %d. %s",
            lp_info.start_lane_id(), clearance_output.status().message());
        return absl::NotFoundError("Scheduler branch deleted.");
      }
    }
  }

  ASSIGN_OR_RETURN(
      const auto path_boundary,
      BuildPathBoundaryFromPose(psmm, drive_passage, plan_start_point,
                                vehicle_geom, st_traj_mgr, lc_state,
                                smooth_result_map, borrow, should_smooth),
      _ << "Fail to build path boundary.");

  return SchedulerOutput{
      .drive_passage = std::move(drive_passage),
      .sl_boundary = std::move(path_boundary),
      .lane_change_state = std::move(lc_state),
      .lane_path_before_lc = neighbor_lp_info.lane_path().BeforeArclength(
          neighbor_lp_info.length_along_route()),
      .length_along_route = lp_info.length_along_route(),
      .should_smooth = should_smooth,
      .borrow_lane = borrow,
      .av_frenet_box_on_drive_passage = std::move(ego_frenet_box),
      .reasons = reasons,
      .clearance_output = std::move(clearance_output)};
}

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput &input,
    const std::vector<LanePathInfo> &target_lp_infos, ThreadPool *thread_pool) {
  SCOPED_QTRACE("ScheduleMultiplePlanTasks");

  std::vector<SchedulerOutput> multi_tasks;
  if (target_lp_infos.size() == 1) {
    const auto &lp_info = target_lp_infos.front();
    ASSIGN_OR_RETURN(
        auto drive_passage,
        BuildDrivePassage(*input.psmm, lp_info.lane_path(),
                          *input.station_anchor,
                          input.sections_info_from_current->planning_horizon(),
                          kDrivePassageKeepBehindLength,
                          FLAGS_planner_consider_all_lanes_virtual),
        _ << "Failed to build drive passage on single target lane path.");

    const bool should_smooth = ShouldSmoothRefLane(
        *input.tl_info_map, drive_passage, input.prev_smooth_state);

    const std::vector<bool> borrow_branches =
        FLAGS_planner_est_scheduler_allow_borrow
            ? std::vector<bool>{false, true}
            : std::vector<bool>{false};
    for (bool borrow : borrow_branches) {
      auto output_or = MakeSchedulerOutput(
          *input.psmm, *input.sections_info_from_current,
          *input.lane_path_infos, drive_passage, lp_info, *input.vehicle_geom,
          *input.planner_params, *input.st_traj_mgr, *input.stalled_objects,
          *input.plan_start_point, *input.prev_lane_path_from_current,
          *input.smooth_result_map, borrow, should_smooth, input.thread_pool);
      if (output_or.ok()) {
        multi_tasks.emplace_back(std::move(output_or).value());
      }
    }
  } else {
    std::vector<absl::StatusOr<SchedulerOutput>> outputs(
        target_lp_infos.size());
    ParallelFor(0, target_lp_infos.size(), thread_pool, [&](int i) {
      const auto &lp_info = target_lp_infos[i];
      auto drive_passage_or = BuildDrivePassage(
          *input.psmm, lp_info.lane_path(), *input.station_anchor,
          input.sections_info_from_current->planning_horizon(),
          kDrivePassageKeepBehindLength,
          FLAGS_planner_consider_all_lanes_virtual);
      if (!drive_passage_or.ok()) {
        return;
      }

      const bool should_smooth = ShouldSmoothRefLane(
          *input.tl_info_map, *drive_passage_or, input.prev_smooth_state);

      outputs[i] = MakeSchedulerOutput(
          *input.psmm, *input.sections_info_from_current,
          *input.lane_path_infos, std::move(drive_passage_or).value(), lp_info,
          *input.vehicle_geom, *input.planner_params, *input.st_traj_mgr,
          *input.stalled_objects, *input.plan_start_point,
          *input.prev_lane_path_from_current, *input.smooth_result_map,
          /*borrow=*/false, should_smooth, input.thread_pool);
    });

    for (auto &output : outputs) {
      if (output.ok()) {
        multi_tasks.emplace_back(std::move(output).value());
      }
    }
  }

  if (multi_tasks.empty()) {
    QLOG(ERROR) << "Unable to schedule multiple tasks.";
    return absl::NotFoundError(
        "Fail to build drive passage on each lane path.");
  }

  return multi_tasks;
}

}  // namespace qcraft::planner
