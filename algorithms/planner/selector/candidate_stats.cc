#include "onboard/planner/selector/candidate_stats.h"

#include <algorithm>
#include <limits>
#include <string>

#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

namespace {

constexpr double kMinObsTimeForLowSpeed = 5.0;  // s.
constexpr double kMaxFollowDistance = 8.0;      // m.
constexpr double kPreviewOverHorizonRatio = 2.5;

int CountContinuousObjects(const std::vector<FrenetBox> &objs_on_target,
                           double block_s) {
  const auto target_it = std::upper_bound(
      objs_on_target.begin(), objs_on_target.end(), block_s,
      [](double val, const auto &box) { return val < box.s_max; });
  auto prev_it = target_it, succ_it = target_it;
  while (prev_it != objs_on_target.begin() &&
         prev_it->s_min - std::prev(prev_it)->s_max < kMaxFollowDistance) {
    prev_it = std::prev(prev_it);
  }
  while (std::next(succ_it) != objs_on_target.end() &&
         std::next(succ_it)->s_min - succ_it->s_max < kMaxFollowDistance) {
    succ_it = std::next(succ_it);
  }

  return succ_it - prev_it + 1;
}

}  // namespace

ProgressStats::ProgressStats(
    const SemanticMapManager &smm,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &results)
    : ego_v(plan_start_point.v()) {
  constexpr double kEgoPassingLatBuffer = 0.4;  // m.
  constexpr double kEgoFrontBuffer = 5.0;       // m.
  const double allow_width = vehicle_geom.width() + kEgoPassingLatBuffer;
  const double front_dist =
      vehicle_geom.front_edge_to_center() + kEgoFrontBuffer;

  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    const auto &passage = scheduler_outputs[idx].drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();

    std::optional<std::string> block_obj = std::nullopt;
    double block_obj_s = 0.0;
    double lowest_v = smm.QueryLaneSpeedLimitById(start_lane_id);
    absl::flat_hash_set<std::string> checked_set;
    std::vector<FrenetBox> objs_on_target;
    for (const auto *traj_ptr :
         results[idx].filtered_traj_mgr->trajectories()) {
      if (checked_set.contains(traj_ptr->object_id())) continue;
      checked_set.emplace(traj_ptr->object_id());

      const auto &obj_pose = *traj_ptr->states().front().traj_point;
      ASSIGN_OR_CONTINUE(const auto dp_tan,
                         passage.QueryTangentAt(obj_pose.pos()));
      if (std::abs(NormalizeAngle(dp_tan.FastAngle() - obj_pose.theta())) >
          M_PI_4) {
        // Just passing through the current lane path.
        continue;
      }

      ASSIGN_OR_CONTINUE(const auto aabbox,
                         passage.QueryFrenetBoxAtContour(traj_ptr->contour()));
      if (aabbox.s_min < 0.0) continue;

      const double l_max = std::clamp(aabbox.l_max, -kDefaultHalfLaneWidth,
                                      kDefaultHalfLaneWidth);
      const double l_min = std::clamp(aabbox.l_min, -kDefaultHalfLaneWidth,
                                      kDefaultHalfLaneWidth);
      if (std::max(kDefaultHalfLaneWidth - l_max,
                   l_min - (-kDefaultHalfLaneWidth)) > allow_width) {
        // Out of target lane path or too small to block the ego vehicle.
        continue;
      }
      objs_on_target.push_back(aabbox);

      const auto &obj_ltb = traj_ptr->long_term_behavior();
      if (obj_ltb.obs_duration < kMinObsTimeForLowSpeed) continue;
      const double obj_ref_v = std::max(obj_ltb.avg_speed, obj_pose.v());

      constexpr double kFollowAccelDecel = 1.0;  // m/s^2.
      const double obj_v_lim =
          std::sqrt(kFollowAccelDecel * (aabbox.s_min - front_dist) +
                    0.5 * (Sqr(ego_v) + Sqr(obj_ref_v)));
      if (obj_v_lim < lowest_v) {
        lowest_v = obj_v_lim;
        block_obj = traj_ptr->object_id();
        block_obj_s = aabbox.center().s;
      }
    }

    if (!lane_speed_map.contains(start_lane_id) ||
        FindOrDie(lane_speed_map, start_lane_id).lane_speed > lowest_v) {
      int count_continuous = 0;
      if (block_obj.has_value()) {
        std::sort(objs_on_target.begin(), objs_on_target.end(),
                  [](const auto &lhs, const auto &rhs) {
                    return lhs.s_min < rhs.s_min;
                  });
        count_continuous = CountContinuousObjects(objs_on_target, block_obj_s);
      }

      lane_speed_map[start_lane_id] =
          LaneSpeedInfo{.lane_speed = lowest_v,
                        .block_obj = block_obj,
                        .continuous_object_num = count_continuous};
      max_lane_speed = std::max(max_lane_speed, lowest_v);
    }
  }
}

RouteLookAheadStats::RouteLookAheadStats(
    const SemanticMapManager &smm, const RouteSectionsInfo &sections_info,
    const std::vector<LanePathInfo> &lp_infos,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &results) {
  route_len = sections_info.length();
  local_horizon = sections_info.planning_horizon() + kLocalMapExtension;
  const auto &front_sec = sections_info.front();

  constexpr double kEpsilon = 1.0;  // m.
  const double max_driving_dist_all = *std::max_element(
      front_sec.driving_distance.begin(), front_sec.driving_distance.end());
  const double preview_horizon = kPreviewOverHorizonRatio * local_horizon;

  absl::flat_hash_set<mapping::ElementId> target_lane_ids;
  for (int i = 0; i < front_sec.lane_ids.size(); ++i) {
    driving_dist_map[front_sec.lane_ids[i]] = front_sec.driving_distance[i];
    if (front_sec.driving_distance[i] >= max_driving_dist_all - kEpsilon ||
        front_sec.driving_distance[i] >= preview_horizon) {
      target_lane_ids.insert(front_sec.lane_ids[i]);
    }
  }
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    const auto start_lane_id =
        scheduler_outputs[idx].drive_passage.lane_path().front().lane_id();
    int lc_num_to_targets;
    if (target_lane_ids.contains(start_lane_id)) {
      lc_num_to_targets = 0;
    } else {
      const int start_lane_idx = FindOrDie(front_sec.id_idx_map, start_lane_id);
      int min_index_diff = std::numeric_limits<int>::max();
      for (const auto target_lane_id : target_lane_ids) {
        const int index_diff = std::abs(
            FindOrDie(front_sec.id_idx_map, target_lane_id) - start_lane_idx);
        min_index_diff = std::min(min_index_diff, index_diff);
      }
      lc_num_to_targets = min_index_diff;
    }
    lc_num_to_targets_map[start_lane_id] = lc_num_to_targets;
    if (lc_num_to_targets < min_lc_num) {
      min_lc_num = lc_num_to_targets;
    }
  }

  absl::flat_hash_map<mapping::ElementId, double> raw_len_along_route_map;
  for (const auto &lp_info : lp_infos) {
    raw_len_along_route_map[lp_info.start_lane_id()] =
        lp_info.length_along_route();
  }
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    const auto start_lane_id =
        scheduler_outputs[idx].drive_passage.lane_path().front().lane_id();
    double length_along_route =
        FindWithDefault(raw_len_along_route_map, start_lane_id, 0.0);

    // Cut off length_along_route if trajectory is blocked by stalled object.
    const auto &end_info = results[idx].trajectory_end_info;
    if (end_info.has_value()) {
      const auto obj_id =
          SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
              end_info->st_traj_id);
      if (stalled_objects.contains(obj_id) &&
          route_len > end_info->end_s + kMinLcLaneLength) {
        length_along_route =
            std::min(length_along_route,
                     std::max(0.0, end_info->end_s - kMinLcLaneLength));
      }
    }
    // Cut off length along route at the end of merging lanes.
    for (const auto &lane_seg :
         scheduler_outputs[idx].drive_passage.lane_path()) {
      if (lane_seg.end_s >= length_along_route) break;

      const auto &lane_info = smm.FindLaneInfoOrDie(lane_seg.lane_id);
      if (lane_info.proto->is_merging()) {
        length_along_route = lane_seg.end_s;
        break;
      }
    }

    len_along_route_map[start_lane_id] = length_along_route;
    if (length_along_route > max_length_along_route) {
      max_length_along_route = length_along_route;
    }
  }
}

}  // namespace qcraft::planner
