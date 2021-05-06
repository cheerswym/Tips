#include "onboard/planner/scheduler/target_lane_path_filter.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <valarray>

#include "absl/container/flat_hash_set.h"
#include "onboard/global/trace.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"

namespace qcraft::planner {

namespace {

constexpr double kLastTargetLanePathReward = -1e6;

std::pair<int, double> ComputeLanePathCost(
    const LanePathInfo &lp_info, const RouteSectionsInfo &route_sections_info,
    const ApolloTrajectoryPointProto &plan_start_point) {
  const double l_offset =
      std::abs(lp_info
                   .ProjectionSLInRange(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                       /*start_s=*/0.0, /*end_s=*/10.0)
                   .l);
  const auto lane_idx_diff = RoundToInt(l_offset / kDefaultLaneWidth);

  double cost = lp_info.path_cost();
  const auto &front_sec = route_sections_info.front();
  // Minor difference based on global route.
  cost += (route_sections_info.planning_horizon() + kLocalMapExtension) /
          front_sec.driving_distance[FindOrDie(front_sec.id_idx_map,
                                               lp_info.start_lane_id())];

  return {lane_idx_diff, cost};
}

int FindMostSimilarLanePathIndexToLastTargetLanePath(
    const std::vector<LanePathInfo> &lp_infos,
    const mapping::LanePath &last_target_lane_path, double ego_v) {
  constexpr double kMinSharedTimeOnLanePath = 3.0;  // s.
  const double shared_len_thres = kMinSharedTimeOnLanePath * ego_v;

  int best_index = -1;
  double max_len_along_route = 0.0;
  for (int i = 0; i < lp_infos.size(); ++i) {
    if (lp_infos[i].start_lane_id() !=
        last_target_lane_path.front().lane_id()) {
      // Assume already aligned.
      continue;
    }

    const auto lane_path = lp_infos[i].lane_path().BeforeArclength(
        lp_infos[i].length_along_route());
    const int lp_size =
        std::min(lane_path.size(), last_target_lane_path.size());
    int last_shared_lane_idx = 0;
    while (last_shared_lane_idx + 1 < lp_size &&
           lane_path.lane_id(last_shared_lane_idx + 1) ==
               last_target_lane_path.lane_id(last_shared_lane_idx + 1)) {
      ++last_shared_lane_idx;
    }
    const double shared_len = lane_path.LaneIndexPointToArclength(
        last_shared_lane_idx,
        std::min(lane_path.lane_segment(last_shared_lane_idx).end_fraction,
                 last_target_lane_path.lane_segment(last_shared_lane_idx)
                     .end_fraction));

    if (shared_len >= shared_len_thres &&
        lp_infos[i].length_along_route() > max_len_along_route) {
      max_len_along_route = lp_infos[i].length_along_route();
      best_index = i;
    }
  }

  return best_index;
}

std::pair<mapping::LanePoint, mapping::LanePoint> FindNeighborLanePoints(
    const RouteSectionsInfo &route_sections_info,
    const mapping::LanePoint &lane_pt) {
  const auto &cur_sec =
      *route_sections_info.FindSegmentContainingLanePointOrNull(lane_pt);
  const int cur_idx = FindOrDie(cur_sec.id_idx_map, lane_pt.lane_id());

  auto left_lane_pt = cur_idx > 0
                          ? mapping::LanePoint(cur_sec.lane_ids[cur_idx - 1],
                                               lane_pt.fraction())
                          : mapping::LanePoint();
  auto right_lane_pt = cur_idx + 1 < cur_sec.lane_ids.size()
                           ? mapping::LanePoint(cur_sec.lane_ids[cur_idx + 1],
                                                lane_pt.fraction())
                           : mapping::LanePoint();
  return {std::move(left_lane_pt), std::move(right_lane_pt)};
}

}  // namespace

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const RouteSectionsInfo &route_sections_info,
    const mapping::LanePath &last_target_lane_path,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &preferred_lane_path,
    std::vector<LanePathInfo> *mutable_lp_infos) {
  SCOPED_QTRACE("FilterMultipleTargetLanePath");

  const auto lp_infos = std::move(*mutable_lp_infos);
  const int n_lps = lp_infos.size();
  // Vector indices correspond to start lanes' indices in the current section.
  std::vector<std::pair<int, double>> lane_path_costs(n_lps);
  for (int i = 0; i < n_lps; ++i) {
    lane_path_costs[i] =
        ComputeLanePathCost(lp_infos[i], route_sections_info, plan_start_point);
  }
  // If the last target lane path is still viable, choose it as one candidate.
  const int last_target_index =
      FindMostSimilarLanePathIndexToLastTargetLanePath(
          lp_infos, last_target_lane_path, plan_start_point.v());
  if (last_target_index != -1) {
    lane_path_costs[last_target_index].second += kLastTargetLanePathReward;

    const auto &front_sec_idx_map = route_sections_info.front().id_idx_map;
    const int last_start_id_idx = FindOrDie(
        front_sec_idx_map, lp_infos[last_target_index].start_lane_id());

    constexpr double kLcPreviewTime = 6.0;     // s.
    constexpr double kMinLcPreviewLen = 20.0;  // m.
    const auto &last_lane_path = lp_infos[last_target_index].lane_path();
    const double preview_length = std::min(
        last_lane_path.length(),
        std::max(kMinLcPreviewLen, kLcPreviewTime * plan_start_point.v()));
    const auto [left_lane_pt, right_lane_pt] = FindNeighborLanePoints(
        route_sections_info,
        last_lane_path.ArclengthToLanePoint(preview_length));
    for (int i = 0; i < n_lps; ++i) {
      if (i == last_target_index) continue;

      if (std::abs(FindOrDie(front_sec_idx_map, lp_infos[i].start_lane_id()) -
                   last_start_id_idx) > 1 ||
          (i < last_target_index &&
           !lp_infos[i].lane_path().ContainsLanePoint(left_lane_pt)) ||
          (i > last_target_index &&
           !lp_infos[i].lane_path().ContainsLanePoint(right_lane_pt))) {
        // For lane paths that split later, do not consider them now.
        lane_path_costs[i].first = INT_MAX;
      }
    }
  }

  std::vector<int> idx_vec(n_lps);
  std::iota(idx_vec.begin(), idx_vec.end(), 0);
  std::stable_sort(
      idx_vec.begin(), idx_vec.end(), [&lane_path_costs](int i1, int i2) {
        return lane_path_costs[i1].first < lane_path_costs[i2].first ||
               (lane_path_costs[i1].first == lane_path_costs[i2].first &&
                lane_path_costs[i1].second < lane_path_costs[i2].second);
      });

  // Keep only the best one for each start lane id.
  absl::flat_hash_set<mapping::ElementId> start_lane_set;
  for (int i = 0; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    if (!start_lane_set.contains(lp_infos[lane_idx].start_lane_id())) {
      start_lane_set.insert(lp_infos[lane_idx].start_lane_id());
      mutable_lp_infos->emplace_back(lp_infos[lane_idx]);
    }
  }

  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  for (int i = 0; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    if (preferred_lanes.contains(lp_infos[lane_idx].start_lane_id())) {
      // Only return the closest preferred lane on existence.
      return {lp_infos[lane_idx]};
    }
  }

  const int res_size = std::min(n_lps, FLAGS_planner_est_parallel_branch_num);
  std::vector<LanePathInfo> results;
  results.reserve(res_size);
  for (int i = 0, prev_lane_diff = -1; i < n_lps; ++i) {
    const int lane_idx = idx_vec[i];
    // Discourage one single lane change over multiple lanes.
    if (lane_path_costs[lane_idx].first > 1) break;
    // Same index diff, choose only the better one.
    if (lane_path_costs[lane_idx].first == prev_lane_diff) continue;

    results.push_back(lp_infos[lane_idx]);
    prev_lane_diff = lane_path_costs[lane_idx].first;
    if (results.size() == res_size) break;
  }

  return results;
}

LanePathInfo ChooseLeastLateralOffsetLanePath(
    const std::vector<LanePathInfo> &lp_infos,
    const std::vector<ApolloTrajectoryPointProto> &trajectory) {
  const int traj_size = trajectory.size();
  const int lp_infos_size = lp_infos.size();
  std::vector<std::valarray<double>> lp_offsets(
      lp_infos_size, std::valarray<double>(traj_size));

  for (int i = 0; i < lp_infos_size; ++i) {
    for (int j = 0; j < traj_size; ++j) {
      lp_offsets[i][j] = std::abs(
          lp_infos[i]
              .ProjectionSL(Vec2dFromApolloTrajectoryPointProto(trajectory[j]))
              .l);
    }
  }
  std::vector<double> lp_offsets_sums;
  lp_offsets_sums.reserve(lp_infos_size);
  std::for_each(lp_offsets.begin(), lp_offsets.end(),
                [&lp_offsets_sums](const auto &offsets) {
                  lp_offsets_sums.push_back(offsets.sum());
                });
  const auto min_element =
      std::min_element(lp_offsets_sums.begin(), lp_offsets_sums.end());
  return lp_infos[std::distance(lp_offsets_sums.begin(), min_element)];
}

}  // namespace qcraft::planner
