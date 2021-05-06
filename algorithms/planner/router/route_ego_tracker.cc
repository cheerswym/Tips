#include "onboard/planner/router/route_ego_tracker.h"

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "onboard/global/clock.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft {
namespace planner {
namespace {

// point to lane distance,same as scheduler.
constexpr double kProjectToLaneRadiusError = kDefaultLaneWidth * 1.5;
// The min reroute distance if reroute when map-match failed
constexpr double kRerouteDistanceThresholdMeters = 200.0;
}  // namespace

absl::StatusOr<PointToCompositeLanePath> FindLaneFromLastCompositeIndex(
    const CompositeLanePath &composite_lane_path,
    const CompositeLanePath::CompositeIndex &last_composite_index,
    absl::Span<const mapping::ElementId> section_lane_ids,
    const mapping::ElementId to_find_lane_id) {
  for (int i = last_composite_index.lane_path_index;
       i < composite_lane_path.num_lane_paths(); ++i) {
    const auto &lane_path = composite_lane_path.lane_path(i);
    int j = (i == last_composite_index.lane_path_index)
                ? last_composite_index.lane_index
                : 0;
    for (; j < lane_path.size(); j++) {
      const auto lane_id = lane_path.lane_id(j);
      for (const auto neighbour_lane_id : section_lane_ids) {
        if (neighbour_lane_id == lane_id) {
          PointToCompositeLanePath point_to_composite_lane_path;
          point_to_composite_lane_path.point_on_route_lane =
              (lane_id == to_find_lane_id);
          point_to_composite_lane_path.composite_index = {i, j};
          // The vehicle is one the lane path.
          point_to_composite_lane_path.route_lane_id = lane_id;
          return point_to_composite_lane_path;
        }
      }
    }
  }
  return absl::NotFoundError(
      absl::StrCat("Cannot find the lane id ", to_find_lane_id));
}

absl::StatusOr<RouteTrackerOutput> GetTrackerInfoOnRoute(
    const RouteManagerState &rm_state, const SemanticMapManager &smm,
    const PoseProto &pose) {
  // point 2 pline closest lane
  const auto level_id = smm.GetLevel();
  const std::vector<mapping::PointToLane> match_lanes =
      mapping::PointToNearLanesAtLevel(
          level_id, smm, Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
          kProjectToLaneRadiusError);

  absl::flat_hash_set<mapping::ElementId> uniq_sections;
  std::vector<mapping::PointToLane> filtered_match_lanes;
  for (const auto &point_to_lane : match_lanes) {
    const auto insert_it =
        uniq_sections.insert(point_to_lane.lane_info->section_id);
    if (insert_it.second) {
      filtered_match_lanes.push_back(point_to_lane);
    }
  }

  for (const auto &point_to_lane : filtered_match_lanes) {
    const mapping::LaneInfo &lane_info = *point_to_lane.lane_info;
    const mapping::LanePoint lane_point = {lane_info.id,
                                           point_to_lane.fraction};
    const auto &section_lane_ids =
        smm.FindSectionInfoOrDie(lane_info.section_id).lane_ids;
    VLOG(3) << "pose:" << pose.ShortDebugString()
            << ", match point:" << lane_point.DebugString()
            << ", lane_id:" << lane_info.id
            << ", distance:" << point_to_lane.dist
            << ",composite index:" << rm_state.composite_index.DebugString();

    const auto point_to_composite_lane_path_or = FindLaneFromLastCompositeIndex(
        rm_state.route->lane_path(), rm_state.composite_index, section_lane_ids,
        lane_point.lane_id());
    if (point_to_composite_lane_path_or.ok()) {
      const auto &composite_index =
          point_to_composite_lane_path_or->composite_index;
      const auto cur_time_micros = absl::ToUnixMicros(Clock::Now());

      const double travel_distance = TravelDistanceBetween(
          smm, rm_state.route->lane_path(), rm_state.composite_index,
          rm_state.last_lane_point.fraction(), composite_index,
          lane_point.fraction());
      VLOG(2) << "travel distance:" << travel_distance
              << ", max_distance:" << kRerouteDistanceThresholdMeters;
      if (rm_state.last_update_time != -1 &&
          !IsTravelMatchedValid(
              rm_state.route->lane_path(), rm_state.composite_index,
              rm_state.last_lane_point.fraction(), composite_index,
              lane_point.fraction(), rm_state.last_update_time, cur_time_micros,
              travel_distance, pose.vel_smooth().x(),
              kRerouteDistanceThresholdMeters)) {
        continue;
      }
      // Map match failed if section matched but the lane restrict is obeyed
      if (!point_to_composite_lane_path_or->point_on_route_lane &&
          !IsTwoLaneConnectedWithBrokenWhites(
              smm, lane_point.lane_id(),
              point_to_composite_lane_path_or->route_lane_id)) {
        VLOG(2) << "Although the vehicle is on the route, but may (not "
                   "sure) obey the traffic rule.";
        break;
      }
      return RouteTrackerOutput{.travel_distance = travel_distance,
                                .composite_index = composite_index,
                                .lane_point = lane_point};
    } else {
      VLOG(3) << "Car to path failed. route lane path:\n"
              << rm_state.route->lane_path().ShortDebugString() << "\n current:"
              << rm_state.route_from_current->lane_path().ShortDebugString()
              << "\n index:" << rm_state.composite_index.DebugString()
              << "\n section ids:" << absl::StrJoin(section_lane_ids, ",")
              << "\n lane_point:" << lane_point.DebugString()
              << "\n pose:" << pose.ShortDebugString()
              << point_to_composite_lane_path_or.status().message();
    }
  }
  return absl::UnavailableError(absl::StrFormat(
      "Failed to track AV. Car pose is: %s \n. Last lane id is: %d",
      pose.pos_smooth().ShortDebugString(),
      rm_state.last_lane_point.lane_id()));
}
}  // namespace planner
}  // namespace qcraft
