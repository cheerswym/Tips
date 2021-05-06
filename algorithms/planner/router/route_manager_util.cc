#include "onboard/planner/router/route_manager_util.h"

#include <memory>

#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_lane_path_finder.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/status_macros.h"
namespace qcraft {
namespace planner {

namespace {
bool IsFallInIntervalsOrTrueIfEmpty(
    const absl::Time time,
    const google::protobuf::RepeatedPtrField<mapping::Interval> &intervals,
    const absl::TimeZone tz = absl::LocalTimeZone()) {
  return intervals.empty() ||
         mapping::SemanticMapManager::IsFallInIntervals(time, intervals, tz);
}
}  // namespace

absl::StatusOr<int> FindNextDestinationIndexViaLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const MultipleStopsRequest &multi_stops_request,
    const std::function<bool(const RouteSectionSequence &)>
        &point_on_route_func) {
  if (multi_stops_request.destination_size() < 2) return 0;
  if (!multi_stops_request.skip_past_stops()) return 0;

  int origin_idx = multi_stops_request.infinite_loop()
                       ? multi_stops_request.destination_size() - 1
                       : 0;
  int next_idx = multi_stops_request.infinite_loop() ? 0 : 1;

  const auto find_lane_point_fn = [&multi_stops_request](int index) {
    const auto index_pair = multi_stops_request.GetIndexPair(index);
    return multi_stops_request
        .lane_point_destinations()[index_pair.first][index_pair.second];
  };

  while (next_idx < multi_stops_request.destination_size()) {
    const auto origin = find_lane_point_fn(origin_idx);
    const auto destination = find_lane_point_fn(next_idx);
    if (origin.Valid() && destination.Valid()) {
      std::vector<mapping::LanePoint> destinations = {destination};
      const auto section_sequence =
          SearchForRouteSectionSequenceAlongLanePoints(
              semantic_map_manager, origin, absl::MakeSpan(destinations),
              /*use_time=*/true);
      if (section_sequence.ok()) {
        VLOG(4) << "Sections to destination No." << next_idx << std::endl
                << section_sequence.value().DebugString();
        if (point_on_route_func(section_sequence.value())) {
          return next_idx;
        }
      }
    }
    ++next_idx;
    ASSIGN_OR_RETURN(const auto orign_next_index,
                     multi_stops_request.GetNextDestinationIndex(origin_idx));
    origin_idx = orign_next_index;
  }

  return 0;
}

absl::StatusOr<MultipleStopsRequest> RecoverRoutingRequest(
    const RecordedRouteProto &recorded_route,
    const SemanticMapManager &semantic_map_manager) {
  if (FLAGS_route_recover_destinations_from_log) {
    if (recorded_route.routing_request().destinations().empty()) {
      if (!recorded_route.routing_request().has_multi_stops()) {
        return absl::NotFoundError(
            "When recover destination from log, request's "
            "destinations are empty and no multi stops.");
      }
      VLOG(2) << "Recover routing request with recorded multiple stops: "
              << recorded_route.routing_request().multi_stops().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager,
                                       recorded_route.routing_request());
    } else {
      VLOG(2) << "Recover routing request with recorded destinations: "
              << recorded_route.routing_request().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager,
                                       recorded_route.routing_request());
    }
  } else {
    if (!recorded_route.routing_request().has_multi_stops()) {
      if (recorded_route.routing_request().destinations().empty()) {
        return absl::NotFoundError(
            "When not recover destination from log, request has no "
            "multi stops and destinations are empty.");
      }

      QLOG(INFO) << "Recover routing request with recorded destinations: "
                 << recorded_route.routing_request().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager,
                                       recorded_route.routing_request());
    } else {
      QLOG(INFO)
          << "Recover routing request with recorded multiple stops: "
          << recorded_route.routing_request().multi_stops().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager,
                                       recorded_route.routing_request());
    }
  }
}

absl::StatusOr<Route> RebuildRouteFromRouteLanePath(
    const SemanticMapManager *semantic_map_manager, int update_id,
    const RoutingRequestProto &routing_request, const CompositeLanePath &rlp,
    bool is_bus, const absl::flat_hash_set<mapping::ElementId> &avoid_lanes) {
  RouteSectionSequence section_seq(rlp, semantic_map_manager);

  absl::flat_hash_set<mapping::ElementId> blacklist = avoid_lanes;

  const absl::Time now = Clock::Now();
  const absl::Time now_plus_5min = now + kLookFutureRestrictDuration;
  for (const auto &section : section_seq.sections()) {
    for (const auto &lane : section.lanes) {
      const auto &lane_info =
          semantic_map_manager->FindLaneInfoOrDie(lane.first);
      if (lane_info.IsPassengerVehicleAvoidLaneType()) {
        blacklist.insert(lane.first);
        continue;
      }
      if (!is_bus) {
        const auto &restrict_range = lane_info.proto->bus_only_day_intervals();
        if (lane_info.Type() == mapping::LaneProto::BUS_ONLY &&
            lane_info.proto != nullptr &&
            (IsFallInIntervalsOrTrueIfEmpty(now, restrict_range) ||
             IsFallInIntervalsOrTrueIfEmpty(now_plus_5min, restrict_range))) {
          blacklist.insert(lane.first);
        }
      }
    }
  }

  RouteLanePathFinderInput input = {.section_sequence = &section_seq,
                                    .lane_id_blacklist = std::move(blacklist),
                                    .start_point = rlp.front(),
                                    .destination_point = rlp.back()};

  auto lane_path_rebuilt =
      FindRouteLanePathOnSectionSequence(input, *semantic_map_manager);

  if (!lane_path_rebuilt.ok()) return lane_path_rebuilt.status();

  return Route(
      semantic_map_manager,
      std::make_unique<CompositeLanePath>(std::move(lane_path_rebuilt.value())),
      std::move(section_seq), routing_request, update_id,
      std::move(input.lane_id_blacklist));
}

std::vector<mapping::ElementId> MayUpdateAvoidLanesByRestict(
    const SemanticMapManager &semantic_map_manager,
    const RouteSectionSequence &section_seq, const absl::Time &begin,
    const absl::Time &end, absl::flat_hash_set<mapping::ElementId> *blacklist) {
  std::vector<mapping::ElementId> added_lanes;
  for (const auto &section : section_seq.sections()) {
    for (const auto &lane : section.lanes) {
      const auto &lane_info =
          semantic_map_manager.FindLaneInfoOrDie(lane.first);
      const auto &restrict_range = lane_info.proto->bus_only_day_intervals();
      if (lane_info.Type() == mapping::LaneProto::BUS_ONLY &&
          lane_info.proto != nullptr) {
        if (IsFallInIntervalsOrTrueIfEmpty(begin, restrict_range) ||
            IsFallInIntervalsOrTrueIfEmpty(end, restrict_range)) {
          auto added_pair = blacklist->insert(lane.first);
          if (added_pair.second) {
            added_lanes.push_back(lane.first);
          }
        } else {
          // Now can on bus-lane
          auto remove_count = blacklist->erase(lane.first);
          if (remove_count > 0) {
            added_lanes.push_back(lane.first);
          }
        }
      }
    }
  }
  return added_lanes;
}

}  // namespace planner
}  // namespace qcraft
