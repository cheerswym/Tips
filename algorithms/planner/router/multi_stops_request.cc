#include "onboard/planner/router/multi_stops_request.h"

#include "absl/types/span.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft {
namespace planner {
namespace {

MultipleStopsRequestProto FromDestinations(
    const ::google::protobuf::RepeatedPtrField<RoutingDestinationProto>
        &destinations) {
  MultipleStopsRequestProto ms;
  ms.set_infinite_loop(false);

  const int n = destinations.size();
  if (n == 0) return ms;

  auto *stop = ms.add_stops();
  // Add the first n - 1 destination as via points.
  for (int i = 0; i + 1 < n; ++i) {
    *stop->add_via_points() = destinations.Get(i);
  }
  // Add the last destination as stop point.
  *stop->mutable_stop_point() = destinations.Get(n - 1);
  stop->set_stop_name(destinations.Get(n - 1).DebugString());
  return ms;
}

}  // namespace

class MultipleStopsRequestBuilder {
 public:
  MultipleStopsRequestBuilder() noexcept = default;

  MultipleStopsRequestBuilder &set_semantic_map_manager(
      const SemanticMapManager *semantic_map_manager) {
    semantic_map_manager_ = semantic_map_manager;
    return *this;
  }

  const SemanticMapManager &semantic_map_manager() const {
    return *semantic_map_manager_;
  }

  MultipleStopsRequestBuilder &set_routing_request_proto(
      const RoutingRequestProto *routing_request_proto) {
    routing_request_proto_ = routing_request_proto;
    return *this;
  }

  const RoutingRequestProto &routing_request_proto() const {
    return *routing_request_proto_;
  }

  absl::StatusOr<MultipleStopsRequest> Build();

 private:
  absl::StatusOr<MultipleStopsRequest> BuildFrom(
      const MultipleStopsRequestProto &multiple_stops_proto,
      const absl::flat_hash_set<mapping::ElementId> &avoid_lanes,
      std::string_view request_id);

  const SemanticMapManager *semantic_map_manager_;
  const RoutingRequestProto *routing_request_proto_;
};

absl::StatusOr<MultipleStopsRequest> MultipleStopsRequestBuilder::BuildFrom(
    const MultipleStopsRequestProto &multiple_stops_proto,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes,
    std::string_view request_id) {
  auto infinite_loop = multiple_stops_proto.has_infinite_loop() &&
                       multiple_stops_proto.infinite_loop();
  if (infinite_loop && multiple_stops_proto.stops_size() <= 1) {
    return absl::InvalidArgumentError("Loop requires at least 2 stops");
  }
  auto skip_past_stops = multiple_stops_proto.has_skip_past_stops()
                             ? multiple_stops_proto.skip_past_stops()
                             : true;
  auto multi_stops_request_proto = multiple_stops_proto;
  std::vector<std::vector<RoutingDestinationProto>> destinations;
  std::vector<std::vector<mapping::LanePoint>> lane_point_destinations;
  destinations.reserve(multiple_stops_proto.stops_size());
  lane_point_destinations.reserve(multiple_stops_proto.stops_size());
  int num_destinations = 0;
  std::vector<std::string> stop_name_list;

  for (const auto &stop : multiple_stops_proto.stops()) {
    destinations.emplace_back();
    lane_point_destinations.emplace_back();
    for (const auto &via : stop.via_points()) {
      destinations.back().emplace_back(via);
      auto via_lane_point_or =
          ParseDestinationProtoToLanePoint(semantic_map_manager(), via);
      if (via_lane_point_or.ok()) {
        lane_point_destinations.back().emplace_back(via_lane_point_or.value());
      } else {
        QLOG(ERROR)
            << "Can not parse destination proto to LanePoint. The error is: "
            << via_lane_point_or.status()
            << "\nThe via point proto is: " << via.ShortDebugString();
        return via_lane_point_or.status();
      }
    }

    RoutingDestinationProto this_stop;
    if (stop.has_stop_point()) {
      this_stop = stop.stop_point();
    } else {
      this_stop.set_named_spot(stop.has_stop_name() ? stop.stop_name() : "");
    }

    destinations.back().push_back(this_stop);
    stop_name_list.emplace_back(stop.has_stop_name() ? stop.stop_name() : "");
    num_destinations += stop.via_points_size() + 1;
    auto stop_lane_point_or =
        ParseDestinationProtoToLanePoint(semantic_map_manager(), this_stop);
    if (stop_lane_point_or.ok()) {
      lane_point_destinations.back().emplace_back(stop_lane_point_or.value());
    } else {
      QLOG(ERROR) << "Can not parse destination proto to LanePoint from "
                  << this_stop.DebugString();
      return stop_lane_point_or.status();
    }
  }
  return MultipleStopsRequest(
      destinations, lane_point_destinations, infinite_loop, skip_past_stops,
      num_destinations, stop_name_list, avoid_lanes, multi_stops_request_proto,
      request_id, true, routing_request_proto());
}

absl::StatusOr<MultipleStopsRequest> MultipleStopsRequestBuilder::Build() {
  absl::flat_hash_set<mapping::ElementId> avoid_lanes;
  const auto &proto = routing_request_proto();
  if (!proto.avoid_lanes().empty()) {
    for (const auto &avoid_lane : proto.avoid_lanes()) {
      avoid_lanes.insert(avoid_lane);
    }
  }
  absl::flat_hash_set<mapping::ElementId> avoid_lanes_from_region =
      FindAvoidLanesFromAvoidRegions(proto, semantic_map_manager());
  avoid_lanes.insert(avoid_lanes_from_region.begin(),
                     avoid_lanes_from_region.end());

  absl::StatusOr<MultipleStopsRequest> multiple_stops_request_or;
  if (!proto.has_multi_stops() && !proto.destinations().empty()) {
    auto multi_stops = FromDestinations(proto.destinations());
    return BuildFrom(multi_stops, avoid_lanes,
                     proto.has_id() ? proto.id() : "");
  } else {
    return BuildFrom(proto.multi_stops(), avoid_lanes,
                     proto.has_id() ? proto.id() : "");
  }
}

absl::StatusOr<MultipleStopsRequest> BuildMultipleStopsRequest(
    const SemanticMapManager &semantic_map_manager,
    const RoutingRequestProto &routing_request_proto) {
  MultipleStopsRequestBuilder builder;
  return builder.set_semantic_map_manager(&semantic_map_manager)
      .set_routing_request_proto(&routing_request_proto)
      .Build();
}

MultipleStopsRequest::MultipleStopsRequest(
    std::vector<std::vector<RoutingDestinationProto>> destinations,
    std::vector<std::vector<mapping::LanePoint>> lane_point_destinations,
    bool infinite_loop, bool skip_past_stops, int num_destinations,
    std::vector<std::string> stop_name_list,
    absl::flat_hash_set<mapping::ElementId> avoid_lanes,
    MultipleStopsRequestProto multi_stops_request_proto,
    std::string_view request_id, bool is_from_routing_request,
    RoutingRequestProto route_request_proto)
    : destinations_(std::move(destinations)),
      lane_point_destinations_(std::move(lane_point_destinations)),
      infinite_loop_(infinite_loop),
      skip_past_stops_(skip_past_stops),
      num_destinations_(num_destinations),
      stop_name_list_(std::move(stop_name_list)),
      avoid_lanes_(std::move(avoid_lanes)),
      multi_stops_request_(std::move(multi_stops_request_proto)),
      request_id_(request_id),
      is_from_routing_request_(is_from_routing_request),
      route_request_proto_(std::move(route_request_proto)) {}

void MultipleStopsRequest::ToRoutingRequestFromMultipleStops(
    RoutingRequestProto *proto) const {
  proto->Clear();
  if (is_from_routing_request()) {
    *proto = route_request_proto_;
    return;
  } else {
    *proto->mutable_multi_stops() = multi_stops_request_;
    for (const auto &avoid_lane : avoid_lanes_) {
      proto->mutable_avoid_lanes()->Add(avoid_lane);
    }
    *proto->mutable_id() = request_id_;
  }
}

int MultipleStopsRequest::GetStopIndex(int dest_index) const {
  const auto index_pair = GetIndexPair(dest_index);
  return dest_index + destinations_[index_pair.first].size() -
         index_pair.second - 1;
}

int MultipleStopsRequest::GetStopSerialNumber(int index) const {
  const auto index_pair = GetIndexPair(index);
  return index_pair.first;
}

absl::StatusOr<RoutingRequestProto>
MultipleStopsRequest::GenerateRoutingRequestProtoToNextStop(
    const SemanticMapManager &semantic_map_manager, int next_dest_index) const {
  const auto idx_pair = GetIndexPair(next_dest_index);
  RoutingRequestProto request;
  if (is_from_routing_request()) {
    request = route_request_proto_;
    request.clear_destinations();
  }
  for (int i = idx_pair.second; i < destinations_[idx_pair.first].size(); ++i) {
    *request.add_destinations() = destination(idx_pair.first, i);
    if (i + 1 < destinations_[idx_pair.first].size()) continue;
    auto destination_proto = request.mutable_destinations()->rbegin();
    if (destination_proto->has_named_spot()) {
      const auto &map = semantic_map_manager.semantic_map();
      const mapping::NamedSpotProto *named_spot = nullptr;
      for (const auto &spot : map.named_spots()) {
        if (spot.name() == destination_proto->named_spot()) {
          named_spot = &spot;
          break;
        }
      }
      if (named_spot == nullptr) {
        const auto err_msg = absl::StrCat("Could not find the named spot ",
                                          destination_proto->named_spot());
        return absl::InvalidArgumentError(err_msg);
      }
    }
  }
  *request.mutable_multi_stops() = multi_stops_request_;
  return request;
}

absl::StatusOr<mapping::LanePoint>
MultipleStopsRequest::ComputeDestinationLanePointByTotalIndex(
    const SemanticMapManager &semantic_map_manager, int index) const {
  const auto index_pair = GetIndexPair(index);
  return ComputeDestinationLanePoint(semantic_map_manager, index_pair.first,
                                     index_pair.second);
}

absl::StatusOr<mapping::LanePoint>
MultipleStopsRequest::ComputeDestinationLanePoint(
    const SemanticMapManager &semantic_map_manager, int stop_index,
    int dest_index) const {
  return ParseDestinationProtoToLanePoint(semantic_map_manager,
                                          destination(stop_index, dest_index));
}

bool MultipleStopsRequest::CheckIndex(int stop_index, int dest_index) const {
  if (stop_index < 0 || stop_index >= destinations_.size()) return false;
  if (dest_index < 0 || dest_index >= destinations_.at(stop_index).size())
    return false;
  return true;
}

bool MultipleStopsRequest::HasNextDestinationIndex(int now_index,
                                                   int *next_index) const {
  if (now_index < 0 || now_index >= num_destinations_) {
    LOG(ERROR) << "now_index is invalid:" << now_index;
    return false;
  }

  int next = now_index + 1;
  if (infinite_loop()) next %= num_destinations_;

  if (next >= num_destinations_) return false;
  if (next_index != nullptr) *next_index = next;
  return true;
}

absl::StatusOr<int> MultipleStopsRequest::GetNextDestinationIndex(
    int now_index) const {
  int next;
  if (HasNextDestinationIndex(now_index, &next)) {
    return next;
  }
  return absl::NotFoundError(
      absl::StrCat("Cannot find next destination,now_index:", now_index));
}

std::pair<int, int> MultipleStopsRequest::GetIndexPair(int index) const {
  for (int stop_idx = 0; stop_idx < destinations_.size(); ++stop_idx) {
    if (index < destinations_[stop_idx].size()) {
      return std::make_pair(stop_idx, index);
    }
    index -= destinations_[stop_idx].size();
  }
  QLOG(ERROR) << "Invalid index input";
  return std::make_pair(0, 0);
}

void MultipleStopsRequest::Clear() {
  destinations_.clear();
  num_destinations_ = 0;
  stop_name_list_.clear();
  lane_point_destinations_.clear();
}
}  // namespace planner
}  // namespace qcraft
