#include "onboard/planner/router/route_manager.h"

#include <algorithm>
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_split.h"
#include "google/protobuf/repeated_field.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/safe_unit.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/off_road_route_searcher.h"
#include "onboard/planner/router/route_ego_tracker.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_reroute.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

DEFINE_bool(router_enable_off_road_search, false,
            "Whether to enable off road searcher.");
namespace qcraft {
namespace planner {
namespace {

//  The min time interval if reroute when map-match failed
constexpr double kResetThresholdMicros = 250 * 1000.0;
// The min travel distance if reset  when map-match failed
constexpr double kDriftDistanceResetThreshold = 20.0;
// The min time interval if reroute when map-match failed
constexpr double kRerouteThresholdMicros = 1500 * 1000.0;
// The max manual driving time interval before reroute even
// map-match success
constexpr double kManualDrivingThresholdMicros = 5000 * 1000.0;

// Computes distance to the end of route.
double ComputeDistanceToRouteEnd(const Route &route_from_current) {
  if (!route_from_current.section_sequence().empty()) {
    // Compute distance along section.
    double cum_len = .0;
    for (const auto &sec : route_from_current.section_sequence().sections()) {
      cum_len += sec.length();
    }
    return cum_len;
  }

  // Compute distance along composite lane path.
  double cum_len = .0;
  for (int i = 0; i < route_from_current.lane_path().num_lane_paths(); i++) {
    cum_len += route_from_current.lane_path().lane_path(i).length();
    if (i > 0)
      cum_len -=
          route_from_current.lane_path().transitions().at(i - 1).overlap_length;
  }
  return cum_len;
}

double ComputeEtaToRouteEnd(const Route &route_from_current,
                            const SemanticMapManager &semantic_map_manager) {
  double cum_time_secs = 0.0;
  for (int lane_path_idx = 0;
       lane_path_idx < route_from_current.lane_path().num_lane_paths();
       lane_path_idx++) {
    const mapping::LanePath &lane_path =
        route_from_current.lane_path().lane_path(lane_path_idx);
    const auto &lane_ids = lane_path.lane_ids();
    if (lane_ids.empty()) {
      continue;
    }

    const auto &first_lane =
        semantic_map_manager.FindLaneInfoOrDie(lane_ids[0]);
    const auto &last_lane =
        semantic_map_manager.FindLaneInfoOrDie(lane_ids.back());
    if (first_lane.id == last_lane.id) {
      cum_time_secs += first_lane.length() *
                       (lane_path.end_fraction() - lane_path.start_fraction()) /
                       first_lane.speed_limit;
      continue;
    }
    cum_time_secs += first_lane.length() * (1.0 - lane_path.start_fraction()) /
                     first_lane.speed_limit;
    for (int i = 1; i < lane_ids.size() - 1; i++) {
      const auto lane_id = lane_ids[i];
      const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(lane_id);
      cum_time_secs += lane_info.length() / lane_info.speed_limit;
    }
    cum_time_secs += last_lane.length() * (lane_path.end_fraction() - 0.0) /
                     last_lane.speed_limit;
  }
  return cum_time_secs;
}

absl::StatusOr<RoutingResultProto> InitRoute(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto,
    const PlannerRoutingRequestProto::InitRouteRequest &request,
    uint64_t request_id) {
  RoutingResultProto result_proto;
  result_proto.set_update_id(request_id);
  // create default destinations
  if (!request.has_routing_request()) {
    result_proto.set_success(false);
    return result_proto;
  }

  const RoutingRequestProto routing_request = request.routing_request();
  result_proto.set_routing_type(RoutingType::INIT_ROUTE);
  if (HasOnlyOneOffroadRequest(routing_request)) {
    result_proto.set_success(true);
    *result_proto.mutable_routing_request() = routing_request;
    result_proto.set_routing_type(RoutingType::PARKING_OFFROAD);
    return result_proto;
  }

  // search for route path
  const auto route_section_lane_pair_or = SearchForRouteLanePathFromPose(
      semantic_map_manager, route_param_proto, request.pose(), routing_request,
      /*blacklist=*/{},
      /*use_time=*/true);
  if (route_section_lane_pair_or.ok()) {
    const auto route_valid_status =
        CheckRouteValidity(route_section_lane_pair_or->second);
    if (route_valid_status.ok()) {
      route_section_lane_pair_or->second.ToProto(
          result_proto.mutable_lane_path());
      *result_proto.mutable_routing_request() = routing_request;
      result_proto.set_success(true);
      return result_proto;
    } else {
      return absl::InternalError("Found a invalid route.");
    }
  } else if (FLAGS_router_enable_off_road_search &&
             absl::IsOutOfRange(route_section_lane_pair_or.status())) {
    // Pose not on road.
    ASSIGN_OR_RETURN(
        const auto multi_stop_request,
        BuildMultipleStopsRequest(semantic_map_manager, routing_request));
    auto off_road_route_or =
        SearchForRouteFromOffRoad(semantic_map_manager, request.pose(),
                                  multi_stop_request, /*search_radius=*/20.0);
    if (off_road_route_or.ok()) {
      // TODO(weijun): Refactor repeated code.
      const auto route_valid_status =
          CheckRouteValidity(off_road_route_or->on_road_route_lane_path);
      if (route_valid_status.ok()) {
        off_road_route_or->on_road_route_lane_path.ToProto(
            result_proto.mutable_lane_path());
        *result_proto.mutable_routing_request() = routing_request;
        off_road_route_or->link_lane_point.ToProto(
            result_proto.mutable_link_lane_point());
        result_proto.set_success(true);
        return result_proto;
      } else {
        return absl::InternalError("Found a invalid route.");
      }
    } else {
      QLOG_EVERY_N_SEC(ERROR, 3)
          << "Init route failed:"
          << route_section_lane_pair_or.status().message()
          << ", Off-road routing also failed:"
          << off_road_route_or.status().message()
          << "routing request:" << routing_request.ShortDebugString();
      return absl::NotFoundError(absl::StrCat(
          "Cannot find any route. pose:", request.pose().ShortDebugString(),
          ", request", request.routing_request().ShortDebugString()));
    }

  } else {
    const Vec3d pos = Vec3dFromProto(request.pose().pos_smooth());
    const Vec3d global =
        semantic_map_manager.coordinate_converter().SmoothToGlobal(pos);

    QEVENT("xiang", "planner_routing_failure", [&](qcraft::QEvent *qevent) {
      qevent->AddField("x", global.x());
      qevent->AddField("y", global.y());
      qevent->AddField("z", global.z());
      qevent->AddField("request", request.routing_request().ShortDebugString());
      qevent->AddField("current_lane_path_len",
                       !route_section_lane_pair_or.ok()
                           ? -1
                           : route_section_lane_pair_or->second.lane_paths()
                                 .front()
                                 .length());
    });
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "Init route failed: cannot find any route path to destinations: "
        << routing_request.ShortDebugString();
    return absl::NotFoundError(absl::StrCat(
        "Cannot search a route.pose:", global.DebugString(), ", request",
        request.routing_request().ShortDebugString()));
  }
  result_proto.set_success(false);
  return result_proto;
}

absl::StatusOr<RoutingResultProto> GenerateRoutingResultProto(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto,
    const PlannerRoutingRequestProto *request) {
  if (request->has_init()) {
    return InitRoute(semantic_map_manager, route_param_proto, request->init(),
                     request->request_id());
  } else {
    return absl::InvalidArgumentError("Invalid routing request type.");
  }
}

absl::StatusOr<RoutingResultProto> GenerateOffRoadRouteResult(
    const RoutingRequestProto &routing_request, int64_t next_request_id) {
  if (HasOnlyOneOffroadRequest(routing_request)) {
    RoutingResultProto result_proto;
    result_proto.set_success(true);
    *result_proto.mutable_routing_request() = routing_request;
    result_proto.set_update_id(next_request_id);
    result_proto.set_routing_type(RoutingType::PARKING_OFFROAD);
    return result_proto;
  }
  return absl::UnavailableError("Only support only one offroad stop yet.");
}

std::optional<TurnSignal> GetTurnSignalOnStart(double dist_from_start) {
  constexpr double kDepartSignalDistance = 10.0;  // meters.
  return (dist_from_start < kDepartSignalDistance)
             ? std::make_optional(TURN_SIGNAL_LEFT)
             : std::nullopt;
}

std::optional<TurnSignal> GetTurnSignalOnEnd(double dist_to_end) {
  constexpr double kStopSignalDistanceAhead = 30.0;  // meters
  return (dist_to_end < kStopSignalDistanceAhead)
             ? std::make_optional(TURN_SIGNAL_RIGHT)
             : std::nullopt;
}

// When the AV current pos to lane failed, return true if out of the distance or
// time threshold, otherwise false.
bool ShouldResetPathInfo(int64 miss_matched_time, double reset_threshold_micros,
                         double drift_distance,
                         double drift_distance_reset_threshold) {
  return miss_matched_time > reset_threshold_micros ||
         drift_distance > drift_distance_reset_threshold;
}

}  // namespace

RouteContentProto RouteManager::route_content_proto() const {
  RouteContentProto content_proto;

  for (const auto &stop_name : data_.multi_stops.stop_name_list()) {
    *content_proto.add_stations_list() = stop_name;
  }

  content_proto.set_next_station(
      data_.next_destination_index >= 0
          ? data_.multi_stops.GetStopSerialNumber(data_.next_destination_index)
          : -1);

  // NOTE: Set a negative distance if destination is not available.
  content_proto.set_distance_to_next_station(
      data_.next_destination_index >= 0 ? data_.distance_to_next_stop : -1.0);

  *content_proto.mutable_routing_request_id() = data_.multi_stops.request_id();

  return content_proto;
}

void RouteManager::InitRequest(
    const std::shared_ptr<const RecordedRouteProto> recorded_route,
    int64_t injected_teleop_micro_secs) {
  // Recover routing request from log
  if (data_.pending_routing_requests.empty() &&
      (FLAGS_route_recover_destinations_from_log ||
       FLAGS_route_recover_multi_stops_from_log)) {
    if (!recorded_route) {
      QLOG(INFO) << "Recovering route request from recorded route: waiting for "
                    "recorded route message teleop secs: "
                 << injected_teleop_micro_secs;
      return;
    }
    if (recorded_route != nullptr) {
      auto status =
          RecoverRoutingRequest(*recorded_route, *semantic_map_manager_);
      if (!status.ok()) {
        QLOG(WARNING) << status.status().message();
        QLOG(INFO) << "Router initialization from recorded route failed.";
      } else {
        data_.pending_routing_requests.push_back(std::move(status).value());
        QLOG(INFO) << "Router initialization done, recorded route recovered.";
      }
    }
  }
  // Create a default routing request if route is empty
  if (data_.pending_routing_requests.empty()) {
    const auto default_request = ConvertAllRouteFlagsToRoutingRequestProto();
    if (default_request.ok()) {
      auto multiple_stops_request_or = BuildMultipleStopsRequest(
          *semantic_map_manager_, default_request.value());
      if (multiple_stops_request_or.ok()) {
        QLOG(INFO) << "Router initialization done, new default request queued.";
        data_.pending_routing_requests.push_back(
            std::move(multiple_stops_request_or).value());
      }
    } else {
      QLOG_EVERY_N_SEC(INFO, 1)
          << "Fail to create default routing request. waiting request...";
    }
  }
}

absl::StatusOr<RouteManagerOutput> RouteManager::Update(
    bool is_bus, const RouteManagerInput &input) {
  SCOPED_QTRACE("RouteManager::Update");
  RouteManagerOutput rm_output;
  bool is_new_route = false;
  bool new_request_arrived = !data_.pending_routing_requests.empty();
  // New route request arrived or initiated by route FLAGS
  if (data_.active_routing_request == nullptr ||
      !data_.pending_routing_requests.empty()) {
    data_.distance_to_next_stop =
        data_.route_from_current
            ? ComputeDistanceToRouteEnd(*data_.route_from_current)
            : std::numeric_limits<double>::infinity();
    data_.eta_secs_to_next_stop =
        data_.route_from_current
            ? ComputeEtaToRouteEnd(*data_.route_from_current,
                                   *semantic_map_manager_)
            : std::numeric_limits<double>::infinity();

    VLOG(3) << "Get route_from_current, dist:" << data_.distance_to_next_stop
            << ", secs:" << data_.eta_secs_to_next_stop
            << ", is_bus:" << is_bus;
    const auto routing_result_or = InitiateRoutingRequest(input);
    if (routing_result_or.ok()) {
      data_.routing_result_proto =
          routing_result_or.value();  // keep routing result
      const bool received =
          ReceivedSuccessfulRoutingResult(routing_result_or.value(), is_bus);
      if (!received) {
        QLOG(ERROR) << "ReceivedSuccessfulRoutingResult failed. ";
        this->last_error_ = route::RouteCreateLanePathFailedError(absl::StrCat(
            "Create route lane path failed.", routing_result_or->update_id()));
        return absl::InternalError("ReceivedSuccessfulRoutingResult");
      } else {
        QLOG(INFO)
            << "ReceivedSuccessfulRoutingResult ok. new route routing result:"
            << routing_result_or->success();
        is_new_route = true;
        data_.is_internal_reroute = false;
      }
    } else {
      this->last_error_ =
          route::RoutePathNotFoundError(routing_result_or.status().ToString());
      VLOG(3) << "Failed initiate routing request: "
              << routing_result_or.status().message();
      if (new_request_arrived) {
        // Return for kickout if we have a new routing request and failed.
        return routing_result_or.status();
      }
    }
  }

  if (data_.has_route()) {
    // Once reached here, we must already have a valid route.
    // next_destination_index must also be valid.
    VLOG(3) << "Get route_from_current, dist:" << data_.distance_to_next_stop
            << ", secs:" << data_.eta_secs_to_next_stop;

    const auto update_tracker_on_route_or =
        GetTrackerInfoOnRoute(data_, *semantic_map_manager_, *input.pose);
    if (update_tracker_on_route_or.ok()) {
      const auto composite_lane_path =
          data_.route->lane_path().AfterCompositeIndexAndLanePoint(
              semantic_map_manager_,
              update_tracker_on_route_or->composite_index,
              update_tracker_on_route_or->lane_point);
      VLOG(3) << "Car to lane path result: composite_index:"
              << update_tracker_on_route_or->composite_index.DebugString()
              << "\n composite_lane_path: "
              << composite_lane_path.ShortDebugString();
      const auto status = UpdateRouteFromCurrent(is_bus, composite_lane_path);
      if (!status.ok()) {
        QLOG(WARNING) << "Failed to update route from current: "
                      << status.message();
      }

      data_.composite_index = update_tracker_on_route_or->composite_index;
      data_.last_lane_point = update_tracker_on_route_or->lane_point;
      data_.route_s += update_tracker_on_route_or->travel_distance;
      data_.last_update_time = absl::ToUnixMicros(Clock::Now());
    } else {
      QLOG_EVERY_N_SEC(WARNING, 2)
          << "Route ego tracker, reason:"
          << update_tracker_on_route_or.status().message();
    }

    int64_t missed_micros =
        absl::ToUnixMicros(Clock::Now()) - data_.last_update_time;
    if (!update_tracker_on_route_or.ok() ||
        missed_micros > kManualDrivingThresholdMicros) {
      if (!update_tracker_on_route_or.ok()) {
        Vec3d pos = Vec3dFromProto(input.pose->pos_smooth());
        Vec3d global =
            semantic_map_manager_->coordinate_converter().SmoothToGlobal(pos);
        QLOG_EVERY_N_SEC(WARNING, 5) << absl::StrCat(
            "Pose to lane failed, off-route now, should need Reroute later, "
            "pose:",
            PoseDebugString(*semantic_map_manager_, *input.pose));
        QEVENT_EVERY_N_SECONDS(
            "xiang", "route_map_match_fail", 10, [&](QEvent *qevent) {
              qevent->AddField("x", global.x());
              qevent->AddField("y", global.y());
              qevent->AddField("z", global.z());
              qevent->AddField("yaw", input.pose->yaw());
              qevent->AddField("vx", input.pose->vel_body().x());
            });
      }
      VLOG(3) << "reach:" << data_.reached_destination_index
              << ", pending:" << data_.pending_next_destination_index
              << ", auto drive state:"
              << IS_AUTO_DRIVE(input.autonomy_state->autonomy_state());
      if (missed_micros > kRerouteThresholdMicros &&
          !IS_AUTO_DRIVE(input.autonomy_state->autonomy_state()) &&
          data_.reached_destination_index !=
              data_.pending_next_destination_index) {
        QLOG_EVERY_N_SEC(WARNING, 1)
            << "reroute, missed_micros:" << missed_micros
            << ", Already reached: " << data_.reached_destination_index
            << ", pending: " << data_.pending_next_destination_index;
        route::RerouteInput reroute_input = {
            .semantic_map_manager = semantic_map_manager_,
            .pose = input.pose.get(),
            .origin_routing_request =
                &(data_.route_from_current->routing_request()),
            .multi_stops = &data_.multi_stops,
            .route_param_proto = route_param_proto_,
            .cur_time_micros = absl::ToUnixMicros(Clock::Now()),
            .route_s = data_.route_s,
            .next_request_id = data_.request_id++,
            .is_bus = is_bus};
        VLOG(3) << "reroute request:" << reroute_input.next_request_id;
        auto reroute_output_or = route::RerouteFromPose(reroute_input);
        if (reroute_output_or.ok()) {
          VLOG(3) << "Update reroute result.";
          data_.route = std::move(reroute_output_or->route);
          data_.route_from_current =
              std::move(reroute_output_or->route_from_current);
          data_.routing_result_proto =
              std::move(reroute_output_or->routing_result_proto);
          ResetRoutePathInfo();
          is_new_route = true;
          data_.is_internal_reroute = true;
          QLOG_EVERY_N_SEC(INFO, 3)
              << "Reroute success." << reroute_input.next_request_id;
        } else {
          last_error_ =
              route::RerouteError(reroute_output_or.status().message());
          VLOG(3) << "reroute error:" << last_error_.ToString();
          return reroute_output_or.status();
        }
      } else if (ShouldResetPathInfo(missed_micros, kResetThresholdMicros,
                                     MicroSecondsToSeconds(missed_micros) *
                                         input.pose->vel_body().x(),
                                     kDriftDistanceResetThreshold)) {
        VLOG(3) << "reset route path. time diff:" << missed_micros
                << ", kResetThresholdMicros:" << kResetThresholdMicros
                << ", kDriftDistanceResetThreshold:"
                << kDriftDistanceResetThreshold;
        ResetRoutePathInfo();
      }
    }
    rm_output.route = data_.route;
    rm_output.route_from_current = data_.route_from_current;
    if (is_new_route && !data_.is_internal_reroute) {
      data_.start_next_route_pos = {input.pose->pos_smooth().x(),
                                    input.pose->pos_smooth().y()};
    }
  }  // namespace planner
  if (data_.routing_result_proto.success()) {
    auto stop_seq_no = data_.multi_stops.GetStopSerialNumber(
        data_.pending_next_destination_index);
    rm_output.destination_stop =
        data_.multi_stops.multi_stops_request_proto().stops(stop_seq_no);
    if (data_.routing_result_proto.has_link_lane_point()) {
      *rm_output.destination_stop.mutable_depart_strategy()
           ->mutable_off_road()
           ->add_specified_onroad_points()
           ->mutable_lane_point() =
          data_.routing_result_proto.link_lane_point();
    }
    VLOG(4) << "Next stop is off road."
            << rm_output.destination_stop.ShortDebugString();
    rm_output.route_content_proto = route_content_proto();
  }
  auto start_signal_opt =
      GetTurnSignalOnStart(data_.start_next_route_pos.DistanceTo(
          {input.pose->pos_smooth().x(), input.pose->pos_smooth().y()}));
  if (start_signal_opt.has_value()) {
    rm_output.signal = *start_signal_opt;
  }
  auto end_signal_opt = GetTurnSignalOnEnd(data_.distance_to_next_stop);
  if (end_signal_opt.has_value()) {
    rm_output.signal = *end_signal_opt;
  }
  data_.is_new_route = is_new_route;
  rm_output.rerouted = is_new_route;
  rm_output.update_id = data_.routing_result_proto.has_update_id()
                            ? data_.routing_result_proto.update_id()
                            : -1;
  return rm_output;
}  // namespace qcraft

absl::Status RouteManager::AddManualReroutingRequest(
    const ReroutingRequestProto &request) {
  auto multiple_stops_request_or = BuildMultipleStopsRequest(
      *semantic_map_manager_, request.routing_request());
  if (multiple_stops_request_or.ok()) {
    data_.pending_routing_requests.push_back(
        std::move(multiple_stops_request_or).value());
    return absl::OkStatus();
  } else {
    return multiple_stops_request_or.status();
  }
}

bool RouteManager::ReceivedSuccessfulRoutingResult(
    const RoutingResultProto &response, bool is_bus) {
  if (!response.success()) {
    return false;
  }
  data_.active_routing_request.reset();
  ResetRoutePathInfo();
  if (!response.success()) {
    QLOG(ERROR) << "Last routing request failed";
    return false;
  }

  const auto multi_stops = data_.pending_multi_stops.empty()
                               ? data_.multi_stops
                               : data_.pending_multi_stops;
  if (!HasOnlyOneOffroadRequest(response.routing_request())) {
    auto route_or = RebuildRouteFromRouteLanePath(
        semantic_map_manager_, response.update_id(), response.routing_request(),
        CompositeLanePath(semantic_map_manager_, response.lane_path()), is_bus,
        multi_stops.avoid_lanes());

    if (!route_or.ok()) {
      QLOG(ERROR) << route_or.status().message();
      return false;
    }
    data_.route = std::make_shared<Route>(std::move(route_or.value()));
    data_.route_from_current = std::make_shared<Route>(
        semantic_map_manager_,
        std::make_unique<CompositeLanePath>(data_.route->lane_path()),
        data_.route->section_sequence(),
        ConvertRoutingRequestToGlobalPoint(*semantic_map_manager_,
                                           data_.route->routing_request()),
        data_.route->update_id(), data_.route->avoid_lanes());

    data_.distance_to_next_stop =
        ComputeDistanceToRouteEnd(*data_.route_from_current);
  }
  QLOG(INFO) << "Successful routing received";
  if (!data_.pending_multi_stops.empty()) {
    data_.multi_stops = std::move(data_.pending_multi_stops);
  }
  data_.next_destination_index = data_.pending_next_destination_index;
  data_.reached_dest_cnt = 0;
  data_.composite_index = {0, 0};
  return true;
}

absl::StatusOr<RoutingResultProto> RouteManager::InitiateRoutingRequest(
    const RouteManagerInput &input) {
  // Take the latest routing request.
  VLOG(2) << "Current pending request size:"
          << data_.pending_routing_requests.size();
  if (!data_.pending_routing_requests.empty()) {
    data_.pending_multi_stops =
        std::move(data_.pending_routing_requests.back());
    data_.pending_routing_requests.clear();

    if (data_.pending_multi_stops.empty()) {
      return absl::NotFoundError("Empty stops in routing request.");
    }
    RoutingRequestProto routing_request;
    data_.pending_multi_stops.ToRoutingRequestFromMultipleStops(
        &routing_request);
    VLOG(2) << "routing request:" << routing_request.DebugString() << ", "
            << data_.pending_multi_stops.destination_size();
    if (HasOnlyOneOffroadRequest(routing_request)) {
      data_.active_routing_request =
          std::make_unique<PlannerRoutingRequestProto>();
      *(data_.active_routing_request->mutable_init()
            ->mutable_routing_request()) = routing_request;
      int64_t request_id = data_.request_id++;
      data_.active_routing_request->set_request_id(request_id);
      data_.pending_next_destination_index = 0;
      data_.reached_destination_index = -1;
      data_.reached_dest_cnt = 0;
      return GenerateOffRoadRouteResult(routing_request, request_id);
    }
    const auto next_destination_index_or =
        FindNextDestinationIndexViaLanePoints(
            *semantic_map_manager_, data_.pending_multi_stops,
            [&input, this](const RouteSectionSequence &section_seq) {
              return section_seq.IsPointOnSections(
                  *semantic_map_manager_,
                  Vec2d(input.pose->pos_smooth().x(),
                        input.pose->pos_smooth().y()),
                  input.pose->yaw(), kDefaultLaneWidth, 10.0);
            });
    if (!next_destination_index_or.ok()) {
      return absl::NotFoundError("Find next destination from pose failed.");
    }
    data_.pending_next_destination_index = next_destination_index_or.value();
    data_.reached_destination_index = -1;
    data_.reached_dest_cnt = 0;

    // sending routing request to first stop
    data_.active_routing_request =
        std::make_unique<PlannerRoutingRequestProto>();
    *(data_.active_routing_request->mutable_init()->mutable_pose()) =
        *input.pose;
    ASSIGN_OR_RETURN(
        auto next_stop_request,
        data_.pending_multi_stops.GenerateRoutingRequestProtoToNextStop(
            *semantic_map_manager_, data_.pending_next_destination_index));
    *(data_.active_routing_request->mutable_init()->mutable_routing_request()) =
        std::move(next_stop_request);
    data_.active_routing_request->set_request_id(data_.request_id++);
    return GenerateRoutingResultProto(*semantic_map_manager_,
                                      *route_param_proto_,
                                      data_.active_routing_request.get());
  }
  if (data_.next_destination_index < 0 || data_.multi_stops.empty()) {
    return absl::NotFoundError("No routing request available yet.");
  }
  const int next_stop_index =
      data_.multi_stops.GetStopIndex(data_.next_destination_index);
  const auto &next_stop = data_.multi_stops.destination(next_stop_index);
  if (next_stop.has_off_road()) {
    VLOG(3) << "Next stop off road, Not Supported."
            << next_stop.ShortDebugString();
    return absl::AlreadyExistsError("Already exist off road stop.");
  }
  if (!has_route()) {
    return absl::NotFoundError("No routing request available yet.");
  }

  // Once reached here, we must already have a valid route.
  // next_destination_index must also be valid.

  const auto next_stop_lane_point_or =
      data_.multi_stops.ComputeDestinationLanePointByTotalIndex(
          *semantic_map_manager_, next_stop_index);
  if (!next_stop_lane_point_or.ok()) {
    return next_stop_lane_point_or.status();
  }

  const auto near_bus_station = [this, &next_stop_lane_point_or](
                                    Vec2d current_pos, double radius) {
    const auto &bus_station_point =
        next_stop_lane_point_or->ComputePos(*semantic_map_manager_);
    return current_pos.DistanceSquareTo(bus_station_point) < radius * radius;
  };

  bool jump_next_stop = FLAGS_route_auto_next_stop &&
                        near_bus_station(Vec2d(input.pose->pos_smooth().x(),
                                               input.pose->pos_smooth().y()),
                                         kReachRouteDestinationRadius);
  // Reroute to next stop if necessary
  data_.goto_next_stop = input.res_button_pressed || jump_next_stop;
  if (input.res_button_pressed) {
    QLOG(INFO) << "RES BUTTON pressed.";
  }

  if (!data_.goto_next_stop) {
    return absl::NotFoundError("No routing request, not go to next stop.");
  }
  if (data_.has_route()) {
    int new_next_dest_idx;
    if (data_.multi_stops.HasNextDestinationIndex(next_stop_index,
                                                  &new_next_dest_idx)) {
      data_.pending_next_destination_index = new_next_dest_idx;
      ASSIGN_OR_RETURN(
          const auto new_request_proto,
          data_.multi_stops.GenerateRoutingRequestProtoToNextStop(
              *semantic_map_manager_, data_.pending_next_destination_index));

      QLOG(INFO) << "new route to next stop "
                 << new_request_proto.ShortDebugString();
      data_.active_routing_request =
          std::make_unique<PlannerRoutingRequestProto>();
      *(data_.active_routing_request->mutable_init()->mutable_pose()) =
          *input.pose;
      *(data_.active_routing_request->mutable_init()
            ->mutable_routing_request()) = new_request_proto;
      data_.active_routing_request->set_request_id(data_.request_id++);
      data_.goto_next_stop = false;
      return GenerateRoutingResultProto(*semantic_map_manager_,
                                        *route_param_proto_,
                                        data_.active_routing_request.get());
    } else {
      return absl::NotFoundError("No routing request, no next stop.");
    }
  } else {
    return absl::NotFoundError("No routing request, not reached bus station.");
  }
}

absl::Status RouteManager::UpdateRouteFromCurrent(
    bool is_bus, const CompositeLanePath &route_path_from_current) {
  auto routing_request = data_.route_from_current->routing_request();
  const auto destinations = ParseDestinationProtoToLanePoints(
      *semantic_map_manager_, routing_request);
  if (!destinations.ok()) {
    return destinations.status();
  }
  const auto &next_destination = destinations->front();

  if (routing_request.destinations_size() > 0) {
    if (double dist = route_path_from_current.front()
                          .ComputePos(*semantic_map_manager_)
                          .DistanceTo(next_destination.ComputePos(
                              *semantic_map_manager_));
        dist < kReachRouteDestinationRadius) {
      data_.reached_destination_index = data_.pending_next_destination_index;
      QLOG_EVERY_N_SEC(INFO, 10)
          << "Reached via point No." << data_.reached_dest_cnt << ", ("
          << next_destination.lane_id() << "," << next_destination.fraction()
          << ")"
          << " distance: " << dist
          << ", reached destination index:" << data_.reached_destination_index;
      if (routing_request.destinations_size() > 1) {
        data_.reached_dest_cnt++;
        routing_request.mutable_destinations()->erase(
            routing_request.destinations().begin());
      }
    }
  }
  if (!is_bus) {
    absl::Time now = Clock::Now();
    absl::Time now_plus_5min = now + kLookFutureRestrictDuration;

    auto changed_lane_ids = MayUpdateAvoidLanesByRestict(
        *semantic_map_manager_, data_.route->section_sequence(), now,
        now_plus_5min, data_.route->mutable_avoid_lanes());
    if (!changed_lane_ids.empty()) {
      QLOG(INFO) << " Changed avoid lanes:"
                 << absl::StrJoin(changed_lane_ids, ",");
    }
  }

  data_.route_from_current = std::make_shared<Route>(
      semantic_map_manager_,
      std::make_unique<CompositeLanePath>(route_path_from_current),
      RouteSectionSequence(route_path_from_current, semantic_map_manager_),
      routing_request, data_.route->update_id(), data_.route->avoid_lanes());

  return absl::OkStatus();
}

void RouteManager::ResetRoutePathInfo() {
  data_.last_update_time = -1;
  data_.composite_index = {0, 0};
  data_.last_lane_point = {mapping::kInvalidElementId, 0.0};
}

bool RouteManager::has_offroad_route() const {
  if (data_.multi_stops.destination_size() > 0 &&
      data_.pending_next_destination_index <
          data_.multi_stops.destination_size()) {
    const int next_stop_index =
        data_.multi_stops.GetStopIndex(data_.pending_next_destination_index);
    const auto &next_stop = data_.multi_stops.destination(next_stop_index);
    return next_stop.has_off_road();
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
