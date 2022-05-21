#include "onboard/lite/portal/server/portal_service_server.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/portal/server/run_event_process.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/v2x/v2x_util.h"

namespace qcraft {
namespace lite {
PortalServiceServer::PortalServiceServer() {
  run_event_process_ = std::make_unique<RunEventProcess>();
}

std::unique_ptr<grpc::Server> PortalServiceServer::StartServer(
    const std::string &addr, LiteModule *lite_module) {
  lite_module_ = lite_module;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
  builder.RegisterService(this);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  return server;
}

void PortalServiceServer::UpdateState(const QState &state) {
  absl::WriterMutexLock lock(&state_mutex_);
  state_ = state;
  state_cond_var_.Signal();
}

void PortalServiceServer::UpdateVehicleInfo(const QVehicleInfo &vehicle_info) {
  absl::WriterMutexLock lock(&vehicle_info_mutex_);
  vehicle_info_ = vehicle_info;
}

void PortalServiceServer::UpdateLogInfo(const QLogInfo &log_info) {
  log_info_ = log_info;
}

void PortalServiceServer::StopServer() {
  stop_notification_.Notify();
  {
    absl::WriterMutexLock lock(&state_mutex_);
    state_cond_var_.Signal();
  }
}

grpc::Status PortalServiceServer::Command(grpc::ServerContext *context,
                                          const CommandRequest *request,
                                          CommandResponse *response) {
  QLOG(INFO) << "command: " << request->DebugString();

  const auto command = request->command();
  const auto timestamp = absl::FormatTime(
      "[%E4Y/%m/%d %H:%M:%E3S] ", absl::FromUnixMicros(request->timestamp()),
      absl::LocalTimeZone());

  const auto run_event_func = [this, request,
                               command](const QRunEvent::Key &key) {
    RunEventRequest run_request;
    run_request.set_timestamp(request->timestamp());
    auto *run_event = run_request.add_run_events();
    run_event->set_key(key);
    if (key == QRunEvent::KEY_QCOMAND_DRIVE_MISSION) {
      run_event->set_value_type(QRunEvent::TYPE_MSG);
      run_event->mutable_routing_request_proto()->CopyFrom(
          command.routing_request_proto());
    } else if (key == QRunEvent::KEY_QCOMAND_SEMANTIC_MAP_MODIFIER) {
      run_event->set_value_type(QRunEvent::TYPE_MSG);
      run_event->mutable_semantic_map_modifier_proto()->CopyFrom(
          command.semantic_map_modifier_proto());
    } else {
      run_event->set_value_type(QRunEvent::TYPE_NONE);
    }
    RunEventResponse response;
    RunEvent(nullptr, &run_request, &response);
  };

  switch (command.type()) {
    case QCommand::ENGAGE: {
      QLOG(WARNING) << timestamp << "execute engage action";
      run_event_func(QRunEvent::KEY_QCOMAND_ENGAGE);
    } break;
    case QCommand::DISENGAGE: {
      QLOG(WARNING) << timestamp << "execute disengage action";
      run_event_func(QRunEvent::KEY_QCOMAND_DISENGAGE);
    } break;
    case QCommand::DOOR_OPEN: {
      QLOG(WARNING) << timestamp << "execute door override";
      run_event_func(QRunEvent::KEY_QCOMAND_DOOR_OPEN);
    } break;
    case QCommand::DOOR_CLOSE: {
      QLOG(WARNING) << timestamp << "execute door override";
      run_event_func(QRunEvent::KEY_QCOMAND_DOOR_CLOSE);
    } break;
    case QCommand::REQUEST_REMOTE_ASSIST: {
      QLOG(WARNING) << timestamp << "requesting remote assist";
      run_event_func(QRunEvent::KEY_QCOMAND_REQUEST_REMOTE_ASSIST);
    } break;
    case QCommand::LANE_CHANGE_LEFT: {
      QLOG(WARNING) << timestamp << "execute lane change";
      run_event_func(QRunEvent::KEY_QCOMAND_LANE_CHANGE_LEFT);
    } break;
    case QCommand::LANE_CHANGE_RIGHT: {
      QLOG(WARNING) << timestamp << "execute lane change";
      run_event_func(QRunEvent::KEY_QCOMAND_LANE_CHANGE_RIGHT);
    } break;
    case QCommand::LANE_CHANGE_FORCE_KEEP: {
      QLOG(WARNING) << timestamp << "force lane keeping";
      run_event_func(QRunEvent::KEY_QCOMAND_LANE_CHANGE_FORCE_KEEP);
    } break;
    case QCommand::LANE_CHANGE_CANCEL: {
      QLOG(WARNING) << timestamp << "interrupt lane change";
      run_event_func(QRunEvent::KEY_QCOMAND_LANE_CHANGE_CANCEL);
    } break;
    case QCommand::NEXT_STATION: {
      QLOG(WARNING) << timestamp << "execute next station";
      run_event_func(QRunEvent::KEY_QCOMAND_NEXT_STATION);
    } break;
    case QCommand::DRIVE_NOTE: {
      QLOG(WARNING) << timestamp << "execute drive note";
      run_event_func(QRunEvent::KEY_QCOMAND_DRIVE_NOTE);
    } break;
    case QCommand::DRIVE_MISSION: {
      QLOG(WARNING) << timestamp << "execute drive mission";
      run_event_func(QRunEvent::KEY_QCOMAND_DRIVE_MISSION);
    } break;
    case QCommand::SEMANTIC_MAP_MODIFIER: {
      QLOG(WARNING) << timestamp << "execute semantic map modifier";
      run_event_func(QRunEvent::KEY_QCOMAND_SEMANTIC_MAP_MODIFIER);
    } break;
    case QCommand::TRAFFIC_LIGHT_IGNORE_ENABLE: {
      QLOG(WARNING) << timestamp << "execute enable ignore traffic light";
      run_event_func(QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_ENABLE);
    } break;
    case QCommand::TRAFFIC_LIGHT_IGNORE_DISABLE: {
      QLOG(WARNING) << timestamp << "execute disable ignore traffic light";
      run_event_func(QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_DISABLE);
    } break;
    case QCommand::LANE_CHANGE_STATIONARY_OBJECTS_ENABLE: {
      QLOG(WARNING) << timestamp
                    << "execute enable lane change stationary objects";
      run_event_func(
          QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_ENABLE);
    } break;
    case QCommand::LANE_CHANGE_STATIONARY_OBJECTS_DISABLE: {
      QLOG(WARNING) << timestamp
                    << "execute disable lane change stationary objects";
      run_event_func(
          QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_DISABLE);
    } break;
    case QCommand::PULL_OVER_ENABLE: {
      QLOG(WARNING) << timestamp << "execute enable pull over";
      run_event_func(QRunEvent::KEY_QCOMAND_PULL_OVER_ENABLE);
    } break;
    case QCommand::PULL_OVER_DISABLE: {
      QLOG(WARNING) << timestamp << "execute enable pull over";
      run_event_func(QRunEvent::KEY_QCOMAND_PULL_OVER_DISABLE);
    } break;
    case QCommand::LEFT_BLINKER_ON: {
      QLOG(WARNING) << timestamp << "execute turn on left blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_LEFT_BLINKER_ON);
    } break;
    case QCommand::LEFT_BLINKER_OFF: {
      QLOG(WARNING) << timestamp << "execute turn off left blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_LEFT_BLINKER_OFF);
    } break;
    case QCommand::LEFT_BLINKER_CANCEL: {
      QLOG(WARNING) << timestamp << "execute cancel left blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_LEFT_BLINKER_CANCEL);
    } break;
    case QCommand::RIGHT_BLINKER_ON: {
      QLOG(WARNING) << timestamp << "execute turn on right blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_ON);
    } break;
    case QCommand::RIGHT_BLINKER_OFF: {
      QLOG(WARNING) << timestamp << "execute turn off right blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_OFF);
    } break;
    case QCommand::RIGHT_BLINKER_CANCEL: {
      QLOG(WARNING) << timestamp << "execute cancel right blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_CANCEL);
    } break;
    case QCommand::EMERGENCY_BLINKER_ON: {
      QLOG(WARNING) << timestamp << "execute turn on emergency blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_ON);
    } break;
    case QCommand::EMERGENCY_BLINKER_OFF: {
      QLOG(WARNING) << timestamp << "execute turn off emergency blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_OFF);
    } break;
    case QCommand::EMERGENCY_BLINKER_CANCEL: {
      QLOG(WARNING) << timestamp << "execute cancel emergency blinker";
      run_event_func(QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_CANCEL);
    } break;
    case QCommand::EMERGENCY_STOP: {
      QLOG(WARNING) << timestamp << "execute emergency stop";
      run_event_func(QRunEvent::KEY_QCOMAND_EMERGENCY_STOP);
    } break;
    case QCommand::MANUAL_CONTROL_ENABLE: {
      QLOG(WARNING) << timestamp << "execute enable manual control";
      run_event_func(QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_ENABLE);
    } break;
    case QCommand::MANUAL_CONTROL_DISABLE: {
      QLOG(WARNING) << timestamp << "execute disable manual control";
      run_event_func(QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_DISABLE);
    } break;
    case QCommand::NUDGE_LEFT:
    case QCommand::NUDGE_RIGHT:
    case QCommand::DRIVE_ROUTE:
    case QCommand::SEMANTIC_MAP_OVERRIDE:
    case QCommand::QHMI_NOTE:
      QLOG(WARNING) << command.type() << "," << timestamp << " not support";
      break;
  }

  response->set_timestamp(absl::ToUnixMicros(Clock::Now()));
  return grpc::Status::OK;
}

grpc::Status PortalServiceServer::State(
    grpc::ServerContext *context, const StateRequest *request,
    grpc::ServerWriter<StateResponse> *writer) {
  StateResponse response;
  auto *state = response.mutable_state();
  while (!stop_notification_.HasBeenNotified()) {
    {
      absl::WriterMutexLock lock(&state_mutex_);
      state_cond_var_.Wait(&state_mutex_);
    }
    if (stop_notification_.HasBeenNotified()) {
      break;
    }
    {
      absl::ReaderMutexLock lock(&state_mutex_);
      state->CopyFrom(state_);
    }

    if (state->type() == QState::SHUTDOWN) {
      break;
    }

    response.set_timestamp(absl::ToUnixMicros(Clock::Now()));
    if (!writer->Write(response)) {
      QLOG(ERROR) << "Failed to forward portal state was caused by network or "
                     "client has been closed.";
      break;
    } else {
      QLOG_EVERY_N_SEC(INFO, 10) << response.DebugString();
    }
  }

  return grpc::Status::OK;
}

grpc::Status PortalServiceServer::Info(grpc::ServerContext *context,
                                       const InfoRequest *request,
                                       InfoResponse *response) {
  response->set_timestamp(absl::ToUnixMicros(Clock::Now()));
  {
    absl::ReaderMutexLock lock(&vehicle_info_mutex_);
    response->mutable_vehicle_info()->CopyFrom(vehicle_info_);
    response->mutable_log_info()->CopyFrom(log_info_);

    auto v2x_params =
        lite_module_->GetRunParams().vehicle_params().v2x_params();
    v2x_util::GetV2xScenes(v2x_params.model(),
                           response->mutable_scene_settings());
  }
  return grpc::Status::OK;
}

grpc::Status PortalServiceServer::RunEvent(grpc::ServerContext *context,
                                           const RunEventRequest *request,
                                           RunEventResponse *response) {
  QLOG(INFO) << "RunEventRequest: " << request->DebugString();

  for (const auto &run_event : request->run_events()) {
    const auto value_type = run_event.value_type();
    switch (value_type) {
      case QRunEvent::TYPE_NONE: {
        break;
      }
      case QRunEvent::TYPE_BOOL: {
        if (!run_event.has_bool_value()) {
          QLOG(ERROR) << "run event value has no bool value "
                      << request->DebugString();
          return grpc::Status::OK;
        }
      } break;
      case QRunEvent::TYPE_INT32: {
        if (!run_event.has_int32_value()) {
          QLOG(ERROR) << "run event value has no int32 value "
                      << request->DebugString();
          return grpc::Status::OK;
        }
      } break;
      case QRunEvent::TYPE_FLOAT: {
        if (!run_event.has_float_value()) {
          QLOG(ERROR) << "run event value has no float value "
                      << request->DebugString();
          return grpc::Status::OK;
        }
      } break;
      case QRunEvent::TYPE_STRING: {
        if (!run_event.has_string_value()) {
          QLOG(ERROR) << "run event value has no string value "
                      << request->DebugString();
          return grpc::Status::OK;
        }
      } break;
      case QRunEvent::TYPE_MSG: {
        if (!run_event.has_routing_request_proto() &&
            !run_event.has_semantic_map_modifier_proto() &&
            !run_event.has_product_setting_proto()) {
          QLOG(ERROR) << "run event value has no msg value "
                      << request->DebugString();
          return grpc::Status::OK;
        }
      } break;
    }
  }

  auto lite_msg_publish = [this](LiteMsgWrapper *msg) {
    QLOG(INFO) << "result: " << msg->DebugString();
    if (lite_module_ != nullptr) {
      QLOG_IF_NOT_OK(WARNING, lite_module_->ForwardLiteMsg(msg));
    }
  };
  // on command
  run_event_process_->OnProcess(*request, lite_msg_publish);

  // record and trans events
  LiteMsgWrapper lite_msg;
  lite_msg.set_tag_number(LiteMsgWrapper::LiteMsgCase::kQRunEventsProto);
  auto *q_run_events_proto = lite_msg.mutable_q_run_events_proto();
  q_run_events_proto->mutable_run_events()->CopyFrom(request->run_events());
  q_run_events_proto->mutable_header()->set_timestamp(request->timestamp());
  lite_msg_publish(&lite_msg);

  response->set_timestamp(absl::ToUnixMicros(Clock::Now()));
  return grpc::Status::OK;
}

void PortalServiceServer::BuildMultiStopsRequestProto(
    const std::string &command_id, const QCommand::DriveRoute &drive_route,
    RoutingRequestProto *routing_request_proto) {
  auto *multi_stops = routing_request_proto->mutable_multi_stops();
  multi_stops->set_infinite_loop(drive_route.infinite_loop());
  multi_stops->set_skip_past_stops(drive_route.skip_past_stops());
  for (const auto &stops : drive_route.stops()) {
    auto *lite_stops = multi_stops->add_stops();
    if (stops.has_stop_name()) {
      lite_stops->set_stop_name(stops.stop_name());
    }

    if (stops.has_stop_point()) {
      auto *global_point =
          lite_stops->mutable_stop_point()->mutable_global_point();
      global_point->CopyFrom(stops.stop_point().geo_point());
    }

    if (stops.via_points_size() > 0) {
      for (const auto &via_point : stops.via_points()) {
        auto *global_point =
            lite_stops->add_via_points()->mutable_global_point();
        global_point->CopyFrom(via_point.geo_point());
      }
    }
  }

  const auto &limited_zone = drive_route.limited_zone();
  for (const auto &lane : limited_zone.lanes()) {
    routing_request_proto->add_avoid_lanes(lane);
  }

  for (const auto &region : limited_zone.regions()) {
    auto *avoid_regions = routing_request_proto->add_avoid_regions();
    avoid_regions->CopyFrom(region);
  }

  routing_request_proto->set_id(command_id);
}

void PortalServiceServer::BuildSemanticMapModificationProto(
    const QCommand::SemanticMapOverride &semantic_map_override,
    SemanticMapModificationProto *semantic_map_modification_proto) {
  if (!semantic_map_override.has_override_zone()) {
    return;
  }

  const auto &override_zone = semantic_map_override.override_zone();
  auto *modifier = semantic_map_modification_proto->mutable_modifier();
  auto *speed_limit_modifier = modifier->mutable_speed_limit_modifier();
  for (const auto &lane : override_zone.lanes()) {
    auto *lane_id_modifier = speed_limit_modifier->add_lane_id_modifier();
    lane_id_modifier->set_lane_id(lane.lane());
    lane_id_modifier->set_override_speed_limit(lane.limited_speed());
  }
  for (const auto &region : override_zone.regions()) {
    auto *region_modifier = speed_limit_modifier->add_region_modifier();
    region_modifier->mutable_region()->CopyFrom(region.region());
    region_modifier->set_override_speed_limit(region.limited_speed());
  }
  speed_limit_modifier->set_max_speed_limit(override_zone.limited_speed());
}

}  // namespace lite
}  // namespace qcraft
