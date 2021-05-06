#include "onboard/planner/router/routing_module.h"

#include <algorithm>
#include <string>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/router/route_engine.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/planner.pb.h"

DECLARE_string(planner_inject_teleop_proto);

namespace qcraft {
namespace planner {

namespace {

static constexpr absl::Duration kRoutingMainLoopInterval =
    absl::Milliseconds(100);
static constexpr int kSecondToMicros = 1000000;
static constexpr double kMaxDistanceMeters = 2.5;  // about 2.5 meters.

bool ValidateLiteHeader(const LiteHeader &header,
                        const absl::Duration &duration) {
  if (header.timestamp() == 0) return true;  // backward data.atibility
  const auto &delay = Clock::Now() - absl::FromUnixMicros(header.timestamp());
  if (delay > duration) {
    QLOG(INFO) << "timestamp " << header.timestamp() << ","
               << absl::ToUnixMicros(Clock::Now()) << ", delay "
               << absl::ToInt64Microseconds(delay);
    return false;
  }
  return true;
}
bool ValidateRemoteAssistMessage(const RemoteAssistToCarProto &ra_to_car) {
  return ValidateLiteHeader(ra_to_car.header(),
                            /*FLAGS_teleop_expire_seconds*/ absl::Seconds(3));
}

void FillRouteResult(const SemanticMapManager &semantic_map_manager,
                     const RouteManagerOutput &route_manager_output,
                     RoutingResultProto *routing_result_proto) {
  routing_result_proto->clear_path_points_from_current();
  if (route_manager_output.route_from_current != nullptr) {
    absl::StatusOr<std::vector<Vec2d>> points_or =
        SimplifyPathPoints(semantic_map_manager,
                           route_manager_output.route_from_current->lane_path(),
                           kMaxDistanceMeters);
    if (points_or.ok()) {
      std::for_each(
          points_or->begin(), points_or->end(),
          [routing_result_proto](const Vec2d &point) {
            point.ToProto(routing_result_proto->add_path_points_from_current());
          });
    } else {
      QLOG(ERROR) << "Cannot generate simplify route path points.";
    }
  }
}

absl::Status CheckRoutePrecondition(const RoutingInput &input) {
  if (input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "Pose is null.");
  }
  if (input.localization_transform == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "localization_transform is null.");
  }
  if (input.autonomy_state == nullptr) {
    return absl::FailedPreconditionError(
        "Check precondition failed. Routing module is not ready. "
        "autonomy_state is null.");
  }

  return absl::OkStatus();
}

}  // namespace

RoutingModule::RoutingModule(LiteClientBase *client) : LiteModule(client) {
  route_state_ = RoutingStateProto::UNINITIALIZED;
}

void RoutingModule::HandleReroutingRequest(
    std::shared_ptr<const ReroutingRequestProto> rerouting_request) {
  input_.rerouting_request = std::move(rerouting_request);
  const auto &request = *(input_.rerouting_request);
  const std::string &request_id =
      request.has_routing_request() && request.routing_request().has_id()
          ? request.routing_request().id()
          : "";
  QLOG(INFO) << absl::StrCat("A new rerouting request arrived. state:",
                             route_state_, "id: ", request_id);
  HmiContentProto hmi_content_proto;
  hmi_content_proto.mutable_route_content()->set_routing_request_id(request_id);
  QLOG_IF_NOT_OK(ERROR, Publish(hmi_content_proto));
}

void RoutingModule::HandleRunEventStates(
    std::shared_ptr<const QRunEventStatesProto> states) {
  const auto &event_state =
      states->states(QRunEventStatesProto::STATE_TYPE_COM_DRIVE_MISSION);
  const auto &event_routing_request =
      event_state.run_event().routing_request_proto();
  if (input_.rerouting_request != nullptr &&
      input_.rerouting_request->routing_request().id() ==
          event_routing_request.id()) {
    return;
  }
  ReroutingRequestProto rerouting_request;
  rerouting_request.mutable_header()->set_timestamp(
      event_state.run_event_state_header().send_timestamp());
  *rerouting_request.mutable_routing_request() = event_routing_request;
  auto rerouting_request_ptr =
      std::make_shared<const ReroutingRequestProto>(rerouting_request);

  input_.rerouting_request = std::move(rerouting_request_ptr);
  const auto &request = *(input_.rerouting_request);
  const std::string &request_id =
      request.has_routing_request() && request.routing_request().has_id()
          ? request.routing_request().id()
          : "";
  QLOG(INFO) << absl::StrCat("A new rerouting request arrived. state:",
                             route_state_, "id: ", request_id);
  HmiContentProto hmi_content_proto;
  hmi_content_proto.mutable_route_content()->set_routing_request_id(request_id);
  QLOG_IF_NOT_OK(ERROR, Publish(hmi_content_proto));
}

void RoutingModule::OnInit() {
  DisableAutoOk();
  if (semantic_map_manager_ == nullptr) {
    QLOG(INFO) << "Load local semantic_map_manager";
    semantic_map_manager_ = std::make_shared<SemanticMapManager>();
    semantic_map_manager_->LoadWholeMap().Build();
  } else {
    QLOG(INFO) << "semantic_map_manager is specified at external.";
  }
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  PlannerParams::Instance()->Init(run_params);
  file_util::TextFileToProto(FLAGS_route_default_params_file,
                             &route_param_proto_);
  QLOG(INFO) << "Route params loaded. file_path: "
             << FLAGS_route_default_params_file
             << ", content: " << route_param_proto_.ShortDebugString();
  route_mgr_ = std::make_unique<RouteManager>(semantic_map_manager_.get(),
                                              &route_param_proto_);
  route_state_ = RoutingStateProto::START;
  QLOG(INFO) << "OnInit() success, Load vehicle params, is_bus:"
             << PlannerParams::IsBus();
}

void RoutingModule::OnSubscribeChannels() {
  Subscribe(&RoutingModule::HandlePose, this);
  Subscribe(&RoutingModule::HandleLocalizationTransform, this);
  Subscribe(&RoutingModule::HandleRemoteAssistToCar, this);
  Subscribe(&RoutingModule::HandleRecordedRoute, this);
  Subscribe(&RoutingModule::HandleAutonomyState, this);
  Subscribe(&RoutingModule::HandleDriverAction, this);
  Subscribe(&RoutingModule::HandleRunEventStates, this);
}

void RoutingModule::OnSetUpTimers() {
  AddTimerOrDie("route_main_loop", std::bind(&RoutingModule::MainLoop, this),
                kRoutingMainLoopInterval,
                /*one_shot=*/false);
}

void RoutingModule::MainLoop() {
  RoutingStateProto routing_state_proto;
  bool route_precondition = true;
  const auto precondition_status = CheckRoutePrecondition(input_);
  if (!precondition_status.ok()) {
    route_state_ = RoutingStateProto::UNAVAILABLE;
    routing_state_proto.set_state(route_state_);
    QLOG_IF_NOT_OK(WARNING, Publish(routing_state_proto));
    route_precondition = false;
  }
  if (input_.rerouting_request != nullptr) {
    absl::Status request_status =
        route_mgr_->AddManualReroutingRequest(*input_.rerouting_request);
    if (!request_status.ok()) {
      QLOG(ERROR) << "Cannot handle the routing request."
                  << input_.rerouting_request->ShortDebugString();
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_SEMANTIC_MAP_ROUTEING_REQUEST_INVALID,
          "It is a map issue, not a route issue. "
          "Cannot hanlde the routing request.",
          request_status.ToString());
    }
  }

  if (!route_precondition || semantic_map_manager_ == nullptr) {
    if (input_.rerouting_request != nullptr) {
      // So we got a routing request but Precondition not satisfied.
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_PLANNER_ROUTING_REQUEST_FAILED,
          "Receive a new request, but the preconditions not statisfied.",
          precondition_status.ToString());
    }
    return;
  }
  semantic_map_manager_->UpdateLocalizationTransform(
      *input_.localization_transform, /*thread_pool=*/nullptr);
  LiteModule::Ok();
  route_state_ = RoutingStateProto::RUNNING;
  routing_state_proto.set_state(route_state_);
  QLOG_IF_NOT_OK(WARNING, Publish(routing_state_proto));

  input_.injected_teleop_micro_secs = absl::ToUnixMicros(Clock::Now());
  if (!route_mgr_->has_route() && !route_mgr_->has_offroad_route()) {
    route_mgr_->InitRequest(input_.recorded_route,
                            input_.injected_teleop_micro_secs);
  }
  if (input_.remote_assist_to_car != nullptr) {
    ProcessTeleopProto(*input_.remote_assist_to_car, route_mgr_.get());
  }
  MaybeInjectTeleopProto(route_mgr_.get(), input_.injected_teleop_micro_secs);
  SCOPED_QTRACE("RouteManagerUpdate");

  route_mgr_->ClearLastError();
  RouteManagerInput route_manager_input(this->input_);
  auto route_mgr_output_or =
      route_mgr_->Update(PlannerParams::IsBus(), route_manager_input);
  int64_t now_microsecs = absl::ToUnixMicros(Clock::Now());
  if (route_mgr_output_or.ok()) {
    RouteManagerOutputProto route_mgr_output_proto;
    route_mgr_output_or->ToProto(&route_mgr_output_proto);
    const auto &header_statusor = Publish(route_mgr_output_proto);
    if (header_statusor.ok()) {
      *(route_mgr_output_proto.mutable_header()) = header_statusor.value();
    }
    if ((now_microsecs - last_publish_microsecs_) >= 1 * kSecondToMicros ||
        route_mgr_output_or->rerouted) {
      if (route_mgr_->MutableRoutingResultProto()->success()) {
        FillRouteResult(*semantic_map_manager_, route_mgr_output_or.value(),
                        route_mgr_->MutableRoutingResultProto());
      }
      QLOG_IF_NOT_OK(WARNING, Publish(route_mgr_->GetRoutingResultProto()));
      last_publish_microsecs_ = now_microsecs;
    } else {
      VLOG(2) << "Ignore publish route manager output result"
              << ", now: " << now_microsecs
              << ", last_publish_time: " << last_publish_microsecs_;
    }
  } else {
    // Check error
    const route::RouteErrorCode &error_code = route_mgr_->GetLastError();
    QLOG(WARNING) << "No route manager output result, "
                  << route_mgr_output_or.status().ToString()
                  << ", now: " << now_microsecs
                  << ", last_publish_time: " << last_publish_microsecs_
                  << ", route_error: " << error_code.ToString();

    if (input_.rerouting_request != nullptr &&
        error_code.code() != route::RouteErrorCode::StatusCode::kOk) {
      std::string error_model;
      switch (route_mgr_output_or.status().code()) {
        case absl::StatusCode::kInternal:
          error_model = "route";
          break;
        case absl::StatusCode::kInvalidArgument:
          error_model = "onboard infra";
          break;
        case absl::StatusCode::kNotFound:
          error_model = "map/localization";
        default:
          error_model = "unknown";
          break;
      }
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
          QIssueSubType::QIST_PLANNER_ROUTING_UPDATE_FAILED,
          absl::StrCat("No route for a new request, model:", error_model),
          error_code.ToString());
    }
  }
  CleanBeforeNextIteration();
}

// Only process Teleop related with route
void RoutingModule::ProcessTeleopProto(
    const RemoteAssistToCarProto &teleop_proto, RouteManager *route_manager) {
  SCOPED_QTRACE("ProcessTeleopProto");
  if (!ValidateRemoteAssistMessage(teleop_proto)) {
    QLOG(ERROR) << "remote assist message validation failed";
  }
  if (teleop_proto.has_driving_action_request() &&
      !teleop_proto.driving_action_request().has_lane_change() &&
      teleop_proto.driving_action_request().has_rerouting()) {
    QLOG_IF_NOT_OK(ERROR,
                   route_manager->AddManualReroutingRequest(
                       teleop_proto.driving_action_request().rerouting()))
        << " Add teleop failed.";
  }
}
void RoutingModule::MaybeInjectTeleopProto(RouteManager *route_manager,
                                           int64_t injected_teleop_micro_secs) {
  SCOPED_QTRACE("MaybeInjectTeleopProto");
  InjectedTeleopProto injected_teleop_proto;
  QCHECK(file_util::StringToProto(FLAGS_planner_inject_teleop_proto,
                                  &injected_teleop_proto));
  for (int i = 0; i < injected_teleop_proto.frames_size(); ++i) {
    if (injected_teleop_proto.frames(i) - injected_teleop_micro_secs <
        50 * 1000) {
      QLOG(INFO) << "Inject Teleop proto at frame(microsecs): "
                 << injected_teleop_micro_secs << ": "
                 << injected_teleop_proto.ShortDebugString();
      ProcessTeleopProto(injected_teleop_proto.contents(i), route_manager);
    }
  }
}
void RoutingModule::CleanBeforeNextIteration() {
  input_.remote_assist_to_car.reset();
  input_.rerouting_request.reset();
  input_.routing_result.reset();
  input_.recorded_route.reset();
}
}  // namespace planner
}  // namespace qcraft
