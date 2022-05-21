#pragma once

#include <grpc++/grpc++.h>

#include <atomic>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "google/protobuf/message.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/portal/proto/portal_service.grpc.pb.h"
#include "onboard/lite/portal/server/run_event_process.h"
#include "onboard/proto/q_state.pb.h"
#include "onboard/proto/q_vehicle_info.pb.h"

namespace qcraft {
namespace lite {

class PortalServiceServer final : public PortalService::Service {
 public:
  PortalServiceServer();

  std::unique_ptr<grpc::Server> StartServer(const std::string &addr,
                                            LiteModule *lite_module);

  void StopServer();

  void UpdateState(const QState &state);

  void UpdateVehicleInfo(const QVehicleInfo &vehicle_info);

  void UpdateLogInfo(const QLogInfo &log_info);

  // grpc interface
  grpc::Status Command(grpc::ServerContext *context,
                       const CommandRequest *request,
                       CommandResponse *response) override;

  grpc::Status State(grpc::ServerContext *context, const StateRequest *request,
                     grpc::ServerWriter<StateResponse> *writer) override;

  grpc::Status Info(grpc::ServerContext *context, const InfoRequest *request,
                    InfoResponse *response) override;

  grpc::Status RunEvent(grpc::ServerContext *context,
                        const RunEventRequest *request,
                        RunEventResponse *response) override;

 private:
  void BuildMultiStopsRequestProto(const std::string &command_id,
                                   const QCommand::DriveRoute &drive_route,
                                   RoutingRequestProto *routing_request_proto);

  void BuildSemanticMapModificationProto(
      const QCommand::SemanticMapOverride &semantic_map_override,
      SemanticMapModificationProto *semantic_map_modification_proto);

 private:
  LiteModule *lite_module_ = nullptr;

  absl::Mutex state_mutex_;
  absl::Notification stop_notification_;
  absl::CondVar state_cond_var_ GUARDED_BY(state_mutex_);
  QState state_ GUARDED_BY(state_mutex_);

  absl::Mutex vehicle_info_mutex_;
  QVehicleInfo vehicle_info_ GUARDED_BY(vehicle_info_mutex_);

  QLogInfo log_info_;

  std::unique_ptr<RunEventProcess> run_event_process_ = nullptr;
};
}  // namespace lite
}  // namespace qcraft
