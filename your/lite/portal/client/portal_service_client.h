#pragma once

#include <grpc++/grpc++.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "onboard/lite/portal/proto/portal_service.grpc.pb.h"

namespace qcraft {
namespace lite {

class PortalServiceClient {
 public:
  explicit PortalServiceClient(const std::string& server_addr);
  explicit PortalServiceClient(std::shared_ptr<grpc::Channel> channel);

  grpc::Status Command(const CommandRequest& request,
                       CommandResponse* response);

  std::unique_ptr<grpc::ClientReader<StateResponse>> State(
      grpc::ClientContext* context, const StateRequest& request);

  grpc::Status Info(const InfoRequest& request, InfoResponse* response);

  grpc::Status RunEvent(const RunEventRequest& request,
                        RunEventResponse* response);

 private:
  std::unique_ptr<PortalService::Stub> stub_;
};

}  // namespace lite
}  // namespace qcraft
