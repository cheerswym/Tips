#include "onboard/lite/portal/client/portal_service_client.h"

#include <grpc++/grpc++.h>

#include <string>
#include <unordered_map>

#include "absl/status/statusor.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"

namespace qcraft {
namespace lite {

PortalServiceClient::PortalServiceClient(const std::string& server_addr)
    : PortalServiceClient(grpc::CreateChannel(
          server_addr, grpc::InsecureChannelCredentials())) {}

PortalServiceClient::PortalServiceClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(PortalService::NewStub(channel)) {}

grpc::Status PortalServiceClient::Command(const CommandRequest& request,
                                          CommandResponse* response) {
  grpc::ClientContext context;
  return stub_->Command(&context, request, response);
}

std::unique_ptr<grpc::ClientReader<StateResponse>> PortalServiceClient::State(
    grpc::ClientContext* context, const StateRequest& request) {
  return stub_->State(context, request);
}

grpc::Status PortalServiceClient::Info(const InfoRequest& request,
                                       InfoResponse* response) {
  grpc::ClientContext context;
  return stub_->Info(&context, request, response);
}

grpc::Status PortalServiceClient::RunEvent(const RunEventRequest& request,
                                           RunEventResponse* response) {
  grpc::ClientContext context;
  return stub_->RunEvent(&context, request, response);
}

}  // namespace lite
}  // namespace qcraft
