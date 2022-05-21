#include <grpc++/grpc++.h>

#include <string>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lite/portal/server/portal_service_server.h"
#include "onboard/proto/port.pb.h"

DEFINE_string(server_addr_ip, "0.0.0.0", "Portal Service server address");
DEFINE_int32(server_addr_port, qcraft::PORTAL_SERVICE_PORT,
             "Portal Service server port");

int main(int argc, char* argv[]) {
  qcraft::InitQCraft(&argc, &argv);
  const auto server_addr =
      absl::StrCat(FLAGS_server_addr_ip, ":", FLAGS_server_addr_port);
  LOG(INFO) << "================================";
  LOG(INFO) << "server_addr: " << server_addr;
  LOG(INFO) << "================================";
  auto server = std::make_unique<qcraft::lite::PortalServiceServer>();
  LOG(INFO) << "Starting portal service server...";
  auto grpc_server = server->StartServer(server_addr, nullptr);
  LOG(INFO) << "Listening on " << server_addr;
  grpc_server->Wait();
  return 0;
}
