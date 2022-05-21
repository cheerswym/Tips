#include "onboard/lite/transport/transport_factory.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace shm {

TEST(TransportFactoryTest, CreateTransmitter) {
  TransportFactory transport_factory;

  NodeConfig node_config;
  RunParamsProtoV2 run_param;
  TransportMode mode = TransportMode::GRPC;
  EXPECT_NE(transport_factory.CreateTransmitter(node_config, run_param, mode),
            nullptr);

  mode = TransportMode::GRPC_STREAM;
  EXPECT_NE(transport_factory.CreateTransmitter(node_config, run_param, mode),
            nullptr);
}

TEST(TransportFactoryTest, CreateReceiver) {
  TransportFactory transport_factory;
  NodeConfig node_config;
  node_config.set_node_name("test_node");
  node_config.set_node_ip("127.0.0.1");
  node_config.set_node_port("10001");
  NodeService::AsyncService async_service_;

  TransportMode mode = TransportMode::GRPC;
  std::shared_ptr<Receiver> rcver =
      transport_factory.CreateReceiver(node_config, mode);
  EXPECT_NE(rcver, nullptr);
  rcver->Enable();

  mode = TransportMode::GRPC_STREAM;
  std::shared_ptr<Receiver> rcver_stream =
      transport_factory.CreateReceiver(node_config, mode);
  EXPECT_NE(rcver_stream, nullptr);
  rcver_stream->Enable();
}

}  // namespace shm
}  // namespace qcraft
