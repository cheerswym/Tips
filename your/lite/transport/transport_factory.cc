#include "onboard/lite/transport/transport_factory.h"

#include "onboard/lite/transport/receiver/grpc_receiver.h"
#include "onboard/lite/transport/receiver/grpc_stream_receiver.h"
#include "onboard/lite/transport/transmitter/grpc_stream_transmitter.h"
#include "onboard/lite/transport/transmitter/grpc_transmitter.h"

namespace qcraft {

std::shared_ptr<Transmitter> TransportFactory::CreateTransmitter(
    const NodeConfig& node_config, const RunParamsProtoV2& run_param,
    const TransportMode& mode) {
  std::shared_ptr<Transmitter> transmitter;
  switch (mode) {
    case TransportMode::GRPC:
      transmitter = std::make_shared<GRPCTransmitter>(node_config, run_param);
      break;
    case TransportMode::GRPC_STREAM:
      transmitter =
          std::make_shared<GRPCStreamTransmitter>(node_config, run_param);
      break;
  }

  return transmitter;
}

std::shared_ptr<Receiver> TransportFactory::CreateReceiver(
    const NodeConfig& node_config, const TransportMode& mode) {
  std::shared_ptr<Receiver> receiver;
  switch (mode) {
    case TransportMode::GRPC:
      receiver = std::make_shared<GRPCReceiver>(node_config);
      break;
    case TransportMode::GRPC_STREAM:
      receiver = std::make_shared<GRPCStreamReceiver>(node_config);
      break;
  }

  return receiver;
}

}  // namespace qcraft
