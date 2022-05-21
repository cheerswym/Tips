#ifndef ONBOARD_LITE_TRANSPORT_TRANSPORT_FACTORY_H_
#define ONBOARD_LITE_TRANSPORT_TRANSPORT_FACTORY_H_

#include <memory>

#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/transport/receiver/receiver.h"
#include "onboard/lite/transport/transmitter/transmitter.h"
#include "onboard/params/vehicle_param_api.h"

namespace qcraft {

enum TransportMode { GRPC, GRPC_STREAM };

class TransportFactory {
 public:
  std::shared_ptr<Transmitter> CreateTransmitter(
      const NodeConfig& node_config, const RunParamsProtoV2& run_param,
      const TransportMode& mode = TransportMode::GRPC);

  std::shared_ptr<Receiver> CreateReceiver(
      const NodeConfig& node_config,
      const TransportMode& mode = TransportMode::GRPC);
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_TRANSPORT_FACTORY_H_
