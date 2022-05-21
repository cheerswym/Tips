#ifndef ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_RECEIVER_H_
#define ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_RECEIVER_H_

#include "onboard/lite/transport/receiver/receiver.h"

namespace qcraft {

class GRPCReceiver : public Receiver {
 public:
  explicit GRPCReceiver(const NodeConfig &node_config);

  virtual ~GRPCReceiver();

  void Enable() override;

  void Disable() override;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_RECEIVER_H_
