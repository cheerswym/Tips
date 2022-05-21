#ifndef ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_TRANSMITTER_H_
#define ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_TRANSMITTER_H_

#include "onboard/lite/transport/transmitter/transmitter.h"

namespace qcraft {

class GRPCTransmitter : public Transmitter {
 public:
  explicit GRPCTransmitter(const NodeConfig &node_config,
                           const RunParamsProtoV2 &);

  virtual ~GRPCTransmitter();

  bool Transmit(const LiteMsgWrapper &lite_msg) override;

  bool Transmit(const ShmMessage &shm_message) override;

 private:
  ::grpc::Status GrpcTransmitLiteMsg(
      ::grpc::ClientContext *context,
      const ::qcraft::TransmitLiteMsgRequest &request,
      ::qcraft::TransmitLiteMsgResponse *response);
  ::grpc::Status GrpcTransmitShmMsg(
      ::grpc::ClientContext *context,
      const ::qcraft::TransmitShmMsgRequest &request,
      ::qcraft::TransmitShmMsgResponse *response);
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_TRANSMITTER_GRPC_TRANSMITTER_H_
