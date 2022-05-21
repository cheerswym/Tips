#include "onboard/lite/transport/transmitter/grpc_stream_transmitter.h"

#include <string>
#include <utility>

#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"

namespace qcraft {

GRPCStreamTransmitter::GRPCStreamTransmitter(const NodeConfig &node_config,
                                             const RunParamsProtoV2 &run_param)
    : Transmitter(node_config, run_param) {}

GRPCStreamTransmitter::~GRPCStreamTransmitter() { Disable(); }

void GRPCStreamTransmitter::Enable() {
  Transmitter::Enable();
  lite_msg_writer_ = std::make_unique<
      TimedStreamWriter<TransmitLiteMsgRequest, TransmitLiteMsgResponse> >(
      std::bind(&NodeService::Stub::TransmitLiteMsgStreamed,
                GetNodeServiceStub(), std::placeholders::_1));
  shm_msg_writer_ = std::make_unique<
      TimedStreamWriter<TransmitShmMsgRequest, TransmitShmMsgResponse> >(
      std::bind(&NodeService::Stub::TransmitShmMsgStreamed,
                GetNodeServiceStub(), std::placeholders::_1));
  lite_msg_writer_->Enable();
  shm_msg_writer_->Enable();
}

void GRPCStreamTransmitter::Disable() {
  if (lite_msg_writer_ != nullptr) {
    lite_msg_writer_->Disable();
  }
  if (shm_msg_writer_ != nullptr) {
    shm_msg_writer_->Disable();
  }
  Transmitter::Disable();
}

bool GRPCStreamTransmitter::Transmit(const LiteMsgWrapper &lite_msg) {
  TransmitLiteMsgRequest request;
  if (!BuildTransmitLiteMsgRequest(lite_msg, &request)) {
    return true;
  }

  int64_t timeout_ms = -1;
  if (GetNodeConfig().has_timeout()) {
    const auto channel_domain =
        LiteMsgConverter::Get().GetLiteMsgChannelDomain(lite_msg);
    if (channel_domain.first == "trace_proto") {
      timeout_ms = GetNodeConfig().timeout() * 4;
    } else {
      timeout_ms = GetNodeConfig().timeout();
    }
  }

  if (!lite_msg_writer_->Transmit(request, timeout_ms)) {
    QLOG_EVERY_N_SEC(WARNING, 3.0)
        << "TransmitLiteMsg: " << GetNodeConfig().node_ip()
        << ", failed. write failed";
    return false;
  }

  return true;
}

bool GRPCStreamTransmitter::Transmit(const ShmMessage &shm_message) {
  TransmitShmMsgRequest request;
  if (!BuildTransmitShmMsgRequest(shm_message, &request)) {
    return true;
  }

  int64_t timeout_ms = -1;
  if (GetNodeConfig().has_timeout()) {
    timeout_ms = GetNodeConfig().timeout();
  }

  if (!shm_msg_writer_->Transmit(request, timeout_ms)) {
    QLOG_EVERY_N_SEC(WARNING, 3.0)
        << "TransmitShmMsg: " << GetNodeConfig().node_ip()
        << ", failed. write failed";
    return false;
  }

  return true;
}

}  // namespace qcraft
