#include "onboard/lite/transport/receiver/grpc_stream_receiver.h"
namespace qcraft {

namespace {
enum { RECEIVE_LITE, RECEIVE_SHM, RECEIVE_QUERY, RECEIVE_MAX };
}  // namespace

GRPCStreamReceiver::GRPCStreamReceiver(const NodeConfig& node_config)
    : Receiver(node_config) {}

GRPCStreamReceiver::~GRPCStreamReceiver() { Disable(); }

void GRPCStreamReceiver::Enable() {
  Receiver::Enable();
  lite_recevier_future_ = std::async(
      std::launch::async,
      [this, complete_queue = completion_queues_[RECEIVE_LITE].get()] {
        const auto receive_context = BuildReceiveContextStreamed<
            NodeService::AsyncService, grpc::ServerContext,
            TransmitLiteMsgRequest, TransmitLiteMsgResponse>(
            &NodeService::AsyncService::RequestTransmitLiteMsgStreamed,
            std::bind(&GRPCStreamReceiver::ReceiveLiteMsg, this,
                      std::placeholders::_1, std::placeholders::_2),
            complete_queue);

        ReceiveLoop(complete_queue);
      });

  shm_receiver_future_ = std::async(
      std::launch::async,
      [this, complete_queue = completion_queues_[RECEIVE_SHM].get()] {
        const auto receive_context = BuildReceiveContextStreamed<
            NodeService::AsyncService, grpc::ServerContext,
            TransmitShmMsgRequest, TransmitShmMsgResponse>(
            &NodeService::AsyncService::RequestTransmitShmMsgStreamed,
            std::bind(&GRPCStreamReceiver::ReceiveShmMsg, this,
                      std::placeholders::_1, std::placeholders::_2),
            complete_queue);

        ReceiveLoop(complete_queue);
      });
}

void GRPCStreamReceiver::Disable() { Receiver::Disable(); }

}  // namespace qcraft
