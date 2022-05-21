#include "onboard/lite/transport/receiver/grpc_receiver.h"

#include "onboard/lite/transport/receiver/receiver_context.h"

namespace qcraft {
namespace {
enum { RECEIVE_LITE, RECEIVE_SHM, RECEIVE_QUERY, RECEIVE_MAX };
}  // namespace

GRPCReceiver::GRPCReceiver(const NodeConfig& node_config)
    : Receiver(node_config) {}

GRPCReceiver::~GRPCReceiver() { Disable(); }

void GRPCReceiver::Enable() {
  Receiver::Enable();
  lite_recevier_future_ = std::async(
      std::launch::async,
      [this, complete_queue = completion_queues_[RECEIVE_LITE].get()] {
        const auto receive_context =
            BuildReceiveContext<NodeService::AsyncService, grpc::ServerContext,
                                TransmitLiteMsgRequest,
                                TransmitLiteMsgResponse>(
                &NodeService::AsyncService::RequestTransmitLiteMsg,
                std::bind(&GRPCReceiver::ReceiveLiteMsg, this,
                          std::placeholders::_1, std::placeholders::_2),
                complete_queue);

        ReceiveLoop(complete_queue);
      });

  shm_receiver_future_ = std::async(
      std::launch::async,
      [this, complete_queue = completion_queues_[RECEIVE_SHM].get()] {
        const auto receive_context =
            BuildReceiveContext<NodeService::AsyncService, grpc::ServerContext,
                                TransmitShmMsgRequest, TransmitShmMsgResponse>(
                &NodeService::AsyncService::RequestTransmitShmMsg,
                std::bind(&GRPCReceiver::ReceiveShmMsg, this,
                          std::placeholders::_1, std::placeholders::_2),
                complete_queue);

        ReceiveLoop(complete_queue);
      });
}

void GRPCReceiver::Disable() { Receiver::Disable(); }

}  // namespace qcraft
