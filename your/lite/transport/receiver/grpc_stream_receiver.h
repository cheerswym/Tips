#ifndef ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_STREAM_RECEIVER_H_
#define ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_STREAM_RECEIVER_H_

#include <memory>

#include "onboard/lite/transport/receiver/receiver.h"
#include "onboard/lite/transport/receiver/receiver_context.h"
// #include "onboard/lite/transport/receiver/grpc_receiver.h"

namespace qcraft {

class GRPCStreamReceiver : public Receiver {
 public:
  explicit GRPCStreamReceiver(const NodeConfig &node_config);

  virtual ~GRPCStreamReceiver();

  void Enable() override;

  void Disable() override;

 private:
  template <typename ServiceType, typename ServerContextType,
            typename RequestType, typename ResponseType>
  std::unique_ptr<ReceiveContext> BuildReceiveContextStreamed(
      std::function<
          void(ServiceType *, ServerContextType *,
               grpc::ServerAsyncReaderWriter<ResponseType, RequestType> *,
               grpc::CompletionQueue *, grpc::ServerCompletionQueue *, void *)>
          request_connect,
      std::function<grpc::Status(const RequestType *, ResponseType *)>
          receive_transmit,
      grpc::ServerCompletionQueue *complete_queue);

 private:
  grpc::ServerContext server_context_;
};

template <typename ServiceType, typename ServerContextType,
          typename RequestType, typename ResponseType>
std::unique_ptr<ReceiveContext> GRPCStreamReceiver::BuildReceiveContextStreamed(
    std::function<
        void(ServiceType *, ServerContextType *,
             grpc::ServerAsyncReaderWriter<ResponseType, RequestType> *,
             grpc::CompletionQueue *, grpc::ServerCompletionQueue *, void *)>
        request_connect,
    std::function<grpc::Status(const RequestType *, ResponseType *)>
        receive_transmit,
    grpc::ServerCompletionQueue *complete_queue) {
  return std::make_unique<ReceiveContextUnaryStreamImpl<
      ServerContextType, RequestType, ResponseType>>(
      std::bind(request_connect, &async_service_, std::placeholders::_1,
                std::placeholders::_2, complete_queue, complete_queue,
                std::placeholders::_3),
      receive_transmit);
}

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_RECEIVER_GRPC_STREAM_RECEIVER_H_
