#ifndef ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_H_
#define ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_H_

#include <grpc++/grpc++.h>

#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/transport/proto/node_service.grpc.pb.h"
#include "onboard/lite/transport/receiver/receiver_context.h"
#include "onboard/node/node_query_struct.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

class Receiver {
 public:
  using MetaData =
      std::variant<EncodedLidarFrameMetaData, EncodedImageMetadata>;

 public:
  explicit Receiver(const NodeConfig &node_config)
      : node_config_(node_config) {}

  virtual ~Receiver() {}

  virtual void Enable();

  virtual void Disable();

  virtual void SubscribLiteMsgCallback(
      const std::map<std::pair<std::string, std::string>, std::string>
          &server_msgs,
      const std::function<void(const LiteHeader &,
                               std::shared_ptr<LiteMsgWrapper>)> &callback);

  virtual void SubscribShmMsgCallback(
      const std::function<void(const LiteHeader &, const Receiver::MetaData &,
                               const std::string &,
                               const LidarParametersProto &)> &callback);

  virtual void QueryCmdCallback(
      const std::function<bool(const NodeQeuryRequest &, NodeQeuryResponse *)>
          &callback);

 protected:
  grpc::Status ReceiveLiteMsg(const TransmitLiteMsgRequest *request,
                              TransmitLiteMsgResponse *response);

  grpc::Status ReceiveShmMsg(const TransmitShmMsgRequest *request,
                             TransmitShmMsgResponse *response);

  grpc::Status ReceiveQueryCmd(const QueryRemoteNodeRequest *request,
                               QueryRemoteNodeResponse *response);

  void ReceiveLoop(grpc::ServerCompletionQueue *complete_queue);

 protected:
  template <typename ServiceType, typename ServerContextType,
            typename RequestType, typename ResponseType>
  std::unique_ptr<ReceiveContext> BuildReceiveContext(
      std::function<void(ServiceType *, ServerContextType *, RequestType *,
                         grpc::ServerAsyncResponseWriter<ResponseType> *,
                         grpc::CompletionQueue *, grpc::ServerCompletionQueue *,
                         void *)>
          request_transmit,
      std::function<grpc::Status(const RequestType *, ResponseType *)>
          receive_transmit,
      grpc::ServerCompletionQueue *complete_queue);

 protected:
  std::future<void> lite_recevier_future_;
  std::future<void> shm_receiver_future_;
  std::vector<std::unique_ptr<grpc::ServerCompletionQueue>> completion_queues_;
  NodeService::AsyncService async_service_;

 private:
  NodeConfig node_config_;
  std::unique_ptr<grpc::Server> server_;

  std::atomic<bool> shutdown_{false};
  std::map<std::pair<std::string, std::string>, std::string> server_msgs_;
  std::function<void(const LiteHeader &, std::shared_ptr<LiteMsgWrapper>)>
      lite_callback_;
  std::function<void(const LiteHeader &, const Receiver::MetaData &,
                     const std::string &, const LidarParametersProto &)>
      shm_callback_;
  std::function<bool(const NodeQeuryRequest &, NodeQeuryResponse *)>
      query_cmd_callback_;
  std::future<void> query_cmd_receiver_future_;
  bool GetCompletionQueueNext(grpc::ServerCompletionQueue *complete_queue,
                              void **tag, bool *ok);
};

template <typename ServiceType, typename ServerContextType,
          typename RequestType, typename ResponseType>
std::unique_ptr<ReceiveContext> Receiver::BuildReceiveContext(
    std::function<void(ServiceType *, ServerContextType *, RequestType *,
                       grpc::ServerAsyncResponseWriter<ResponseType> *,
                       grpc::CompletionQueue *, grpc::ServerCompletionQueue *,
                       void *)>
        request_transmit,
    std::function<grpc::Status(const RequestType *, ResponseType *)>
        receive_transmit,
    grpc::ServerCompletionQueue *complete_queue) {
  return std::make_unique<
      ReceiveContextUnaryImpl<ServerContextType, RequestType, ResponseType>>(
      std::bind(request_transmit, &async_service_, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3, complete_queue,
                complete_queue, std::placeholders::_4),
      receive_transmit);
}

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_RECEIVER_RECEIVER_H_
