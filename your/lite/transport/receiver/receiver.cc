#include "onboard/lite/transport/receiver/receiver.h"

#include "onboard/camera/utils/camera_image.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"
#include "onboard/node/node_util.h"
#include "onboard/params/utils/param_util.h"

namespace qcraft {

namespace {
enum { RECEIVE_LITE, RECEIVE_SHM, RECEIVE_QUERY, RECEIVE_MAX };
}  // namespace

void Receiver::Enable() {
  grpc::ServerBuilder builder;
  std::string server_address("0.0.0.0:" + node_config_.node_port());
  builder.AddListeningPort(server_address.c_str(),
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&async_service_);

  for (size_t index = 0; index < RECEIVE_MAX; index++) {
    completion_queues_.push_back(builder.AddCompletionQueue());
  }

  server_ = builder.BuildAndStart();
  QCHECK(server_) << "Failed to call BuildAndStart";

  query_cmd_receiver_future_ = std::async(
      std::launch::async,
      [this, complete_queue = completion_queues_[RECEIVE_QUERY].get()] {
        const auto receive_context =
            BuildReceiveContext<NodeService::AsyncService, grpc::ServerContext,
                                QueryRemoteNodeRequest,
                                QueryRemoteNodeResponse>(
                &NodeService::AsyncService::RequestQueryRemoteNode,
                std::bind(&Receiver::ReceiveQueryCmd, this,
                          std::placeholders::_1, std::placeholders::_2),
                complete_queue);

        ReceiveLoop(complete_queue);
      });
}

void Receiver::Disable() {
  shutdown_ = true;
  server_->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(3));
  for (auto&& completion_queue : completion_queues_) {
    completion_queue->Shutdown();
  }

  lite_recevier_future_.wait();
  shm_receiver_future_.wait();
  query_cmd_receiver_future_.wait();

  for (auto&& completion_queue : completion_queues_) {
    bool ok;
    void* tag;
    while (completion_queue->Next(&tag, &ok)) {
    }
  }
}

void Receiver::SubscribLiteMsgCallback(
    const std::map<std::pair<std::string, std::string>, std::string>&
        server_msgs,
    const std::function<void(const LiteHeader&,
                             std::shared_ptr<LiteMsgWrapper>)>& callback) {
  server_msgs_ = server_msgs;
  lite_callback_ = callback;
}

void Receiver::SubscribShmMsgCallback(
    const std::function<void(const LiteHeader&, const Receiver::MetaData&,
                             const std::string&, const LidarParametersProto&)>&
        callback) {
  shm_callback_ = callback;
}

void Receiver::QueryCmdCallback(
    const std::function<bool(const NodeQeuryRequest&, NodeQeuryResponse*)>&
        callback) {
  query_cmd_callback_ = callback;
}

bool Receiver::GetCompletionQueueNext(
    grpc::ServerCompletionQueue* complete_queue, void** tag, bool* ok) {
  return complete_queue->Next(tag, ok);
}

void Receiver::ReceiveLoop(grpc::ServerCompletionQueue* complete_queue) {
  bool ok;
  void* tag;
  if (!GetCompletionQueueNext(complete_queue, &tag, &ok)) {
    return;
  }

  grpc::CompletionQueue::NextStatus status;
  ReceiveContext* receive_context;
  do {
    receive_context = static_cast<ReceiveContext*>(tag);
    if (shutdown_) {
      break;
    }

    SCOPED_QTRACE("DoThenAsyncNext");
    status = complete_queue->DoThenAsyncNext(
        [&, receive_context, ok]() {
          SCOPED_QTRACE("NextState");
          if (!receive_context->NextState(ok)) {
            SCOPED_QTRACE("Reset");
            receive_context->Reset();
          }
        },
        &tag, &ok, gpr_inf_future(GPR_CLOCK_REALTIME));
  } while (status != grpc::CompletionQueue::SHUTDOWN);
}

grpc::Status Receiver::ReceiveLiteMsg(const TransmitLiteMsgRequest* request,
                                      TransmitLiteMsgResponse* response) {
  auto lite_msg = std::make_shared<LiteMsgWrapper>(request->lite_message());
  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(*lite_msg);
  SCOPED_QTRACE_ARG1("ReceiveLiteMsg", "channel_domain", channel_domain.first);
  if (server_msgs_.find(channel_domain) == server_msgs_.end()) {
    return grpc::Status(grpc::StatusCode::NOT_FOUND,
                        "Node: " + node_config_.node_name() +
                            " don't use this channel: " + channel_domain.first);
  }

  if (lite_callback_) {
    LiteMsgConverter::Get()
        .GetMutableLiteMsgWrapperHeader(lite_msg.get())
        ->set_full_node_name(request->header().full_node_name());
    lite_callback_(request->header(), lite_msg);
  }

  response->set_success(true);
  return grpc::Status::OK;
}

grpc::Status Receiver::ReceiveShmMsg(const TransmitShmMsgRequest* request,
                                     TransmitShmMsgResponse* response) {
  const auto shm_msg_type = request->shm_msg_type();
  SCOPED_QTRACE_ARG1("ReceiveShmMsg", "shm_msg_type", shm_msg_type);
  if (shm_callback_) {
    switch (shm_msg_type) {
      case SHM_ENCODED_LIDAR_FRAME: {
        shm_callback_(request->header(), request->encoded_lidar_frame_meta(),
                      request->data(),
                      param_util::ConvertLidarParams(request->lidar_params()));
      } break;
      case SHM_ENCODED_IMAGE: {
        shm_callback_(request->header(), request->encoded_image_meta(),
                      request->data(), LidarParametersProto());
      } break;
      case SHM_LIDAR_FRAME:
      case SHM_DECODED_IMAGE:
      case SHM_UNKNOWN:
      case SHM_UNUSED:
        QLOG(FATAL) << "Unexpected shared memory message type.";
        break;
    }
  }

  response->set_success(true);
  return grpc::Status::OK;
}

grpc::Status Receiver::ReceiveQueryCmd(const QueryRemoteNodeRequest* request,
                                       QueryRemoteNodeResponse* response) {
  if (!query_cmd_callback_) {
    return grpc::Status(grpc::StatusCode::NOT_FOUND,
                        "Node: " + node_config_.node_name() +
                            " had not provided query_cmd_callback_");
  }
  NodeQeuryRequest cb_request;
  NodeQeuryResponse cb_response;
  cb_request.full_node_name = request->full_node_name();
  switch (request->query_type()) {
    case QueryRemoteNodeRequest::QUERY_NODE_INPUTS:
      cb_request.query_cmd = NodeQueryCmdType::QUERY_NODE_INPUTS;
      break;
    default:
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                          "Node " + node_config_.node_name() +
                              " can not recognize the query_type");
  }
  bool status = query_cmd_callback_(cb_request, &cb_response);
  if (status) {
    response->set_full_node_name(GetFullNodeName(node_config_));
    response->set_success(true);
    for (const auto& msg_decl : cb_response.node_inputs) {
      auto* tmp_ent = response->add_node_inputs();
      tmp_ent->set_field_name(msg_decl.field_name);
      tmp_ent->set_channel(msg_decl.channel);
      tmp_ent->set_domain(msg_decl.domain);
    }
  }
  return grpc::Status::OK;
}

}  // namespace qcraft
