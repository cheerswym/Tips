#include "onboard/lite/transport/receiver/grpc_receiver.h"

#include <cstring>
#include <functional>
#include <iostream>

#include "cpp_free_mock/cpp_free_mock.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/transport/receiver/receiver.h"

namespace qcraft {

template <typename T, typename... U>
size_t GetAddress(std::function<T(U...)> f) {
  typedef T(fnType)(U...);
  fnType** fnPointer = f.template target<fnType*>();
  return static_cast<size_t>(*fnPointer);
}

class GRPCReceiverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    NodeConfig node_config;
    node_config.set_node_name("test_node");
    node_config.set_node_ip("127.0.0.1");
    node_config.set_node_port("10001");
    receiver_ = std::make_unique<GRPCReceiver>(node_config);
  }

  void TearDown() override { CLEAR_MOCKER(); }
  std::unique_ptr<GRPCReceiver> receiver_;
};

TEST_F(GRPCReceiverTest, RegCb) {
  receiver_->Enable();
  std::map<std::pair<std::string, std::string>, std::string>
      subscribe_server_msgs;
  std::function<void(const LiteHeader&, std::shared_ptr<LiteMsgWrapper>)>
      lite_callback([](const LiteHeader& header,
                       std::shared_ptr<LiteMsgWrapper> lite_msg) {});
  EXPECT_EQ(receiver_->lite_callback_, nullptr);
  receiver_->SubscribLiteMsgCallback(subscribe_server_msgs, lite_callback);
  EXPECT_NE(receiver_->lite_callback_, nullptr);

  std::function<void(const LiteHeader&, const Receiver::MetaData&,
                     const std::string&, const LidarParametersProto&)>
      shm_callback([](const LiteHeader& header, const Receiver::MetaData& data,
                      const std::string& str,
                      const LidarParametersProto& proto) {});
  EXPECT_EQ(receiver_->shm_callback_, nullptr);
  receiver_->SubscribShmMsgCallback(shm_callback);
  EXPECT_NE(receiver_->shm_callback_, nullptr);

  std::function<bool(const NodeQeuryRequest&, NodeQeuryResponse*)>
  query_cmd_callback([](const NodeQeuryRequest& request,
                        NodeQeuryResponse* response) -> bool { return true; });
  EXPECT_EQ(receiver_->query_cmd_callback_, nullptr);
  receiver_->QueryCmdCallback(query_cmd_callback);
  EXPECT_NE(receiver_->query_cmd_callback_, nullptr);
}

TEST_F(GRPCReceiverTest, ReceiveLoop) {
  receiver_->Enable();
  grpc::ServerCompletionQueue complete_queue;

  absl::SleepFor(absl::Seconds(1));
  auto mocker = MOCKER(&GRPCReceiver::GetCompletionQueueNext);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(receiver_.get(), testing::_, testing::_,
                                     testing::_))
      .Times(testing::Exactly(2))
      .WillOnce(testing::Return(false))
      .WillRepeatedly(testing::Return(true));
  receiver_->ReceiveLoop(&complete_queue);

  receiver_->shutdown_ = true;
  receiver_->ReceiveLoop(&complete_queue);
}

TEST_F(GRPCReceiverTest, ReceiveLiteMsg) {
  receiver_->Enable();

  TransmitLiteMsgRequest request;
  TransmitLiteMsgResponse response;
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;

  test_data.mutable_header()->set_timestamp(123);
  test_data.mutable_header()->set_channel("test_channel");
  test_data.mutable_header()->set_domain("test_domain");
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  request.mutable_lite_message()->CopyFrom(lite_msg);
  request.mutable_header()->set_channel("test_channel");
  request.mutable_header()->set_domain("test_domain");
  request.mutable_header()->set_full_node_name("test_node");
  EXPECT_EQ(grpc::StatusCode::NOT_FOUND,
            receiver_->ReceiveLiteMsg(&request, &response).error_code());

  std::function<void(const LiteHeader&, std::shared_ptr<LiteMsgWrapper>)>
      lite_callback([](const LiteHeader& header,
                       std::shared_ptr<LiteMsgWrapper> lite_msg) {});
  std::map<std::pair<std::string, std::string>, std::string>
      subscribe_server_msgs;

  subscribe_server_msgs.insert(
      std::make_pair(std::make_pair("test_channel", "test_domain"), ""));
  receiver_->SubscribLiteMsgCallback(subscribe_server_msgs, lite_callback);
  EXPECT_TRUE(receiver_->ReceiveLiteMsg(&request, &response).ok());
}

TEST_F(GRPCReceiverTest, ReceiveShmMsg) {
  receiver_->Enable();

  TransmitShmMsgRequest request;
  TransmitShmMsgResponse response;

  std::function<void(const LiteHeader&, const Receiver::MetaData&,
                     const std::string&, const LidarParametersProto&)>
      shm_callback([](const LiteHeader& header, const Receiver::MetaData& data,
                      const std::string& str,
                      const LidarParametersProto& proto) {});
  EXPECT_EQ(receiver_->shm_callback_, nullptr);
  receiver_->SubscribShmMsgCallback(shm_callback);
  EXPECT_NE(receiver_->shm_callback_, nullptr);

  request.set_shm_msg_type(SHM_ENCODED_LIDAR_FRAME);
  EXPECT_TRUE(receiver_->ReceiveShmMsg(&request, &response).ok());
  request.set_shm_msg_type(SHM_LIDAR_FRAME);
  EXPECT_DEATH(receiver_->ReceiveShmMsg(&request, &response), ".*");
  request.set_shm_msg_type(SHM_ENCODED_IMAGE);
  EXPECT_TRUE(receiver_->ReceiveShmMsg(&request, &response).ok());
  request.set_shm_msg_type(SHM_DECODED_IMAGE);
  EXPECT_DEATH(receiver_->ReceiveShmMsg(&request, &response), ".*");
  request.set_shm_msg_type(SHM_UNKNOWN);
  EXPECT_DEATH(receiver_->ReceiveShmMsg(&request, &response), ".*");
  request.set_shm_msg_type(SHM_UNKNOWN);
  EXPECT_DEATH(receiver_->ReceiveShmMsg(&request, &response), ".*");
}

TEST_F(GRPCReceiverTest, ReceiveQueryCmdOk) {
  QueryRemoteNodeRequest request;
  QueryRemoteNodeResponse response;

  receiver_->Enable();
  EXPECT_EQ(grpc::StatusCode::NOT_FOUND,
            receiver_->ReceiveQueryCmd(&request, &response).error_code());

  std::function<bool(const NodeQeuryRequest&, NodeQeuryResponse*)>
  query_cmd_callback(
      [](const NodeQeuryRequest& request, NodeQeuryResponse* response) -> bool {
        struct NodeInputMsgDecl data = {
            .field_name = "test_filed",
            .channel = "test_channel",
            .domain = "test_domain",
        };
        response->node_inputs.push_back(data);
        return true;
      });
  EXPECT_EQ(receiver_->query_cmd_callback_, nullptr);
  receiver_->QueryCmdCallback(query_cmd_callback);

  request.set_full_node_name("test_node");
  request.set_query_type(QueryRemoteNodeRequest::QUERY_NODE_INPUTS);
  EXPECT_TRUE(receiver_->ReceiveQueryCmd(&request, &response).ok());
}

TEST_F(GRPCReceiverTest, Context) {
  receiver_->Enable();

  std::unique_ptr<ReceiveContextUnaryImpl<
      grpc::ServerContext, TransmitLiteMsgRequest, TransmitLiteMsgResponse>>
      context;
  grpc::ServerCompletionQueue complete_queue;

  auto mocker =
      MOCKER(&grpc::Service::RequestAsyncUnary<qcraft::TransmitLiteMsgRequest>);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(&receiver_->async_service_, testing::_,
                                     testing::_, testing::_, testing::_,
                                     testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return());

  ::grpc::Status mock_data(::grpc::StatusCode::OK, "test message");
  auto mocker_recv = MOCKER(&qcraft::GRPCReceiver::ReceiveLiteMsg);
  EXPECT_CALL(*mocker_recv,
              MOCK_FUNCTION(receiver_.get(), testing::_, testing::_))
      .WillRepeatedly(testing::Return(mock_data));

  auto f0 =
      [&complete_queue, this](
          ::grpc::ServerContext* context,
          ::qcraft::TransmitLiteMsgRequest* request,
          grpc::ServerAsyncResponseWriter<::qcraft::TransmitLiteMsgResponse>*
              response,
          void* tag) {
        receiver_->async_service_.RequestTransmitLiteMsg(
            context, request, response, &complete_queue, &complete_queue, tag);
      };
  auto f1 = [this](const ::qcraft::TransmitLiteMsgRequest* request,
                   ::qcraft::TransmitLiteMsgResponse* response) {
    return receiver_->ReceiveLiteMsg(request, response);
  };

  context = std::make_unique<ReceiveContextUnaryImpl<
      grpc::ServerContext, TransmitLiteMsgRequest, TransmitLiteMsgResponse>>(
      f0, f1);
  EXPECT_NE(context, nullptr);

  auto mocker_trans = MOCKER(&grpc::ServerAsyncResponseWriter<
                             qcraft::TransmitLiteMsgResponse>::Finish);
  EXPECT_CALL(*mocker_trans, MOCK_FUNCTION(&context->response_transmit_,
                                           testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return());

  EXPECT_FALSE(context->Transimit(false));
  EXPECT_TRUE(context->Transimit(true));
  EXPECT_FALSE(context->Complete(false));
  EXPECT_FALSE(context->NextState(false));
  context->Reset();
}

}  // namespace qcraft
