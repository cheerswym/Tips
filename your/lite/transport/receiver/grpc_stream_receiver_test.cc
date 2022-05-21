#include "onboard/lite/transport/receiver/grpc_stream_receiver.h"

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

class GRPCStreamReceiverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    NodeConfig node_config;
    node_config.set_node_name("test_node");
    node_config.set_node_ip("127.0.0.1");
    node_config.set_node_port("10001");
    receiver_ = std::make_unique<GRPCStreamReceiver>(node_config);
  }

  void TearDown() override { CLEAR_MOCKER(); }
  std::unique_ptr<GRPCStreamReceiver> receiver_;
};

TEST_F(GRPCStreamReceiverTest, StreamContext) {
  receiver_->Enable();

  std::unique_ptr<ReceiveContextUnaryStreamImpl<
      grpc::ServerContext, TransmitLiteMsgRequest, TransmitLiteMsgResponse>>
      context;
  grpc::ServerCompletionQueue complete_queue;

  auto mocker = MOCKER(&grpc::Service::RequestAsyncBidiStreaming);
  EXPECT_CALL(*mocker,
              MOCK_FUNCTION(&receiver_->async_service_, testing::_, testing::_,
                            testing::_, testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return());

  auto f0 = [&complete_queue, this](
                ::grpc::ServerContext* context,
                ::grpc::ServerAsyncReaderWriter<qcraft::TransmitLiteMsgResponse,
                                                qcraft::TransmitLiteMsgRequest>*
                    stream,
                void* tag) {
    receiver_->async_service_.RequestTransmitLiteMsgStreamed(
        context, stream, &complete_queue, &complete_queue, tag);
  };
  auto f1 = [this](const ::qcraft::TransmitLiteMsgRequest* request,
                   ::qcraft::TransmitLiteMsgResponse* response) {
    return receiver_->ReceiveLiteMsg(request, response);
  };

  context = std::make_unique<ReceiveContextUnaryStreamImpl<
      grpc::ServerContext, TransmitLiteMsgRequest, TransmitLiteMsgResponse>>(
      f0, f1);
  EXPECT_NE(context, nullptr);

  auto mocker_trans = MOCKER(&grpc::internal::Call::PerformOps);
  EXPECT_CALL(*mocker_trans,
              MOCK_FUNCTION(&context->stream_->call_, testing::_))
      .WillRepeatedly(testing::Return());
  ::grpc::Status mock_data(::grpc::StatusCode::OK, "test message");
  auto mocker_recv = MOCKER(&qcraft::GRPCStreamReceiver::ReceiveLiteMsg);
  EXPECT_CALL(*mocker_recv,
              MOCK_FUNCTION(receiver_.get(), testing::_, testing::_))
      .WillRepeatedly(testing::Return(mock_data));

  EXPECT_FALSE(context->RequestTransmit(false));
  EXPECT_FALSE(context->ReceiveTransmit(false));
  EXPECT_FALSE(context->NextState(false));
  EXPECT_TRUE(context->RequestTransmit(true));
  EXPECT_TRUE(context->ReceiveTransmit(true));
  context->Reset();
}

}  // namespace qcraft
