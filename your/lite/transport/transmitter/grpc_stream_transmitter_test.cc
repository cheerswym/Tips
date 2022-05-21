#include "onboard/lite/transport/transmitter/grpc_stream_transmitter.h"

#include "cpp_free_mock/cpp_free_mock.h"
#include "gmock/gmock.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_message_util.h"

DECLARE_string(q_run_mode);

namespace qcraft {

class GRPCStreamTransTest : public ::testing::Test {
 protected:
  void SetUp() override {
    NodeConfig node_config;
    RunParamsProtoV2 run_param;
    transmitter =
        std::make_unique<GRPCStreamTransmitter>(node_config, run_param);
  }

  void TearDown() override { CLEAR_MOCKER(); }
  std::unique_ptr<GRPCStreamTransmitter> transmitter;
};

TEST_F(GRPCStreamTransTest, LiteMsgTransmit) {
  EXPECT_NE(transmitter, nullptr);
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));
  transmitter->Enable();
  EXPECT_FALSE(transmitter->Transmit(lite_msg));

  FLAGS_q_run_mode = "onboard";
  EXPECT_FALSE(transmitter->Transmit(lite_msg));
}

TEST_F(GRPCStreamTransTest, ShmMsg) {
  EXPECT_NE(transmitter, nullptr);
  auto shm_msg = ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "shm_test");

  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);
  transmitter->Enable();
  EXPECT_FALSE(transmitter->Transmit(*shm_msg));

  FLAGS_q_run_mode = "onboard";
  EXPECT_FALSE(transmitter->Transmit(*shm_msg));
}

TEST_F(GRPCStreamTransTest, SendQueryCmd) {
  NodeQeuryRequest request;
  NodeQeuryResponse response;

  request.full_node_name = "test_node_input";
  request.query_cmd = (enum NodeQueryCmdType)1;
  EXPECT_FALSE(transmitter->SendQueryCmd(request, &response));
}

TEST_F(GRPCStreamTransTest, BuildTransmitShmMsgRequest) {
  TransmitShmMsgRequest request;

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "shm_test1");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      EncodedImageMetadata::encoded_image_meta);
  EXPECT_TRUE(transmitter->BuildTransmitShmMsgRequest(*shm_msg, &request));
}

TEST_F(GRPCStreamTransTest, UpdateRequestHeader) {
  LiteHeader origin;
  LiteHeader header;

  EXPECT_TRUE(transmitter->UpdateRequestHeader(origin, &header));

  FLAGS_q_run_mode = "onboard";
  EXPECT_TRUE(transmitter->UpdateRequestHeader(origin, &header));

  origin.set_full_node_name("test_node_input");
  EXPECT_FALSE(transmitter->UpdateRequestHeader(origin, &header));
}

TEST_F(GRPCStreamTransTest, LiteMsgWrapper) {
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  auto mocker = MOCKER(&GRPCStreamTransmitter::BuildTransmitLiteMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(false));
  transmitter->Enable();

  TransmitLiteMsgRequest request;
  EXPECT_TRUE(transmitter->Transmit(lite_msg));
  mocker->RestoreToReal();

  testing::Mock::VerifyAndClearExpectations(transmitter.get());
}

TEST_F(GRPCStreamTransTest, SendQueryCmdHookFalse) {
  NodeQeuryRequest request;
  NodeQeuryResponse response;

  ::grpc::Status mock_data(::grpc::StatusCode::UNKNOWN, "test message");
  auto mocker = MOCKER(&GRPCStreamTransmitter::QueryRemoteNodeStatus);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_,
                                     testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(mock_data));
  request.full_node_name = "test_node_input";
  request.query_cmd = NodeQueryCmdType::QUERY_NODE_INPUTS;
  EXPECT_FALSE(transmitter->SendQueryCmd(request, &response));
}

TEST_F(GRPCStreamTransTest, SendQueryCmdHookOk) {
  NodeQeuryRequest request;
  NodeQeuryResponse response;

  QueryRemoteNodeResponse rpc_response;
  rpc_response.set_success(true);
  rpc_response.set_full_node_name("test_node_input");

  QueryRemoteNodeResponse_NodeInput node_input;
  node_input.set_field_name("test_file");
  node_input.set_channel("test_channel");
  node_input.set_domain("test_domain");
  rpc_response.add_node_inputs()->CopyFrom(node_input);

  ::grpc::Status mock_data(::grpc::StatusCode::OK, "test message");
  auto mocker = MOCKER(&GRPCStreamTransmitter::QueryRemoteNodeStatus);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_,
                                     testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::DoAll(testing::SetArgPointee<3>(rpc_response),
                               testing::Return(mock_data)));
  request.full_node_name = "test_node_input";
  request.query_cmd = NodeQueryCmdType::QUERY_NODE_INPUTS;
  EXPECT_TRUE(transmitter->SendQueryCmd(request, &response));
}

TEST_F(GRPCStreamTransTest, ShmMsgMock) {
  EXPECT_NE(transmitter, nullptr);

  auto mocker = MOCKER(&GRPCStreamTransmitter::BuildTransmitShmMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(false));

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "shm_test6");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);
  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(*shm_msg));
  mocker->RestoreToReal();

  testing::Mock::VerifyAndClearExpectations(transmitter.get());
}

TEST_F(GRPCStreamTransTest, BuildTransmitShmMsgRequestHookEncodeLidar) {
  TransmitShmMsgRequest request;
  LidarParametersProto data;

  auto mocker = MOCKER((LidarParametersProto(*)(
      const RunParamsProtoV2 &, LidarId))param_util::GetLidarParams);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillRepeatedly(testing::Return(data));

  auto shm_msg = ShmMessage::CreateToWrite(1024, SHM_ENCODED_LIDAR_FRAME,
                                           "shm_test_hook3");
  EXPECT_TRUE(transmitter->BuildTransmitShmMsgRequest(*shm_msg, &request));
}

}  // namespace qcraft
