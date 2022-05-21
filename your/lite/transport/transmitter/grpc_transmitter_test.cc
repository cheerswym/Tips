#include "onboard/lite/transport/transmitter/grpc_transmitter.h"

#include <memory>

#include "cpp_free_mock/cpp_free_mock.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_message_util.h"

DECLARE_string(q_run_mode);

namespace qcraft {

class GRPCTransTest : public ::testing::Test {
 protected:
  void SetUp() override {
    NodeConfig node_config;
    RunParamsProtoV2 run_param;

    node_config.set_timeout(10);
    transmitter = std::make_unique<GRPCTransmitter>(node_config, run_param);
  }

  void TearDown() override { CLEAR_MOCKER(); }
  std::unique_ptr<GRPCTransmitter> transmitter;
};

TEST_F(GRPCTransTest, LiteMsgTransmit) {
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

TEST_F(GRPCTransTest, LiteMsgTransmit2) {
  EXPECT_NE(transmitter, nullptr);
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;

  test_data.mutable_header()->set_timestamp(123);
  test_data.mutable_header()->set_channel("trace_proto");
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  transmitter->Enable();
  EXPECT_FALSE(transmitter->Transmit(lite_msg));
}

TEST_F(GRPCTransTest, ShmMsg) {
  EXPECT_NE(transmitter, nullptr);
  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "trans_test");

  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);
  transmitter->Enable();
  EXPECT_FALSE(transmitter->Transmit(*shm_msg));

  FLAGS_q_run_mode = "onboard";
  EXPECT_FALSE(transmitter->Transmit(*shm_msg));
}

TEST_F(GRPCTransTest, SendQueryCmd) {
  NodeQeuryRequest request;
  NodeQeuryResponse response;

  request.full_node_name = "test_node_input";
  request.query_cmd = (enum NodeQueryCmdType)1;

  EXPECT_FALSE(transmitter->SendQueryCmd(request, &response));
}

TEST_F(GRPCTransTest, BuildTransmitShmMsgRequest) {
  TransmitShmMsgRequest request;

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "trans_test2");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      EncodedImageMetadata::encoded_image_meta);
  EXPECT_TRUE(transmitter->BuildTransmitShmMsgRequest(*shm_msg, &request));

  auto shm_msg3 =
      ShmMessage::CreateToWrite(256, SHM_DECODED_IMAGE, "decoded_image", "");
  EXPECT_DEATH(transmitter->BuildTransmitShmMsgRequest(*shm_msg3, &request),
               ".*");

  auto shm_msg4 = ShmMessage::CreateToWrite(1024, SHM_UNKNOWN, "trans_test4");
  EXPECT_DEATH(transmitter->BuildTransmitShmMsgRequest(*shm_msg4, &request),
               ".*");

  auto shm_msg5 = ShmMessage::CreateToWrite(1024, SHM_UNUSED, "trans_test5");
  EXPECT_DEATH(transmitter->BuildTransmitShmMsgRequest(*shm_msg5, &request),
               ".*");
}

TEST_F(GRPCTransTest, UpdateRequestHeader) {
  LiteHeader origin;
  LiteHeader header;

  EXPECT_TRUE(transmitter->UpdateRequestHeader(origin, &header));

  FLAGS_q_run_mode = "onboard";
  EXPECT_TRUE(transmitter->UpdateRequestHeader(origin, &header));

  origin.set_full_node_name("test_node_input");
  EXPECT_FALSE(transmitter->UpdateRequestHeader(origin, &header));
}

TEST_F(GRPCTransTest, LiteMsgWrapper) {
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitLiteMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(false));

  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(lite_msg));
  mocker->RestoreToReal();

  testing::Mock::VerifyAndClearExpectations(transmitter.get());
}

TEST_F(GRPCTransTest, LiteMsgWrapperHookError) {
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitLiteMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(true));

  TransmitLiteMsgResponse rpc_response;
  rpc_response.set_success(true);
  ::grpc::Status mock_data(::grpc::StatusCode::DEADLINE_EXCEEDED,
                           "test message");
  auto mocker_status = MOCKER(&GRPCTransmitter::GrpcTransmitLiteMsg);
  EXPECT_CALL(*mocker_status, MOCK_FUNCTION(transmitter.get(), testing::_,
                                            testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::DoAll(testing::SetArgPointee<3>(rpc_response),
                               testing::Return(mock_data)));
  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(lite_msg));
  mocker->RestoreToReal();

  testing::Mock::VerifyAndClearExpectations(transmitter.get());
}

TEST_F(GRPCTransTest, LiteMsgWrapperHookOk) {
  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitLiteMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(true));

  TransmitLiteMsgResponse rpc_response;
  rpc_response.set_success(true);
  ::grpc::Status mock_data(::grpc::StatusCode::OK, "test message");
  auto mocker_status = MOCKER(&GRPCTransmitter::GrpcTransmitLiteMsg);
  EXPECT_CALL(*mocker_status, MOCK_FUNCTION(transmitter.get(), testing::_,
                                            testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::DoAll(testing::SetArgPointee<3>(rpc_response),
                               testing::Return(mock_data)));
  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(lite_msg));
  mocker->RestoreToReal();

  testing::Mock::VerifyAndClearExpectations(transmitter.get());
}

TEST_F(GRPCTransTest, ShmMsgHookRequest) {
  EXPECT_NE(transmitter, nullptr);

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitShmMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(false));

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "trans_test2");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);

  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(*shm_msg));
}

TEST_F(GRPCTransTest, ShmMsgHookOk) {
  EXPECT_NE(transmitter, nullptr);

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitShmMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(true));

  TransmitShmMsgResponse rpc_response;
  rpc_response.set_success(true);
  ::grpc::Status mock_data(::grpc::StatusCode::OK, "test message");
  auto mocker_status = MOCKER(&GRPCTransmitter::GrpcTransmitShmMsg);
  EXPECT_CALL(*mocker_status, MOCK_FUNCTION(transmitter.get(), testing::_,
                                            testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::DoAll(testing::SetArgPointee<3>(rpc_response),
                               testing::Return(mock_data)));

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "trans_test4");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);

  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(*shm_msg));
}

TEST_F(GRPCTransTest, ShmMsgHookError) {
  EXPECT_NE(transmitter, nullptr);

  auto mocker = MOCKER(&GRPCTransmitter::BuildTransmitShmMsgRequest);
  EXPECT_CALL(*mocker, MOCK_FUNCTION(transmitter.get(), testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::Return(true));

  TransmitShmMsgResponse rpc_response;
  rpc_response.set_success(true);
  ::grpc::Status mock_data(::grpc::StatusCode::DEADLINE_EXCEEDED,
                           "test message");
  auto mocker_status = MOCKER(&GRPCTransmitter::GrpcTransmitShmMsg);
  EXPECT_CALL(*mocker_status, MOCK_FUNCTION(transmitter.get(), testing::_,
                                            testing::_, testing::_))
      .Times(testing::Exactly(1))
      .WillOnce(testing::DoAll(testing::SetArgPointee<3>(rpc_response),
                               testing::Return(mock_data)));

  auto shm_msg =
      ShmMessage::CreateToWrite(1024, SHM_ENCODED_IMAGE, "trans_test4");
  shm_msg->mutable_shm_msg_metadata()->MutableExtension(
      qcraft::EncodedImageMetadata::encoded_image_meta);

  transmitter->Enable();
  EXPECT_TRUE(transmitter->Transmit(*shm_msg));
}

}  // namespace qcraft
