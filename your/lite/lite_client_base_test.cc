#include "onboard/lite/lite_client_base.h"

#include <functional>

#include "gmock/gmock.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "gtest/gtest.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

class LiteClientBaseTest : public LiteClientBase {
 public:
  virtual ~LiteClientBaseTest() {}

  explicit LiteClientBaseTest(LiteModuleName module_name,
                              folly::Executor *executor = nullptr,
                              const std::string &domain = "")
      : LiteClientBase(module_name, executor, domain) {}

  void SleepUntil(absl::Duration duration) override {}

 private:
  // return diff value by channel name
  bool PubInternal(
      const std::string &channel, const std::string &domain,
      std::unique_ptr<google::protobuf::Message> message) override {
    bool ret = true;
    if (channel == "test_channel_2") {
      ret = false;
    }
    return ret;
  }

  void SubscribeInternal(const std::string &channel, const std::string &domain,
                         std::unique_ptr<SubCallback> sub_callback) override {}

  void StopInternal() override {}

 protected:
  struct SimpleStruct {
    int size;
    std::array<double, 10> numbers;
    std::array<char, 10> chars;
    REGISTER_SHM_MSG(SimpleStruct)
  };
};

TEST(LiteClientBaseTest, SubscribeShmMsg) {
  LiteClientBaseTest client_base(TEST_LITE_MODULE_1);

  std::function<void(std::shared_ptr<ShmMessage>)> callback =
      [](std::shared_ptr<ShmMessage> msg) -> void { (void)msg; };

  client_base.SubscribeShmMsg(callback, "test_channel", "test_domain");
  EXPECT_EQ(
      client_base.GetSubDomainChannels().count("/test_domain/test_channel"),
      (int64_t)1);
}

TEST(LiteClientBaseTest, SubscribeMsgWithCallback) {
  LiteClientBaseTest client_base(TEST_LITE_MODULE_1);

  client_base.SubscribeMsgWithCallback("test_channel", "test_domain", nullptr);
  EXPECT_EQ(
      client_base.GetSubDomainChannels().count("/test_domain/test_channel"),
      (int64_t)1);
}

TEST(LiteClientBaseTest, PublishShmMsg) {
  LiteClientBaseTest client_base(TEST_LITE_MODULE_1);

  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<LiteClientBaseTest::SimpleStruct>(
          SHM_UNKNOWN, "shm_simple_struct");

  EXPECT_TRUE(client_base.PublishShmMsg(shm_stru1.get()).ok());
  EXPECT_TRUE(
      client_base.PublishShmMsg(shm_stru1.get(), "test_channel", "test_domain")
          .ok());
  EXPECT_FALSE(
      client_base
          .PublishShmMsg(shm_stru1.get(), "test_channel_2", "test_domain")
          .ok());
}

TEST(LiteClientBaseTest, ForwardLiteMsg) {
  LiteClientBaseTest client_base(TEST_LITE_MODULE_1);

  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));

  // check
  EXPECT_TRUE(client_base.ForwardLiteMsg(&lite_msg).ok());
  EXPECT_TRUE(
      client_base.ForwardLiteMsg("test_channel", "test_domain", &lite_msg)
          .ok());
  EXPECT_FALSE(
      client_base.ForwardLiteMsg("test_channel_2", "test_domain", &lite_msg)
          .ok());
}

TEST(LiteClientBaseTest, PublishLiteMsgWithMeta) {
  LiteClientBaseTest client_base(TEST_LITE_MODULE_1);

  LiteMsgWrapper lite_msg;
  LiteMsgSimpleProto test_data;
  test_data.mutable_header()->set_timestamp(123);

  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
      test_data, "lite_msg_simple_proto", &lite_msg));
  EXPECT_TRUE(
      client_base
          .PublishLiteMsgWithMeta("test_channel", "test_domain", &lite_msg)
          .ok());
  EXPECT_FALSE(
      client_base
          .PublishLiteMsgWithMeta("test_channel_2", "test_domain", &lite_msg)
          .ok());
}

}  // namespace qcraft
