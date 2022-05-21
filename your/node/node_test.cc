#include "onboard/node/node.h"

#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gtest/gtest.h"
#include "onboard/conf/run_conf_man.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/transport/shm/shm_flags.h"
#include "onboard/lite/transport/shm/shm_manager.h"
#include "onboard/params/param_manager.h"

DECLARE_string(node_name);
DECLARE_string(name_space);
DECLARE_bool(enable_node_verbose_log);
DECLARE_bool(enable_query_remote_node);

// INFO(liyu): run 'sudo ntpdate cn.pool.ntp.org' to sync time clock on
// different node.
namespace qcraft {

namespace {
constexpr int32 kShmTagNumber = 6;
const char kLiteMsgData[] = "Publish LiteMsg, failed_issues = %05d";
const char kShmMsgData[] = "Publish ShmMsg, Image = %05d";
const auto kDummyIssueCallback =
    [](QIssueSeverity severity, QIssueType type, QIssueSubType sub_type,
       const std::string& message, const std::string& args_message) {};

class NodeTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_enable_node_verbose_log = true;
    FLAGS_node_name = "main_node";
    FLAGS_name_space = "omc";
    FLAGS_enable_query_remote_node = false;
    FLAGS_transport_managed_shared_memory_size = 32LL * 1024 * 1024;

    static std::once_flag flag;
    std::call_once(flag, []() {
      qcraft::shm::ShmManager::Instance()->DestroyAndCreateShm();
    });
  }

  virtual void TearDown() {}

 public:
  std::string CombineDomainChannel(const std::string& domain,
                                   const std::string& channel) {
    return absl::StrCat("/", domain, "/", channel);
  }

  void UpdateMetadata(LiteHeader* header, const std::string& channel,
                      const std::string& domain) {
    header->set_timestamp(absl::ToUnixMicros(absl::Now()));
    header->set_channel(channel);
    header->set_domain(domain);
    header->set_seq_number(seq_number_.fetch_add(1));
    header->set_module_id(
        static_cast<int32_t>(LiteModuleName::AUTONOMY_STATE_MODULE));
    header->set_tag_number(LiteMsgConverter::Get().GetTagNumber(channel));
  }

  template <typename T>
  void UpdateMetadata(LiteHeader* header, const std::string& channel,
                      const std::string& domain) {
    if (ProtoMessageToName<T>() != channel) {
      header->set_channel(channel);
    } else {
      header->set_channel("");
    }

    header->set_domain(domain);
    header->set_timestamp(absl::ToUnixMicros(absl::Now()));
    header->set_seq_number(seq_number_.fetch_add(1));
    header->set_module_id(
        static_cast<int32_t>(LiteModuleName::IMAGE_PUBLISHER_MODULE));
    header->set_tag_number(LiteMsgConverter::Get().GetTagNumber<T>());
  }

  void SubscribeMsgWithCallback(
      const std::string& channel, const std::string& domain,
      const std::string& field_name,
      std::function<void(std::shared_ptr<const LiteMsgWrapper>)> lite_callback,
      std::function<void(std::shared_ptr<const ShmMessage>)> shm_callback) {
    LOG(INFO) << "SubscribeMsgWithCallback::channel: " << channel
              << ", domain:" << domain;
    const auto domain_channel = CombineDomainChannel(domain, channel);

    msg_callbacks_[domain_channel].push_back(
        std::make_unique<LiteMsgWrapperCallback>(
            [lite_callback = std::move(lite_callback),
             shm_callback = std::move(shm_callback)](
                std::shared_ptr<const LiteMsgWrapper> lite_msg) {
              if (lite_msg->tag_number() == kShmTagNumber) {
                const ShmMessageMetadata& shm_message_metadata =
                    static_cast<const ShmMessageMetadata&>(
                        LiteMsgConverter::Get().GetLiteMsgField(*lite_msg));
                std::shared_ptr<const ShmMessage> shm_message =
                    ShmMessage::CreateToRead(shm_message_metadata);
                ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(
                    shm_message_metadata);
                if (shm_callback != nullptr) {
                  shm_callback(std::move(shm_message));
                }
              } else {
                lite_callback(std::move(lite_msg));
              }
            },
            field_name));
  }

  void PublishLiteMsg(size_t index) {
    AutonomyStateProto autonomy_state;
    autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
    auto* reason = autonomy_state.mutable_reason();

    std::string failed_issues = absl::StrFormat(kLiteMsgData, index);
    reason->add_failed_issues(failed_issues);

    std::string channel("autonomy_state_proto"), domain;
    UpdateMetadata(autonomy_state.mutable_header(), channel, domain);

    const auto domain_channel = CombineDomainChannel(domain, channel);
    for (auto& callback : msg_callbacks_[domain_channel]) {
      callback->Call(callback->ParseFrom(
          std::make_shared<AutonomyStateProto>(autonomy_state)));
    }
  }

  void PublishShmMsg(size_t index) {
    std::string channel("encoded_image"), domain;

    EncodedImageMetadata encoded_image_meta;
    encoded_image_meta.set_camera_id(CameraId::CAM_FRONT_LEFT);

    auto* image_info = encoded_image_meta.mutable_image_info();
    image_info->set_width(1920);
    image_info->set_height(1080);

    std::string encoded_image = absl::StrFormat(kShmMsgData, index);
    std::unique_ptr<ShmMessage> encoded_image_shm = ShmMessage::CreateToWrite(
        encoded_image.size(), SHM_ENCODED_IMAGE, channel);
    memcpy(encoded_image_shm->mutable_buffer(), encoded_image.data(),
           encoded_image.size());
    *encoded_image_shm->mutable_shm_msg_metadata()->MutableExtension(
        EncodedImageMetadata::encoded_image_meta) = encoded_image_meta;

    UpdateMetadata<ShmMessageMetadata>(
        encoded_image_shm->mutable_shm_msg_metadata()->mutable_header(),
        channel, domain);

    const auto domain_channel = CombineDomainChannel(domain, channel);
    for (auto& callback : msg_callbacks_[domain_channel]) {
      callback->Call(callback->ParseFrom(std::make_shared<ShmMessageMetadata>(
          encoded_image_shm->shm_msg_metadata())));
    }
  }

  void TestTransmit(int transmit_count, int freq) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    nodes_ =
        Node::CreateNodes(FLAGS_node_name, FLAGS_name_space, *param_manager);
    for (auto& node : nodes_) {
      node->OnInit(kDummyIssueCallback);
    }

    int lite_msg_count = 0, shm_msg_count = 0;
    for (auto& node : nodes_) {
      node->OnSubscribeChannels(
          std::bind(&NodeTest::SubscribeMsgWithCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4,
                    std::placeholders::_5),
          [&lite_msg_count](std::shared_ptr<LiteMsgWrapper> lite_msg) {
            ++lite_msg_count;

            EXPECT_TRUE(lite_msg->has_autonomy_state_proto());
            const auto& autonomy_state = lite_msg->autonomy_state_proto();
            QLOG_EVERY_N_SEC(INFO, 1.0)
                << autonomy_state.autonomy_state() << ", "
                << autonomy_state.reason().failed_issues(0);
          },
          [&shm_msg_count](std::shared_ptr<ShmMessage> shm_message) {
            ++shm_msg_count;

            std::string data;
            data.resize(shm_message->buffer_size());
            memcpy(data.data(), shm_message->buffer(),
                   shm_message->buffer_size());
            EXPECT_EQ(data.size(), sizeof(kShmMsgData));

            const auto& shm_message_metadata = shm_message->shm_msg_metadata();
            EXPECT_EQ(shm_message_metadata.shm_msg_type(), SHM_ENCODED_IMAGE);
            const auto& encoded_image_meta = shm_message_metadata.GetExtension(
                EncodedImageMetadata::encoded_image_meta);
            QLOG_EVERY_N_SEC(INFO, 1.0)
                << data.data() << ", "
                << encoded_image_meta.image_info().width() << ", "
                << encoded_image_meta.image_info().height();
          });
    }

    for (auto& node : nodes_) {
      node->OnSetUpTimers();
    }

    int sleep_ms = 1000 / freq * 1000;
    if (FLAGS_node_name == "main_node") {
      for (size_t index = 0; index < transmit_count; index++) {
        PublishLiteMsg(index);
        usleep(sleep_ms);
      }
      sleep(40);
    } else {
      // wait 10 seconds for node_1 startup
      sleep(10);
      for (size_t index = 0; index < transmit_count; index++) {
        PublishShmMsg(index);
        usleep(sleep_ms);
      }
      sleep(30);
    }

    QLOG(INFO) << "### lite_fail_rate: "
               << ((lite_msg_count != 0) ? (((transmit_count - lite_msg_count) *
                                             1.0 / transmit_count) *
                                            100)
                                         : 0)
               << "%";
    QLOG(INFO) << "### shm_fail_rate: "
               << ((shm_msg_count != 0)
                       ? (((transmit_count * 2 - shm_msg_count) * 1.0 /
                           (transmit_count * 2)) *
                          100)
                       : 0)
               << "%";
  }

 public:
  std::vector<std::unique_ptr<Node>> nodes_;
  std::unordered_map<std::string, std::vector<std::unique_ptr<SubCallback>>>
      msg_callbacks_;

  std::atomic<uint64_t> seq_number_ = 0;
};

TEST_F(NodeTest, TestInit) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  nodes_ = Node::CreateNodes(FLAGS_node_name, FLAGS_name_space, *param_manager);
  for (auto& node : nodes_) {
    node->OnInit(kDummyIssueCallback);
  }
  nodes_.clear();
}

TEST_F(NodeTest, TestSubscribeMsg) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  nodes_ = Node::CreateNodes(FLAGS_node_name, FLAGS_name_space, *param_manager);
  for (auto& node : nodes_) {
    node->OnInit(kDummyIssueCallback);
  }

  for (auto& node : nodes_) {
    node->OnSubscribeChannels(
        std::bind(&NodeTest::SubscribeMsgWithCallback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5),
        [](std::shared_ptr<LiteMsgWrapper> lite_msg) {
          LOG(INFO) << "Publish::LiteMsg";
        },
        [](std::shared_ptr<ShmMessage> shm_message) {
          LOG(INFO) << "Publish::ShmMsg";
        });
  }

  nodes_.clear();
}

// TEST_F(NodeTest, TestNormalTransimit) { TestTransmit(100, 10); }

// TEST_F(NodeTest, TestStressTransimit) { TestTransmit(10000, 200); }

}  // namespace
}  // namespace qcraft
