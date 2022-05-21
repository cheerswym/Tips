#include "onboard/vis/vantage_forwarding_module.h"

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/vis/ark/ark_server/ark_client_man.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/spin_reader.h"
#include "onboard/lidar/spin_util.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/proto/lidar.pb.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/proto/port.pb.h"

DEFINE_string(
    onboard_forward_white_list_channels, "",
    "The channel names of lite msgs to be forwarded. When the flag is "
    "empty, all lite msgs will be forwarded.");
DEFINE_string(
    onboard_forward_black_list_channels,
    "raw_encoded_image,lidar_spin,decoded_image",
    "The channel names of lite msgs to filter out when running onboard.");
namespace qcraft {
namespace {
constexpr double kMessageStalenessThresholdUs = 200000;  // us
constexpr int kMaxNumLiteMessageInBuffer = 500;
constexpr int kMaxNumShmMessageInBuffer = 50;

bool IsNotEmpty(
    boost::circular_buffer<std::shared_ptr<const LiteMsgWrapper>> *queue) {
  return !queue->empty();
}

bool IsNotEmpty(
    boost::circular_buffer<
        std::pair<ShmMessageMetadata, std::shared_ptr<const ShmMessage>>>
        *queue) {
  return !queue->empty();
}

bool IsLiteChannel(const std::string &channel) {
  return channel.find("/") == std::string::npos;
}

}  // namespace

VantageForwardingModule::VantageForwardingModule(LiteClientBase *lite_client)
    : LiteModule(lite_client),
      lite_msg_queue_(kMaxNumLiteMessageInBuffer),
      shm_message_queue_(kMaxNumShmMessageInBuffer) {}

VantageForwardingModule::~VantageForwardingModule() {
  stop_notification_.Notify();
  shm_publisher_future_.wait();
  lite_publisher_future_.wait();
}

void VantageForwardingModule::OnInit() {
  {
    const RunParamsProtoV2 run_params = GetRunParams();
    vantage_client_man::CreateVantageClientMan(param_manager());
    update_run_params_ = vantage_client_man::UpdateRunParams(run_params);
    if (!IsOnboardMode()) {
      ark_client_man::CreateArkClientMan(param_manager());
      ark_client_man::UpdateRunParams(run_params);
    }
  }

  onboard_filter_white_list_channels_ =
      absl::StrSplit(FLAGS_onboard_forward_white_list_channels, ",",
                     [](absl::string_view &sp) {
                       sp = absl::StripAsciiWhitespace(sp);
                       return !sp.empty();
                     });
  onboard_filter_black_list_channels_ =
      absl::StrSplit(FLAGS_onboard_forward_black_list_channels, ",",
                     [](absl::string_view &sp) {
                       sp = absl::StripAsciiWhitespace(sp);
                       return !sp.empty();
                     });
}

void VantageForwardingModule::OnSubscribeChannels() {
  DisableInputOutputChecker();
  const auto config = (IsOnboardMode()) && !OnTestBenchForRsim()
                          ? GetAllModuleConfigs()
                          : LoadModuleConfigsFromAllDeclared();
  auto channel_domain_to_field_name =
      GetOutputsByAllNodesFromModuleConfigs(config);

  QCHECK(!channel_domain_to_field_name.empty()) << "No module configs provided";
  for (const auto &kv : channel_domain_to_field_name) {
    if (IsInLiteMsgBlackList(kv.first.first)) {
      VLOG(2) << "Field " << kv.first.first
              << " is in blacklist. Will not forward to Vantage.";
      continue;
    }

    // The channel names of lite msgs to be forwarded.
    if (!onboard_filter_white_list_channels_.empty() &&
        !ContainsKey(onboard_filter_white_list_channels_, kv.first.first)) {
      continue;
    }

    // skip filtered channel.
    if (ContainsKey(onboard_filter_black_list_channels_, kv.first.first)) {
      continue;
    }

    QLOG(INFO) << "Vantage forward subscribes " << kv.first.first;
    SubscribeMsgWithCallback(
        kv.first.first, kv.first.second, kv.second,
        [this](std::shared_ptr<const LiteMsgWrapper> lite_msg) {
          this->OnLiteMessage(lite_msg);
        },
        [this](std::shared_ptr<const ShmMessage> shm_msg) {
          this->OnShmMessage(shm_msg);
        });
  }
}

void VantageForwardingModule::OnSetUpTimers() {
  lite_publisher_future_ = std::async([&] {
    const auto duration = absl::Seconds(1);
    while (!stop_notification_.HasBeenNotified()) {
      std::shared_ptr<const LiteMsgWrapper> lite_msg;
      {
        absl::MutexLock m(&mutex_);
        if (!mutex_.AwaitWithTimeout(
                absl::Condition(IsNotEmpty, &lite_msg_queue_), duration)) {
          continue;
        }
        lite_msg = std::move(lite_msg_queue_.front());
        lite_msg_queue_.pop_front();
      }

      ForwardLiteMessage(*lite_msg);
    }
  });

  shm_publisher_future_ = std::async([&] {
    const auto duration = absl::Seconds(1);
    while (!stop_notification_.HasBeenNotified()) {
      ShmMessageMetadata shm_message_metadata;
      std::shared_ptr<const ShmMessage> shm_message;
      {
        absl::MutexLock m(&mutex_);
        if (!mutex_.AwaitWithTimeout(
                absl::Condition(IsNotEmpty, &shm_message_queue_), duration)) {
          continue;
        }
        std::tie(shm_message_metadata, shm_message) =
            shm_message_queue_.front();
        shm_message_queue_.pop_front();
        shm_message_cnt_by_type_[ShmMsgType_Name(
            shm_message_metadata.shm_msg_type())]--;
      }

      QCHECK_NOTNULL(shm_message);
      ForwardShmMessage(shm_message->shm_msg_metadata(), *shm_message);
    }
  });

  pull_user_input_runner_ =
      std::make_unique<PeriodicRunner>(absl::Milliseconds(100), true);
  pull_user_input_runner_->Start([this]() {
    std::vector<LiteMsgWrapper> lite_msgs;
    QCOUNTER_SPAN("PullUserInputMessages");
    vantage_client_man::PullUserInputMessages(&lite_msgs);
    if (!IsOnboardMode()) {
      ark_client_man::PullUserInputMessages(&lite_msgs);
    }
    for (auto &lite_msg : lite_msgs) {
      QLOG(INFO) << "Received user input messges: " << lite_msg.DebugString();
      QLOG_IF_NOT_OK(WARNING, ForwardLiteMsg(&lite_msg));
    }
  });
}

void VantageForwardingModule::OnLiteMessage(
    std::shared_ptr<const LiteMsgWrapper> lite_msg) {
  absl::MutexLock lock(&mutex_);

  latest_message_ts_ =
      LiteMsgConverter::Get().GetLiteMsgWrapperHeader(*lite_msg).timestamp();
  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(*lite_msg);
  if (!IsLiteChannel(channel_domain.first)) {
    return;
  }
  if (channel_domain.first == "update_run_params_proto") {
    LiteModule::UpdateRunParams(lite_msg->update_run_params_proto());

    const RunParamsProtoV2 run_params = GetRunParams();
    update_run_params_ = vantage_client_man::UpdateRunParams(run_params);
    if (!IsOnboardMode()) {
      ark_client_man::UpdateRunParams(run_params);
    }
  }

  if (lite_msg_queue_.size() == kMaxNumLiteMessageInBuffer) {
    static int lite_message_discarded = 0;
    lite_message_discarded++;
    QCOUNTER("lite_message_discarded", 1);
    QLOG_EVERY_N_SEC(WARNING, 3.0)
        << "lite_msg_queue_.size(): " << lite_msg_queue_.size()
        << " total discard lite_message: " << lite_message_discarded;
  }
  lite_msg_queue_.push_back(std::move(lite_msg));
}

void VantageForwardingModule::OnShmMessage(
    std::shared_ptr<const ShmMessage> shm_message) {
  absl::MutexLock lock(&mutex_);
  const ShmMessageMetadata &shm_message_metadata =
      shm_message->shm_msg_metadata();
  latest_message_ts_ = shm_message_metadata.header().timestamp();
  if (shm_message_queue_.size() == kMaxNumShmMessageInBuffer) {
    static int shm_message_discarded = 0;
    shm_message_discarded++;
    QCOUNTER("shm_message_discarded", 1);
    QLOG_EVERY_N_SEC(WARNING, 3.0)
        << "shm_message_queue_.size():" << shm_message_queue_.size()
        << "total discard shm_message: " << shm_message_discarded;
  }

  shm_message_queue_.push_back({shm_message_metadata, shm_message});
  shm_message_cnt_by_type_[ShmMsgType_Name(
      shm_message_metadata.shm_msg_type())]++;
}

void VantageForwardingModule::ForwardLiteMessage(
    const LiteMsgWrapper &lite_msg) {
  const uint64_t message_ts =
      LiteMsgConverter::Get().GetLiteMsgWrapperHeader(lite_msg).timestamp();
  // Drop stale messages to avoid lagging.
  if (IsOnboardMode() &&
      latest_message_ts_ > message_ts + kMessageStalenessThresholdUs) {
    return;
  }

  SCOPED_QTRACE("VantageForwardingModule::ForwardLiteMessage");
  vantage_client_man::UpdateLiteMessage(lite_msg);
  if (!IsOnboardMode()) {
    ark_client_man::UpdateLiteMessage(lite_msg);
  }
}

void VantageForwardingModule::ForwardShmMessage(
    const ShmMessageMetadata &shm_message_metadata,
    const ShmMessage &shm_message) {
  const uint64_t message_ts = shm_message_metadata.header().timestamp();
  // Drop stale messages to avoid lagging.
  if (latest_message_ts_ > message_ts + kMessageStalenessThresholdUs) {
    return;
  }

  int msgs_in_queue = 0;
  const auto shm_type = ShmMsgType_Name(shm_message_metadata.shm_msg_type());
  {
    absl::ReaderMutexLock l(&mutex_);
    msgs_in_queue = shm_message_cnt_by_type_[shm_type];
  }

  if (!update_run_params_) {
    const RunParamsProtoV2 run_params = GetRunParams();
    update_run_params_ = vantage_client_man::UpdateRunParams(run_params);
    if (!IsOnboardMode()) {
      ark_client_man::UpdateRunParams(run_params);
    }
  }

  SCOPED_QTRACE_ARG2("VantageForwardingModule::ForwardShmMessage", "shm_type",
                     shm_type, "msgs_in_queue", msgs_in_queue);
  update_run_params_ =
      vantage_client_man::UpdateShmMessage(shm_message_metadata, shm_message);
  if (!IsOnboardMode()) {
    ark_client_man::UpdateShmMessage(shm_message_metadata, shm_message);
  }
}

}  // namespace qcraft
