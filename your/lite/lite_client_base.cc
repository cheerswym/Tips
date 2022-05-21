#include "onboard/lite/lite_client_base.h"

namespace qcraft {

LiteClientBase::~LiteClientBase() {}

LiteClientBase::LiteClientBase(LiteModuleName module_name,
                               folly::Executor* executor,
                               const std::string& domain, bool is_sim)
    : module_name_(module_name),
      executor_(executor),
      domain_(domain),
      seq_number_(0),
      check_input_output_message_(false),
      skip_updating_metadata_(false),
      is_sim_(is_sim) {
  timer_manager_ = std::make_unique<TimerManager>();
  callback_manager_ = std::make_unique<CallbackManager>(executor);
}

void LiteClientBase::SubscribeShmMsg(
    std::function<void(std::shared_ptr<ShmMessage>)> shm_cb,
    const std::string& channel, const std::string& domain) {
  std::function<void(std::shared_ptr<const ShmMessageMetadata>)> cb =
      [cb = std::move(shm_cb)](std::shared_ptr<const ShmMessageMetadata> msg) {
        std::shared_ptr<ShmMessage> shm_msg = ShmMessage::CreateToRead(*msg);
        if (shm_msg == nullptr) {
          LOG(ERROR) << "shm_msg is nullptr";
        } else {
          ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(
              shm_msg->shm_msg_metadata());
          cb(std::move(shm_msg));
        }
      };
  SubscribeInternalWithChecker(
      channel, domain,
      std::make_unique<LiteMsgCallback<ShmMessageMetadata>>(std::move(cb)));
  // this->Subscribe(std::move(cb)).SetChannel(channel).SetDomain(domain);
}

void LiteClientBase::SubscribeMsgWithCallback(
    const std::string& channel, const std::string& domain,
    std::unique_ptr<SubCallback> sub_callback) {
  SubscribeInternalWithChecker(channel, domain, std::move(sub_callback));
}

// Share memory publisher and subcriber, won't own the pointer.
absl::StatusOr<LiteHeader> LiteClientBase::PublishShmMsg(ShmMessage* shm_msg) {
  const std::string& channel = shm_msg->shm_msg_metadata().header().channel();
  const std::string& domain = shm_msg->shm_msg_metadata().header().domain();
  return PublishShmMsg(shm_msg, channel, domain);
}

absl::StatusOr<LiteHeader> LiteClientBase::PublishShmMsg(
    ShmMessage* shm_msg, const std::string& channel,
    const std::string& domain) {
  // Update metadata before publish.
  UpdateMetadata<ShmMessageMetadata>(
      shm_msg->mutable_shm_msg_metadata()->mutable_header(), channel, domain_);
  shm_msg->mutable_shm_msg_metadata()->mutable_header()->set_is_shm_msg(true);
  // Before send, update share memory manager.
  ShmFactory::GetShmManager()->BeforeSendSharedMemoryMsg(
      shm_msg->shm_msg_metadata());

  const auto header = shm_msg->mutable_shm_msg_metadata()->header();
  // Publish shm msg proto.
  if (!PubInternalWithChecker(channel, domain, shm_msg->shm_msg_metadata())) {
    return absl::AbortedError("PubInternalWithChecker() fails");
  }
  return header;
}

absl::StatusOr<LiteHeader> LiteClientBase::ForwardLiteMsg(
    LiteMsgWrapper* lite_msg) {
  auto [channel, domain] =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(*lite_msg);
  return ForwardLiteMsg(channel, domain, lite_msg);
}

absl::StatusOr<LiteHeader> LiteClientBase::ForwardLiteMsg(
    const std::string& channel, const std::string& domain,
    LiteMsgWrapper* lite_msg) {
  LiteHeader* header =
      LiteMsgConverter::Get().GetMutableLiteMsgWrapperHeader(lite_msg);
  header->set_channel(channel);
  header->set_domain(domain);
  QCHECK(header->has_timestamp()) << LiteModuleName_Name(header->module_id())
                                  << ":" << channel << " has no timestamp.";
  QCHECK(header->timestamp() > 0) << LiteModuleName_Name(header->module_id())
                                  << ":" << channel << " timestamp is 0.";
  if (!PubInternalWithChecker(
          channel, domain,
          LiteMsgConverter::Get().GetLiteMsgField(*lite_msg))) {
    return absl::AbortedError("PubInternalWithChecker() fails");
  }
  return *header;
}

absl::StatusOr<LiteHeader> LiteClientBase::PublishLiteMsgWithMeta(
    const std::string& channel, const std::string& domain,
    LiteMsgWrapper* lite_msg) {
  auto* mutable_header =
      LiteMsgConverter::Get().GetMutableLiteMsgWrapperHeader(lite_msg);
  if (!mutable_header) {
    return absl::AbortedError("Fails to get mutable header from lite_msg");
  }

  // Update metadata before publish.
  UpdateMetadata(mutable_header, channel, domain);
  if (!PubInternalWithChecker(
          channel, domain,
          LiteMsgConverter::Get().GetLiteMsgField(*lite_msg))) {
    return absl::AbortedError("PubInternalWithChecker() fails");
  }
  return *mutable_header;
}

// Add a timer, die if same timer_name has been added before. If `delay` is
// greater than absl::ZeroDuration(), timer will start after waiting for
// `delay`. See TimerManager::ExecuteTimers().
LiteTimer& LiteClientBase::AddTimerOrDie(const std::string& timer_name,
                                         std::function<void()> cb,
                                         absl::Duration delay,
                                         absl::Duration interval,
                                         bool one_shot) {
  return timer_manager_->AddTimerOrDie(timer_name, std::move(cb), delay,
                                       interval, one_shot);
}

void LiteClientBase::RemoveTimer(const std::string& timer_name) {
  timer_manager_->RemoveTimer(timer_name);
}

// Schedule Callback
void LiteClientBase::Schedule(std::function<void()> cb) {
  callback_manager_->ScheduleCallback(std::move(cb));
}

void LiteClientBase::SetInputOutputChecker(
    const PubSubMessageMap& input_message_map,
    const PubSubMessageMap& output_message_map,
    const LiteModuleConfig& module_config) {
  module_config_ = module_config;
  check_input_output_message_ = true;
  input_message_map_ = input_message_map;
  output_message_map_ = output_message_map;
}

// Not thread safe, usually call it before start using client.
void LiteClientBase::DisableInputOutputChecker() {
  check_input_output_message_ = false;
}

// Not thread safe, usually call it before start using client.
void LiteClientBase::DisableUpdatingMetadata() {
  skip_updating_metadata_ = true;
}

// Returns whether the client has disabled metadata updating.
bool LiteClientBase::IsDisableUpdatingMetadata() {
  return skip_updating_metadata_;
}

// Return module name.
LiteModuleName LiteClientBase::GetModuleName() const { return module_name_; }

std::unordered_map<std::string, int64_t> LiteClientBase::GetSubDomainChannels()
    const {
  return sub_domain_channels_;
}

folly::Executor* LiteClientBase::GetExecutor() const { return executor_; }

bool LiteClientBase::IsSimClient() const { return is_sim_; }

void LiteClientBase::AddSubDomainChannel(const std::string& domain,
                                         const std::string& channel) {
  const auto domain_channel = CombineDomainChannel(domain, channel);
  if (sub_domain_channels_.count(domain_channel) == 0) {
    sub_domain_channels_[domain_channel] = 1;
  } else {
    sub_domain_channels_[domain_channel]++;
  }
}

void LiteClientBase::Stop() { StopInternal(); }

void LiteClientBase::UpdateMetadata(LiteHeader* header,
                                    const std::string& channel,
                                    const std::string& domain) {
  if (skip_updating_metadata_) return;
  header->set_timestamp(absl::ToUnixMicros(Clock::Now()));
  header->set_channel(channel);
  header->set_domain(domain);
  header->set_seq_number(seq_number_.fetch_add(1));
  header->set_module_id(static_cast<int32_t>(module_name_));
  header->set_tag_number(LiteMsgConverter::Get().GetTagNumber(channel));
}

bool LiteClientBase::PubInternalWithChecker(
    const std::string& channel, const std::string& domain,
    std::unique_ptr<google::protobuf::Message> message) {
  VLOG(5) << "PubInternalWithChecker, " << module_name_ << "," << domain << ":"
          << channel << ", tag_num: "
          << LiteMsgConverter::Get().GetLiteMsgTagNumber(*message);

  if (check_input_output_message_) {
    if ((channel_check_whitelist_.count(channel) == 0) &&
        !ContainsKey(output_message_map_, std::make_pair(channel, domain))) {
      LOG(FATAL) << "Publish the channel=" << channel << ",domain=" << domain
                 << " is not registered in lite module config: "
                 << module_config_.DebugString();
    }
  }
  return PubInternal(channel, domain, std::move(message));
}

bool LiteClientBase::PubInternalWithChecker(
    const std::string& channel, const std::string& domain,
    const google::protobuf::Message& message) {
  std::unique_ptr<google::protobuf::Message> lite_msg_copy(message.New());
  lite_msg_copy->CopyFrom(message);
  return PubInternalWithChecker(channel, domain, std::move(lite_msg_copy));
}

void LiteClientBase::SubscribeInternalWithChecker(
    const std::string& channel, const std::string& domain,
    std::unique_ptr<SubCallback> sub_callback) {
  VLOG(5) << "--";
  VLOG(5) << "SubscribeInternalWithChecker:" << domain << ":" << channel;
  if (check_input_output_message_) {
    if ((channel_check_whitelist_.count(channel) == 0) &&
        !ContainsKey(input_message_map_, std::make_pair(channel, domain))) {
      LOG(FATAL) << "Subscribe the channel=" << channel << ",domain=" << domain
                 << " is not registered in lite module config: "
                 << module_config_.DebugString();
    }
  }

  AddSubDomainChannel(domain, channel);

  ShmFactory::GetShmManager()->RegisterShmMsgSubscriber(channel, domain);

  return SubscribeInternal(channel, domain, std::move(sub_callback));
}

}  // namespace qcraft
