#include "onboard/lite/lite_transport.h"

#include <algorithm>
#include <functional>
#include <future>
#include <utility>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/global/trace.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_debug_util.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
LiteTransport::LiteTransport(MessageHub* message_hub)
    : message_hub_(message_hub) {}

bool LiteTransport::ReceiveNextMessage(absl::Duration duration) {
  QCOUNTER_SPAN("Lite_ReceiveNextMessage");
  const auto enter_time = Clock::Now();
  if (duration < absl::ZeroDuration()) {
    return false;
  }

  if (Clock::IsSimClock()) {
    // In simulation clock, will still sleep 10ms at least to wait for walltime
    // for receiving messages.
    duration = std::min(duration, absl::Milliseconds(10));
  }

  msg_mu_.Lock();
  // Optimize the wait time.
  const auto actual_wait = duration - (Clock::Now() - enter_time);
  // Check if msg_mu waited for the correct duration.
  if (!msg_mu_.AwaitWithTimeout(absl::Condition(IsNotEmpty, &message_callback_),
                                actual_wait)) {
    msg_mu_.Unlock();
    return false;
  }
  ReceiveMsgCallback recv_callback = message_callback_.front();
  message_callback_.pop();
  msg_mu_.Unlock();
  QCOUNTER_SPAN("Lite_ReceiveNextMessage_CALLBACK");
  if (recv_callback.message.index() == 0) {
    recv_callback.callback->Call(recv_callback.callback->ParseFrom(
        std::get<std::shared_ptr<google::protobuf::Message>>(
            recv_callback.message)));
  } else if (recv_callback.message.index() == 1) {
    recv_callback.callback->Call(recv_callback.callback->ParseFrom(
        *std::get<std::shared_ptr<std::string>>(recv_callback.message)));
  } else {
    QLOG(FATAL) << "Unsupport type";
  }
  return true;
}

bool LiteTransport::PublishMessage(
    const std::string& channel, const std::string& domain,
    std::unique_ptr<google::protobuf::Message> message) {
  SCOPED_QTRACE_ARG1("LiteTransport::PublishMessage", "channel", channel);

  if (msg_modifier_man_) {
    msg_modifier_man_->Process(channel, message.get());
  }
  return message_hub_->DispatchMessage(CombineDomainChannel(domain, channel),
                                       std::move(message));
}

void LiteTransport::SubscribeMessage(
    const std::string& channel, const std::string& domain,
    std::unique_ptr<SubCallback> sub_callback) {
  const auto& combined_domain_channel = CombineDomainChannel(domain, channel);
  SubCallback* callback = nullptr;
  {
    absl::MutexLock l(&reg_callback_mu_);
    registered_callbacks_.emplace_back(std::move(sub_callback));
    callback = registered_callbacks_.back().get();
  }
  MessageHub::PushMsgCallback func =
      [this, callback = callback](
          const std::string& domain_channel,
          const std::variant<std::shared_ptr<google::protobuf::Message>,
                             std::shared_ptr<std::string>>& message) {
        PushMessageCallback(callback, domain_channel, message);
      };
  message_hub_->RegisterDomainChannel(combined_domain_channel, std::move(func));
}

void LiteTransport::SetMsgModifierMan(MessageModifierMan* msg_modifier_man) {
  msg_modifier_man_ = msg_modifier_man;
}

void LiteTransport::PushMessageCallback(
    SubCallback* callback, const std::string& domain_channel,
    const std::variant<std::shared_ptr<google::protobuf::Message>,
                       std::shared_ptr<std::string>>& message) {
  ReceiveMsgCallback msg_callback;
  msg_callback.message = message;
  msg_callback.callback = callback;
  msg_callback.domain_channel = domain_channel;
  msg_callback.recv_time_ = Clock::Now();
  auto message_callback_size = 0;
  {
    QCOUNTER_SPAN("LiteTransport::PushMessageCallback");
    absl::MutexLock lm(&msg_mu_);
    message_callback_.push(msg_callback);
    message_callback_size = message_callback_.size();
  }
  const size_t MAX_CALLBACK_SIZE = 1000;
  if (message_callback_size > MAX_CALLBACK_SIZE) {
    QLOG(ERROR) << "Too many message_callback:" << message_callback_size
                << ", domain_channel:" << domain_channel;
  }
  QCOUNTER("pending_callback_size", message_callback_size);
}

MessageHub* LiteTransport::GetMessageHub() const { return message_hub_; }

void LiteTransport::Stop() { message_hub_->Reset(); }

}  // namespace qcraft
