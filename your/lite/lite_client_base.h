#ifndef ONBOARD_LITE_LITE_CLIENT_BASE_H_
#define ONBOARD_LITE_LITE_CLIENT_BASE_H_

// TODO(kun): More features unfinished:
// 0, Timer callerbacks.
// 1, Unsubscribe a message
// 2, Message Rate Checker
// 3, Specify Executor for Sub or Pub
// 4, single main thread check.
// ...

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "folly/Executor.h"
#include "google/protobuf/message.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/clock.h"
#include "onboard/lite/check.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/lite_timer.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/message/message_hub.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

// A map from {channel, domain} pair to expected min and max execution
// intervals.
typedef std::map<std::pair<std::string, std::string>,
                 std::pair<absl::Duration, absl::Duration>>
    PubSubMessageMap;

class LiteClientBase {
 public:
  virtual ~LiteClientBase();

  explicit LiteClientBase(LiteModuleName module_name, folly::Executor* executor,
                          const std::string& domain, bool is_sim = false);

  // Entry of receiving messages, execute timer and callbacks.
  // Dispatcher will call it, individual module won't call it at most of cases.
  virtual void SleepUntil(absl::Duration duration) = 0;

  // TODO(kun): It's not thread-safe, add thread check, should sub at main
  template <typename T>
  void Subscribe(std::function<void(std::shared_ptr<const T>)> cb) {
    const auto& channel = ProtoMessageToName<T>();
    Subscribe<T>(std::move(cb), channel);
  }

  template <typename T>
  void Subscribe(std::function<void(std::shared_ptr<const T>)> cb,
                 const std::string& channel) {
    const auto domain_channel = CombineDomainChannel(domain_, channel);
    SubscribeMsgWithCallback(
        channel, domain_, std::make_unique<LiteMsgCallback<T>>(std::move(cb)));
  }

  // TODO(kun): Support more forms of subscribe, like: method pointer.
  template <typename T, typename C>
  void Subscribe(void (C::*pmethod)(std::shared_ptr<const T>), C* pclass) {
    std::function<void(std::shared_ptr<const T>)> cb =
        [pmethod, pclass](std::shared_ptr<const T> msg) {
          (pclass->*pmethod)(msg);
        };
    Subscribe<T>(std::move(cb));
  }

  template <typename T, typename C>
  void Subscribe(void (C::*pmethod)(std::shared_ptr<const T>), C* pclass,
                 const std::string& channel) {
    std::function<void(std::shared_ptr<const T>)> cb =
        [pmethod, pclass](std::shared_ptr<const T> msg) {
          (pclass->*pmethod)(msg);
        };
    Subscribe<T>(std::move(cb), channel);
  }

  void SubscribeShmMsg(std::function<void(std::shared_ptr<ShmMessage>)> shm_cb,
                       const std::string& channel,
                       const std::string& domain = "");

  void SubscribeMsgWithCallback(const std::string& channel,
                                const std::string& domain,
                                std::unique_ptr<SubCallback> sub_callback);

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg) {
    return Publish(lite_msg, ProtoMessageToName<T>(), domain_);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(std::unique_ptr<T> lite_msg) {
    return Publish(std::move(lite_msg), ProtoMessageToName<T>(), domain_);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg,
                                     const std::string& channel) {
    return Publish(lite_msg, channel, domain_);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(std::unique_ptr<T> lite_msg,
                                     const std::string& channel) {
    return Publish(std::move(lite_msg), channel, domain_);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(std::unique_ptr<T> lite_msg,
                                     const std::string& channel,
                                     const std::string& domain) {
    VLOG(5) << "lite_client_base: Publish:" << domain << ":" << channel << ":"
            << lite_msg->DebugString();
    // Update metadata before publish.
    UpdateMetadata<T>(lite_msg->mutable_header(), channel, domain);

    const auto header = lite_msg->header();
    // proto::Message, update metadata.
    if (!PubInternalWithChecker(channel, domain, std::move(lite_msg))) {
      return absl::AbortedError("PubInternalWithChecker() fails");
    }
    return header;
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg,
                                     const std::string& channel,
                                     const std::string& domain) {
    std::unique_ptr<T> lite_msg_copy(lite_msg.New());
    lite_msg_copy->CopyFrom(lite_msg);
    return Publish(std::move(lite_msg_copy), channel, domain);
  }

  // Share memory publisher and subcriber, won't own the pointer.
  absl::StatusOr<LiteHeader> PublishShmMsg(ShmMessage* shm_msg);

  absl::StatusOr<LiteHeader> PublishShmMsg(ShmMessage* shm_msg,
                                           const std::string& channel,
                                           const std::string& domain);

  absl::StatusOr<LiteHeader> ForwardLiteMsg(LiteMsgWrapper* lite_msg);

  absl::StatusOr<LiteHeader> ForwardLiteMsg(const std::string& channel,
                                            const std::string& domain,
                                            LiteMsgWrapper* lite_msg);

  absl::StatusOr<LiteHeader> PublishLiteMsgWithMeta(const std::string& channel,
                                                    const std::string& domain,
                                                    LiteMsgWrapper* lite_msg);

  // Add a timer, die if same timer_name has been added before. If `delay` is
  // greater than absl::ZeroDuration(), timer will start after waiting for
  // `delay`. See TimerManager::ExecuteTimers().
  LiteTimer& AddTimerOrDie(const std::string& timer_name,
                           std::function<void()> cb, absl::Duration delay,
                           absl::Duration interval, bool one_shot = true);

  template <typename C>
  LiteTimer& AddTimerOrDie(const std::string& timer_name, void (C::*pmethod)(),
                           C* pclass, absl::Duration delay,
                           absl::Duration interval, bool one_shot = true) {
    std::function<void()> cb = [pmethod, pclass]() { (pclass->*pmethod)(); };
    return AddTimerOrDie(timer_name, std::move(cb), delay, interval, one_shot);
  }

  void RemoveTimer(const std::string& timer_name);

  // Schedule Callback
  void Schedule(std::function<void()> cb);

  void SetInputOutputChecker(const PubSubMessageMap& input_message_map,
                             const PubSubMessageMap& output_message_map,
                             const LiteModuleConfig& module_config);

  // Not thread safe, usually call it before start using client.
  void DisableInputOutputChecker();

  // Not thread safe, usually call it before start using client.
  void DisableUpdatingMetadata();

  // Returns whether the client has disabled metadata updating.
  bool IsDisableUpdatingMetadata();

  // Return module name.
  LiteModuleName GetModuleName() const;

  folly::Executor* GetExecutor() const;

  bool IsSimClient() const;

  std::unordered_map<std::string, int64_t> GetSubDomainChannels() const;

  void AddSubDomainChannel(const std::string& domain,
                           const std::string& channel);

  void Stop();

 protected:
  const LiteModuleName module_name_;
  folly::Executor* executor_;
  std::unique_ptr<TimerManager> timer_manager_;
  std::unique_ptr<CallbackManager> callback_manager_;

  // all subscription.
  std::unordered_map<std::string, int64_t> sub_domain_channels_;

 private:
  // In simulation mode should override this function.
  template <typename T>
  void UpdateMetadata(LiteHeader* header, const std::string& channel,
                      const std::string& domain) {
    header->set_channel(channel);
    header->set_domain(domain);
    if (skip_updating_metadata_) return;
    header->set_timestamp(absl::ToUnixMicros(Clock::Now()));
    header->set_domain(domain);
    header->set_seq_number(seq_number_.fetch_add(1));
    header->set_module_id(static_cast<int32_t>(module_name_));
    header->set_tag_number(LiteMsgConverter::Get().GetTagNumber<T>());
  }

  void UpdateMetadata(LiteHeader* header, const std::string& channel,
                      const std::string& domain);

  bool PubInternalWithChecker(
      const std::string& channel, const std::string& domain,
      std::unique_ptr<google::protobuf::Message> message);

  bool PubInternalWithChecker(const std::string& channel,
                              const std::string& domain,
                              const google::protobuf::Message& message);

  void SubscribeInternalWithChecker(const std::string& channel,
                                    const std::string& domain,
                                    std::unique_ptr<SubCallback> sub_callback);

 private:
  // override by lite_client
  // TODO(kun): Return Status, build own status library.
  virtual bool PubInternal(
      const std::string& channel, const std::string& domain,
      std::unique_ptr<google::protobuf::Message> message) = 0;

  virtual void SubscribeInternal(const std::string& channel,
                                 const std::string& domain,
                                 std::unique_ptr<SubCallback> sub_callback) = 0;

  virtual void StopInternal() = 0;

 private:
  const std::string domain_;
  std::atomic<uint64_t> seq_number_;
  // If true will check input and output message's channel and domain
  // registered, if message is not registerred, it will check failed.
  bool check_input_output_message_;
  // If true will not update metadata and keep lite msg's saved metadata.
  bool skip_updating_metadata_;
  PubSubMessageMap input_message_map_;
  PubSubMessageMap output_message_map_;
  LiteModuleConfig module_config_;

  bool is_sim_ = false;

  const std::unordered_set<std::string> publishing_whitelist_ = {
      "system_state_proto", "counter_proto", "log_proto", "q_events_proto"};

  const std::unordered_set<std::string> channel_check_whitelist_ = {
      "trace_proto",
      "system_info_proto",
      "execution_issue_proto",
      "system_command_proto",
      "system_state_proto",
      "counter_proto",
      "log_proto",
      "q_events_proto",
      "autonomy_state_proto"};
};

}  // namespace qcraft
#endif  // ONBOARD_LITE_LITE_CLIENT_BASE_H_
