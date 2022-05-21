#ifndef ONBOARD_LITE_LITE_CLIENT_H_
#define ONBOARD_LITE_LITE_CLIENT_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/time/time.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_transport.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport.h"

namespace qcraft {

class LiteClient : public LiteClientBase {
 public:
  virtual ~LiteClient() {}

  explicit LiteClient(LiteModuleName module_name,
                      std::unique_ptr<Transport> transport,
                      folly::Executor *executor = nullptr,
                      const std::string &domain = "")
      : LiteClientBase(module_name, executor, domain),
        transport_(std::move(transport)) {}

  // Entry of receiving messages, execute timer and callbacks.
  // Dispatcher will call it, individual module won't call it at most of cases.
  void SleepUntil(absl::Duration duration) override { Dispatch(duration); }

  MessageHub *GetMessageHub() const;

 private:
  void Dispatch(absl::Duration duration);

  bool PubInternal(
      const std::string &channel, const std::string &domain,
      std::unique_ptr<google::protobuf::Message> message) override {
    // TODO(kun): Return util::Status instead.
    return transport_->PublishMessage(channel, domain, std::move(message));
  }

  void SubscribeInternal(const std::string &channel, const std::string &domain,
                         std::unique_ptr<SubCallback> sub_callback) override {
    transport_->SubscribeMessage(channel, domain, std::move(sub_callback));
  }

  void StopInternal() override { transport_->Stop(); }

  std::unique_ptr<Transport> transport_;
};

}  // namespace qcraft
#endif  // ONBOARD_LITE_LITE_CLIENT_H_
