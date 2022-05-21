#ifndef ONBOARD_LITE_LITE_CALLBACKS_H_
#define ONBOARD_LITE_LITE_CALLBACKS_H_

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <string_view>  // NOLINT
#include <utility>
#include <variant>  // NOLINT

#include "glog/logging.h"
#include "onboard/base/base.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/utils/errors.h"

namespace qcraft {

class SubCallback {
 public:
  SubCallback() {}

  virtual ~SubCallback() {}

  virtual std::shared_ptr<const void> ParseFrom(
      const std::variant<std::string_view, std::shared_ptr<void>>&
          data_or_msg) = 0;

  virtual void Call(std::shared_ptr<const void> msg) = 0;
};

template <typename T>
class LiteMsgCallback : public SubCallback {
 public:
  typedef std::function<void(std::shared_ptr<const T>)> MsgCallback;

  // TODO(kun): Support run with executor.
  explicit LiteMsgCallback(MsgCallback cb)
      : SubCallback(), cb_(std::move(cb)) {}

  std::shared_ptr<const void> ParseFrom(
      const std::variant<std::string_view, std::shared_ptr<void>>&
          data_or_msg) {
    auto index = data_or_msg.index();
    if (index == 0) {
      std::shared_ptr<T> msg = std::make_shared<T>();
      std::string_view data = std::get<std::string_view>(data_or_msg);
      // Optimize with arena?
      msg->ParseFromArray(data.data(), data.size());
      std::shared_ptr<const void> rt = msg;
      return rt;
    } else {
      return std::get<std::shared_ptr<void>>(data_or_msg);
    }
  }

  void Call(std::shared_ptr<const void> msg) {
    std::shared_ptr<const T> cast_msg = std::static_pointer_cast<const T>(msg);
    cb_(cast_msg);
  }

 private:
  const MsgCallback cb_;
};

// Majorly used for logger to save lite messages.
class LiteMsgWrapperCallback : public SubCallback {
 public:
  typedef std::function<void(std::shared_ptr<const LiteMsgWrapper>)>
      MsgCallback;

  LiteMsgWrapperCallback(MsgCallback cb, const std::string& field_name)
      : cb_(std::move(cb)), field_name_(field_name) {}

  template <typename C>
  LiteMsgWrapperCallback(
      void (C::*pmethod)(std::shared_ptr<const LiteMsgWrapper>), C* pclass,
      const std::string& field_name)
      : cb_([pmethod, pclass](std::shared_ptr<const LiteMsgWrapper> msg) {
          (pclass->*pmethod)(std::move(msg));
        }),
        field_name_(field_name) {}

  std::shared_ptr<const void> ParseFrom(
      const std::variant<std::string_view, std::shared_ptr<void>>&
          data_or_msg) {
    auto index = data_or_msg.index();
    std::shared_ptr<LiteMsgWrapper> msg = std::make_shared<LiteMsgWrapper>();
    if (index == 0) {
      CHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
          std::get<std::string_view>(data_or_msg), field_name_, msg.get()));
    } else {
      CHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(
          *std::static_pointer_cast<google::protobuf::Message>(
              std::get<std::shared_ptr<void>>(data_or_msg)),
          field_name_, msg.get()));
    }
    return msg;
  }

  void Call(std::shared_ptr<const void> msg) {
    std::shared_ptr<const LiteMsgWrapper> cast_msg =
        std::static_pointer_cast<const LiteMsgWrapper>(msg);
    cb_(cast_msg);
  }

 private:
  const MsgCallback cb_;
  const std::string field_name_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_CALLBACKS_H_
