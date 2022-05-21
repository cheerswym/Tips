#include "onboard/lite/sidecar/user_input_sidecar.h"

#include "onboard/utils/thread_util.h"

namespace qcraft {

UserInputSideCar::UserInputSideCar() : callback_([](std::string str) {}) {}

void UserInputSideCar::Start(LiteModule *lite_module) {
  thread_ = std::make_unique<std::thread>([this] {
    QSetThreadName("UserInputSideCar");
    while (true) {
      std::string str;
      std::cin >> str;
      callback_(str);
      if (str.compare("bye") == 0 || str.compare("stop") == 0) {
        break;
      }
    }
  });
}

void UserInputSideCar::SetCallBack(
    const std::function<void(const std::string &str)> &callback) {
  callback_ = callback;
}

void UserInputSideCar::Stop(LiteModule *lite_module) { thread_->join(); }

SideCar::Type UserInputSideCar::GetType() { return SideCar::Type::USER_INPUT; }
}  // namespace qcraft
