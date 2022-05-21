#ifndef ONBOARD_LITE_SIDECAR_USER_INPUT_SIDECAR_H_
#define ONBOARD_LITE_SIDECAR_USER_INPUT_SIDECAR_H_

#include <memory>
#include <string>

#include "onboard/lite/lite_module.h"

namespace qcraft {
// This side car start a sepearted thread to listen the stdin, it will callback
// immediately . This is useful to handle stdin in the module(like main
// AutonomyModule), since we can't handle stdin in the main loop as it will be
// blocking.
class UserInputSideCar : public SideCar {
 public:
  UserInputSideCar();
  void Start(LiteModule *lite_module) override;
  void Stop(LiteModule *lite_module) override;

  void SetCallBack(const std::function<void(const std::string &str)> &callback);

  Type GetType() override;

 private:
  std::unique_ptr<std::thread> thread_;
  std::function<void(const std::string &str)> callback_;
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_SIDECAR_USER_INPUT_SIDECAR_H_
