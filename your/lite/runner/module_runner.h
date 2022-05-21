#ifndef ONBOARD_LITE_RUNNER_MODULE_RUNNER_H_
#define ONBOARD_LITE_RUNNER_MODULE_RUNNER_H_

#include "onboard/lite/runner/runner.h"

namespace qcraft {
namespace lite {
class ModuleRunner final : public Runner {
 public:
  bool Init() override;
  int Run() override;
  int Release() override;
  void SignalHandler(int signum) override;

  virtual ~ModuleRunner() {}

 private:
};
}  // namespace lite
}  // namespace qcraft

#endif  // ONBOARD_LITE_RUNNER_MODULE_RUNNER_H_
