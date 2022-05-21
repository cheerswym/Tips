#ifndef ONBOARD_LITE_RUNNER_AUTONOMY_RUNNER_H_
#define ONBOARD_LITE_RUNNER_AUTONOMY_RUNNER_H_

#include "onboard/lite/runner/runner.h"

namespace qcraft {
namespace lite {
class AutonomyRunner final : public Runner {
 public:
  bool Init() override;
  int Run() override;
  int Release() override;
  void SignalHandler(int signum) override;

  virtual ~AutonomyRunner() {}

 private:
  void SetRsimFlags();
};
}  // namespace lite
}  // namespace qcraft

#endif  // ONBOARD_LITE_RUNNER_AUTONOMY_RUNNER_H_
