#ifndef ONBOARD_LITE_RUNNER_RUNNER_H_
#define ONBOARD_LITE_RUNNER_RUNNER_H_

#include <signal.h>

#include <memory>
#include <string>

#include "folly/Executor.h"
#include "onboard/eval/collectors/qevent_log_collector.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/transport/message/shm_message_hub.h"

namespace qcraft {
namespace lite {
class Runner {
 public:
  virtual bool Init() = 0;
  virtual int Run() = 0;
  virtual int Release() = 0;
  virtual void SignalHandler(int signum) = 0;

  virtual ~Runner() {}

 protected:
  static std::string SigNumToString(int signum) {
    switch (signum) {
      case SIGINT:
        return "SIGINT";
      default:
        return "UNDEFINED";
    }
  }

  std::shared_ptr<ClockBase> clock_;
  std::unique_ptr<QEventLogCollector> qevent_collector_;
  std::unique_ptr<ShmMessageHub> hub_;
  std::unique_ptr<folly::Executor> executor_;
  std::unique_ptr<LiteClient> lite_client_;
  std::unique_ptr<LiteModule> lite_module_;
};
}  // namespace lite
}  // namespace qcraft

#endif  // ONBOARD_LITE_RUNNER_RUNNER_H_
