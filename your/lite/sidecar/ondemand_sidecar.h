#ifndef ONBOARD_LITE_SIDECAR_ONDEMAND_SIDECAR_H_
#define ONBOARD_LITE_SIDECAR_ONDEMAND_SIDECAR_H_

#include <memory>

#include "onboard/ap_common/util/blocking_queue.h"
#include "onboard/lite/lite_module.h"

namespace qcraft {
// OnDemandSideCar, run a job that outside the schedule of module.
// Currently dump trace when execution issue.
class OnDemandSideCar : public SideCar {
 public:
  OnDemandSideCar();
  void Start(LiteModule *lite_module) override;
  void AddJob(const std::function<void(LiteModule *)> &func) override;
  void Stop(LiteModule *lite_module) override;
  Type GetType() override;

 private:
  std::unique_ptr<std::thread> thread_;
  absl::Notification stop_notification_;
  apollo::common::util::BlockingQueue<std::function<void(LiteModule *)>> queue_;
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_SIDECAR_ONDEMAND_SIDECAR_H_
