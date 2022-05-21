#ifndef ONBOARD_LITE_SIDECAR_PERIODIC_SIDECAR_H_
#define ONBOARD_LITE_SIDECAR_PERIODIC_SIDECAR_H_

#include <memory>

#include "onboard/lite/lite_module.h"
#include "onboard/utils/periodic_runner.h"

namespace qcraft {
// PerodicSideCar perodically run jobs that outside the schedule of module.
// Current contain collecting counter and qlog.
class PerodicSideCar : public SideCar {
  void Start(LiteModule *lite_module) override;

  void Stop(LiteModule *lite_module) override;

  SideCar::Type GetType() override;

 private:
  // counter, qevent, log, system_info
  std::unique_ptr<PeriodicRunner> runner_;
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_SIDECAR_PERIODIC_SIDECAR_H_
