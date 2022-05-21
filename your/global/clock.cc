#include "onboard/global/clock.h"

#include <memory>
#include <utility>

#include "absl/synchronization/mutex.h"

namespace qcraft {

class GlobalClock {
 public:
  GlobalClock() { cur_clock_ = std::make_shared<SystemClock>(); }

  ClockBase &CurrentClock() {
    absl::MutexLock l(&mu_);
    return *cur_clock_.get();
  }

  void SetGlobalClock(std::shared_ptr<ClockBase> new_clock) {
    absl::MutexLock l(&mu_);
    cur_clock_ = new_clock;
  }

 private:
  absl::Mutex mu_;
  std::shared_ptr<ClockBase> cur_clock_;
};

namespace {
static GlobalClock *g_clock = new GlobalClock();
}  // namespace

ClockBase &GetGlobalClock() { return g_clock->CurrentClock(); }

void SetGlobalClock(std::shared_ptr<ClockBase> clock) {
  g_clock->SetGlobalClock(std::move(clock));
}

}  // namespace qcraft
