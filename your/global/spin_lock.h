#ifndef ONBOARD_GLOBAL_SPIN_LOCK_H_
#define ONBOARD_GLOBAL_SPIN_LOCK_H_

#include <atomic>

#include "onboard/base/macros.h"

namespace qcraft {

class SpinLock {
 public:
  void Lock() {
    // Acquire the spin lock.
    while (UNLIKELY(flag_.test_and_set(std::memory_order_acquire))) {
      // Wait for the lock to be released.
    }
  }
  void Unlock() { flag_.clear(std::memory_order_release); }

 private:
  std::atomic_flag flag_;
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SPIN_LOCK_H_
