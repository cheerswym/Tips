#ifndef ONBOARD_LITE_TRANSPORT_SHM_CONDITION_NOTIFIER_H_
#define ONBOARD_LITE_TRANSPORT_SHM_CONDITION_NOTIFIER_H_

#include <sys/types.h>

#include <atomic>
#include <cstdint>

#include "onboard/base/base.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
namespace shm {

class ConditionNotifier {
 public:
  explicit ConditionNotifier();
  virtual ~ConditionNotifier();
  void Shutdown();
  bool Write(const ReadableInfo& info, int subscriber_num,
             uint64_t current_write_seq, uint64_t current_write_index);
  bool Read(int timeout_ms, ReadableInfo* info, int* index);
  void ReaderCatchup();
  bool GetWriteIndex(uint64_t* current_write_seq,
                     uint64_t* current_write_index);

 private:
  bool TryWrite(const ReadableInfo& info, int subscriber_num,
                uint64_t current_write_seq, uint64_t current_write_index);
  bool TryRead(ReadableInfo* info, int* index);

  int64_t Index(int n) { return n % kNotifierSlotSize; }

  Indicator* indicator_ = nullptr;
  int64_t my_read_seq_ = -1;
  std::atomic<bool> is_shutdown_ = {false};
};
}  // namespace shm
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_CONDITION_NOTIFIER_H_
