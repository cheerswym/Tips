#include "onboard/lite/transport/shm/condition_notifier.h"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <thread>

#include "glog/logging.h"
#include "onboard/base/base.h"

namespace qcraft {
namespace shm {

namespace {
constexpr auto kMinSleepTime = 1000;  // unit: us
}

ConditionNotifier::ConditionNotifier() {
  indicator_ = ShmManager::Instance()->CreateNotifierIndicator();
}

void ConditionNotifier::ReaderCatchup() {
  my_read_seq_ = indicator_->next_write_seq.load() - 1;
}

ConditionNotifier::~ConditionNotifier() { Shutdown(); }

void ConditionNotifier::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }
}

bool ConditionNotifier::TryWrite(const ReadableInfo& info, int subscriber_num,
                                 uint64_t current_write_seq,
                                 uint64_t current_write_index) {
  shm::ShmManager::Instance()->GarbageCollection(current_write_index);

  // write info first and then index ref count
  indicator_->infos[current_write_index] = info;
  shm::ShmManager::Instance()->IncreaseIndexRefCount(current_write_index,
                                                     subscriber_num);
  indicator_->seqs[current_write_index] = current_write_seq;
  return true;
}

bool ConditionNotifier::GetWriteIndex(uint64_t* write_seq,
                                      uint64_t* write_index) {
  if (is_shutdown_.load()) {
    return false;
  }
  *write_seq = indicator_->next_write_seq.fetch_add(1);
  *write_index = Index(*write_seq);
  return true;
}

bool ConditionNotifier::Write(const ReadableInfo& info, int subscriber_num,
                              uint64_t current_write_seq,
                              uint64_t current_write_index) {
  CHECK(TryWrite(info, subscriber_num, current_write_seq, current_write_index));
  return true;
}

bool ConditionNotifier::TryRead(ReadableInfo* info, int* index) {
  uint64_t target_seq = indicator_->seqs[Index(my_read_seq_ + 1)].load();
  if (my_read_seq_ >= (int64_t)target_seq) {
    return false;
  }

  if (target_seq - my_read_seq_ >= kNotifierSlotSize - 1) {
    LOG(FATAL) << "the fast writer has overlap the reader";
    return false;
  }

  my_read_seq_ += 1;
  *info = indicator_->infos[Index(my_read_seq_)];
  *index = Index(my_read_seq_);
  return true;
}

bool ConditionNotifier::Read(int timeout_ms, ReadableInfo* info, int* index) {
  VLOG(4) << "Read:" << my_read_seq_;
  if (is_shutdown_.load()) {
    return false;
  }

  int64_t us = kMinSleepTime;
  int64_t total_us = 0;
  while (!TryRead(info, index)) {
    if (is_shutdown_.load()) {
      return false;
    }
    usleep(us);
    total_us += us;
    if (total_us > timeout_ms * 1000) {
      return false;
    }
  }
  return true;
}

}  // namespace shm
}  // namespace qcraft
