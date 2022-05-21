#include "onboard/global/counter.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "boost/circular_buffer.hpp"
#include "folly/SharedMutex.h"
#include "folly/ThreadLocal.h"
#include "folly/logging/RateLimiter.h"
#include "onboard/global/spin_lock.h"
#include "onboard/math/util.h"
#include "onboard/utils/map_util.h"

// This is a temporary fix to TSAN link error.
namespace folly {
template class SharedMutexImpl<true>;
template class SharedMutexImpl<false>;
}  // namespace folly

namespace qcraft {
class CounterImpl {
 public:
  CounterProto GetCounterOutput();

  // This function is thread safe.
  void AddCounterEvent(const char* name, int64_t val);

  void AddCounterEvent(const CounterEvent& counter_event);

 private:
  struct CounterPerThread {
    // Trace collected per thread.
    static constexpr int kCounterBufferSizePerThread = 100000;
    boost::circular_buffer<CounterEvent> counter_events_per_thread{
        kCounterBufferSizePerThread};

    // Spin lock to avoid data race.
    SpinLock spin_lock;
  };
  class CounterTag;
  folly::ThreadLocal<CounterPerThread, CounterTag> counter_per_thread_;
};

void CounterImpl::AddCounterEvent(const char* name, int64_t val) {
  CounterEvent counter_event{name, val, absl::ToUnixMillis(absl::Now())};
  counter_per_thread_->spin_lock.Lock();

  counter_per_thread_->counter_events_per_thread.push_back(counter_event);

  // Release the spin lock.
  counter_per_thread_->spin_lock.Unlock();
}

void CounterImpl::AddCounterEvent(const CounterEvent& counter_event) {
  counter_per_thread_->spin_lock.Lock();

  counter_per_thread_->counter_events_per_thread.push_back(counter_event);

  // Release the spin lock.
  counter_per_thread_->spin_lock.Unlock();
}

CounterProto CounterImpl::GetCounterOutput() {
  CounterProto counter_proto;
  std::map<std::string, CounterItem> counter_items;

  auto accessor = counter_per_thread_.accessAllThreads();
  for (auto& counter_per_thread : accessor) {
    boost::circular_buffer<CounterEvent> counter_events_per_thread(
        CounterPerThread::kCounterBufferSizePerThread);

    counter_per_thread.spin_lock.Lock();

    counter_events_per_thread.swap(
        counter_per_thread.counter_events_per_thread);

    // Release the spin lock.
    counter_per_thread.spin_lock.Unlock();

    if (counter_events_per_thread.full()) {
      LOG(INFO) << "circular buffer is full";
    }
    for (const auto& counter_event : counter_events_per_thread) {
      const std::string& name = counter_event.name;
      auto* counter_item = FindOrNull(counter_items, name);
      if (counter_item != nullptr) {
        if (counter_event.value > counter_item->max) {
          counter_item->max = counter_event.value;
        } else if (counter_event.value < counter_item->min) {
          counter_item->min = counter_event.value;
        }
        counter_item->sum += counter_event.value;
        counter_item->count +=
            std::max(counter_event.fields.size(), static_cast<size_t>(1));
        counter_item->last_timestamp_ms = counter_event.timestamp_ms;
        if (!counter_event.fields.empty()) {
          for (const auto& kv : counter_event.fields) {
            auto* item_field = FindOrNull(counter_item->fields, kv.first);
            if (item_field == nullptr) {
              counter_item->fields[kv.first] = std::make_shared<CounterItem>();
              counter_item->fields[kv.first]->name = kv.first;
              counter_item->fields[kv.first]->first_timestamp_ms =
                  counter_event.timestamp_ms;
              counter_item->fields[kv.first]->last_timestamp_ms =
                  counter_event.timestamp_ms;
              counter_item->fields[kv.first]->max = kv.second;
              counter_item->fields[kv.first]->min = kv.second;
              counter_item->fields[kv.first]->sum = kv.second;
              counter_item->fields[kv.first]->count = 1;
            } else {
              (*item_field)->last_timestamp_ms = counter_event.timestamp_ms;
              if (kv.second > (*item_field)->max) {
                (*item_field)->max = kv.second;
              } else if (kv.second < (*item_field)->min) {
                (*item_field)->min = kv.second;
              }
              (*item_field)->sum += kv.second;
              (*item_field)->count += 1;
            }
          }
        }
      } else {
        CounterItem item;
        item.name = name;
        item.first_timestamp_ms = counter_event.timestamp_ms;
        item.last_timestamp_ms = counter_event.timestamp_ms;
        item.max = counter_event.value;
        item.min = counter_event.value;
        item.sum = counter_event.value;
        item.count =
            std::max(counter_event.fields.size(), static_cast<size_t>(1));
        if (!counter_event.fields.empty()) {
          for (const auto& kv : counter_event.fields) {
            item.fields[kv.first] = std::make_shared<CounterItem>();
            item.fields[kv.first]->name = kv.first;
            item.fields[kv.first]->first_timestamp_ms =
                counter_event.timestamp_ms;
            item.fields[kv.first]->last_timestamp_ms =
                counter_event.timestamp_ms;
            item.fields[kv.first]->max = kv.second;
            item.fields[kv.first]->min = kv.second;
            item.fields[kv.first]->sum = kv.second;
            item.fields[kv.first]->count = 1;
          }
        }
        counter_items[name] = item;
      }
    }
  }

  if (counter_items.empty()) {
    return counter_proto;
  }

  // Collect trace and publish.
  for (auto& [name, counter_item] : counter_items) {
    CounterItemProto* counter_item_proto = counter_proto.add_item();
    counter_item_proto->set_name(counter_item.name);
    counter_item_proto->set_max(counter_item.max);
    counter_item_proto->set_min(counter_item.min);
    counter_item_proto->set_sum(counter_item.sum);
    counter_item_proto->set_count(counter_item.count);
    counter_item_proto->set_first_timestamp_ms(counter_item.first_timestamp_ms);
    counter_item_proto->set_last_timestamp_ms(counter_item.last_timestamp_ms);
    auto* mutable_fields = counter_item_proto->mutable_fields();
    for (const auto& kv : counter_item.fields) {
      (*mutable_fields)[kv.first].set_name(kv.second->name);
      (*mutable_fields)[kv.first].set_max(kv.second->max);
      (*mutable_fields)[kv.first].set_min(kv.second->min);
      (*mutable_fields)[kv.first].set_sum(kv.second->sum);
      (*mutable_fields)[kv.first].set_count(kv.second->count);
      (*mutable_fields)[kv.first].set_first_timestamp_ms(
          kv.second->first_timestamp_ms);
      (*mutable_fields)[kv.first].set_last_timestamp_ms(
          kv.second->last_timestamp_ms);
    }
  }
  return counter_proto;
}

Counter::Counter() : impl_(std::make_unique<CounterImpl>()) {}

CounterProto Counter::GetCounterOutput() { return impl_->GetCounterOutput(); }

void Counter::AddCounterEventInternal(const char* name, int64_t val) {
  impl_->AddCounterEvent(name, val);
}

void Counter::AddCounterEventInternal(const CounterEvent& counter_event) {
  impl_->AddCounterEvent(counter_event);
}

}  // namespace qcraft
