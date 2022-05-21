#ifndef ONBOARD_GLOBAL_COUNTER_H_
#define ONBOARD_GLOBAL_COUNTER_H_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/time/clock.h"
#include "glog/logging.h"
#include "onboard/base/macros.h"
#include "onboard/global/singleton.h"
#include "onboard/proto/counter.pb.h"
namespace qcraft {

#define CONCAT_TOKEN_(foo, bar) CONCAT_TOKEN_IMPL_(foo, bar)
#define CONCAT_TOKEN_IMPL_(foo, bar) foo##bar

#define QCOUNTER(name, val)                             \
  static_assert(name "", "Must pass a string literal"); \
  qcraft::Counter::Instance()->AddCounterEvent(name, val)

#define QCOUNTER_EVENT(counter_event) \
  qcraft::Counter::Instance()->AddCounterEvent(counter_event)

#define QCOUNTER_SPAN(name)                                                \
  static_assert(name "", "Must pass a string literal");                    \
  qcraft::CounterEventWrapper CONCAT_TOKEN_(scoped_trace_, __LINE__)(name, \
                                                                     false)

#define QCOUNTER_FUNC()                                                        \
  qcraft::CounterEventWrapper CONCAT_TOKEN_(scoped_trace_, __LINE__)(__func__, \
                                                                     false)

#define QCOUNTER_AND_PRINT_SPAN(name)                   \
  static_assert(name "", "Must pass a string literal"); \
  qcraft::CounterEventWrapper CONCAT_TOKEN_(scoped_trace_, __LINE__)(name, true)

struct CounterItem {
  std::string name;
  int64_t first_timestamp_ms;
  int64_t last_timestamp_ms;
  int64_t max;
  int64_t min;
  int64_t sum;
  int64_t count;
  std::unordered_map<std::string, std::shared_ptr<CounterItem>> fields;
};

struct CounterEvent {
  const char* name;
  int64_t value;
  int64_t timestamp_ms;  // ms
  std::unordered_map<std::string, int64_t> fields;
};

class CounterImpl;
class Counter {
 public:
  // Get the counter proto object.
  CounterProto GetCounterOutput();

  // Add counter event. This function is thread safe.
  template <typename T>
  void AddCounterEvent(const char* name, T val) {
    static_assert(std::is_integral<T>::value, "Required Integral Type");
    AddCounterEventInternal(name, val);
  }

  void AddCounterEvent(const CounterEvent& counter_event) {
    AddCounterEventInternal(counter_event);
  }

 private:
  void AddCounterEventInternal(const char* name, int64_t val);
  void AddCounterEventInternal(const CounterEvent& counter_event);
  std::unique_ptr<CounterImpl> impl_;
  DECLARE_SINGLETON(Counter);
};

class CounterEventWrapper {
 public:
  explicit CounterEventWrapper(const char* name, bool print = true) {
    print_ = print;
    name_ = name;
    start_ = std::chrono::steady_clock().now();
  }

  ~CounterEventWrapper() {
    auto now = std::chrono::steady_clock().now();
    Counter::Instance()->AddCounterEvent(
        name_,
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_)
            .count());
    if (print_) {
      LOG(ERROR) << " !!! " << name_ << " cost :"
                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - start_)
                        .count()
                 << " ms.";
    }
  }

 private:
  bool print_;
  std::chrono::steady_clock::time_point start_;
  const char* name_;
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_COUNTER_H_
