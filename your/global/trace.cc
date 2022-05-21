#include "onboard/global/trace.h"

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>

#include "boost/circular_buffer.hpp"
#include "folly/SharedMutex.h"
#include "folly/ThreadLocal.h"
#include "folly/logging/RateLimiter.h"
#include "onboard/global/counter.h"
#include "onboard/global/spin_lock.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/node/node_util.h"

// This is a temporary fix to TSAN link error.
namespace folly {
template class SharedMutexImpl<true>;
template class SharedMutexImpl<false>;
}  // namespace folly

namespace qcraft {

DECLARE_bool(ftrace_compress_enable);

namespace {

constexpr double kMaxTraceDuration = 3.0;  // s
constexpr int kTraceBufferSizePerThread = 10000;

std::string GetThreadIdStr(std::thread::id thread_id) {
  std::ostringstream ss;
  ss << thread_id;
  return ss.str();
}
}  // namespace

class TraceImpl {
 public:
  // This function is thread safe.
  void AddTraceEvent(const TraceEvent& trace_event) {
    QCOUNTER_SPAN("AddTraceEvent");
    trace_per_thread_->spin_lock.Lock();

    if (UNLIKELY(trace_per_thread_->completed_trace_events.empty())) {
      trace_per_thread_->thread_id = std::this_thread::get_id();
    }
    trace_per_thread_->incompleted_trace_events.push_back(trace_event);

    // Release the spin lock.
    trace_per_thread_->spin_lock.Unlock();

    if (trace_event.use_ftrace) {
      FTrace::Instance()->MarkStart(trace_event);
    }
  }

  // This function is thread safe.
  void CompleteTraceEvent() {
    QCOUNTER_SPAN("MarkEnd");
    const char* trace_event_name;
    int trace_duration_ms = 0;
    bool use_ftrace = true;

    trace_per_thread_->spin_lock.Lock();

    // Update the latest trace event.
    CHECK(!trace_per_thread_->incompleted_trace_events.empty());
    auto& trace_event = trace_per_thread_->incompleted_trace_events.back();
    trace_event.end = ToUnixDoubleSeconds(absl::Now());

    trace_event_name = trace_event.name;
    // Trace event is in second, need convert to ms.
    trace_duration_ms = RoundToInt((trace_event.end - trace_event.start) * 1e3);
    use_ftrace = trace_event.use_ftrace;

    // Finish this trace event.
    trace_per_thread_->completed_trace_events.push_back(trace_event);
    trace_per_thread_->incompleted_trace_events.pop_back();

    // Release the spin lock.
    trace_per_thread_->spin_lock.Unlock();

    Counter::Instance()->AddCounterEvent(trace_event_name, trace_duration_ms);

    if (use_ftrace) {
      FTrace::Instance()->MarkEnd();
    }
  }

  void AddMetaEvent(const MetaEvent& meta_event) {
    trace_per_thread_->spin_lock.Lock();
    trace_per_thread_->meta_events.insert(meta_event);
    trace_per_thread_->spin_lock.Unlock();
  }

  bool ShouldDump();

  std::optional<TraceProto> DumpTrace(const std::string& description);

  std::optional<TraceProto> DumpFTrace(const std::string& description);

 private:
  struct TracePerThread {
    SpinLock spin_lock;
    std::thread::id thread_id;
    std::vector<TraceEvent> incompleted_trace_events;
    boost::circular_buffer<TraceEvent> completed_trace_events{
        kTraceBufferSizePerThread};
    std::unordered_set<MetaEvent, MetaEventOp, MetaEventOp> meta_events;
  };
  class TraceTag;
  folly::ThreadLocal<TracePerThread, TraceTag> trace_per_thread_;
};

bool TraceImpl::ShouldDump() { return FTrace::Instance()->ShouldDump(); }

std::optional<TraceProto> TraceImpl::DumpTrace(const std::string& description) {
  std::vector<MetaEventProto> all_meta_events;
  std::map<std::thread::id, std::vector<TraceEvent>> all_trace_events;

  const auto now = ToUnixDoubleSeconds(absl::Now());
  auto accessor = trace_per_thread_.accessAllThreads();
  for (auto& trace_per_thread : accessor) {
    std::thread::id thread_id;
    std::vector<TraceEvent> incompleted_trace_events;
    boost::circular_buffer<TraceEvent> completed_trace_events(
        kTraceBufferSizePerThread);
    std::unordered_set<MetaEvent, MetaEventOp, MetaEventOp> meta_events;

    trace_per_thread.spin_lock.Lock();
    {
      thread_id = trace_per_thread.thread_id;
      incompleted_trace_events = trace_per_thread.incompleted_trace_events;
      completed_trace_events.swap(trace_per_thread.completed_trace_events);
      meta_events.swap(trace_per_thread.meta_events);
    }
    trace_per_thread.spin_lock.Unlock();

    for (const auto& meta_event : meta_events) {
      MetaEventProto proto;
      proto.set_process_id(meta_event.process_id);
      proto.set_process_name(meta_event.process_name);
      proto.set_thread_id(meta_event.thread_id);
      proto.set_thread_name(meta_event.thread_name);
      all_meta_events.push_back(std::move(proto));
    }

    auto& trace_events_for_thread = all_trace_events[thread_id];
    for (auto& trace_event : incompleted_trace_events) {
      if (trace_event.start >= now - kMaxTraceDuration) {
        trace_event.end = now;
        trace_events_for_thread.push_back(trace_event);
      }
    }

    for (const auto& trace_event : completed_trace_events) {
      if (trace_event.start >= now - kMaxTraceDuration) {
        trace_events_for_thread.push_back(trace_event);
      }
    }
  }

  if (all_meta_events.empty() && all_trace_events.empty()) {
    return std::nullopt;
  }

  TraceProto trace_proto;
  trace_proto.set_module(GetFullNodeName());
  trace_proto.set_description(description);
  // Collect trace and publish.
  for (auto& meta_event : all_meta_events) {
    auto* new_meta_event = trace_proto.add_meta_event();
    *new_meta_event = meta_event;
  }

  for (auto& [thread_id, trace_events] : all_trace_events) {
    TraceEventsPerThreadProto* trace_events_per_thread_proto =
        trace_proto.add_trace();
    trace_events_per_thread_proto->set_thread_id(GetThreadIdStr(thread_id));
    trace_events_per_thread_proto->set_process_id(std::to_string(getpid()));
    for (const auto& trace_event : trace_events) {
      TraceEventProto* trace_event_proto =
          trace_events_per_thread_proto->add_trace_events();
      trace_event_proto->set_name(trace_event.name);
      for (int i = 0; i < trace_event.num_args; ++i) {
        auto* arg = trace_event_proto->add_args();
        arg->set_key(trace_event.args[i].key);
        arg->set_value(trace_event.args[i].value);
      }
      trace_event_proto->set_start(trace_event.start);
      trace_event_proto->set_end(trace_event.end);
    }
  }

  return trace_proto;
}

std::optional<TraceProto> TraceImpl::DumpFTrace(
    const std::string& description) {
  std::string kernel_event;
  if (!FTrace::Instance()->DumpTrace(&kernel_event)) {
    return std::nullopt;
  }

  if (kernel_event.empty()) {
    return std::nullopt;
  }

  TraceProto trace_proto;
  trace_proto.set_module(GetFullNodeName());
  trace_proto.set_description(description);
  if (FLAGS_ftrace_compress_enable) {
    std::string compressed;
    SCOPED_QTRACE("compress_trace");
    QLOG(INFO) << "Kernel event size before compression: "
               << kernel_event.size();
    if (trace_util::Compress(kernel_event, &compressed)) {
      QLOG(INFO) << "Kernel event size after compression: "
                 << compressed.size();
      trace_proto.set_kernel_event(std::move(compressed));
    } else {
      trace_proto.set_kernel_event(std::move(kernel_event));
    }
  } else {
    trace_proto.set_kernel_event(std::move(kernel_event));
  }

  return trace_proto;
}

Trace::Trace() : impl_(std::make_unique<TraceImpl>()) {}

void Trace::AddTraceEvent(const TraceEvent& trace_event) {
  impl_->AddTraceEvent(trace_event);
}

void Trace::CompleteTraceEvent() { impl_->CompleteTraceEvent(); }

void Trace::AddMetaEvent(const MetaEvent& meta_event) {
  impl_->AddMetaEvent(meta_event);
}

bool Trace::ShouldDump() { return impl_->ShouldDump(); }

std::optional<TraceProto> Trace::DumpTrace(const std::string& description) {
  return impl_->DumpTrace(description);
}

std::optional<TraceProto> Trace::DumpFTrace(const std::string& description) {
  return impl_->DumpFTrace(description);
}

}  // namespace qcraft
