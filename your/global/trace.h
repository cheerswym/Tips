#ifndef ONBOARD_GLOBAL_TRACE_H_
#define ONBOARD_GLOBAL_TRACE_H_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "onboard/base/macros.h"
#include "onboard/global/counter.h"
#include "onboard/global/ftrace.h"
#include "onboard/lite/check.h"
#include "onboard/proto/trace.pb.h"
#include "onboard/utils/time_util.h"

#define CONCAT_TOKEN_(foo, bar) CONCAT_TOKEN_IMPL_(foo, bar)
#define CONCAT_TOKEN_IMPL_(foo, bar) foo##bar

// For onboard tracing.

#define FUNC_QTRACE() \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(__func__, true)

#define FUNC_QTRACE_LITE() \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(__func__, false)

#define SCOPED_QTRACE(name)                             \
  static_assert(name "", "Must pass a string literal"); \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(name, true)

#define SCOPED_QTRACE_ARG1(name, key1, val1)                             \
  static_assert(name "", "Must pass a string literal");                  \
  static_assert(key1 "", "Must pass a string literal for argument key"); \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(            \
      name, key1, qcraft::trace_util::ToString(val1), true)

#define SCOPED_QTRACE_ARG2(name, key1, val1, key2, val2)                 \
  static_assert(name "", "Must pass a string literal");                  \
  static_assert(key1 "", "Must pass a string literal for argument key"); \
  static_assert(key2 "", "Must pass a string literal for argument key"); \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(            \
      name, key1, qcraft::trace_util::ToString(val1), key2,              \
      qcraft::trace_util::ToString(val2), true)

#define SCOPED_QTRACE_ARG3(name, key1, val1, key2, val2, key3, val3)     \
  static_assert(name "", "Must pass a string literal");                  \
  static_assert(key1 "", "Must pass a string literal for argument key"); \
  static_assert(key2 "", "Must pass a string literal for argument key"); \
  static_assert(key3 "", "Must pass a string literal for argument key"); \
  qcraft::ScopedTrace CONCAT_TOKEN_(scoped_trace_, __LINE__)(            \
      name, key1, qcraft::trace_util::ToString(val1), key2,              \
      qcraft::trace_util::ToString(val2), key3,                          \
      qcraft::trace_util::ToString(val3), true)

#define SCOPED_QTRACE_IF(name, cond)                                \
  static_assert(name "", "Must pass a string literal");             \
  std::unique_ptr<qcraft::ScopedTrace> CONCAT_TOKEN_(scoped_trace_, \
                                                     __LINE__) =    \
      (cond) ? std::make_unique<qcraft::ScopedTrace>(name, true) : nullptr;

namespace qcraft {

struct MetaEvent {
  std::string process_id;
  std::string thread_id;
  std::string process_name;
  std::string thread_name;
};
struct MetaEventOp {
  bool operator()(const MetaEvent& e1, const MetaEvent& e2) const {
    return e1.process_id == e2.process_id && e1.thread_id == e2.thread_id &&
           e1.process_name == e2.process_name &&
           e1.thread_name == e2.thread_name;
  }
  size_t operator()(const MetaEvent& e) const {
    const size_t v1 = std::hash<std::string>()(e.process_id);
    const size_t v2 = std::hash<std::string>()(e.thread_id);
    const size_t v3 = std::hash<std::string>()(e.process_name);
    const size_t v4 = std::hash<std::string>()(e.thread_name);
    return v1 ^ v2 ^ v3 ^ v4;
  }
};

class TraceImpl;
class Trace {
 public:
  // This function is thread safe.
  void AddTraceEvent(const TraceEvent& trace_event);

  void CompleteTraceEvent();

  void AddMetaEvent(const MetaEvent& meta_event);

  bool ShouldDump();

  /**
   * Dump trace with a rate limitor.
   *
   * @return the trace, if ratelimitor does not pass, will return nullopt.
   */
  std::optional<TraceProto> DumpTrace(const std::string& description);

  std::optional<TraceProto> DumpFTrace(const std::string& description);

 private:
  std::unique_ptr<TraceImpl> impl_;

  DECLARE_SINGLETON(Trace);
};

class ScopedTrace {
 public:
  explicit ScopedTrace(const char* name, bool use_ftrace) {
    TraceEvent trace_event;
    trace_event.name = name;
    trace_event.start = ToUnixDoubleSeconds(absl::Now());
    trace_event.use_ftrace = use_ftrace;

    Trace::Instance()->AddTraceEvent(trace_event);
  }

  ScopedTrace(const char* name, const char* key1, const std::string& val1,
              bool use_ftrace) {
    QCHECK_LE(val1.size(), TraceEvent::TraceArg::kValueMaxLength);
    TraceEvent trace_event;
    trace_event.name = name;
    trace_event.start = ToUnixDoubleSeconds(absl::Now());
    trace_event.num_args = 1;
    trace_event.args[0].key = key1;
    snprintf(trace_event.args[0].value, val1.size() + 1, "%s", val1.c_str());
    trace_event.use_ftrace = use_ftrace;

    Trace::Instance()->AddTraceEvent(trace_event);
  }

  ScopedTrace(const char* name, const char* key1, const std::string& val1,
              const char* key2, const std::string& val2, bool use_ftrace) {
    QCHECK_LE(val1.size(), TraceEvent::TraceArg::kValueMaxLength);
    QCHECK_LE(val2.size(), TraceEvent::TraceArg::kValueMaxLength);
    TraceEvent trace_event;
    trace_event.name = name;
    trace_event.start = ToUnixDoubleSeconds(absl::Now());
    trace_event.num_args = 2;
    trace_event.args[0].key = key1;
    snprintf(trace_event.args[0].value, val1.size() + 1, "%s", val1.c_str());
    trace_event.args[1].key = key2;
    snprintf(trace_event.args[1].value, val2.size() + 1, "%s", val2.c_str());
    trace_event.use_ftrace = use_ftrace;

    Trace::Instance()->AddTraceEvent(trace_event);
  }

  ScopedTrace(const char* name, const char* key1, const std::string& val1,
              const char* key2, const std::string& val2, const char* key3,
              const std::string& val3, bool use_ftrace) {
    QCHECK_LE(val1.size(), TraceEvent::TraceArg::kValueMaxLength);
    QCHECK_LE(val2.size(), TraceEvent::TraceArg::kValueMaxLength);
    QCHECK_LE(val3.size(), TraceEvent::TraceArg::kValueMaxLength);
    TraceEvent trace_event;
    trace_event.name = name;
    trace_event.start = ToUnixDoubleSeconds(absl::Now());
    trace_event.num_args = 3;
    trace_event.args[0].key = key1;
    snprintf(trace_event.args[0].value, val1.size() + 1, "%s", val1.c_str());
    trace_event.args[1].key = key2;
    snprintf(trace_event.args[1].value, val2.size() + 1, "%s", val2.c_str());
    trace_event.args[2].key = key3;
    snprintf(trace_event.args[2].value, val3.size() + 1, "%s", val3.c_str());
    trace_event.use_ftrace = use_ftrace;

    Trace::Instance()->AddTraceEvent(trace_event);
  }

  ~ScopedTrace() { Trace::Instance()->CompleteTraceEvent(); }
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_TRACE_H_
