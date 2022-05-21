#ifndef ONBOARD_GLOBAL_LOGGING_H_
#define ONBOARD_GLOBAL_LOGGING_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iosfwd>

#include "onboard/base/macros.h"

#define QLOG_WITH_MODULE(severity, dump_now) \
  QLOG_WITH_MODULE_##severity(severity, dump_now)

#define QLOG_WITH_MODULE_INFO(severity, dump_now)                        \
  qcraft::BufferedLoggerWrapper(__FILE__, __LINE__,                      \
                                qcraft::LogSeverity::severity, dump_now) \
      .stream()
#define QLOG_WITH_MODULE_WARNING(severity, dump_now)                     \
  qcraft::BufferedLoggerWrapper(__FILE__, __LINE__,                      \
                                qcraft::LogSeverity::severity, dump_now) \
      .stream()
#define QLOG_WITH_MODULE_ERROR(severity, dump_now)                       \
  qcraft::BufferedLoggerWrapper(__FILE__, __LINE__,                      \
                                qcraft::LogSeverity::severity, dump_now) \
      .stream()
#define QLOG_WITH_MODULE_FATAL(severity, dump_now)                            \
  qcraft::FatalBufferedLoggerWrapper(__FILE__, __LINE__,                      \
                                     qcraft::LogSeverity::severity, dump_now) \
      .stream()

#define QLOG_WITH_MODULE_QISSUE(severity, dump_now)                      \
  qcraft::BufferedLoggerWrapper(__FILE__, __LINE__,                      \
                                qcraft::LogSeverity::severity, dump_now) \
      .stream()

// This class is used to explicitly ignore values in the conditional
// logging macros.  This avoids compiler warnings like "value computed
// is not used" and "statement has no effect".
class LogMessageVoidifier {
 public:
  LogMessageVoidifier() {}
  // This has to be an operator with a precedence lower than << but
  // higher than ?:
  void operator&(std::ostream&) {}
};

class LogEveryNState {
 public:
  bool ShouldLog(int n) {
    if (n <= 0) {
      return false;
    }
    const uint32_t value = counter_.load(std::memory_order_relaxed);
    counter_.store(value + 1, std::memory_order_relaxed);
    return (value % n == 0);
  }

 private:
  std::atomic<uint32_t> counter_{0};
};

class LogEveryNSecState {
 public:
  bool ShouldLog(double seconds) {
    // const auto now = absl::Now();
    const auto now = std::chrono::steady_clock().now();
    const int64_t now_cycles =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch())
            .count();
    int64_t next_cycles = next_log_time_cycles_.load(std::memory_order_relaxed);
    do {
      if (now_cycles <= next_cycles) {
        return false;
      }
    } while (!next_log_time_cycles_.compare_exchange_weak(
        next_cycles,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            (now + std::chrono::milliseconds(static_cast<int>(seconds * 1000)))
                .time_since_epoch())
            .count(),
        std::memory_order_relaxed, std::memory_order_relaxed));
    return true;
  }

 private:
  // Cycle count according to CycleClock that we should next log at.
  std::atomic<int64_t> next_log_time_cycles_{0};
};

#define LOGGING_INTERNAL_STATEFUL_CONDITION(kind, condition, arg)           \
  for (bool logging_internal_stateful_condition_do_log(condition);          \
       logging_internal_stateful_condition_do_log;                          \
       logging_internal_stateful_condition_do_log = false)                  \
    for (static Log##kind##State logging_internal_stateful_condition_state; \
         logging_internal_stateful_condition_do_log &&                      \
         logging_internal_stateful_condition_state.ShouldLog(arg);          \
         logging_internal_stateful_condition_do_log = false)

#define QLOG_WITH_MODULE_IF(severity, condition)                              \
  static_cast<void>(0),                                                       \
      LIKELY(!(condition))                                                    \
          ? (void)0                                                           \
          : LogMessageVoidifier() &                                           \
                qcraft::BufferedLoggerWrapper(                                \
                    __FILE__, __LINE__, qcraft::LogSeverity::severity, false) \
                    .stream()

#define QLOG_WITH_MODULE_NOW_IF(severity, condition)                         \
  static_cast<void>(0),                                                      \
      LIKELY(!(condition))                                                   \
          ? (void)0                                                          \
          : LogMessageVoidifier() &                                          \
                qcraft::BufferedLoggerWrapper(                               \
                    __FILE__, __LINE__, qcraft::LogSeverity::severity, true) \
                    .stream()

#endif  // ONBOARD_GLOBAL_LOGGING_H_
