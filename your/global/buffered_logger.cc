#include "onboard/global/buffered_logger.h"

#include <algorithm>
#include <cstdlib>
#include <deque>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "folly/SharedMutex.h"
#include "folly/ThreadLocal.h"
#include "glog/logging.h"
#include "onboard/global/car_common.h"
#include "onboard/global/spin_lock.h"
#include "onboard/utils/stacktrace.h"

namespace folly {
// Explicitly instantiate SharedMutex here:
template class SharedMutexImpl<true>;
template class SharedMutexImpl<false>;
}  // namespace folly

namespace qcraft {
namespace {

std::string GetThreadIdStr(std::thread::id thread_id) {
  std::ostringstream ss;
  ss << thread_id;
  return ss.str();
}

LogItemProto::LogSeverity GetProtoSeverity(LogSeverity severity) {
  switch (severity) {
    case INFO:
      return LogItemProto::INFO;
    case WARNING:
      return LogItemProto::WARNING;
    case ERROR:
      return LogItemProto::ERROR;
    case FATAL:
      return LogItemProto::FATAL;
    case QISSUE:
      return LogItemProto::QISSUE;
  }
}

google::LogSeverity GetGlogSeverity(LogSeverity severity) {
  switch (severity) {
    case INFO:
      return google::INFO;
    case WARNING:
      return google::WARNING;
    case ERROR:
      return google::ERROR;
    case FATAL:
      return google::FATAL;
    case QISSUE:
      return google::ERROR;
  }
}
}  // namespace

class GlogDummy : public BufferedLoggerImpl {
 public:
  LogProto DumpLogProto(const std::string& module_name) override {
    LogProto proto;
    auto* log_item = proto.add_log_item();
    log_item->set_module_name(module_name);
    return proto;
  }

  // This function is thread safe.
  void AddLog(const char* file_name, int file_line, LogSeverity severity,
              int64_t capture_timestamp_ms, const std::thread::id& thread_id,
              const std::string& msg, bool dump_now) override {
    google::LogMessage(file_name, file_line, GetGlogSeverity(severity)).stream()
        << msg;
  }

  void SetFatalHandler(
      const std::function<void(const LogItem&)>& fatal_handler) override {}

  void SetDumpHandler(const std::function<void()>& dump_handler) override {}

  virtual ~GlogDummy() {}
};

class BufferedLoggerWithThreadLocal : public BufferedLoggerImpl {
 public:
  LogProto DumpLogProto(const std::string& module_name) override {
    LogProto proto;
    auto accessor = log_per_thread_.accessAllThreads();
    for (auto& log_per_thread : accessor) {
      log_per_thread.spin_lock.Lock();
      log_per_thread.active_buf.swap(log_per_thread.flushing_buf);
      log_per_thread.spin_lock.Unlock();
      auto& flushing_buf = log_per_thread.flushing_buf;
      while (!flushing_buf.empty()) {
        const auto& item = flushing_buf.front();
        auto* log_item = proto.add_log_item();
        log_item->set_file_name(item.file_name);
        log_item->set_file_line(item.file_line);
        log_item->set_severity(GetProtoSeverity(item.severity));
        log_item->set_module_name(module_name);
        log_item->set_capture_timestamp_ms(item.capture_timestamp_ms);
        log_item->set_thread_id(GetThreadIdStr(item.thread_id));
        log_item->set_msg(item.msg);
        flushing_buf.pop_front();
      }
    }
    return proto;
  }

  // This function is thread safe.
  void AddLog(const char* file_name, int file_line, LogSeverity severity,
              int64_t capture_timestamp_ms, const std::thread::id& thread_id,
              const std::string& msg, bool dump_now) override {
    std::stringstream ss;
    if (severity == FATAL) {
      ss << msg << "\n";
      GetStackTrace(ss);
    } else {
      ss << msg;
    }
    log_per_thread_->spin_lock.Lock();
    log_per_thread_->active_buf.emplace_back(file_name, file_line, severity,
                                             capture_timestamp_ms, thread_id,
                                             ss.str());
    log_per_thread_->spin_lock.Unlock();
    const LogItem item(file_name, file_line, severity, capture_timestamp_ms,
                       thread_id, ss.str());
    if (severity == FATAL) {
      fatal_handler_(item);
    } else if (dump_now) {
      dump_handler_();
      google::LogMessage(file_name, file_line, GetGlogSeverity(severity))
              .stream()
          << msg;
    } else {
      // for non_fatal log, also print on the screen.
      google::LogMessage(file_name, file_line, GetGlogSeverity(severity))
              .stream()
          << msg;
    }
  }

  void SetFatalHandler(
      const std::function<void(const LogItem&)>& fatal_handler) override {
    fatal_handler_ = fatal_handler;
  }

  void SetDumpHandler(const std::function<void()>& dump_handler) override {
    dump_handler_ = dump_handler;
  }

  virtual ~BufferedLoggerWithThreadLocal() {}

 private:
  struct PerThread {
    std::deque<LogItem> active_buf;
    std::deque<LogItem> flushing_buf;
    SpinLock spin_lock;
  };
  class Tag;

  std::function<void(const LogItem&)> fatal_handler_ =
      [](const LogItem& log_item) {
        LOG(ERROR) << "In onboard model, make sure setting up fatal handler";
        LOG(ERROR) << "Here we just crash and print what is in the fatal log:";
        const auto time = absl::FromUnixMillis(log_item.capture_timestamp_ms);
        LOG(FATAL) << "[FATAL] "
                   << absl::FormatTime("%E4Y/%m/%d %H:%M:%E3S ", time,
                                       absl::LocalTimeZone())
                   << log_item.thread_id << " " << log_item.file_name << ":"
                   << log_item.file_line << "]" << log_item.msg;
      };

  std::function<void()> dump_handler_ = []() {
    LOG(ERROR) << "Please set dump_handler.";
  };

  folly::ThreadLocal<PerThread, Tag> log_per_thread_;
};

BufferedLoggerWrapper::~BufferedLoggerWrapper() {
  BufferedLogger::Instance()->AddLog(
      file_name_, file_line_, severity_, absl::ToUnixMillis(absl::Now()),
      std::this_thread::get_id(), stream_.str(), dump_now_);
}

FatalBufferedLoggerWrapper::~FatalBufferedLoggerWrapper() {
  BufferedLogger::Instance()->AddLog(
      file_name_, file_line_, severity_, absl::ToUnixMillis(absl::Now()),
      std::this_thread::get_id(), stream_.str(), dump_now_);
  exit(1);
}

LogProto BufferedLogger::DumpLogProto(const std::string& module_name) {
  return impl_->DumpLogProto(module_name);
}

void BufferedLogger::AddLog(const char* file_name, int file_line,
                            LogSeverity severity, int64_t capture_timestamp_ms,
                            const std::thread::id& thread_id,
                            const std::string& msg, bool dump_now) {
  impl_->AddLog(file_name, file_line, severity, capture_timestamp_ms, thread_id,
                msg, dump_now);
}

void BufferedLogger::SetFatalHandler(
    const std::function<void(const LogItem&)>& fatal_handler) {
  impl_->SetFatalHandler(fatal_handler);
}

void BufferedLogger::SetDumpHandler(const std::function<void()>& dump_handler) {
  impl_->SetDumpHandler(dump_handler);
}

BufferedLogger::BufferedLogger()
    : impl_(std::make_unique<BufferedLoggerWithThreadLocal>()) {
  if (IsDSimMode()) {
    impl_.reset(new GlogDummy());
  }
}

}  // namespace qcraft
