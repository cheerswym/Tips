#ifndef ONBOARD_GLOBAL_BUFFERED_LOGGER_H_
#define ONBOARD_GLOBAL_BUFFERED_LOGGER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "onboard/global/singleton.h"
#include "onboard/proto/buffered_log.pb.h"

namespace qcraft {
enum LogSeverity { INFO, WARNING, ERROR, FATAL, QISSUE };

struct LogItem {
  LogItem(const char* file_name_, int file_line_, LogSeverity severity_,
          int64_t capture_timestamp_ms_, const std::thread::id& thread_id_,
          const std::string& msg_)
      : file_name(file_name_),
        file_line(file_line_),
        severity(severity_),
        capture_timestamp_ms(capture_timestamp_ms_),
        thread_id(thread_id_),
        msg(msg_) {}

  const char* const file_name;
  const int file_line;
  const LogSeverity severity;
  const int64_t capture_timestamp_ms;
  const std::thread::id thread_id;
  const std::string msg;
};

class BufferedLoggerImpl {
 public:
  virtual LogProto DumpLogProto(const std::string& module_name) = 0;
  virtual void AddLog(const char* file_name, int file_line,
                      LogSeverity severity, int64_t capture_timestamp_ms,
                      const std::thread::id& thread_id, const std::string& msg,
                      bool dump_now) = 0;
  virtual void SetFatalHandler(
      const std::function<void(const LogItem&)>& fatal_handler) = 0;
  virtual void SetDumpHandler(const std::function<void()>& dump_handler) = 0;
  virtual ~BufferedLoggerImpl() {}
};

class BufferedLogger {
 public:
  // Dump the log collected in proto format.
  LogProto DumpLogProto(const std::string& module_name);

  // Add Log , thread save
  void AddLog(const char* file_name, int file_line, LogSeverity severity,
              int64_t capture_timestamp_ms, const std::thread::id& thread_id,
              const std::string& msg, bool dump_now);

  void SetFatalHandler(
      const std::function<void(const LogItem&)>& fatal_handler);

  void SetDumpHandler(const std::function<void()>& dump_handler);

 private:
  std::unique_ptr<BufferedLoggerImpl> impl_;
  DECLARE_SINGLETON(BufferedLogger);
};

class BufferedLoggerWrapper {
 public:
  BufferedLoggerWrapper(const char* file_name, int file_line,
                        LogSeverity severity, bool dump_now = false)
      : file_name_(file_name),
        file_line_(file_line),
        severity_(severity),
        dump_now_(dump_now) {}

  ~BufferedLoggerWrapper();

  std::ostream& stream() { return stream_; }

 protected:
  const char* file_name_;
  const int file_line_;
  const LogSeverity severity_;
  std::ostringstream stream_;
  const bool dump_now_;
};

class FatalBufferedLoggerWrapper : public BufferedLoggerWrapper {
 public:
  FatalBufferedLoggerWrapper(const char* file_name, int file_line,
                             LogSeverity severity, bool dump_now = false)
      : BufferedLoggerWrapper(file_name, file_line, severity, dump_now) {}

  __attribute__((noreturn)) ~FatalBufferedLoggerWrapper();
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_BUFFERED_LOGGER_H_
