#include "onboard/global/trace_util.h"

#include <chrono>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "boost/filesystem/operations.hpp"
#include "boost/process/system.hpp"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/utils/temp_file.h"

namespace qcraft {

namespace {

class TimeCount {
 public:
  TimeCount(const std::string &tag, int count, bool verbose = false)
      : tag_(tag),
        count_(count),
        verbose_(verbose),
        start_(std::chrono::system_clock::now()) {}

  ~TimeCount() {
    auto end = std::chrono::system_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start_)
            .count();
    const auto duration_per_count = duration * 1.0 / count_;
    if ((duration_per_count > 0) && (verbose_ || duration_per_count > 800.0)) {
      LOG(ERROR) << absl::StrFormat("%s: %f ms", tag_, duration_per_count);
    }
  }

 private:
  std::string tag_;
  int count_;
  bool verbose_;
  std::chrono::system_clock::time_point start_;
};

class TraceUtilTest : public testing::Test {
 protected:
  void RecoverFile(const std::string &ftrace_filename,
                   const std::string &recovered_ftrace_filename) {
    std::string kernel_event;
    {
      std::ifstream ifs(ftrace_filename);
      std::ostringstream oss;
      oss << ifs.rdbuf();
      kernel_event = oss.str();
    }

    std::string recovered_kernel_event;
    {
      TimeCount time_count(ftrace_filename, 1, true);
      trace_util::RecoverKernelEvent(kernel_event, &recovered_kernel_event);
    }

    std::ofstream ofs(recovered_ftrace_filename);
    ofs << recovered_kernel_event;
  }

  void TimestampFile(const std::string &ftrace_filename,
                     const std::string &timestamp_ftrace_filename) {
    std::string kernel_event;
    {
      std::ifstream ifs(ftrace_filename);
      std::ostringstream oss;
      oss << ifs.rdbuf();
      kernel_event = oss.str();
    }

    std::string timestamp_kernel_event;
    {
      TimeCount time_count(ftrace_filename, 1, true);
      trace_util::TimestampToKernelEvent(kernel_event, &timestamp_kernel_event);
    }

    std::ofstream ofs(timestamp_ftrace_filename);
    ofs << timestamp_kernel_event;
  }

  void DumpRecoveredToHtml(const std::string &ftrace_filename,
                           const std::string &html_ftrace_filename) {
    const auto recovered_ftrace_filename(ftrace_filename + ".recovered");
    RecoverFile(ftrace_filename, recovered_ftrace_filename);
    {
      const TempFile recovered_ftrace_file(recovered_ftrace_filename);
      const TempFile html_ftrace_file(html_ftrace_filename);
      const auto trace2html = absl::StrCat(
          "/usr/local/catapult/tracing/bin/trace2html ",
          recovered_ftrace_filename, " --output=", html_ftrace_filename);
      boost::process::system(trace2html);
    }
  }

  void DumpTimestampToHtml(const std::string &ftrace_filename,
                           const std::string &html_ftrace_filename) {
    const auto timestamp_ftrace_filename(ftrace_filename + ".timestamp");
    TimestampFile(ftrace_filename, timestamp_ftrace_filename);
    {
      const TempFile timestamp_ftrace_file(timestamp_ftrace_filename);
      const TempFile html_ftrace_file(html_ftrace_filename);
      const auto trace2html = absl::StrCat(
          "/usr/local/catapult/tracing/bin/trace2html ",
          timestamp_ftrace_filename, " --output=", html_ftrace_filename);
      boost::process::system(trace2html);
    }
  }
};

TEST_F(TraceUtilTest, TestRecoverKernelEvent) {
  DumpRecoveredToHtml("onboard/global/data/test_recover.ftrace",
                      "onboard/global/data/test_recover.ftrace.html");
}

TEST_F(TraceUtilTest, TestTimestamToKernelEvent) {
  DumpTimestampToHtml("onboard/global/data/test_timestamp.ftrace",
                      "onboard/global/data/test_timestamp.ftrace.html");
}

}  // namespace
}  // namespace qcraft
