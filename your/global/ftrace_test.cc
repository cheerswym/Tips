#include "onboard/global/ftrace.h"

#include <chrono>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "boost/filesystem/operations.hpp"
#include "boost/process/system.hpp"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/global/trace_util.h"
#include "onboard/utils/temp_file.h"

namespace qcraft {

DECLARE_bool(ftrace_enable);
DECLARE_int32(ftrace_buffer_size_kb);
DECLARE_int32(ftrace_dump_trace_interval);

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

class FTraceTest : public testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_ftrace_buffer_size_kb = 512;
    FLAGS_ftrace_dump_trace_interval = 3;

    static std::once_flag flag;
    std::call_once(flag, []() { FTrace::Instance()->Init(); });
  }

  virtual void TearDown() {
    FLAGS_ftrace_buffer_size_kb = 64;
    FLAGS_ftrace_dump_trace_interval = 0;

    // FTrace::Instance()->Uninit();
  }

  void TraceMark(const std::string &prefix, int useconds, int stack_depth) {
    for (int index = 0; index < stack_depth; index++) {
      const auto name = absl::StrFormat("%s_%d", prefix, index);
      TraceEvent trace_event = {.name = name.c_str(),
                                .num_args = 2,
                                .args[0].key = "key_1",
                                .args[0].value = "value_1",
                                .args[1].key = "key_2",
                                .args[1].value = "value_2"};
      FTrace::Instance()->MarkStart(trace_event);
      if (useconds > 0) {
        usleep(useconds / 10);
      }
    }

    for (int index = 0; index < stack_depth; index++) {
      if (useconds > 0) {
        usleep(useconds / 10);
      }
      FTrace::Instance()->MarkEnd();
    }
  }

  void TraceMarks(const std::string &prefix, int threads_count,
                  int traces_count, int stack_depth, bool sync) {
    std::vector<std::thread> mark_threads;
    for (int index = 0; index < threads_count; index++) {
      mark_threads.push_back(std::thread(
          [this](const std::string &prefix, int useconds, int traces_count,
                 int stack_depth) {
            const auto name_prefix =
                absl::StrFormat("%s_%ul", prefix, pthread_self());

            int count = traces_count;
            while (count--) {
              std::string name;
              if (count == traces_count - 1) {
                name = absl::StrFormat("B_%s_%d", name_prefix, count);
              } else if (count == 0) { /* code */
                name = absl::StrFormat("E_%s_%d", name_prefix, count);
              } else {
                name = absl::StrFormat("M_%s_%d", name_prefix, count);
              }

              TraceMark(name, useconds, stack_depth);
            }
          },
          prefix, (threads_count - index) * 1000, traces_count,
          index % stack_depth + 1));
      usleep(index * 1000);
    }

    if (sync) {
      for (auto &worker : mark_threads) {
        worker.join();
      }
    } else {
      for (auto &worker : mark_threads) {
        worker.detach();
      }
    }
  }

  std::string DumpTrace(const std::string &ftrace_filename) {
    std::string kernel_event;
    {
      TimeCount time_count(ftrace_filename + "::dumptrace", 1);
      EXPECT_TRUE(FTrace::Instance()->DumpTrace(&kernel_event));
    }

    return kernel_event;
  }

  bool DumpToFile(const std::string &ftrace_filename) {
    const auto kernel_event = DumpTrace(ftrace_filename);
    if (kernel_event.empty()) {
      return false;
    }

    EXPECT_GT(kernel_event.size(), 300 * 1024);

    {
      TimeCount time_count(ftrace_filename + "::writetrace", 1);
      std::ofstream ofs(ftrace_filename);
      ofs << kernel_event;
    }

    return true;
  }

  void DumpToHtml(const std::string &ftrace_filename,
                  const std::string &html_ftrace_filename) {
    if (!FLAGS_ftrace_enable) {
      return;
    }

    if (DumpToFile(ftrace_filename)) {
      const TempFile ftrace_file(ftrace_filename);
      const auto trace2html =
          absl::StrCat("/usr/local/catapult/tracing/bin/trace2html ",
                       ftrace_filename, " --output=", html_ftrace_filename);
      boost::process::system(trace2html);
    }
  }
};

TEST_F(FTraceTest, TestTraceMark) {
  int count = 10000;
  {
    TimeCount time_count("test_trace_mark", count, true);
    while (count--) {
      TraceMark("test_trace_mark", 0, 3);
    }
  }
}

TEST_F(FTraceTest, TestTraceSingleThread) {
  {
    TimeCount time_count("test_trace_single_thread_mark", 1, true);
    TraceMark("test_trace", 100000, 10);
  }

  {
    TimeCount time_count("test_trace_single_thread_dump", 1, true);
    DumpToHtml("/qcraft/TestTraceSingleThread.ftrace",
               "/qcraft/TestTraceSingleThread.ftrace.html");
  }
}

TEST_F(FTraceTest, TestTraceMultiThread_V1) {
  if (!FLAGS_ftrace_enable) {
    return;
  }

  {
    TimeCount time_count("test_multithread_marks_v1", 1, true);
    TraceMarks("test_multithread", 30, 70, 10, true);
  }

  {
    TimeCount time_count("test_multithread_dump_html", 1, true);
    DumpToHtml("/qcraft/TestTraceMultiThread.ftrace",
               "/qcraft/TestTraceMultiThread.ftrace.html");
  }
}

TEST_F(FTraceTest, TestTraceMultiThread_V2) {
  if (!FLAGS_ftrace_enable) {
    return;
  }

  {
    TimeCount time_count("test_multithread_marks_v2", 1, true);
    TraceMarks("test_multithread", 30, 70, 10, false);
  }

  auto dump_count = 10;
  while (dump_count--) {
    const auto ftrace_filename =
        absl::StrFormat("/qcraft/TestTraceMultiThread_%d.ftrace", dump_count);
    const auto html_ftrace_filename = absl::StrFormat(
        "/qcraft/TestTraceMultiThread_%d.ftrace.html", dump_count);

    TimeCount time_count(ftrace_filename, 1, true);
    DumpToHtml(ftrace_filename, html_ftrace_filename);
  }
}

TEST_F(FTraceTest, TestTraceMultiThread_V3) {
  if (!FLAGS_ftrace_enable) {
    return;
  }

  {
    TimeCount time_count("test_multithread_marks_v3", 1, true);
    TraceMarks("test_multithread", 30, 70, 10, false);
  }

  int threads_count = 30;
  std::vector<std::thread> dump_threads;
  for (int index = 0; index < threads_count; index++) {
    dump_threads.push_back(std::thread(
        [this, threads_count, index](int useconds) {
          int dump_count = threads_count;
          while (dump_count--) {
            usleep(useconds);
            const auto ftrace_filename = absl::StrFormat(
                "/qcraft/TestTraceMultiThread_%d_%d.ftrace", index, dump_count);
            const auto html_ftrace_filename = absl::StrFormat(
                "/qcraft/TestTraceMultiThread_%d_%d.ftrace.html", index,
                dump_count);

            TimeCount time_count(ftrace_filename, 1, true);
            DumpToHtml(ftrace_filename, html_ftrace_filename);
          }
        },
        1000 * 100 * index));
  }

  for (auto &worker : dump_threads) {
    worker.join();
  }
}

TEST_F(FTraceTest, TestTraceMultiProcess) {
  if (!FLAGS_ftrace_enable) {
    return;
  }

  static const int kProcessCount = 10;

  std::vector<pid_t> pids;
  pids.reserve(kProcessCount);
  for (int index = 0; index < kProcessCount; ++index) {
    if ((pids[index] = fork()) < 0) {
      perror("fork");
      abort();
    } else if (pids[index] == 0) {
      LOG(INFO) << "[C][B]Trace, getpid(): " << getpid();
      {
        {
          const auto name =
              absl::StrFormat("test_multiprocess_marks_%d", getpid());
          TimeCount time_count(name, 1, true);
          // reduce trace size due to process count.
          TraceMarks(name, 30, 14, 10, false);
        }

        int threads_count = 30;
        std::vector<std::thread> dump_threads;
        for (int index = 0; index < threads_count; index++) {
          dump_threads.push_back(std::thread(
              [this, threads_count, index](int useconds) {
                auto dump_count = threads_count;
                while (dump_count--) {
                  usleep(useconds);
                  const auto ftrace_filename = absl::StrFormat(
                      "/qcraft/TestTraceMultiProcess_%d_%d_%d.ftrace", getpid(),
                      index, dump_count);
                  const auto html_ftrace_filename = absl::StrFormat(
                      "/qcraft/TestTraceMultiProcess_%d_%d_%d.ftrace.html",
                      getpid(), index, dump_count);

                  TimeCount time_count(ftrace_filename, 1, true);
                  DumpToFile(ftrace_filename);
                }
              },
              1000 * 100 * index));
        }

        for (auto &worker : dump_threads) {
          worker.join();
        }
      }
      LOG(INFO) << "[C][E]Trace, getpid(): " << getpid();

      exit(0);
    }
  }

  LOG(INFO) << "[P][B]TraceMarks";
  TraceMarks("test_multiprocess_marks", 30, 70, 10, true);
  LOG(INFO) << "[P][E]TraceMarks";
  LOG(INFO) << "[P][B]DumpToFile";
  DumpToFile("/qcraft/TestTraceMultiProcess.ftrace");
  LOG(INFO) << "[P][E]DumpToFile";

  /* Wait for children to exit. */
  int status;
  for (int index = 0; index < kProcessCount; ++index) {
    int pid = wait(&status);
    LOG(INFO) << absl::StrFormat("Child with PID %d exited with status 0x%x",
                                 pid, status);
  }
}

TEST_F(FTraceTest, TestTraceCompress) {
  if (!FLAGS_ftrace_enable) {
    return;
  }

  {
    TimeCount time_count("test_trace_single_thread_mark", 1, true);
    TraceMark("test_trace", 200000, 50);
  }

  std::string ftrace_filename("/qcraft/TestTraceCompress.ftrace");
  const auto kernel_event = DumpTrace(ftrace_filename);
  if (kernel_event.empty()) {
    return;
  }

  {
    // std::ofstream ofs_origin(ftrace_filename);
    // ofs_origin << kernel_event;

    std::string compressed;
    {
      TimeCount time_count(ftrace_filename + "::compress", 1, true);
      EXPECT_TRUE(trace_util::Compress(kernel_event, &compressed));
    }

    // std::ofstream ofs_compressed(ftrace_filename + ".compress");
    // ofs_compressed << compressed;
    EXPECT_EQ(compressed[0], trace_util::kCompressTag);

    TraceProto trace_proto;
    trace_proto.set_kernel_event(compressed);

    std::string compressed_proto = trace_proto.kernel_event();

    EXPECT_EQ(compressed_proto.compare(compressed), 0);

    EXPECT_EQ(compressed_proto[0], trace_util::kCompressTag);

    // std::ofstream ofs_compressed_proto(ftrace_filename + ".compress_proto");
    // ofs_compressed_proto << compressed_proto;

    std::string decompressed;
    {
      TimeCount time_count(ftrace_filename + "::decompress", 1, true);
      EXPECT_TRUE(trace_util::Decompress(compressed_proto, &decompressed));
    }

    // std::ofstream ofs_decompressed(ftrace_filename + ".decompress");
    // ofs_decompressed << decompressed;

    EXPECT_EQ(kernel_event.compare(decompressed), 0);
  }
}

}  // namespace
}  // namespace qcraft
