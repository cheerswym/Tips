#include "onboard/global/ftrace.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/time/clock.h"
#include "folly/ThreadLocal.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/thread_util.h"

namespace qcraft {

DEFINE_bool(ftrace_enable, false, "true is for enable ftrace; false is diable");
DEFINE_bool(ftrace_compress_enable, true,
            "true is for enable compress ftrace buffers; false is diable");
DEFINE_bool(ftrace_overwrite_enable, true,
            "true is for overwrite ftrace buffers; false is stop tracing while "
            "buffer have filled up");
DEFINE_int32(ftrace_buffer_size_kb, 1024,
             "ftrace buffer size per cpu");  // 3s data cache
DEFINE_bool(ftrace_buffer_size_per_cpu_enable, false,
            "true is for applying buffer_size_kb per cpu; false is applying "
            "for all cpu");
DEFINE_string(ftrace_events, "sched", "ftrace events, like 'sched,irqoff'");
DEFINE_string(ftrace_functions, "",
              "ftrace functions, like 'do_filp_open,filp_close'");
DEFINE_int32(ftrace_dump_trace_interval, 2, "ftrace dump trace interval");
DEFINE_int32(ftrace_instances_size, 5, "ftrace instance size");

namespace {

const char kTracingPath[] = "/sys/kernel/tracing/";
const char kDebugTracingPath[] = "/sys/kernel/debug/tracing/";

const char kPerCPU[] = "per_cpu";
const char kTraceClockPath[] = "trace_clock";
const char kTraceBufferSizePath[] = "buffer_size_kb";
const char kTracingOverwriteEnablePath[] = "options/overwrite";
const char kCurrentTracerPath[] = "current_tracer";
#if defined(__x86_64__)
const char kFuncgraphAbsTimePath[] = "options/funcgraph-abstime";
const char kFuncgraphCpuPath[] = "options/funcgraph-cpu";
const char kFuncgraphProcPath[] = "options/funcgraph-proc";
#endif
const char kFtraceFilterPath[] = "set_ftrace_filter";
const char kTracingOnPath[] = "tracing_on";
const char kTracePath[] = "trace";
const char kTraceMarkerPath[] = "trace_marker";
const char kTraceInstancesPath[] = "instances";

const char kShmFTraceName[] = "qcraft_ftrace";
const char kShmFTraceSegmentName[] = "qcraft_ftrace_segment";
const int kShmFTraceSize = 4096;

typedef enum { OPT, REQ } requiredness;

const std::unordered_map<std::string,
                         std::vector<std::pair<requiredness, std::string>>>
    kTracingCategories = {
        // CPU Scheduling
        {"sched",
         {
             {REQ, "events/sched/sched_switch/enable"},
             {REQ, "events/sched/sched_wakeup/enable"},
             //  {OPT, "events/sched/sched_waking/enable"},
             //  {OPT, "events/sched/sched_process_hang/enable"},
             //  {OPT, "events/sched/sched_pi_setprio/enable"},
             //  {OPT, "events/sched/sched_stat_runtime/enable"},
             //  {OPT, "events/sched/sched_stat_blocked/enable"},
             //  {OPT, "events/sched/sched_stat_iowait/enable"},
             //  {OPT, "events/sched/sched_stat_sleep/enable"},
             //  {OPT, "events/sched/sched_stat_wait/enable"},
             //  {OPT, "events/sched/sched_wait_task/enable"},
             //  {OPT, "events/sched/sched_process_exit/enable"},
             //  {OPT, "events/cgroup/enable"},
             //  {OPT, "events/oom/oom_score_adj_update/enable"},
             //  {OPT, "events/task/task_rename/enable"},
             //  {OPT, "events/task/task_newtask/enable"},
         }},
        // IRQ Events
        {"irq",
         {
             {REQ, "events/irq/enable"},
         }},
        // I2C Events
        {"i2c",
         {
             {REQ, "events/i2c/enable"},
             {REQ, "events/i2c/i2c_read/enable"},
             {REQ, "events/i2c/i2c_write/enable"},
             {REQ, "events/i2c/i2c_result/enable"},
             {REQ, "events/i2c/i2c_reply/enable"},
         }},
        // CPU Frequency
        {"freq",
         {
             {REQ, "events/power/cpu_frequency/enable"},
             {OPT, "events/power/clock_set_rate/enable"},
             {OPT, "events/power/clock_disable/enable"},
             {OPT, "events/power/clock_enable/enable"},
             {OPT, "events/clk/clk_set_rate/enable"},
             {OPT, "events/clk/clk_disable/enable"},
             {OPT, "events/clk/clk_enable/enable"},
             {OPT, "events/power/cpu_frequency_limits/enable"},
             {OPT, "events/power/suspend_resume/enable"},
             {OPT, "events/cpuhp/cpuhp_enter/enable"},
             {OPT, "events/cpuhp/cpuhp_exit/enable"},
         }},
        // CPU Idle
        {"idle",
         {
             {REQ, "events/power/cpu_idle/enable"},
         }},
        // Disk I/O
        {"disk",
         {
             {REQ, "events/block/block_rq_issue/enable"},
             {REQ, "events/block/block_rq_complete/enable"},
             {OPT, "events/ext4/ext4_da_write_begin/enable"},
             {OPT, "events/ext4/ext4_da_write_end/enable"},
             {OPT, "events/ext4/ext4_sync_file_enter/enable"},
             {OPT, "events/ext4/ext4_sync_file_exit/enable"},
         }},
        // eMMC commands
        {"mmc",
         {
             {REQ, "events/mmc/enable"},
         }},
// Synchronization
#if defined(__x86_64__)
        {"sync",
         {
             {REQ, "events/dma_fence/enable"},
         }},
#endif
        // Kernel Workqueues
        {"workq",
         {
             {REQ, "events/workqueue/enable"},
         }},
        // Kernel Memory Reclaim
        {"memreclaim",
         {
             {REQ, "events/vmscan/mm_vmscan_direct_reclaim_begin/enable"},
             {REQ, "events/vmscan/mm_vmscan_direct_reclaim_end/enable"},
             {REQ, "events/vmscan/mm_vmscan_kswapd_wake/enable"},
             {REQ, "events/vmscan/mm_vmscan_kswapd_sleep/enable"},
         }},
        // Voltage and Current Regulators
        {"regulators",
         {
             {REQ, "events/regulator/enable"},
         }},
        // Page cache
        {"pagecache",
         {
             {REQ, "events/filemap/enable"},
         }},
        // Thermal event
        {"thermal",
         {
             {REQ, "events/thermal/thermal_temperature/enable"},
             {OPT, "events/thermal/cdev_update/enable"},
         }},
};

}  // namespace

class FTraceDevice {
 public:
  FTraceDevice() {}
  virtual ~FTraceDevice() {}

  virtual bool SetTraceOverwriteEnable(bool enable) = 0;

  virtual bool SetTraceBufferSizeKB(int size) = 0;

  virtual bool SetTraceClock() = 0;

  virtual bool SetTraceFuncs(const std::string& funcs) = 0;

  virtual bool SetTraceFuncsEnable(bool enable) = 0;

  virtual bool DisableTraceEvents() = 0;

  virtual bool EnableTraceEvents(const std::string& names) = 0;

  virtual bool StartTrace() = 0;

  virtual bool SyncTrace() = 0;

  virtual bool MarkTrace(const std::string& tag) = 0;

  virtual bool ClearTrace() = 0;

  virtual bool StopTrace() = 0;

  virtual bool DumpTrace(std::string* data) = 0;
};

class FTraceNull : public FTraceDevice {
 public:
  FTraceNull() {}
  virtual ~FTraceNull() {}

  virtual bool SetTraceOverwriteEnable(bool enable) { return true; }

  virtual bool SetTraceBufferSizeKB(int size) { return true; }

  virtual bool SetTraceClock() { return true; }

  virtual bool SetTraceFuncs(const std::string& funcs) { return true; }

  virtual bool SetTraceFuncsEnable(bool enable) { return true; }

  virtual bool DisableTraceEvents() { return true; }

  virtual bool EnableTraceEvents(const std::string& names) { return true; }

  virtual bool StartTrace() { return true; }

  virtual bool SyncTrace() { return true; }

  virtual bool MarkTrace(const std::string& tag) { return true; }

  virtual bool ClearTrace() { return true; }

  virtual bool StopTrace() { return true; }

  virtual bool DumpTrace(std::string* data) { return true; }
};

class FTraceImpl : public FTraceDevice {
 public:
  FTraceImpl() {
    root_path_ = GetRootPath();
    QCHECK(!root_path_.empty());
  }

  explicit FTraceImpl(const std::string& name) {
    root_path_ = GetRootPath();
    QCHECK(!root_path_.empty());
    instance_path_ = GetInstancePath(root_path_, name);
    QCHECK(!instance_path_.empty());
  }

  ~FTraceImpl() {
    if (!instance_path_.empty()) {
      rmdir(instance_path_.c_str());
    }

    auto accessor = ftrace_per_thread_.accessAllThreads();
    for (auto& ftrace_per_thread : accessor) {
      std::fclose(ftrace_per_thread.trace_marker_file);
    }
  }

  virtual bool SetTraceOverwriteEnable(bool enable) {
    return WriteBool(kTracingOverwriteEnablePath, enable);
  }

  virtual bool SetTraceBufferSizeKB(int size) {
#if defined(__x86_64__)
    constexpr int kCPUCount = 16;
#else
    constexpr int kCPUCount = 8;
#endif
    constexpr int kCPUBindBoundary = 8;
    if (FLAGS_ftrace_buffer_size_per_cpu_enable) {
      auto ok = true;
      for (size_t index = 0; index < kCPUCount; index++) {
        const auto buffer_size_kb_path =
            absl::StrCat(kPerCPU, "/cpu", index, "/", kTraceBufferSizePath);
        if (size <= 1) {
          ok &= WriteStr(buffer_size_kb_path, "1");
        } else {
          const auto buffer_size = absl::StrFormat(
              "%d", (index < kCPUBindBoundary) ? size : (size << 2));
          ok &= WriteStr(buffer_size_kb_path, buffer_size);
        }
      }
      return ok;
    } else {
      const auto buffer_size = absl::StrFormat("%d", size < 1 ? 1 : size);
      return WriteStr(kTraceBufferSizePath, buffer_size);
    }
  }

  virtual bool SetTraceClock() {
    std::ifstream clock_file(GetTracePath() + kTraceClockPath);
    std::string clock((std::istreambuf_iterator<char>(clock_file)),
                      std::istreambuf_iterator<char>());

    std::string new_clock;
    if (clock.find("boot") != std::string::npos) {
      new_clock = "boot";
    } else if (clock.find("mono") != std::string::npos) {
      new_clock = "mono";
    } else {
      new_clock = "global";
    }

    auto begin = clock.find('[') + 1;
    auto end = clock.find(']');
    if (new_clock.compare(0, std::string::npos, clock, begin, end - begin) ==
        0) {
      return true;
    }
    return WriteStr(kTraceClockPath, new_clock);
  }

  virtual bool SetTraceFuncs(const std::string& funcs) {
    auto ok = TruncateFile(kFtraceFilterPath);
    const std::vector<std::string> my_funcs =
        absl::StrSplit(funcs, ",", absl::SkipWhitespace());
    for (const auto& func : my_funcs) {
      ok &= AppendStr(kFtraceFilterPath, func);
    }
    return ok;
  }

  virtual bool SetTraceFuncsEnable(bool enable) {
#if defined(__x86_64__)
    auto ok = WriteBool(root_path_ + kFuncgraphAbsTimePath, enable);
    ok &= WriteBool(root_path_ + kFuncgraphCpuPath, enable);
    ok &= WriteBool(root_path_ + kFuncgraphProcPath, enable);
#else
    auto ok = true;
#endif
    ok &=
        WriteStr(root_path_ + kCurrentTracerPath, enable ? "function" : "nop");
    return ok;
  }

  virtual bool DisableTraceEvents() {
    auto ok = true;
    for (const auto& tracing_category : kTracingCategories) {
      for (const auto& trace_event : tracing_category.second) {
        ok &= WriteBool(trace_event.second, false);
      }
    }
    return ok;
  }

  virtual bool EnableTraceEvents(const std::string& names) {
    auto ok = true;
    const std::vector<std::string> my_names =
        absl::StrSplit(names, ",", absl::SkipWhitespace());
    for (const auto& name : my_names) {
      ok &= EnableTraceEvent(name);
    }
    return ok;
  }

  virtual bool StartTrace() { return SetTracingEnabled(true); }

  virtual bool SyncTrace() {
    auto ok = true;

    float parent_ts = SystemTime(CLOCK_MONOTONIC) / 1000000000.0f;
    const auto parent_with_ts =
        absl::StrFormat("trace_event_clock_sync: parent_ts=%f\n", parent_ts);
    ok &= WriteStr(GetTraceMarkerFile(), parent_with_ts);

    auto realtime_ts = SystemTime(CLOCK_REALTIME) / 1000000;
    const auto realtime_with_ts = absl::StrFormat(
        "trace_event_clock_sync: realtime_ts=%" PRId64 "\n", realtime_ts);
    ok &= WriteStr(GetTraceMarkerFile(), realtime_with_ts);

    return ok;
  }

  virtual bool MarkTrace(const std::string& tag) {
    return WriteStr(GetTraceMarkerFile(), tag);
  }

  virtual bool ClearTrace() { return TruncateFile(kTracePath); }

  virtual bool StopTrace() { return SetTracingEnabled(false); }

  virtual bool DumpTrace(std::string* data) {
    return ReadFile(kTracePath, data);
  }

 private:
  FILE* GetTraceMarkerFile() {
    if (ftrace_per_thread_->trace_marker_file == nullptr) {
      auto trace_marker_path(GetTracePath() + kTraceMarkerPath);
      ftrace_per_thread_->trace_marker_file =
          std::fopen(trace_marker_path.c_str(), "w");
    }
    return ftrace_per_thread_->trace_marker_file;
  }

  bool SetTracingEnabled(bool enable) {
    return WriteBool(kTracingOnPath, enable);
  }

  bool EnableTraceEvent(const std::string& name) {
    const auto* tracing_category = FindOrNull(kTracingCategories, name);
    if (tracing_category == nullptr) {
      QLOG(ERROR) << absl::StrFormat("Unknown tracing category \"%s\"", name);
      return false;
    }

    for (const auto& trace_event : *tracing_category) {
      bool required = trace_event.first == REQ;
      const auto& file_name = trace_event.second;
      if (WriteBool(file_name, true)) {
        return true;
      } else if (required) {
        QLOG(ERROR) << absl::StrFormat("Writing tracing event %s", file_name);
        return false;
      }
    }

    return true;
  }

  bool WriteBool(const std::string& file_name, bool enable) {
    return WriteStr(file_name, enable ? "1" : "0");
  }

  bool WriteStr(const std::string& file_name, const std::string& str) {
    return WriteFile(file_name, str, "w");
  }

  bool WriteStr(FILE* file, const std::string& str) {
    return WriteFile(file, str);
  }

  bool AppendStr(const std::string& file_name, const std::string& str) {
    return WriteFile(file_name, str, "a");
  }

  bool TruncateFile(const std::string& file_name) {
    auto trace_path(GetTracePath() + file_name);
    auto fd = creat(trace_path.c_str(), 0);
    if (fd == -1) {
      QLOG(ERROR) << absl::StrFormat(
          "Truncating %s: %s (%d)", trace_path.c_str(), strerror(errno), errno);
      return false;
    }

    close(fd);

    return true;
  }

  bool WriteFile(const std::string& file_name, const std::string& str,
                 const std::string& modes) {
    auto trace_path(GetTracePath() + file_name);
    if (file_name.find("/sys/") != -1) {
      trace_path = file_name;
    }

    std::unique_ptr<std::FILE, std::function<void(std::FILE*)>> file(
        std::fopen(trace_path.c_str(), modes.c_str()),
        [](std::FILE* fp) { std::fclose(fp); });
    if (file == nullptr) {
      QLOG(ERROR) << absl::StrFormat("[%d]Opening %s: %s (%d)", getpid(),
                                     trace_path, strerror(errno), errno);
      return false;
    }

    if (TEMP_FAILURE_RETRY(std::fwrite(str.c_str(), 1, str.size(),
                                       file.get()) != str.size())) {
      QLOG(ERROR) << absl::StrFormat("[%d][%u]Writing %s: %s %s (%d)", getpid(),
                                     pthread_self(), trace_path, str,
                                     strerror(errno), errno);

      return false;
    }

    return true;
  }

  bool WriteFile(std::FILE* file, const std::string& str) {
    if (file == nullptr) {
      return false;
    }
    if (TEMP_FAILURE_RETRY(std::fwrite(str.c_str(), 1, str.size(), file) !=
                           str.size())) {
      return false;
    }
    std::fflush(file);
    return true;
  }

  bool ReadFile(const std::string& file_name, std::string* data) {
    auto trace_path(GetTracePath() + file_name);
    std::unique_ptr<std::FILE, std::function<void(std::FILE*)>> file(
        std::fopen(trace_path.c_str(), "r"),
        [](std::FILE* fp) { std::fclose(fp); });

    if (file == nullptr) {
      QLOG(ERROR) << absl::StrFormat("[%d]Opening %s: %s (%d)", getpid(),
                                     trace_path, strerror(errno), errno);
      return false;
    }

    const int expected_size = getpagesize();
    const int cpu_num = sysconf(_SC_NPROCESSORS_CONF);
    const int data_ratio = 8;
    const int reserved_size =
        data_ratio * cpu_num * FLAGS_ftrace_buffer_size_kb * 1024;

    std::string result;
    const int capacity =
        (reserved_size + expected_size - 1) / expected_size * expected_size;
    result.reserve(capacity);
    bool ok = true;
    int num_reads = 0;
    const auto now = absl::Now();
    while (true) {
      const auto prev_size = result.size();
      if (prev_size + expected_size > result.capacity()) {
        QLOG(WARNING) << absl::StrFormat(
            "It seems we haven't reserved sufficient space: %d vs %d",
            prev_size + expected_size, result.capacity());
      }
      result.resize(prev_size + expected_size);
      auto read_size = TEMP_FAILURE_RETRY(
          std::fread(const_cast<char*>(result.data() + prev_size), 1,
                     expected_size, file.get()));
      ++num_reads;
      if (read_size <= 0) {
        if (read_size < 0 && errno != EAGAIN) {
          LOG(ERROR) << absl::StrFormat("[%d][%u]Read %s: %s (%d)", getpid(),
                                        pthread_self(), trace_path,
                                        strerror(errno), errno);
          ok = false;
        }
        result.resize(prev_size);
        break;
      } else if (read_size < expected_size) {
        result.resize(prev_size + read_size);
      }
    }
    QLOG(INFO) << absl::StrFormat(
        "Read the ftrace file %d times of %lld bytes, using %.3f seconds",
        num_reads, result.size(), absl::ToDoubleSeconds(absl::Now() - now));

    if (ok) {
      *data = std::move(result);
    }

    return ok;
  }

  std::string GetTracePath() {
    if (instance_path_.empty()) {
      return root_path_;
    } else {
      return instance_path_;
    }
  }

  std::string GetInstancePath(const std::string& root_path,
                              const std::string& name) {
    auto trace_path = root_path + kTraceInstancesPath + "/" + name + "/";
    struct stat st;
    auto ret = stat(trace_path.c_str(), &st);
    if (ret < 0) {
      QCHECK_GE(mkdir(trace_path.c_str(), 0666), 0);
    }

    return trace_path;
  }

  std::string GetRootPath() {
    auto tracefs =
        access((std::string(kTracingPath) + kTraceMarkerPath).c_str(), F_OK) !=
        -1;
    auto debugfs =
        access((std::string(kDebugTracingPath) + kTraceMarkerPath).c_str(),
               F_OK) != -1;

    if (!tracefs && !debugfs) {
      return MountTraceDevice();
    }

    if (tracefs) {
      return kTracingPath;
    } else {
      return kDebugTracingPath;
    }
  }

  int64_t SystemTime(clockid_t clk_id) {
    struct timespec t;
    clock_gettime(clk_id, &t);
    return int64_t{t.tv_sec} * 1000000000 + t.tv_nsec;
  }

  std::string MountTraceDevice() {
    const char kTracefsPath[] = "/sys/kernel/tracing";
    const char kDebugfsPath[] = "/sys/kernel/debug";

    struct stat st;
    auto ret = stat(kTracefsPath, &st);
    if (ret >= 0) {
      ret = mount("nodev", kTracefsPath, "tracefs", 0, NULL);
      if (ret >= 0) {
        return kTracingPath;
      }
    }

    ret = stat(kDebugfsPath, &st);
    if (ret >= 0) {
      ret = mount("nodev", kDebugfsPath, "debugfs", 0, NULL);
      if (ret >= 0) {
        return kDebugTracingPath;
      }
    }

    QLOG(ERROR) << "Did not find trace folder";
    return "";
  }

 private:
  std::string root_path_;
  std::string instance_path_;

  struct FTracePerThread {
    std::FILE* trace_marker_file;
  };
  class FTraceTag;
  folly::ThreadLocal<FTracePerThread, FTraceTag> ftrace_per_thread_;
};  // namespace qcraft

FTrace::FTrace() {
  for (int index = 0; index < FLAGS_ftrace_instances_size; ++index) {
    if (FLAGS_ftrace_enable) {
      std::string name = absl::StrFormat("qcraft_ftrace_%d", index);
      devices_.push_back(std::make_unique<FTraceImpl>(name));
    } else {
      devices_.push_back(std::make_unique<FTraceNull>());
    }
  }

  if (FLAGS_ftrace_enable) {
    shm_ = std::make_unique<bip::managed_shared_memory>(
        bip::open_or_create, kShmFTraceName, kShmFTraceSize);
    segment_ = shm_->find_or_construct<FTraceSegment>(kShmFTraceSegmentName)();
  }
}

FTrace::~FTrace() {}

bool FTrace::Init() {
  if (!Uninit()) {
    return false;
  }

  if (!Start()) {
    return false;
  }

  if (FLAGS_ftrace_enable) {
    shm_ = std::make_unique<bip::managed_shared_memory>(
        bip::create_only, kShmFTraceName, kShmFTraceSize);
    segment_ = shm_->find_or_construct<FTraceSegment>(kShmFTraceSegmentName)();
  }

  return true;
}

bool FTrace::Uninit() {
  if (shm_ != nullptr) {
    shm_ = nullptr;
    bip::shared_memory_object::remove(kShmFTraceName);
  }

  if (!Stop()) {
    return false;
  }

  return true;
}

void FTrace::MarkStart(const std::string& event) {
  if (segment_ == nullptr) {
    return;
  }

  if (segment_->dumping.load()) {
    QLOG(INFO) << absl::StrFormat(
        "[B]FTrace event dropped, trace_event: %s, %d", event, getpid());
    return;
  }

  auto tag = absl::StrFormat("B|%d|%s", getpid(), event);
  devices_[GetDeviceIndex()]->MarkTrace(tag);
}

void FTrace::MarkStart(const TraceEvent& trace_event) {
  if (segment_ == nullptr) {
    return;
  }

  if (segment_->dumping.load()) {
    QLOG(INFO) << absl::StrFormat(
        "[B]FTrace event dropped, trace_event: %s, %d", trace_event.name,
        getpid());
    return;
  }

  auto tag = absl::StrFormat("B|%d|%s", getpid(), trace_event.name);
  for (int index = 0; index < trace_event.num_args; ++index) {
    if (index == 0) {
      tag = absl::StrFormat("%s|%s=%s", tag, trace_event.args[index].key,
                            trace_event.args[index].value);
    } else {
      tag = absl::StrFormat("%s;%s=%s", tag, trace_event.args[index].key,
                            trace_event.args[index].value);
    }
  }

  devices_[GetDeviceIndex()]->MarkTrace(tag);
}

void FTrace::MarkEnd() {
  if (segment_ == nullptr) {
    return;
  }

  if (segment_->dumping.load()) {
    QLOG(INFO) << absl::StrFormat("[E]FTrace event dropped, %d", getpid());
    return;
  }

  devices_[GetDeviceIndex()]->MarkTrace("E");
}

bool FTrace::DumpTrace(std::string* data) {
  QCHECK(data != nullptr);
  if (segment_ == nullptr) {
    return true;
  }
  segment_->time_dump.store(absl::ToUnixSeconds(absl::Now()) +
                            FLAGS_ftrace_dump_trace_interval);

  auto expected = false;
  if (!segment_->dumping.compare_exchange_strong(expected, true)) {
    QLOG(INFO) << absl::StrFormat("FTrace dump dropped(1), %d", getpid());
    return true;
  }

  if (segment_->dumping_count.load() >= FLAGS_ftrace_instances_size) {
    QLOG(ERROR) << absl::StrFormat("FTrace dump dropped(2), %d", getpid());
    segment_->dumping.store(false);
    return false;
  }

  auto ok = true;
  segment_->dumping_count.fetch_add(1);
  {
    //! The order cann't be changed.
    auto index = AddDeviceIndex();
    // trace cpu consumption for start device
    {
      auto tag = absl::StrFormat("B|%d|StartDevice", getpid());
      devices_[index]->MarkTrace(tag);
      ok &= StartDevice();
      devices_[index]->MarkTrace("E");
    }
    segment_->dumping.store(false);

    // trace cpu consumption for total dump
    {
      MarkStart("TotalDumpTrace");

      ok &= devices_[index]->StopTrace();

      // trace cpu consumption for dump trace
      {
        MarkStart("DumpTrace");
        ok &= devices_[index]->DumpTrace(data);
        MarkEnd();
      }

      // trace cpu consumption for clear trace
      {
        MarkStart("ClearTrace");
        ok &= devices_[index]->ClearTrace();
        MarkEnd();
      }

      MarkEnd();
    }
  }
  segment_->dumping_count.fetch_sub(1);

  return ok;
}

bool FTrace::ShouldDump() {
  if (segment_ == nullptr) {
    return false;
  }
  if (absl::ToUnixSeconds(absl::Now()) < segment_->time_dump.load()) {
    return false;
  }
  return true;
}

bool FTrace::StartDevice() {
  auto index = GetDeviceIndex();
  auto ok = devices_[index]->StartTrace();
  ok &= devices_[index]->SyncTrace();
  if (!ok) {
    static const int kTryMaxCount = 6;
    int try_count = 0;
    do {
      // TODO(liyu) sleep not a good solution, need improve it.
      usleep(330 * 1000);
      ok = devices_[index]->SyncTrace();
      QLOG(INFO) << absl::StrFormat("FTrace try to sync, %d %d", getpid(),
                                    try_count);
    } while (!ok && ++try_count < kTryMaxCount);
  }

  return ok;
}

bool FTrace::Start() {
  std::atomic<int> ok = true;
  std::vector<std::thread> start_threads;
  for (int index = 0; index < devices_.size(); ++index) {
    start_threads.push_back(std::thread([this, &ok, index]() {
      QSetThreadName("ftrace_thread");
      ok &= devices_[index]->SetTraceOverwriteEnable(
          FLAGS_ftrace_overwrite_enable);
      ok &= devices_[index]->SetTraceBufferSizeKB(FLAGS_ftrace_buffer_size_kb);
      ok &= devices_[index]->SetTraceClock();
      ok &= devices_[index]->SetTraceFuncs(FLAGS_ftrace_functions);
      ok &=
          devices_[index]->SetTraceFuncsEnable(!FLAGS_ftrace_functions.empty());
      ok &= devices_[index]->EnableTraceEvents(FLAGS_ftrace_events);
      if (index == 0) {
        ok &= devices_[index]->StartTrace();
      }
      ok &= devices_[index]->ClearTrace();
      ok &= devices_[index]->SyncTrace();
    }));
  }

  for (auto& thread : start_threads) {
    thread.join();
  }

  return ok;
}

bool FTrace::Stop() {
  auto ok = true;
  for (int index = 0; index < devices_.size(); ++index) {
    ok &= devices_[index]->StopTrace();
    ok &= devices_[index]->DisableTraceEvents();
    ok &= devices_[index]->SetTraceFuncsEnable(false);
    ok &= devices_[index]->SetTraceBufferSizeKB(1);
    ok &= devices_[index]->SetTraceOverwriteEnable(true);
  }

  return ok;
}

int FTrace::GetDeviceIndex() {
  return segment_->index_device.load() % FLAGS_ftrace_instances_size;
}

int FTrace::AddDeviceIndex() {
  return segment_->index_device.fetch_add(1) % FLAGS_ftrace_instances_size;
}
};  // namespace qcraft
