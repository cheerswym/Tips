#ifndef ONBOARD_GLOBAL_FTRACE_H_
#define ONBOARD_GLOBAL_FTRACE_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "boost/interprocess/managed_shared_memory.hpp"
#include "onboard/base/macros.h"
#include "onboard/global/singleton.h"
#include "onboard/global/trace_util.h"

namespace qcraft {

namespace bip = boost::interprocess;

class FTraceDevice;
class FTrace {
 public:
  ~FTrace();

  // init kernel trace
  // NOTE： if you call it, it will lead to restart the ftrace.
  bool Init();

  // unit kernel trace
  // NOTE: if you call it, it will lead to that ftrace does not work.
  bool Uninit();

  // mark a tag, like 'trace_start' in kernel trace
  void MarkStart(const std::string& event);

  // mark a tag, like 'B|ppid|event|key1=value1;key2=value2' in kernel trace
  void MarkStart(const TraceEvent& trace_event);

  // mark a tag, like 'E' in kernel trace
  void MarkEnd();

  // dump kernel trace into event
  bool DumpTrace(std::string* event);

  bool ShouldDump();

 private:
  bool Start();

  bool Stop();

  bool StartDevice();

 private:
  int GetDeviceIndex();
  int AddDeviceIndex();

 private:
  std::vector<std::unique_ptr<FTraceDevice>> devices_;

  struct FTraceSegment {
    std::atomic<bool> dumping;
    std::atomic<int> dumping_count;
    std::atomic<int64_t> time_dump;
    std::atomic<int> index_device;
  };

  FTraceSegment* segment_ = nullptr;
  std::unique_ptr<bip::managed_shared_memory> shm_;

  DECLARE_SINGLETON(FTrace);
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_FTRACE_H_
