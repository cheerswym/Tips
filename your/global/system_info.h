#ifndef ONBOARD_GLOBAL_SYSTEM_H_
#define ONBOARD_GLOBAL_SYSTEM_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#if defined(__x86_64__)
#include "nvml.h"  // NOLINT
#endif

#include "onboard/global/singleton.h"
#include "onboard/global/system_io.h"
#include "onboard/proto/system_info.pb.h"
#include "onboard/proto/system_status_info.pb.h"

namespace qcraft {

// Get sytem info including gpu, cpu, memeory , network.
class SystemInfo {
 public:
  SystemInfoProto GetModuleSystemInfoProto(const std::string& module_name);

  SystemInfoProto GetSharedSystemInfoProto();

  void GetGpuDriverProto(GPUDriverProto* gpu_driver_proto);

 private:
  void SetModuleCPUInfo(CPUInfoProto* proto, const std::string& module);

  void SetModuleIoInfo(IoInfoProto* proto, const std::string& module);

  void SetTotalCPUInfo(SystemInfoProto* system_info_proto);

  void SetNetworkInfoProto(NetworkInfoProto* proto);

  void SetGPUInfo(GPUInfoProto* proto);

  void SetGPUDriverInfo(GPUDriverProto* proto);

  int64_t GetNetworkStats(const std::string& device, const std::string& metric);

 private:
  void InitCpu();

  void InitGpu();

 private:
  int64_t last_self_cpu_ = -1;
  bool io_init_ = false;
  ProcessIoInfo last_self_io_;

  std::vector<uint64_t> last_total_cpu_;
  std::vector<uint64_t> current_total_cpu_;

  // key is {device, metric}, value is last value;
  std::map<std::pair<std::string, std::string>, int64_t>
      device_metric_latest_value_;

#if defined(__x86_64__)
  std::vector<nvmlDevice_t> devices_;
#endif

  DECLARE_SINGLETON(SystemInfo);
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SYSTEM_H_
