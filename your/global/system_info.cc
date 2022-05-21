#include "onboard/global/system_info.h"

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "onboard/global/system_cpu.h"
#include "onboard/global/system_mem.h"
#include "onboard/global/system_misc.h"
#include "onboard/node/node_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace {
constexpr char kSysteClassNet[] = "/sys/class/net/";
// bytes
constexpr char kStatsTxBytes[] = "/statistics/tx_bytes";
constexpr char kStatsRxBytes[] = "/statistics/rx_bytes";
// packets
constexpr char kStatsTxPackets[] = "/statistics/tx_packets";
constexpr char kStatsRxPackets[] = "/statistics/rx_packets";
// errors
constexpr char kStatsTxErrors[] = "/statistics/tx_errors";
constexpr char kStatsRxErrors[] = "/statistics/rx_errors";
// dropped
constexpr char kStatsTxDropped[] = "/statistics/tx_dropped";
constexpr char kStatsRxDropped[] = "/statistics/rx_dropped";
}  // namespace

SystemInfo::SystemInfo() {
  InitCpu();
  InitGpu();
}

void SystemInfo::InitCpu() {
  // Cpu cores and total_cpu
  const static int total_cpu_number = sysconf(_SC_NPROCESSORS_CONF) + 1;
  last_total_cpu_.resize(total_cpu_number);
  current_total_cpu_.resize(total_cpu_number);
}

SystemInfoProto SystemInfo::GetModuleSystemInfoProto(
    const std::string& module_name) {
  SystemInfoProto system_info;

  // cpu per module
  SetModuleCPUInfo(system_info.mutable_cpu_info(),
                   absl::StrCat(GetFullNodeName(), "/", module_name));

  // mem per module
  auto* mem_info = system_info.mutable_mem_info();
  uint64_t self_mem_kb = GetSelfMemUsage() >> 10;  // B -> kB
  mem_info->set_used_kb(self_mem_kb);
  mem_info->set_module(absl::StrCat(GetFullNodeName(), "/", module_name));

  // io per moudle
  SetModuleIoInfo(system_info.mutable_io_info(),
                  absl::StrCat(GetFullNodeName(), "/", module_name));

  return system_info;
}

SystemInfoProto SystemInfo::GetSharedSystemInfoProto() {
  SystemInfoProto system_info;

  // cpu
  SetTotalCPUInfo(&system_info);

  // gpu
  auto* gpu_info = system_info.mutable_gpu_info();
  SetGPUInfo(gpu_info);
  for (size_t index = 0; index < gpu_info->gpu_device_info_size(); index++) {
    auto* gpu_device_info = gpu_info->mutable_gpu_device_info(index);
    gpu_device_info->set_name(
        absl::StrCat(GetFullNodeName(), "/", gpu_device_info->name()));
  }

  // mem
  auto* mem = system_info.mutable_mem_info();
  const auto total_mem = GetTotalMemUsage();
  mem->set_used_kb(total_mem.total_mem_kb - total_mem.available_mem_kb);
  mem->set_module(GetFullNodeName());
  mem->set_total_kb(total_mem.total_mem_kb);

  // network
  SetNetworkInfoProto(system_info.mutable_network_info());

  // misc
  SetMiscInfo(system_info.mutable_misc_info());

  return system_info;
}

void SystemInfo::SetModuleIoInfo(IoInfoProto* proto,
                                 const std::string& module) {
  const auto current_self_io = GetSelfIoInfo();
  if (!io_init_) {
    io_init_ = true;
    proto->set_rchar(current_self_io.rchar);
    proto->set_wchar(current_self_io.wchar);
    proto->set_syscr(current_self_io.syscr);
    proto->set_syscw(current_self_io.syscw);
    proto->set_read_bytes(current_self_io.read_bytes);
    proto->set_write_bytes(current_self_io.write_bytes);
    proto->set_cancelled_write_bytes(current_self_io.cancelled_write_bytes);
  } else {
    proto->set_rchar(current_self_io.rchar - last_self_io_.rchar);
    proto->set_wchar(current_self_io.wchar - last_self_io_.wchar);
    proto->set_syscr(current_self_io.syscr - last_self_io_.syscr);
    proto->set_syscw(current_self_io.syscw - last_self_io_.syscw);
    proto->set_read_bytes(current_self_io.read_bytes -
                          last_self_io_.read_bytes);
    proto->set_write_bytes(current_self_io.write_bytes -
                           last_self_io_.write_bytes);
    proto->set_cancelled_write_bytes(current_self_io.cancelled_write_bytes -
                                     last_self_io_.cancelled_write_bytes);
  }
  proto->set_module(module);

  last_self_io_ = current_self_io;
}

void SystemInfo::SetModuleCPUInfo(CPUInfoProto* proto,
                                  const std::string& module) {
  const auto current_self_cpu = GetSelfCPU();
  if (last_self_cpu_ != -1) {
    const int64_t diff = current_self_cpu - last_self_cpu_;
    proto->set_usage_amount(diff);
  } else {
    proto->set_usage_amount(0);
  }
  proto->set_module(module);
  last_self_cpu_ = current_self_cpu;
}

void SystemInfo::SetTotalCPUInfo(SystemInfoProto* system_info_proto) {
  int64_t diff = 0;
  GetTotalCPU(&current_total_cpu_);
  std::vector<uint32_t> curren_cpu_freq = GetCPUFreq();
  CPUInfoProto* proto = nullptr;
  for (int i = 0; i < current_total_cpu_.size(); i++) {
    proto = system_info_proto->add_cpu_info_cores();
    if (i < curren_cpu_freq.size()) {
      proto->add_cpu_freqs(curren_cpu_freq[i]);
    }
    if (last_total_cpu_[i] != 0) {
      diff = current_total_cpu_[i] - last_total_cpu_[i];
      proto->set_usage_amount(diff);
    } else {
      proto->set_usage_amount(0);
    }
    if (i == 0) {
      proto->set_module(absl::StrCat(GetFullNodeName(), "/total"));
    } else {
      proto->set_module(absl::StrCat(GetFullNodeName(), "/cpu", i - 1));
    }
    last_total_cpu_[i] = current_total_cpu_[i];
  }
}

void SystemInfo::SetNetworkInfoProto(NetworkInfoProto* proto) {
  try {
    for (const auto& entry :
         boost::filesystem::directory_iterator(kSysteClassNet)) {
      auto network_device_info = proto->add_network_device_info();
      const std::string path = entry.path().string();
      const std::string device = entry.path().filename().string();
      network_device_info->set_name(device);
      network_device_info->set_tx(GetNetworkStats(path, kStatsTxBytes) >>
                                  10);  // kB
      network_device_info->set_rx(GetNetworkStats(path, kStatsRxBytes) >>
                                  10);  // kB
      network_device_info->set_tx_packets(
          GetNetworkStats(path, kStatsTxPackets));
      network_device_info->set_rx_packets(
          GetNetworkStats(path, kStatsRxPackets));
      network_device_info->set_tx_errors(GetNetworkStats(path, kStatsTxErrors));
      network_device_info->set_rx_errors(GetNetworkStats(path, kStatsRxErrors));
      network_device_info->set_tx_dropped(
          GetNetworkStats(path, kStatsTxDropped));
      network_device_info->set_rx_dropped(
          GetNetworkStats(path, kStatsRxDropped));
    }

    proto->set_module(GetFullNodeName());
  } catch (boost::filesystem::filesystem_error& e) {
    LOG(ERROR) << e.what();
  }
}

int64_t SystemInfo::GetNetworkStats(const std::string& device_path,
                                    const std::string& metric) {
  std::string str;
  if (!file_util::GetFileContent(absl::StrCat(device_path, metric), &str)) {
    return 0;
  }
  const auto value = std::stol(str);
  const auto key = std::make_pair(device_path, metric);
  if (const auto* last_value_ptr =
          FindOrNull(device_metric_latest_value_, key)) {
    const auto result = value - *last_value_ptr;
    device_metric_latest_value_[key] = value;
    return result;
  } else {
    device_metric_latest_value_[key] = value;
    return 0;
  }
}

void SystemInfo::GetGpuDriverProto(GPUDriverProto* gpu_driver_proto) {
  SetGPUDriverInfo(gpu_driver_proto);
}

}  // namespace qcraft
