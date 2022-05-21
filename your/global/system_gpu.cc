#include "absl/strings/str_format.h"
#include "onboard/global/system_info.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace {
const unsigned int kMaxProcessNumOnGPU = 128;
const unsigned int kMaxProcessNameLength = 128;

void SetEachTypeProcessMem(GPUDeviceInfoProto* device_info,
                           unsigned int process_num,
                           nvmlProcessInfo_t* process_infos) {
  for (int i = 0; i < process_num; i++) {
    unsigned int pid = process_infos[i].pid;
    char process_name[kMaxProcessNameLength];
    if (nvmlSystemGetProcessName(pid, process_name, kMaxProcessNameLength) !=
        NVML_SUCCESS) {
      continue;
    }
    auto* gpu_mem = device_info->add_process_gpu_mem();
    gpu_mem->set_memory_used_byte(process_infos[i].usedGpuMemory);
    gpu_mem->set_process_name(process_name);
  }
}

void SetGpuProcessMem(nvmlDevice_t& device, GPUDeviceInfoProto* device_info,
                      int index) {
  nvmlProcessInfo_t process_infos[kMaxProcessNumOnGPU];
  unsigned int process_num = kMaxProcessNumOnGPU;
  QCHECK_EQ(nvmlDeviceGetComputeRunningProcesses_v2(device, &process_num,
                                                    process_infos),
            NVML_SUCCESS);
  SetEachTypeProcessMem(device_info, process_num, process_infos);
}

}  // namespace

void SystemInfo::InitGpu() {
  // TODO(liyu) scripts/lite_integration_test/lite_integration_test.sh crashed
  // if we used QCHECK_EQ
  const auto result = nvmlInit();
  if (result != NVML_SUCCESS) {
    return;
  }

  uint count;
  QCHECK_EQ(nvmlDeviceGetCount(&count), NVML_SUCCESS);
  devices_.resize(count);
  for (uint index = 0; index < count; ++index) {
    nvmlDevice_t device;
    nvmlDeviceGetHandleByIndex(index, &device);
    devices_[index] = device;
  }
}

void SystemInfo::SetGPUInfo(GPUInfoProto* proto) {
  for (size_t index = 0; index < devices_.size(); index++) {
    auto device = devices_[index];
    auto* device_info = proto->add_gpu_device_info();
    char name[NVML_DEVICE_NAME_BUFFER_SIZE];
    QCHECK_EQ(nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE),
              NVML_SUCCESS);
    device_info->set_name(absl::StrFormat("%s-%d", name, index));

    nvmlUtilization_t utilization;
    QCHECK_EQ(nvmlDeviceGetUtilizationRates(device, &utilization),
              NVML_SUCCESS);
    device_info->set_gpu_utilization(utilization.gpu);
    device_info->set_memory_utilization(utilization.memory);

    nvmlMemory_t mem;
    QCHECK_EQ(nvmlDeviceGetMemoryInfo(device, &mem), NVML_SUCCESS);
    device_info->set_memory_free_byte(mem.free);
    device_info->set_memory_total_byte(mem.total);

    SetGpuProcessMem(device, device_info, index);

    unsigned int fan_speed;
    if (nvmlDeviceGetFanSpeed(device, &fan_speed) == NVML_SUCCESS) {
      device_info->set_fan_speed(fan_speed);
    }

    unsigned int temperature;
    QCHECK_EQ(nvmlDeviceGetTemperature(
                  device, nvmlTemperatureSensors_t::NVML_TEMPERATURE_GPU,
                  &temperature),
              NVML_SUCCESS);
    device_info->set_temperature(temperature);

    unsigned int power;
    QCHECK_EQ(nvmlDeviceGetPowerUsage(device, &power), NVML_SUCCESS);
    device_info->set_power_usage(power / 1000);

    unsigned int max_clock;
    QCHECK_EQ(nvmlDeviceGetMaxClockInfo(device, NVML_CLOCK_SM, &max_clock),
              NVML_SUCCESS);
    device_info->set_max_clock(max_clock);

    unsigned int current_clock;
    QCHECK_EQ(nvmlDeviceGetClockInfo(device, NVML_CLOCK_SM, &current_clock),
              NVML_SUCCESS);
    device_info->set_current_clock(current_clock);
  }
}

void SystemInfo::SetGPUDriverInfo(GPUDriverProto* proto) {
  for (size_t index = 0; index < devices_.size(); index++) {
    auto device = devices_[index];
    auto* driver_info = proto->add_gpu_driver_info();

    char drvier_version[NVML_SYSTEM_DRIVER_VERSION_BUFFER_SIZE];
    QCHECK_EQ(nvmlSystemGetDriverVersion(
                  drvier_version, NVML_SYSTEM_DRIVER_VERSION_BUFFER_SIZE),
              NVML_SUCCESS);
    driver_info->set_driver_version(drvier_version);

    char nvml_version[NVML_SYSTEM_NVML_VERSION_BUFFER_SIZE];
    QCHECK_EQ(nvmlSystemGetNVMLVersion(nvml_version,
                                       NVML_SYSTEM_NVML_VERSION_BUFFER_SIZE),
              NVML_SUCCESS);
    driver_info->set_nvml_version(nvml_version);

    int cuda_driver_version;
    QCHECK_EQ(nvmlSystemGetCudaDriverVersion(&cuda_driver_version),
              NVML_SUCCESS);
    driver_info->set_cuda_version(cuda_driver_version);

    char uuid[NVML_DEVICE_UUID_V2_BUFFER_SIZE];
    QCHECK_EQ(nvmlDeviceGetUUID(device, uuid, NVML_DEVICE_UUID_V2_BUFFER_SIZE),
              NVML_SUCCESS);
    driver_info->set_uuid(uuid);
  }
}

}  // namespace qcraft
