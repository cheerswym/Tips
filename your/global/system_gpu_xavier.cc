#include <regex>
#include <string>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "glog/logging.h"
#include "onboard/global/system_info.h"
#include "onboard/utils/file_util.h"

namespace qcraft {

namespace {
constexpr char kSystemDevicesThermal[] = "/sys/devices/virtual/thermal/";
constexpr char kSystemDevicesGPULoad[] = "/sys/devices/gpu.0/load";
constexpr char kSystemBusGPUPowerUsage[] =
    "/sys/bus/i2c/devices/1-0040/iio_device/in_power0_input";
}  // namespace

void SystemInfo::InitGpu() {}

void SystemInfo::SetGPUInfo(GPUInfoProto* proto) {
  try {
    for (const auto& entry :
         boost::filesystem::directory_iterator(kSystemDevicesThermal)) {
      const std::regex e("(thermal_zone\\d+)");
      const auto& path = entry.path();
      std::smatch sm;
      if (!std::regex_match(path.filename().string(), sm, e)) {
        continue;
      }

      std::string name;
      file_util::GetFileContent(absl::StrCat(path.string(), "/type"), &name);
      name.erase(name.find_last_not_of(" \n\r\t") + 1);
      if (name.compare("GPU-therm") == 0) {
        std::string input;
        file_util::GetFileContent(absl::StrCat(path.string(), "/temp"), &input);
        auto* device_info = proto->add_gpu_device_info();
        device_info->set_name(name);
        device_info->set_temperature(std::stoi(input) / 1000);

        file_util::GetFileContent(kSystemDevicesGPULoad, &input);
        device_info->set_gpu_utilization(std::stoi(input) / 10);

        file_util::GetFileContent(kSystemBusGPUPowerUsage, &input);
        device_info->set_power_usage(std::stoi(input) / 1000);
      }
    }
  } catch (boost::filesystem::filesystem_error& e) {
    LOG(ERROR) << e.what();
  }
}

void SystemInfo::SetGPUDriverInfo(GPUDriverProto* proto) {}

}  // namespace qcraft
