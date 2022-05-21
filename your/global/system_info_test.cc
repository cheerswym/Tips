#include "onboard/global/system_info.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(SystemInfoTest, TestGetSharedSystemInfoProto) {
  int count = 0;
  while (count++ < 3) {
    const auto shared_system_info =
        qcraft::SystemInfo::Instance()->GetSharedSystemInfoProto();
    LOG(INFO) << "Shared system info: " << shared_system_info.DebugString();
  }
}

TEST(SystemInfoTest, TestGetModuleSystemInfoProto) {
  int count = 0;
  while (count++ < 3) {
    const auto module_system_info =
        qcraft::SystemInfo::Instance()->GetModuleSystemInfoProto(
            "SystemInfoTest");
    LOG(INFO) << "Module system info: " << module_system_info.DebugString();
  }
}

TEST(SystemInfoTest, TestGetGpuDriverProto) {
  int count = 0;
  while (count++ < 3) {
    GPUDriverProto gpu_driver;
    qcraft::SystemInfo::Instance()->GetGpuDriverProto(&gpu_driver);
    LOG(INFO) << "GPU driver info: " << gpu_driver.DebugString();
  }
}

}  // namespace
}  // namespace qcraft
