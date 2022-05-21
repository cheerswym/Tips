#include "onboard/global/system_status.h"

#include <functional>
#include <string>
#include <unordered_map>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "boost/process.hpp"
#include "boost/system/error_code.hpp"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace {
TEST(SystemStatusInfoTest, TestGetOmcCheckInfoProto) {
  int count = 0;
  qcraft::SystemStatusInfo system_status_info;

  while (count++ < 3) {
    const auto proto = system_status_info.GetOmcStatusInfoProto();

    CHECK_EQ(proto.kernel().node(), "omc");
    CHECK_EQ(proto.kernel().kernel(), "Linux");
    LOG(INFO) << "Omc system check info: " << proto.DebugString();
  }
}

}  // namespace
}  // namespace qcraft
