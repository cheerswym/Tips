#include <fstream>
#include <iostream>
#include <vector>

#include "absl/strings/str_split.h"
#include "boost/filesystem.hpp"
#include "boost/process.hpp"
#include "boost/system/error_code.hpp"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/camera/v3/camera_util_v4l2.h"
#include "onboard/global/system_status.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace {

const uint32_t ColorSpacesRGB = 8;
const uint32_t SizeImage = 4147200;
const uint32_t FmtYUYV = 1448695129;

TEST(SystemStatusInfoTest, TestGetObcCheckInfoProto) {
  int count = 0;
  qcraft::SystemStatusInfo system_status_info;

  while (count++ < 3) {
    const auto proto = system_status_info.GetObcStatusInfoProto();
    CHECK_EQ(proto.kernel().node(), "obc");
    CHECK_EQ(proto.kernel().kernel(), "Linux");

    CHECK_EQ(proto.xavier().product(), "geac91");

    CHECK_EQ(proto.sensor().cameras_size(), 4);
    for (const auto &camera : proto.sensor().cameras()) {
      CHECK_EQ(camera.status().status(), "ok;");
      CHECK_EQ(camera.streams_size(), 2);
      for (const auto &stream : camera.streams()) {
        CHECK_EQ(stream.cap().driver(), "tegra-video");
        CHECK_EQ(stream.cap().version(), 0x4098C);
        CHECK_EQ(stream.fmt_setting().fmt().width(), 1920);
        CHECK_EQ(stream.fmt_setting().fmt().height(), 1080);
        CHECK_EQ(stream.fmt_setting().fmt().bytes_perline(), 3840);
        CHECK_EQ(stream.fmt_setting().fmt().colorspace(), ColorSpacesRGB);
        CHECK_EQ(stream.fmt_setting().fmt().sizeimage(), SizeImage);
        CHECK_EQ(stream.fmt_setting().fmt().pixel_format(), FmtYUYV);
      }
    }
    LOG(INFO) << "obc info: " << proto.DebugString();
  }
}

}  // namespace
}  // namespace qcraft
