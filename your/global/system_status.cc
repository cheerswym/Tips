#include "onboard/global/system_status.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "boost/filesystem.hpp"
#include "boost/process.hpp"
#include "boost/system/error_code.hpp"
#include "glog/logging.h"
#include "onboard/camera/v3/camera_util_v4l2.h"
#include "onboard/global/system_info.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft {

SystemStatusInfo::SystemStatusInfo() {}

SystemStatusInfo::~SystemStatusInfo() {}

std::string SystemStatusInfo::GetCommandResult(const std::string &cmd) {
  boost::process::ipstream pipe_stream;
  boost::process::child c(cmd, boost::process::std_out > pipe_stream);
  c.wait();
  if (c.exit_code() != EXIT_SUCCESS) {
    LOG(ERROR) << "Failed to get cmd result: " << cmd;
    return "";
  }
  std::string str;
  std::getline(pipe_stream, str);
  return str;
}

void SystemStatusInfo::GetKernelInfo(const std::string &node,
                                     KernelInfo *proto) {
  CHECK(proto != nullptr);
  proto->set_node(node);
  proto->set_kernel(GetCommandResult("uname -s"));
  proto->set_hostname(GetCommandResult("uname -n"));
  proto->set_release(GetCommandResult("uname -r"));
  proto->set_version(GetCommandResult("uname -v"));
  proto->set_machine(GetCommandResult("uname -m"));
  proto->set_date(GetCommandResult("date +%F_%T"));
  struct sysinfo s_info;
  if (sysinfo(&s_info) == 0) {
    proto->set_uptime(s_info.uptime);
    time_t boot_time = time(0) - s_info.uptime;
    char buff[64] = {0};
    ctime_r(&boot_time, buff);
    proto->set_reboot_date(buff);
  }
  LOG(INFO) << "Finish to get kernel info.";
}

int32_t SystemStatusInfo::OpenDevice(const std::string &device) {
  int32_t fd = open(device.c_str(), O_RDWR, 0);
  if (fd == -1) {
    LOG(ERROR) << "Failed to open: " << device;
    return -1;
  }
  LOG(INFO) << "Success to open: " << device << " fd: " << fd;
  return fd;
}

void SystemStatusInfo::CloseDevice(int32_t fd) {
  if (fd != -1) {
    if (close(fd) == -1) {
      LOG(ERROR) << "Faild close device. " << fd;
    }
  }
}

void SystemStatusInfo::GetVideoStreamCapability(int32_t fd,
                                                VideoStreamCapability *proto) {
  CHECK(proto != nullptr);
  v4l2_capability cap;
  camera::v3::SetZero(&cap);
  if (camera::v3::xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
    LOG(ERROR) << "Failed to query capability: " << fd;
    return;
  }

  proto->set_driver(reinterpret_cast<char *>(cap.driver));
  proto->set_card(reinterpret_cast<char *>(cap.card));
  proto->set_bus_info(reinterpret_cast<char *>(cap.bus_info));
  proto->set_version(cap.version);
  proto->set_capabilities(cap.capabilities);
  proto->set_dev_caps(cap.device_caps);
}

void SystemStatusInfo::GetVideoStreamFormatDesc(int32_t fd,
                                                VideoStreamParam *proto) {
  CHECK(proto != nullptr);
  v4l2_fmtdesc fmt;
  camera::v3::SetZero(&fmt);
  fmt.index = 0;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (camera::v3::xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
    VideoStreamFormatDescription *desc = proto->add_fmts();
    desc->set_index(fmt.index);
    desc->set_type(fmt.type);
    desc->set_flags(fmt.flags);
    desc->set_description(reinterpret_cast<char *>(fmt.description));
    desc->set_pixel_format(fmt.pixelformat);
    fmt.index++;
  }
}

void SystemStatusInfo::GetVideoStreamFormatSetting(int32_t fd,
                                                   VideoStreamFormat *proto) {
  CHECK(proto != nullptr);
  v4l2_format fmt;
  camera::v3::SetZero(&fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (camera::v3::xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
    LOG(ERROR) << "Failed to get format setting: " << fd;
    return;
  }
  v4l2_pix_format *pix_format = reinterpret_cast<v4l2_pix_format *>(&fmt.fmt);

  proto->set_type(fmt.type);
  PixFormat *pix = proto->mutable_fmt();
  pix->set_width(pix_format->width);
  pix->set_height(pix_format->height);
  pix->set_pixel_format(pix_format->pixelformat);
  pix->set_field(pix_format->field);
  pix->set_bytes_perline(pix_format->bytesperline);
  pix->set_sizeimage(pix_format->sizeimage);
  pix->set_colorspace(pix_format->colorspace);
}

void SystemStatusInfo::GetVideoStreamInputDesc(int32_t fd,
                                               VideoStreamParam *proto) {
  CHECK(proto != nullptr);
  v4l2_input input;
  camera::v3::SetZero(&input);
  input.index = 0;
  while (camera::v3::xioctl(fd, VIDIOC_ENUMINPUT, &input) == 0) {
    VideoStreamInputDescription *desc = proto->add_inputs();
    desc->set_index(input.index);
    desc->set_name(reinterpret_cast<char *>(input.name));
    switch (input.type) {
      case V4L2_INPUT_TYPE_TUNER:
        desc->set_type(INPUT_TYPE_TUNER);
        break;
      case V4L2_INPUT_TYPE_CAMERA:
        desc->set_type(INPUT_TYPE_CAMERA);
        break;
      case V4L2_INPUT_TYPE_TOUCH:
        desc->set_type(INPUT_TYPE_TOUCH);
        break;
    }
    desc->set_std(input.std);
    desc->set_status(input.status);
    std::string status;

    if (input.status == 0) {
      status = "ok;";
    } else {
      // general
      if (input.status & V4L2_IN_ST_NO_POWER) status += "no power;";
      if (input.status & V4L2_IN_ST_NO_SIGNAL) status += "no signal;";
      if (input.status & V4L2_IN_ST_NO_COLOR) status += "no color;";

      // sensor orientation
      if (input.status & V4L2_IN_ST_HFLIP) status += "flip horizontally;";
      if (input.status & V4L2_IN_ST_VFLIP) status += "flip vertically;";

      // analog
      if (input.status & V4L2_IN_ST_NO_H_LOCK) status += "no horizontal lock;";
      if (input.status & V4L2_IN_ST_COLOR_KILL) status += "color killer;";
      if (input.status & V4L2_IN_ST_NO_V_LOCK) status += "no vertical lock;";
      if (input.status & V4L2_IN_ST_NO_STD_LOCK)
        status += "no standard format lock;";

      // digital
      if (input.status & V4L2_IN_ST_NO_SYNC)
        status += "no synchronization lock;";
      if (input.status & V4L2_IN_ST_NO_EQU) status += "No equalizer lock;";
      if (input.status & V4L2_IN_ST_NO_CARRIER)
        status += "carrier recovery failed;";

      // VCR and set-top box
      if (input.status & V4L2_IN_ST_MACROVISION)
        status += "mcrovision detected;";
      if (input.status & V4L2_IN_ST_NO_ACCESS)
        status += "conditional access denied;";
      if (input.status & V4L2_IN_ST_VTR) status += "VTR time constant;";
    }
    cameras_[reinterpret_cast<char *>(input.name)] = status;
    desc->set_capabilities(input.capabilities);
    input.index++;
  }
}

void SystemStatusInfo::CheckVideoStreamInput() {
  CHECK_EQ(std::system("ls /dev/video* > video.txt"), 0);
  std::ifstream fin("video.txt", std::ios::in);
  std::string dev_node;
  while (fin >> dev_node) {
    // dev node name, eg. /dev/video0
    VideoStreamParam stream;
    LOG(INFO) << "Prepare to get video node info: " << dev_node;
    stream.set_name(dev_node);
    auto fd = OpenDevice(dev_node);
    GetVideoStreamCapability(fd, stream.mutable_cap());
    LOG(INFO) << "Success to get capability.";
    GetVideoStreamFormatDesc(fd, &stream);
    LOG(INFO) << "Success to get format description.";
    GetVideoStreamFormatSetting(fd, stream.mutable_fmt_setting());
    LOG(INFO) << "Success to get format setting.";
    GetVideoStreamInputDesc(fd, &stream);
    LOG(INFO) << "Success to get input description.";
    stream.set_input_setting(GetVideoStreamInputSetting(fd));
    LOG(INFO) << "Success to get input setting.";
    GetVideoStreamStandardDesc(fd, &stream);
    LOG(INFO) << "Success to get standard description.";

    for (const auto &input : stream.inputs()) {
      streams_[input.name()].push_back(stream);
    }
    CloseDevice(fd);
  }
  fin.close();
  std::remove("video.txt");
}

int32_t SystemStatusInfo::GetVideoStreamInputSetting(int32_t fd) {
  int32_t input = 0;
  if (camera::v3::xioctl(fd, VIDIOC_G_INPUT, &input) < 0) {
    LOG(ERROR) << "Failed to get input: " << fd;
    return -1;
  }
  return input;
}

void SystemStatusInfo::GetVideoStreamStandardDesc(int32_t fd,
                                                  VideoStreamParam *proto) {
  CHECK(proto != nullptr);
  v4l2_standard std;
  camera::v3::SetZero(&std);
  std.index = 0;
  while (camera::v3::xioctl(fd, VIDIOC_ENUMSTD, &std) == 0) {
    VideoStreamStandardDescription *desc = proto->add_standards();
    desc->set_index(std.index);
    desc->set_std_id(std.id);
    desc->set_name(reinterpret_cast<char *>(std.name));
    desc->set_framelines(std.framelines);
    FramePeriod *fp = desc->mutable_frameperiod();
    fp->set_numerator(std.frameperiod.numerator);
    fp->set_denominator(std.frameperiod.denominator);
    std.index++;
  }
}

void SystemStatusInfo::GetCameraSetting(SensorInfo *proto) {
  CHECK(proto != nullptr);
  CheckVideoStreamInput();
  CHECK_EQ(streams_.size(), cameras_.size());
  for (const auto &[camera_name, camera_status] : cameras_) {
    CameraSetting *camera = proto->add_cameras();
    camera->mutable_status()->set_name(camera_name);
    camera->mutable_status()->set_status(camera_status);
    for (const auto &stream : streams_[camera_name]) {
      *camera->add_streams() = stream;
    }
  }
}

void SystemStatusInfo::GetXavierInfo(XavierInfo *proto) {
  CHECK(proto != nullptr);
  std::string cmd_result = GetCommandResult("cat /proc/version");

  const std::vector<std::string> strs = absl::StrSplit(cmd_result, "[");
  const std::vector<std::string> substrs =
      absl::StrSplit(strs[1].substr(0, strs[1].size() - 2), ", ");
  for (auto str : substrs) {
    const std::vector<std::string> infos = absl::StrSplit(str, ":");
    if (infos[0] == "product") {
      proto->set_product(infos[1]);
    } else if (infos[0] == "hardware") {
      proto->set_hardware(infos[1]);
    } else if (infos[0] == "jetpack") {
      proto->set_jetpack(infos[1]);
    } else if (infos[0] == "soft") {
      proto->set_software(infos[1]);
    } else if (infos[0] == "type") {
      proto->set_type(infos[1]);
    } else {
      LOG(INFO) << "Wrong key: " << infos[0] << " value: " << infos[1];
    }
  }
}

ObcStatusInfoProto SystemStatusInfo::GetObcStatusInfoProto() {
  cameras_.clear();
  streams_.clear();
  ObcStatusInfoProto proto;
  GetKernelInfo("obc", proto.mutable_kernel());
  LOG(INFO) << "Success to get kernel info.";
  GetXavierInfo(proto.mutable_xavier());
  LOG(INFO) << "Success to get xavier info.";
  GetCameraSetting(proto.mutable_sensor());
  LOG(INFO) << "Success to get camera setting.";
  return proto;
}

void SystemStatusInfo::GetCpuInfo(CpuInfo *proto) {
  std::ifstream fin("/proc/cpuinfo", std::ios::in);
  std::string str;
  constexpr char kProcessor[] = "processor";
  constexpr char kModel[] = "model name";
  constexpr char kCpuCores[] = "cpu cores";
  int32_t processor_num = 0, core_num = 0;
  std::string model_name;
  while (std::getline(fin, str)) {
    if (str.find(kProcessor) != std::string::npos) {
      processor_num++;
    }
    if ((str.find(kModel) != std::string::npos) && model_name.empty()) {
      model_name = str.substr(str.find(":") + 2);
    }

    if ((str.find(kCpuCores) != std::string::npos) && core_num == 0) {
      core_num = std::stoi(str.substr(str.find(":") + 2));
    }
  }
  fin.close();
  proto->set_cpu_num(processor_num);
  proto->set_core_num(core_num);
  proto->set_model(model_name);
}

OmcStatusInfoProto SystemStatusInfo::GetOmcStatusInfoProto() {
  OmcStatusInfoProto proto;
  GetKernelInfo("omc", proto.mutable_kernel());
  GetCpuInfo(proto.mutable_cpu());
  SystemInfo::Instance()->GetGpuDriverProto(proto.mutable_gpu_driver());
  LOG(INFO) << "Success to get omc info.";
  return proto;
}

}  // namespace qcraft
