#ifndef ONBOARD_GLOBAL_SYSTEM_STATUS_H
#define ONBOARD_GLOBAL_SYSTEM_STATUS_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/proto/system_status_info.pb.h"

namespace qcraft {

class SystemStatusInfo {
 public:
  SystemStatusInfo();

  ~SystemStatusInfo();

  // collect xavier status info to proto.
  ObcStatusInfoProto GetObcStatusInfoProto();

  // collect omc status info to proto.
  OmcStatusInfoProto GetOmcStatusInfoProto();

 private:
  std::string GetCommandResult(const std::string& cmd);

  void GetKernelInfo(const std::string& node, KernelInfo* proto);

  int OpenDevice(const std::string& device);

  void CloseDevice(int fd);

  void GetVideoStreamCapability(int fd, VideoStreamCapability* proto);

  void GetVideoStreamFormatDesc(int fd, VideoStreamParam* proto);

  void GetVideoStreamFormatSetting(int fd, VideoStreamFormat* proto);

  void GetVideoStreamInputDesc(int fd, VideoStreamParam* proto);

  int32_t GetVideoStreamInputSetting(int fd);

  void CheckVideoStreamInput();

  void GetVideoStreamStandardDesc(int fd, VideoStreamParam* proto);

  void GetCameraSetting(SensorInfo* proto);

  void GetXavierInfo(XavierInfo* proto);

  void GetCpuInfo(CpuInfo* proto);
  // {camera_name, camera_status};
  std::unordered_map<std::string, std::string> cameras_;
  std::unordered_map<std::string, std::vector<VideoStreamParam>> streams_;
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SYSTEM_STATUS_H
