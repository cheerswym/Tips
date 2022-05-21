#ifndef ONBOARD_LITE_SERVICE_SERVER_H_
#define ONBOARD_LITE_SERVICE_SERVER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/spin_util.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/service/server_base.h"
#include "onboard/lite/transport/transport_factory.h"
#include "onboard/node/node_query_struct.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
class Server {
 public:
  explicit Server(const NodeConfig& node_config);

  Server() = delete;

  virtual ~Server();

  bool Init();

  void SubscribLiteMsgCallback(
      const std::map<std::pair<std::string, std::string>, std::string>&
          server_msgs,
      const std::function<void(std::shared_ptr<LiteMsgWrapper>)>& callback);

  void SubscribShmMsgCallback(
      const std::function<void(std::shared_ptr<ShmMessage>)>& callback);

  void QueryCmdCallback(
      const std::function<bool(const NodeQeuryRequest&, NodeQeuryResponse*)>&
          callback);

  void Destroy();

 public:
  const std::string& ServerName() const { return server_base_->ServerName(); }

 private:
  const SpinProcessor& GetSpinProcessor(
      const LidarParametersProto& lidar_params)
      LOCKS_EXCLUDED(spin_processors_mutex_);

  std::unique_ptr<ShmMessage> BuildLidarFrame(
      const LiteHeader& lite_header,
      const EncodedLidarFrameMetaData& encoded_lidar_frame_metadata,
      const std::string& data, const LidarParametersProto& lidar_params);

 private:
  NodeConfig node_config_;
  std::unique_ptr<ServerBase> server_base_;
  std::shared_ptr<Receiver> receiver_;

  absl::Mutex spin_processors_mutex_;
  std::map<LidarId, std::unique_ptr<SpinProcessor>> spin_processors_
      GUARDED_BY(spin_processors_mutex_);

  spin_util::SpinAssembler spin_assembler_;
  spin_util::PointCloudAssembler point_cloud_assembler_;

  std::string lidar_frame_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_SERVICE_SERVER_H_
