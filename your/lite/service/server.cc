#include "onboard/lite/service/server.h"

#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/logging.h"

DECLARE_bool(enable_node_verbose_log);
DECLARE_bool(enable_node_publish_lidar_frame);
DECLARE_bool(enable_node_publish_encoded_lidar_frame);
DECLARE_string(transport_mode);

namespace qcraft {

Server::Server(const NodeConfig& node_config)
    : node_config_(node_config),
      server_base_(std::make_unique<ServerBase>(node_config.node_ip() + ":" +
                                                node_config.node_port())) {}

Server::~Server() { Destroy(); }

bool Server::Init() {
  TransportFactory transport_factory;
  TransportMode transport_mode;
  if (FLAGS_transport_mode == "grpc_stream") {
    transport_mode = TransportMode::GRPC_STREAM;
  } else if (FLAGS_transport_mode == "grpc") {
    transport_mode = TransportMode::GRPC;
  } else {
    QLOG(FATAL) << "Unknown transport_mode. expect grpc, grpc_stream";
  }
  receiver_ = transport_factory.CreateReceiver(node_config_, transport_mode);
  if (receiver_ == nullptr) {
    return false;
  }

  receiver_->Enable();

  return true;
}

void Server::SubscribLiteMsgCallback(
    const std::map<std::pair<std::string, std::string>, std::string>&
        server_msgs,
    const std::function<void(std::shared_ptr<LiteMsgWrapper>)>& callback) {
  receiver_->SubscribLiteMsgCallback(
      server_msgs,
      [callback = callback](const LiteHeader& lite_header,
                            std::shared_ptr<LiteMsgWrapper> lite_msg) {
        // TODO(liyu) consider statistics from different node
        const auto delta =
            absl::ToUnixMicros(absl::Now()) - lite_header.timestamp();
        QCOUNTER("lite_recv_delay_us", delta);  // microseconds
        if (FLAGS_enable_node_verbose_log) {
          QLOG_EVERY_N_SEC(INFO, 1.0)
              << "lite_recv_delay_ms: " << delta * 1.0 / 1000;
        }

        callback(lite_msg);
      });
}

void Server::SubscribShmMsgCallback(
    const std::function<void(std::shared_ptr<ShmMessage>)>& callback) {
  receiver_->SubscribShmMsgCallback(
      [callback = callback, this](
          const LiteHeader& lite_header, const Receiver::MetaData& metadata,
          const std::string& data, const LidarParametersProto& lidar_params) {
        // TODO(liyu) consider statistics from different node
        const auto delta =
            absl::ToUnixMicros(absl::Now()) - lite_header.timestamp();
        QCOUNTER("shm_recv_delay_us", delta);  // microseconds
        if (FLAGS_enable_node_verbose_log) {
          QLOG_EVERY_N_SEC(INFO, 1.0)
              << "shm_recv_delay_ms: " << delta * 1.0 / 1000;
        }

        switch (metadata.index()) {
          case 0: {
            const auto& encoded_lidar_frame_metadata =
                std::get<EncodedLidarFrameMetaData>(metadata);
            if (FLAGS_enable_node_publish_encoded_lidar_frame) {
              auto shm_encoded_lidar_frame = ShmMessage::CreateToWrite(
                  data.size(), SHM_ENCODED_LIDAR_FRAME, lite_header.channel(),
                  lite_header.domain());
              memcpy(shm_encoded_lidar_frame->mutable_buffer(), data.c_str(),
                     data.size());
              *shm_encoded_lidar_frame->mutable_shm_msg_metadata()
                   ->MutableExtension(
                       EncodedLidarFrameMetaData::encoded_lidar_frame_meta) =
                  encoded_lidar_frame_metadata;
              shm_encoded_lidar_frame->mutable_shm_msg_metadata()
                  ->mutable_header()
                  ->set_full_node_name(lite_header.full_node_name());
              callback(std::move(shm_encoded_lidar_frame));
            }

            // TODO(liyu) maybe we need move to another module for performance.
            if (encoded_lidar_frame_metadata.encoding_type() == LDE_SNAPPY) {
              if (FLAGS_enable_node_publish_lidar_frame) {
                auto shm_lidar_frame =
                    BuildLidarFrame(lite_header, encoded_lidar_frame_metadata,
                                    data, lidar_params);
                callback(std::move(shm_lidar_frame));
              }
            }
          } break;
          case 1: {
            const auto& encoded_image_metadata =
                std::get<EncodedImageMetadata>(metadata);
            auto shm_encoded_image = ShmMessage::CreateToWrite(
                data.size(), SHM_ENCODED_IMAGE, lite_header.channel(),
                lite_header.domain());
            memcpy(shm_encoded_image->mutable_buffer(), data.c_str(),
                   data.size());
            *shm_encoded_image->mutable_shm_msg_metadata()->MutableExtension(
                EncodedImageMetadata::encoded_image_meta) =
                encoded_image_metadata;
            shm_encoded_image->mutable_shm_msg_metadata()
                ->mutable_header()
                ->set_full_node_name(lite_header.full_node_name());
            callback(std::move(shm_encoded_image));
          } break;
        }
      });
}

void Server::QueryCmdCallback(
    const std::function<bool(const NodeQeuryRequest&, NodeQeuryResponse*)>&
        callback) {
  receiver_->QueryCmdCallback(callback);
}

void Server::Destroy() {
  if (receiver_ != nullptr) {
    receiver_->Disable();
  }

  receiver_.reset();
}

const SpinProcessor& Server::GetSpinProcessor(
    const LidarParametersProto& lidar_params) {
  absl::MutexLock lock(&spin_processors_mutex_);
  const LidarId lidar_id = lidar_params.installation().lidar_id();
  if (!ContainsKey(spin_processors_, lidar_id)) {
    spin_processors_[lidar_id] = std::make_unique<SpinProcessor>(lidar_params);
  }
  return *spin_processors_[lidar_id];
}

std::unique_ptr<ShmMessage> Server::BuildLidarFrame(
    const LiteHeader& lite_header,
    const EncodedLidarFrameMetaData& encoded_lidar_frame_metadata,
    const std::string& data, const LidarParametersProto& lidar_params) {
  std::unique_ptr<ShmMessage> shm_lidar_frame;
  spin_util::DecodeWithSnappy(data, &lidar_frame_);
  if (encoded_lidar_frame_metadata.data_type() == LDD_SPIN) {
    shm_lidar_frame = Spin::MakeSpinOnShm(static_cast<LidarModel>(
        static_cast<int>(encoded_lidar_frame_metadata.type())));
    spin_assembler_.AssembleSpinFromEncodedString(
        lidar_frame_, GetSpinProcessor(lidar_params),
        /*estimate_normal=*/false, shm_lidar_frame->mutable_value<Spin>());
  } else {
    shm_lidar_frame = PointCloud::MakePointCloudOnShm();
    point_cloud_assembler_.AssemblePointCloudFromEncodedString(
        lidar_frame_, lidar_params,
        encoded_lidar_frame_metadata.compat_version(),
        /*estimate_normal=*/false,
        shm_lidar_frame->mutable_value<PointCloud>());
  }
  const auto channel = shm_lidar_frame->shm_msg_metadata().header().channel();
  const auto domain = shm_lidar_frame->shm_msg_metadata().header().domain();
  auto* spin_metadata =
      shm_lidar_frame->mutable_shm_msg_metadata()->MutableExtension(
          SpinMetadata::spin_meta);
  spin_metadata->set_is_partial(encoded_lidar_frame_metadata.is_partial());
  spin_metadata->set_num_scans(encoded_lidar_frame_metadata.num_scans());
  spin_metadata->set_num_points(encoded_lidar_frame_metadata.num_points());
  spin_metadata->set_type(encoded_lidar_frame_metadata.type());
  spin_metadata->set_id(encoded_lidar_frame_metadata.id());
  spin_metadata->set_data_type(encoded_lidar_frame_metadata.data_type());
  spin_metadata->set_compat_version(
      encoded_lidar_frame_metadata.compat_version());
  auto* header = shm_lidar_frame->mutable_shm_msg_metadata()->mutable_header();
  *header = lite_header;
  header->set_channel(channel);
  header->set_domain(domain);
  header->set_full_node_name(lite_header.full_node_name());

  return shm_lidar_frame;
}

}  // namespace qcraft
