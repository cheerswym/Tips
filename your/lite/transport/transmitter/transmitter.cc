#include "onboard/lite/transport/transmitter/transmitter.h"

#include <string>
#include <utility>

#include "onboard/global/car_common.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"
#include "onboard/node/node_util.h"

namespace {
gpr_timespec GetTransmitDeadline(int64_t timeout) {
  return gpr_time_add(gpr_now(GPR_CLOCK_MONOTONIC),
                      gpr_time_from_millis(timeout, GPR_TIMESPAN));
}
}  // namespace

namespace qcraft {

Transmitter::Transmitter(const NodeConfig &node_config,
                         const RunParamsProtoV2 &run_param)
    : node_config_(node_config), run_param_(run_param) {}

Transmitter::~Transmitter() { Disable(); }

void Transmitter::Enable() {
  std::string server_address(node_config_.node_ip() + ":" +
                             node_config_.node_port());
  stub_ = NodeService::NewStub(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
}

void Transmitter::Disable() {
  if (stub_ != nullptr) {
    stub_.reset();
  }
}

bool Transmitter::SendQueryCmd(const NodeQeuryRequest &request,
                               NodeQeuryResponse *response) {
  grpc::ClientContext context;
  constexpr int64_t kQueryRemoteNodeTimeoutMs = 200;  // ms
  context.set_deadline(GetTransmitDeadline(kQueryRemoteNodeTimeoutMs));
  QueryRemoteNodeRequest rpc_request;
  QueryRemoteNodeResponse rpc_response;
  rpc_request.set_full_node_name(request.full_node_name);
  switch (request.query_cmd) {
    case NodeQueryCmdType::QUERY_NODE_INPUTS:
      rpc_request.set_query_type(QueryRemoteNodeRequest::QUERY_NODE_INPUTS);
      break;
    default:
      QLOG(WARNING) << "Transmitter::SendQueryCmd can not recognize the "
                       "query_cmd type, enum value ("
                    << request.query_cmd << ").";
      return false;
  }
  const auto status =
      QueryRemoteNodeStatus(&context, rpc_request, &rpc_response);
  if (!status.ok()) {
    QLOG(WARNING) << "Transmitter::SendQueryCmd to " << node_config_.node_ip()
                  << " failed, " << status.error_message();
    return false;
  }
  if (rpc_response.success()) {
    if (NodeQueryCmdType::QUERY_NODE_INPUTS == request.query_cmd) {
      response->node_inputs.clear();
      for (const auto &node_input : rpc_response.node_inputs()) {
        NodeInputMsgDecl msg_decl;
        msg_decl.field_name = node_input.field_name();
        msg_decl.channel = node_input.channel();
        msg_decl.domain = node_input.domain();
        response->node_inputs.push_back(msg_decl);
      }
    }
  }
  return true;
}

bool Transmitter::BuildTransmitLiteMsgRequest(const LiteMsgWrapper &lite_msg,
                                              TransmitLiteMsgRequest *request) {
  request->mutable_lite_message()->CopyFrom(lite_msg);

  const auto &header =
      LiteMsgConverter::Get().GetLiteMsgWrapperHeader(lite_msg);
  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(lite_msg);
  if (channel_domain.first == "update_run_params_proto") {
    absl::WriterMutexLock l(&run_param_mutex_);
    run_param_ = RunParamsProtoV2::construct(
        lite_msg.update_run_params_proto().run_params());
  }

  return UpdateRequestHeader(header, request->mutable_header());
}

bool Transmitter::BuildTransmitShmMsgRequest(const ShmMessage &shm_message,
                                             TransmitShmMsgRequest *request) {
  std::string data;
  const auto &shm_message_metadata = shm_message.shm_msg_metadata();
  switch (shm_message_metadata.shm_msg_type()) {
    case SHM_ENCODED_LIDAR_FRAME: {
      data.resize(shm_message.buffer_size());
      memcpy(data.data(), shm_message.buffer(), shm_message.buffer_size());
      const auto encoded_lidar_frame_meta = shm_message_metadata.GetExtension(
          EncodedLidarFrameMetaData::encoded_lidar_frame_meta);
      *request->mutable_encoded_lidar_frame_meta() = encoded_lidar_frame_meta;

      LidarParametersProto lidar_params;
      {
        absl::ReaderMutexLock lock(&run_param_mutex_);
        lidar_params = param_util::GetLidarParams(
            run_param_, encoded_lidar_frame_meta.id());
      }
      request->mutable_lidar_params()->CopyFrom(
          param_util::ConvertLidarParamsToV1(lidar_params));
    } break;
    case SHM_ENCODED_IMAGE: {
      data.resize(shm_message.buffer_size());
      memcpy(data.data(), shm_message.buffer(), shm_message.buffer_size());
      *request->mutable_encoded_image_meta() =
          shm_message_metadata.GetExtension(
              EncodedImageMetadata::encoded_image_meta);
    } break;
    case SHM_LIDAR_FRAME:
    case SHM_DECODED_IMAGE:
    case SHM_UNKNOWN:
    case SHM_UNUSED:
      QLOG(FATAL) << "Unexpected shared memory message type.";
  }

  request->set_shm_msg_type(shm_message_metadata.shm_msg_type());
  request->set_data(std::move(data));

  return UpdateRequestHeader(shm_message_metadata.header(),
                             request->mutable_header());
}

bool Transmitter::UpdateRequestHeader(const LiteHeader &origin,
                                      LiteHeader *header) {
  if (IsOnboardMode()) {
    if (origin.has_full_node_name()) {
      return false;
    }
  }

  header->set_timestamp(absl::ToUnixMicros(absl::Now()));
  header->set_channel(origin.channel());
  header->set_domain(origin.domain());
  header->set_seq_number(origin.seq_number());
  header->set_module_id(origin.module_id());
  header->set_tag_number(origin.tag_number());
  header->set_full_node_name(GetFullNodeName());
  return true;
}

::grpc::Status Transmitter::QueryRemoteNodeStatus(
    ::grpc::ClientContext *context,
    const ::qcraft::QueryRemoteNodeRequest &request,
    ::qcraft::QueryRemoteNodeResponse *response) {
  return stub_->QueryRemoteNode(context, request, response);
}

}  // namespace qcraft
