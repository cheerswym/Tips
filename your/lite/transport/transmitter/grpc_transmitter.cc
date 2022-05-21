#include "onboard/lite/transport/transmitter/grpc_transmitter.h"

#include "onboard/global/clock.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"

namespace {
gpr_timespec GetTransmitDeadline(int64_t timeout) {
  return gpr_time_add(gpr_now(GPR_CLOCK_MONOTONIC),
                      gpr_time_from_millis(timeout, GPR_TIMESPAN));
}
}  // namespace

namespace qcraft {

GRPCTransmitter::GRPCTransmitter(const NodeConfig &node_config,
                                 const RunParamsProtoV2 &run_param)
    : Transmitter(node_config, run_param) {}

GRPCTransmitter::~GRPCTransmitter() {}

bool GRPCTransmitter::Transmit(const LiteMsgWrapper &lite_msg) {
  TransmitLiteMsgRequest request;
  if (!BuildTransmitLiteMsgRequest(lite_msg, &request)) {
    return true;
  }

  TransmitLiteMsgResponse response;
  grpc::ClientContext context;
  if (GetNodeConfig().has_timeout()) {
    const auto channel_domain =
        LiteMsgConverter::Get().GetLiteMsgChannelDomain(lite_msg);
    if (channel_domain.first == "trace_proto") {
      context.set_deadline(GetTransmitDeadline(GetNodeConfig().timeout() * 4));
    } else {
      context.set_deadline(GetTransmitDeadline(GetNodeConfig().timeout()));
    }
  }

  const auto status = GrpcTransmitLiteMsg(&context, request, &response);
  if (!status.ok()) {
    const auto error_code = status.error_code();
    if (error_code == grpc::StatusCode::DEADLINE_EXCEEDED) {
      const auto &lite_header =
          LiteMsgConverter::Get().GetLiteMsgWrapperHeader(lite_msg);
      const auto name = lite_header.full_node_name() + "/" +
                        LiteModuleName_Name(lite_header.module_id()) + "/" +
                        lite_header.channel() + "/" + lite_header.domain();
      CounterEvent counter_event{"lite_transmit_timeout", 1,
                                 absl::ToUnixMillis(Clock::Now())};
      counter_event.fields[name] = 1;
      QCOUNTER_EVENT(counter_event);
      return true;
    } else {
      QLOG_EVERY_N_SEC(WARNING, 3.0)
          << "TransmitLiteMsg: " << GetNodeConfig().node_ip() << ", "
          << status.error_message();
    }

    return false;
  }

  CHECK(response.success());
  return true;
}

bool GRPCTransmitter::Transmit(const ShmMessage &shm_message) {
  TransmitShmMsgRequest request;
  if (!BuildTransmitShmMsgRequest(shm_message, &request)) {
    return true;
  }

  TransmitShmMsgResponse response;
  grpc::ClientContext context;
  if (GetNodeConfig().has_timeout()) {
    context.set_deadline(GetTransmitDeadline(GetNodeConfig().timeout()));
  }

  const auto status = GrpcTransmitShmMsg(&context, request, &response);
  if (!status.ok()) {
    const auto error_code = status.error_code();
    if (error_code == grpc::StatusCode::DEADLINE_EXCEEDED) {
      const auto &shm_message_metadata = shm_message.shm_msg_metadata();
      const auto &lite_header = shm_message_metadata.header();
      const auto name = lite_header.full_node_name() + "/" +
                        LiteModuleName_Name(lite_header.module_id()) + "/" +
                        lite_header.channel() + "/" + lite_header.domain();
      CounterEvent counter_event{"shm_transmit_timeout", 1,
                                 absl::ToUnixMillis(Clock::Now())};
      counter_event.fields[name] = 1;
      QCOUNTER_EVENT(counter_event);
      return true;
    } else {
      QLOG_EVERY_N_SEC(WARNING, 3.0)
          << "TransmitShmMsg: " << GetNodeConfig().node_ip() << ", "
          << status.error_message();
    }

    return false;
  }

  CHECK(response.success());
  return true;
}

::grpc::Status GRPCTransmitter::GrpcTransmitLiteMsg(
    ::grpc::ClientContext *context,
    const ::qcraft::TransmitLiteMsgRequest &request,
    ::qcraft::TransmitLiteMsgResponse *response) {
  return GetNodeServiceStub().TransmitLiteMsg(context, request, response);
}

::grpc::Status GRPCTransmitter::GrpcTransmitShmMsg(
    ::grpc::ClientContext *context,
    const ::qcraft::TransmitShmMsgRequest &request,
    ::qcraft::TransmitShmMsgResponse *response) {
  return GetNodeServiceStub().TransmitShmMsg(context, request, response);
}

}  // namespace qcraft
