#include "onboard/lite/service/client.h"

#include <algorithm>

#include "onboard/global/counter.h"
#include "onboard/lite/logging.h"

DECLARE_bool(enable_node_verbose_log);
DECLARE_string(transport_mode);

namespace qcraft {

namespace {
const int kEnableTransmitInterval = 10;  // unit: ms
}

Client::Client(const NodeConfig &node_config)
    : node_config_(node_config),
      client_base_(std::make_unique<ClientBase>(node_config.node_ip() + ":" +
                                                node_config.node_port())),
      enable_transmit_interval_(kEnableTransmitInterval),
      enable_transmit_timepoint_(0) {}

Client::~Client() { Destroy(); }

bool Client::Init(const RunParamsProtoV2 &run_param) {
  TransportFactory transport_factory;
  absl::WriterMutexLock lock(&mutex_);
  TransportMode transport_mode;
  if (FLAGS_transport_mode == "grpc_stream") {
    transport_mode = TransportMode::GRPC_STREAM;
  } else if (FLAGS_transport_mode == "grpc") {
    transport_mode = TransportMode::GRPC;
  } else {
    QLOG(FATAL) << "Unknown transport_mode. expect grpc, grpc_stream";
  }
  transmitter_ = transport_factory.CreateTransmitter(node_config_, run_param,
                                                     transport_mode);
  if (transmitter_ == nullptr) {
    return false;
  }

  transmitter_->Enable();
  return true;
}

bool Client::PublishLiteMsg(const LiteMsgWrapper &lite_msg) {
  if (!CheckTransmitStatus()) {
    return false;
  }

  bool status = false;
  const auto transmit_time = absl::ToUnixMicros(absl::Now());
  {
    absl::ReaderMutexLock lock(&mutex_);
    status = transmitter_->Transmit(lite_msg);
  }
  const auto delta = absl::ToUnixMicros(absl::Now()) - transmit_time;
  QCOUNTER("lite_transmit_us", delta);  // microseconds
  if (FLAGS_enable_node_verbose_log) {
    QLOG_EVERY_N_SEC(INFO, 1.0) << "lite_transmit_ms: " << delta * 1.0 / 1000;
  }

  UpdateTransmitStatus(status);
  return status;
}

bool Client::PublishShmMsg(const ShmMessage &shm_message) {
  if (!CheckTransmitStatus()) {
    return false;
  }

  bool status = false;
  const auto transmit_time = absl::ToUnixMicros(absl::Now());
  {
    absl::ReaderMutexLock lock(&mutex_);
    status = transmitter_->Transmit(shm_message);
  }
  const auto delta = absl::ToUnixMicros(absl::Now()) - transmit_time;
  QCOUNTER("shm_transmit_us", delta);  // microseconds
  if (FLAGS_enable_node_verbose_log) {
    QLOG_EVERY_N_SEC(INFO, 1.0) << "shm_transmit_ms: " << delta * 1.0 / 1000;
  }

  UpdateTransmitStatus(status);
  return status;
}

void Client::Destroy() {
  absl::WriterMutexLock lock(&mutex_);
  if (transmitter_ != nullptr) {
    transmitter_->Disable();
  }

  transmitter_.reset();
}

bool Client::CheckTransmitStatus() {
  if (enable_transmit_timepoint_ == 0) {
    return true;
  }

  if (absl::ToUnixMillis(absl::Now()) < enable_transmit_timepoint_) {
    return false;
  }

  {
    absl::WriterMutexLock lock(&mutex_);
    transmitter_->Enable();
  }
  return true;
}

void Client::UpdateTransmitStatus(bool status) {
  if (status) {
    enable_transmit_interval_ = kEnableTransmitInterval;
    enable_transmit_timepoint_ = 0;
  } else {
    enable_transmit_interval_ = std::min(enable_transmit_interval_ * 2, 3000);
    enable_transmit_timepoint_ =
        absl::ToUnixMillis(absl::Now()) + enable_transmit_interval_;
  }
}

bool Client::SendQueryCmd(const NodeQeuryRequest &request,
                          NodeQeuryResponse *response) {
  if (!CheckTransmitStatus()) {
    return false;
  }

  bool status = false;
  {
    absl::ReaderMutexLock lock(&mutex_);
    status = transmitter_->SendQueryCmd(request, response);
  }

  UpdateTransmitStatus(status);
  return status;
}

}  // namespace qcraft
