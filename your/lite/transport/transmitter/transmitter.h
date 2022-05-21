#ifndef ONBOARD_LITE_TRANSPORT_TRANSMITTER_TRANSMITTER_H_
#define ONBOARD_LITE_TRANSPORT_TRANSMITTER_TRANSMITTER_H_

#include <grpc++/grpc++.h>

#include <memory>

#include "absl/synchronization/mutex.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/transport/proto/node_service.grpc.pb.h"
#include "onboard/node/node_query_struct.h"
#include "onboard/params/utils/param_util.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
class Transmitter {
 public:
  explicit Transmitter(const NodeConfig &node_config,
                       const RunParamsProtoV2 &run_param);
  ~Transmitter();

  virtual void Enable();

  virtual void Disable();

  virtual bool Transmit(const LiteMsgWrapper &lite_msg) = 0;

  virtual bool Transmit(const ShmMessage &shm_message) = 0;

  bool SendQueryCmd(const NodeQeuryRequest &request,
                    NodeQeuryResponse *response);

 protected:
  NodeService::Stub GetNodeServiceStub() const { return *stub_.get(); }

  const NodeConfig &GetNodeConfig() { return node_config_; }

  bool BuildTransmitLiteMsgRequest(const LiteMsgWrapper &lite_msg,
                                   TransmitLiteMsgRequest *request);

  bool BuildTransmitShmMsgRequest(const ShmMessage &shm_message,
                                  TransmitShmMsgRequest *request);

 private:
  bool UpdateRequestHeader(const LiteHeader &shm_header,
                           LiteHeader *request_header);

  ::grpc::Status QueryRemoteNodeStatus(
      ::grpc::ClientContext *context,
      const ::qcraft::QueryRemoteNodeRequest &request,
      ::qcraft::QueryRemoteNodeResponse *response);

 private:
  NodeConfig node_config_;

  absl::Mutex run_param_mutex_;
  RunParamsProtoV2 run_param_ GUARDED_BY(&run_param_mutex_);

  std::unique_ptr<NodeService::Stub> stub_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_TRANSMITTER_TRANSMITTER_H_
