#ifndef ONBOARD_LITE_SERVICE_CLIENT_H_
#define ONBOARD_LITE_SERVICE_CLIENT_H_

#include <memory>
#include <string>

#include "absl/synchronization/mutex.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/service/client_base.h"
#include "onboard/lite/transport/transport_factory.h"
#include "onboard/node/node_query_struct.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
class Client {
 public:
  explicit Client(const NodeConfig &node_config);

  Client() = delete;

  virtual ~Client();

  bool Init(const RunParamsProtoV2 &run_param) LOCKS_EXCLUDED(mutex_);

  bool PublishLiteMsg(const LiteMsgWrapper &lite_msg) LOCKS_EXCLUDED(mutex_);

  bool PublishShmMsg(const ShmMessage &shm_message) LOCKS_EXCLUDED(mutex_);

  void Destroy() LOCKS_EXCLUDED(mutex_);

  bool SendQueryCmd(const NodeQeuryRequest &request,
                    NodeQeuryResponse *response) LOCKS_EXCLUDED(mutex_);

 public:
  const std::string &ServerName() const { return client_base_->ServerName(); }

  bool ServerIsReady() const { return client_base_->ServerIsReady(); }

 private:
  bool CheckTransmitStatus() LOCKS_EXCLUDED(mutex_);
  void UpdateTransmitStatus(bool status) LOCKS_EXCLUDED(mutex_);

 private:
  NodeConfig node_config_;
  std::unique_ptr<ClientBase> client_base_;

  absl::Mutex mutex_;
  std::shared_ptr<Transmitter> transmitter_ GUARDED_BY(mutex_);

  int enable_transmit_interval_;
  int64_t enable_transmit_timepoint_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_SERVICE_CLIENT_H_
