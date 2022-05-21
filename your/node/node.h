#ifndef ONBOARD_NODE_NODE_H_
#define ONBOARD_NODE_NODE_H_

#include <future>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "onboard/lite/service/client.h"
#include "onboard/lite/service/server.h"
#include "onboard/node/node_query_struct.h"
#include "onboard/params/param_manager.h"

namespace qcraft {
class Node {
 public:
  using CollectionChannelDomainToFieldName =
      std::map<std::pair<std::string, std::string>, std::string>;
  using MsgChannelDomain = std::pair<std::string, std::string>;

  virtual ~Node();

  void OnInit();

  void OnSubscribeChannels(
      const std::function<void(
          const std::string &, const std::string &, const std::string &,
          std::function<void(std::shared_ptr<const LiteMsgWrapper>)>,
          std::function<void(std::shared_ptr<const ShmMessage>)>)> &subscribe,
      const std::function<void(std::shared_ptr<LiteMsgWrapper>)> &lite_callback,
      const std::function<void(std::shared_ptr<ShmMessage>)> &shm_callback);

  void OnSetUpTimers();

 private:
  void OnModuleShmMessage(std::shared_ptr<const ShmMessage> shm_msg);

  void OnModuleLiteMessage(std::shared_ptr<const LiteMsgWrapper> lite_msg);

  void ForwardLiteMessage(const LiteMsgWrapper &lite_msg);

  void ForwardShmMessage(const ShmMessage &shm_message);

  void QueryRemoteNodesInputsAndWaitForReturns();

  bool QueryRemoteNodeInputs(const std::string &node_name);

  bool ProcessQueryCmd(const NodeQeuryRequest &, NodeQeuryResponse *);

  CollectionChannelDomainToFieldName GetHostNodeInputsCollection();

  CollectionChannelDomainToFieldName GetHostNodeOutputsCollection();

  CollectionChannelDomainToFieldName GetClientNodeInputsCollection(
      const NodeConfig &client_node_config);

  // if failed to query node inputs, assume a minimal set as failure fallback.
  CollectionChannelDomainToFieldName
  GetClientNodeInputsQueryFailedFallbackCollection(
      const NodeConfig &client_node_config);

 private:
  explicit Node(const NodeConfig &node_config,
                const ParamManager &param_manager);

 public:
  static std::vector<std::unique_ptr<Node>> CreateNodes(
      const std::string &node_name, const std::string &name_space,
      const ParamManager &param_manager);

 private:
  NodeConfig node_config_;

  std::unique_ptr<Server> server_;
  std::unordered_map<std::string, std::shared_ptr<Client>> clients_;
  std::map<MsgChannelDomain, std::vector<std::shared_ptr<Client>>>
      publish_clients_;

  std::map<std::string, CollectionChannelDomainToFieldName>
      client_nodes_inputs_by_query_;
  CollectionChannelDomainToFieldName host_node_inputs_for_query_;
  bool query_remote_asked_and_failed_flag_ = false;
  std::map<std::string, bool> client_node_name_to_query_status_;

  absl::Mutex msg_mutex_;
  std::atomic<uint64_t> latest_message_ts_{0};
  std::queue<std::shared_ptr<const LiteMsgWrapper>> lite_msg_queue_
      GUARDED_BY(msg_mutex_);
  std::queue<std::shared_ptr<const ShmMessage>> shm_message_queue_
      GUARDED_BY(msg_mutex_);
  std::map<int, int64> shm_message_count_ GUARDED_BY(&msg_mutex_);

  absl::Notification stop_notification_;
  std::future<void> lite_publisher_future_;
  std::future<void> shm_publisher_future_;
  // node internal fault notice to others
  std::future<void> fault_publisher_future_;

  const ParamManager &param_manager_;
};

}  // namespace qcraft

#endif  // ONBOARD_NODE_NODE_H_
