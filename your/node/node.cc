#include "onboard/node/node.h"

#include "absl/strings/match.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/node/node_util.h"
#include "onboard/params/param_manager.h"

DEFINE_bool(enable_node_verbose_log, false,
            "enable node system verbos log to debug issue.");
DEFINE_bool(enable_node_input_config, true,
            "collect node inputs by explicit config on module_configs.");
DEFINE_bool(enable_node_publish_lidar_frame, true,
            "enable node decode lidar frame for localization.");
DEFINE_bool(enable_node_publish_encoded_lidar_frame, false,
            "enable node publish encoded lidar frame for playback.");
DEFINE_bool(enable_query_remote_node, true,
            "query remote node inputs on node init.");
DEFINE_double(query_remote_node_timeout_s, 30.0,
              "query remote nodes timeout in sec.");
DEFINE_string(transport_mode, "grpc_stream",
              "lite transport mode. should be one of: grpc, grpc_stream");

namespace qcraft {

namespace {
constexpr double kMessageStalenessThresholdUs = 200 * 1000;  // us

bool IsNotEmpty(std::queue<std::shared_ptr<const LiteMsgWrapper>> *queue) {
  return !queue->empty();
}

bool IsNotEmpty(std::queue<std::shared_ptr<const ShmMessage>> *queue) {
  return !queue->empty();
}

std::string GetChannel(const Node::MsgChannelDomain &channel_domain) {
  return channel_domain.first;
}

std::string GetDomain(const Node::MsgChannelDomain &channel_domain) {
  return channel_domain.second;
}

std::string ToFormattedStr(const Node::CollectionChannelDomainToFieldName
                               &channel_domain_to_field_name) {
  std::string str("\n ========CollectionChannelDomain========\n");
  for (const auto &[channel_domain, field_name] :
       channel_domain_to_field_name) {
    const auto &channel = GetChannel(channel_domain);
    const auto &domain = GetDomain(channel_domain);
    (void)field_name;  // [[ maybe_unused ]]
    str = absl::StrCat(str, "  ", channel);
    if (!domain.empty() && (channel != domain)) {
      str = absl::StrCat(str, "_", domain);
    }
    str = absl::StrCat(str, "\n");
  }
  str = absl::StrCat(str, " =======================================\n");
  return str;
}

}  // namespace

Node::Node(const NodeConfig &node_config, const ParamManager &param_manager)
    : node_config_(node_config), param_manager_(param_manager) {}

Node::~Node() {
  stop_notification_.Notify();
  if (shm_publisher_future_.valid()) {
    shm_publisher_future_.wait();
  }

  if (lite_publisher_future_.valid()) {
    lite_publisher_future_.wait();
  }

  if (fault_publisher_future_.valid()) {
    fault_publisher_future_.wait();
  }

  for (const auto &client : clients_) {
    client.second->Destroy();
  }

  server_->Destroy();
}

void Node::OnInit() {
  const auto nodes_run_config =
      LoadNodesConfigFromCurrentNodeFile(param_manager_);
  for (const auto &node_config : nodes_run_config.nodes()) {
    if (IsThisNodeInterface(node_config_, node_config.node_ip())) {
      QCHECK(server_ == nullptr);
      auto server = std::make_unique<Server>(node_config);
      if (server->Init()) {
        server_ = std::move(server);
      } else {
        QLOG(FATAL) << "Failed to create server, Name: "
                    << GetFullNodeName(node_config);
      }
    } else if (IsConnectThisNodeDirectly(node_config_, node_config.node_ip())) {
      auto client = std::make_unique<Client>(node_config);
      RunParamsProtoV2 run_params;
      param_manager_.GetRunParams(&run_params);
      if (client->Init(run_params)) {
        clients_[GetFullNodeName(node_config)] = std::move(client);
      } else {
        QLOG(FATAL) << "Failed to create client, Name: "
                    << GetFullNodeName(node_config);
      }
    } else {
      QLOG(INFO) << "Skip this node: " << node_config.node_ip()
                 << " was caused by don't connect directly.";
    }
  }

  QCHECK(server_) << "Failed to create server, Name: "
                  << GetFullNodeName(node_config_);

  // prepare host node service and start to talk with client nodes
  host_node_inputs_for_query_ = GetHostNodeInputsCollection();
  server_->QueryCmdCallback(std::bind(&Node::ProcessQueryCmd, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
  if (IsOnboardMode()) {
    QueryRemoteNodesInputsAndWaitForReturns();
  }
}

void Node::OnSubscribeChannels(
    const std::function<void(
        const std::string &, const std::string &, const std::string &,
        std::function<void(std::shared_ptr<const LiteMsgWrapper>)>,
        std::function<void(std::shared_ptr<const ShmMessage>)>)> &subscribe,
    const std::function<void(std::shared_ptr<LiteMsgWrapper>)> &lite_callback,
    const std::function<void(std::shared_ptr<ShmMessage>)> &shm_callback) {
  CollectionChannelDomainToFieldName server_outputs;
  CollectionChannelDomainToFieldName subscribe_server_msgs;
  const auto nodes_run_config =
      LoadNodesConfigFromCurrentNodeFile(param_manager_);
  for (const auto &node_config : nodes_run_config.nodes()) {
    if (IsThisNodeInterface(node_config_, node_config.node_ip())) {
      server_outputs = GetHostNodeOutputsCollection();
      subscribe_server_msgs = GetHostNodeInputsCollection();
      break;
    }
  }

  if (FLAGS_enable_node_verbose_log) {
    QLOG(INFO) << "server_output: " << ToFormattedStr(server_outputs);
    QLOG(INFO) << "subscribe_server_msgs: "
               << ToFormattedStr(subscribe_server_msgs);
  }

  CollectionChannelDomainToFieldName subscribe_module_msgs;
  for (auto &&node_config : nodes_run_config.nodes()) {
    if (!IsThisNode(node_config_, node_config.node_name(),
                    node_config.name_space()) &&
        IsConnectThisNodeDirectly(node_config_, node_config.node_ip())) {
      const auto &client_inputs = GetClientNodeInputsCollection(node_config);
      for (const auto &[channel_domain, field_name] : client_inputs) {
        if (IsThisNodeMessage(GetChannel(channel_domain))) {
          continue;
        }
        if (ContainsKey(server_outputs, channel_domain)) {
          subscribe_module_msgs[channel_domain] = field_name;
          publish_clients_[channel_domain].push_back(
              clients_[GetFullNodeName(node_config)]);
        }
      }
    }
  }

  if (FLAGS_enable_node_verbose_log) {
    QLOG(INFO) << "subscribe_module_msgs: "
               << ToFormattedStr(subscribe_module_msgs);
  }

  for (const auto &subscribe_msg : subscribe_module_msgs) {
    subscribe(
        subscribe_msg.first.first, subscribe_msg.first.second,
        subscribe_msg.second,
        [this](std::shared_ptr<const LiteMsgWrapper> lite_msg) {
          this->OnModuleLiteMessage(lite_msg);
        },
        [this](std::shared_ptr<const ShmMessage> shm_msg) {
          this->OnModuleShmMessage(shm_msg);
        });
  }

  server_->SubscribLiteMsgCallback(
      subscribe_server_msgs, [lite_callback = lite_callback](
                                 std::shared_ptr<LiteMsgWrapper> lite_msg) {
        lite_callback(lite_msg);
      });

  server_->SubscribShmMsgCallback(
      [shm_callback = shm_callback](std::shared_ptr<ShmMessage> shm_message) {
        shm_callback(shm_message);
      });
}

void Node::OnSetUpTimers() {
  lite_publisher_future_ = std::async(std::launch::async, [&] {
    const auto duration = absl::Seconds(1);
    while (!stop_notification_.HasBeenNotified()) {
      std::shared_ptr<const LiteMsgWrapper> lite_msg;
      {
        absl::MutexLock lock(&msg_mutex_);
        if (!msg_mutex_.AwaitWithTimeout(
                absl::Condition(IsNotEmpty, &lite_msg_queue_), duration)) {
          continue;
        }

        lite_msg = lite_msg_queue_.front();
        lite_msg_queue_.pop();
        QCHECK_NOTNULL(lite_msg);
      }

      ForwardLiteMessage(*lite_msg);
    }
  });

  shm_publisher_future_ = std::async(std::launch::async, [&] {
    const auto duration = absl::Seconds(1);
    while (!stop_notification_.HasBeenNotified()) {
      std::shared_ptr<const ShmMessage> shm_message;
      {
        absl::MutexLock lock(&msg_mutex_);
        if (!msg_mutex_.AwaitWithTimeout(
                absl::Condition(IsNotEmpty, &shm_message_queue_), duration)) {
          continue;
        }

        shm_message = shm_message_queue_.front();
        shm_message_queue_.pop();
        QCHECK_NOTNULL(shm_message);
        shm_message_count_[shm_message->shm_msg_metadata().shm_msg_type()]--;
      }

      ForwardShmMessage(*shm_message);
    }
  });

  fault_publisher_future_ = std::async(std::launch::async, [&] {
    bool has_fault_to_publish = query_remote_asked_and_failed_flag_;
    if (!has_fault_to_publish) {
      return;
    }
    auto &&module_configs = LoadModuleConfigsFromCurrentLaunchFile();
    bool has_autonomy_state_module =
        std::any_of(module_configs.begin(), module_configs.end(),
                    [](const LiteModuleConfig &cfg) -> bool {
                      return (cfg.module_name() == AUTONOMY_STATE_MODULE);
                    });
    constexpr int kTagExecutionIssueProto = 27;
    while (!stop_notification_.HasBeenNotified()) {
      if (query_remote_asked_and_failed_flag_) {
        for (const auto &[node_name, query_ok] :
             client_node_name_to_query_status_) {
          if (query_ok) {
            continue;
          }
          auto severity = QIssueSeverity::QIS_ERROR;
          auto type = QIssueType::QIT_EXCEPTION;
          auto sub_type = QIssueSubType::QIST_NODE_INPUT_QUERY_FAILED;
          auto message = absl::StrCat(
              "Query node inputs failed. Host: ", GetFullNodeName(node_config_),
              " -> Client: ", node_name);
          QISSUEX(severity, type, sub_type, message);
          if (!has_autonomy_state_module) {
            QLOG(ERROR) << "severity: " << QIssueSeverity_Name(severity)
                        << " type: " << QIssueType_Name(type)
                        << " sub_type: " << QIssueSubType_Name(sub_type)
                        << " message: " << message;
          }
          auto lite_msg = std::make_shared<LiteMsgWrapper>();
          lite_msg->set_tag_number(kTagExecutionIssueProto);
          auto issue =
              lite_msg->mutable_execution_issue_proto()->mutable_issue();
          issue->set_severity(severity);
          issue->set_type(type);
          issue->set_sub_type(sub_type);
          issue->set_message(message);
          auto header =
              lite_msg->mutable_execution_issue_proto()->mutable_header();
          header->set_timestamp(absl::ToUnixMicros(Clock::Now()));
          header->set_tag_number(kTagExecutionIssueProto);
          header->set_module_id(NODE_MODULE);
          {
            absl::MutexLock lock(&msg_mutex_);
            lite_msg_queue_.push(std::move(lite_msg));
          }
        }  // for
      }    // if
      absl::SleepFor(absl::Seconds(1));
    }  // while
  });
}

void Node::OnModuleShmMessage(std::shared_ptr<const ShmMessage> shm_msg) {
  const ShmMessageMetadata &shm_msg_metadata = shm_msg->shm_msg_metadata();
  latest_message_ts_ = shm_msg_metadata.header().timestamp();
  int queue_size = 0;
  {
    absl::MutexLock lock(&msg_mutex_);
    shm_message_queue_.push(std::move(shm_msg));
    shm_message_count_[shm_msg_metadata.shm_msg_type()]++;
    queue_size = shm_message_queue_.size();
  }

  // TODO(liyu) consider statistics from different node
  QCOUNTER("shm_message_queue_size", queue_size);
  if (FLAGS_enable_node_verbose_log) {
    QLOG_EVERY_N_SEC(INFO, 3.0) << "shm_msg_size: " << queue_size;
  }
}

void Node::OnModuleLiteMessage(std::shared_ptr<const LiteMsgWrapper> lite_msg) {
  latest_message_ts_ =
      LiteMsgConverter::Get().GetLiteMsgWrapperHeader(*lite_msg).timestamp();
  int queue_size = 0;
  {
    absl::MutexLock lock(&msg_mutex_);
    lite_msg_queue_.push(std::move(lite_msg));
    queue_size = lite_msg_queue_.size();
  }

  // TODO(liyu) consider statistics from different node
  QCOUNTER("lite_msg_queue_size", queue_size);
  if (FLAGS_enable_node_verbose_log) {
    QLOG_EVERY_N_SEC(INFO, 3.0) << " lite_msg_size: " << queue_size;
  }
}

void Node::ForwardLiteMessage(const LiteMsgWrapper &lite_msg) {
  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(lite_msg);
  const auto &clients = publish_clients_[channel_domain];
  for (const auto &client : clients) {
    if (!client->ServerIsReady()) {
      continue;
    }

    SCOPED_QTRACE_ARG1("Node::ForwardLiteMessage", "server_name",
                       client->ServerName());
    if (!client->PublishLiteMsg(lite_msg)) {
      QCOUNTER("lite_message_drop_count", 1);
    }
  }
}

void Node::ForwardShmMessage(const ShmMessage &shm_message) {
  const auto &shm_msg_metadata = shm_message.shm_msg_metadata();
  const auto &lite_header = shm_msg_metadata.header();
  const auto message_ts = lite_header.timestamp();
  // Drop stale messages to avoid lagging.
  if (latest_message_ts_ > message_ts + kMessageStalenessThresholdUs) {
    QCOUNTER("shm_message_timeout_count", 1);
    return;
  }

  int msgs_in_queue = 0;
  const auto shm_msg_type = shm_msg_metadata.shm_msg_type();
  {
    absl::ReaderMutexLock lock(&msg_mutex_);
    msgs_in_queue = shm_message_count_[shm_msg_type];
  }

  const auto &clients = publish_clients_[std::make_pair(lite_header.channel(),
                                                        lite_header.domain())];
  for (const auto &client : clients) {
    if (!client->ServerIsReady()) {
      continue;
    }

    SCOPED_QTRACE_ARG3("Node::ForwardShmMessage", "server_name",
                       client->ServerName(), "shm_msg_type", shm_msg_type,
                       "msgs_in_queue", msgs_in_queue);
    if (!client->PublishShmMsg(shm_message)) {
      QCOUNTER("shm_message_drop_count", 1);
    }
  }
}

std::vector<std::unique_ptr<Node>> Node::CreateNodes(
    const std::string &node_name, const std::string &name_space,
    const ParamManager &param_manager) {
  QCHECK(!node_name.empty()) << "Node name can't be empty.";
  QCHECK(!name_space.empty()) << "Name space can't be empty.";
  std::vector<std::unique_ptr<Node>> nodes;
  const auto nodes_run_config =
      LoadNodesConfigFromCurrentNodeFile(param_manager);
  for (const auto &node_config : nodes_run_config.nodes()) {
    if (IsThisNode(node_config, node_name, name_space)) {
      nodes.push_back(
          std::unique_ptr<Node>(new Node(node_config, param_manager)));
    }
  }

  QCHECK(!nodes.empty()) << "Failed to create node, Name: "
                         << GetFullNodeName(node_name, name_space);
  return nodes;
}

Node::CollectionChannelDomainToFieldName Node::GetHostNodeInputsCollection() {
  if (!host_node_inputs_for_query_.empty()) {
    return host_node_inputs_for_query_;
  }
  // TODO(sweif): if not enable_node_input_config, still get host node
  // module_configs by params for matching original behavior.
  // future we can always get actual module_configs by
  // LoadModuleConfigsFromCurrentLaunchFile.
  const auto &module_configs =
      FLAGS_enable_node_input_config
          ? LoadModuleConfigsFromCurrentLaunchFile()
          : LoadModuleConfigsFromLaunchFile(
                node_config_.lite_launch_config_filename());
  return (FLAGS_enable_node_input_config
              ? GetNodeInputsFromModuleConfigs(module_configs)
              : GetInputsFromModuleConfigs(module_configs));
}

Node::CollectionChannelDomainToFieldName Node::GetHostNodeOutputsCollection() {
  // TODO(sweif): if not enable_node_input_config, still get host node
  // module_configs by params for matching original behavior.
  // future we can always get actual module_configs by
  // LoadModuleConfigsFromCurrentLaunchFile.
  const auto &module_configs =
      FLAGS_enable_node_input_config
          ? LoadModuleConfigsFromCurrentLaunchFile()
          : LoadModuleConfigsFromLaunchFile(
                node_config_.lite_launch_config_filename());
  return GetOutputsFromModuleConfigs(module_configs);
}

Node::CollectionChannelDomainToFieldName Node::GetClientNodeInputsCollection(
    const NodeConfig &client_node_config) {
  const auto &client_node_name = GetFullNodeName(client_node_config);
  if (FLAGS_enable_query_remote_node &&
      ContainsKey(client_node_name_to_query_status_, client_node_name)) {
    if (query_remote_asked_and_failed_flag_ &&
        client_node_name_to_query_status_[client_node_name] == false) {
      return GetClientNodeInputsQueryFailedFallbackCollection(
          client_node_config);
    } else {
      // return per prior query results
      return client_nodes_inputs_by_query_[client_node_name];
    }
  } else {
    // compose client inputs by static configs
    const auto &module_configs = LoadModuleConfigsFromLaunchFile(
        client_node_config.lite_launch_config_filename());
    return (FLAGS_enable_node_input_config
                ? GetNodeInputsFromModuleConfigs(module_configs)
                : GetInputsFromModuleConfigs(module_configs));
  }
}

Node::CollectionChannelDomainToFieldName
Node::GetClientNodeInputsQueryFailedFallbackCollection(
    const NodeConfig &client_node_config) {
  const CollectionChannelDomainToFieldName kClientOmcChannels = {
      {std::make_pair("node_state_proto", ""), "node_state_proto"},
      {std::make_pair("execution_issue_proto", ""), "execution_issue_proto"},
      {std::make_pair("log_proto", ""), "log_proto"}};
  const CollectionChannelDomainToFieldName kClientObcChannels = {
      {std::make_pair("node_state_proto", ""), "node_state_proto"},
      {std::make_pair("autonomy_state_proto", ""), "autonomy_state_proto"}};
  const CollectionChannelDomainToFieldName kClientOtherChannels = {};

  auto is_client_of = [&client_node_config](absl::string_view ns) -> bool {
    return absl::StrContains(
        absl::AsciiStrToLower(client_node_config.name_space()), ns);
  };
  const auto &fallback_channels =
      is_client_of("omc") ? kClientOmcChannels
                          : (is_client_of("xavier") ? kClientObcChannels
                                                    : kClientOtherChannels);
  QLOG(WARNING) << "Assumed client inputs as query failed fallback. "
                << GetFullNodeName(client_node_config)
                << " fallback_channels: " << ToFormattedStr(fallback_channels);
  return fallback_channels;
}

void Node::QueryRemoteNodesInputsAndWaitForReturns() {
  if (!FLAGS_enable_query_remote_node) {
    query_remote_asked_and_failed_flag_ = false;
    return;
  }
  const absl::Time deadline =
      Clock::Now() + absl::Seconds(FLAGS_query_remote_node_timeout_s);
  auto all_ok = [this]() -> bool {
    using ElemTy = std::pair<std::string, bool>;
    return std::all_of(client_node_name_to_query_status_.begin(),
                       client_node_name_to_query_status_.end(),
                       [](const ElemTy &elem) -> bool { return elem.second; });
  };

  do {
    for (const auto &[node_name, client_ptr] : clients_) {
      if (!ContainsKey(client_node_name_to_query_status_, node_name)) {
        client_node_name_to_query_status_[node_name] = false;
        client_nodes_inputs_by_query_[node_name] = {};
      }
      if (!client_node_name_to_query_status_[node_name]) {
        if (QueryRemoteNodeInputs(node_name)) {
          client_node_name_to_query_status_[node_name] = true;
        }
      }
    }
    if (all_ok()) {
      QLOG(INFO) << "Succeed to query all remote nodes.";
      query_remote_asked_and_failed_flag_ = false;
      return;
    }
  } while (Clock::Now() < deadline);

  QLOG(ERROR) << "Failed to complete query all remote nodes in "
              << absl::Seconds(FLAGS_query_remote_node_timeout_s);
  query_remote_asked_and_failed_flag_ = true;
}

bool Node::QueryRemoteNodeInputs(const std::string &node_name) {
  auto client = clients_[node_name];
  QCHECK_NOTNULL(client);
  if (FLAGS_enable_node_verbose_log) {
    QLOG_EVERY_N_SEC(INFO, 1.0)
        << "Attemp to query node inputs to " << node_name;
  }
  NodeQeuryRequest request;
  NodeQeuryResponse response;
  request.full_node_name = GetFullNodeName(node_config_);
  request.query_cmd = NodeQueryCmdType::QUERY_NODE_INPUTS;
  bool status = client->SendQueryCmd(request, &response);
  if (status) {
    QCHECK(ContainsKey(client_nodes_inputs_by_query_, node_name));
    auto &client_inputs = client_nodes_inputs_by_query_[node_name];
    for (const auto &node_input : response.node_inputs) {
      const auto &filed_name = node_input.field_name;
      const auto &channel = node_input.channel;
      const auto &domain = node_input.domain;
      client_inputs[std::make_pair(channel, domain)] = filed_name;
    }
    if (FLAGS_enable_node_verbose_log) {
      QLOG(INFO) << "Succeed query node inputs to " << node_name
                 << ToFormattedStr(client_inputs);
    }
    return true;
  }
  return false;
}

bool Node::ProcessQueryCmd(const NodeQeuryRequest &request,
                           NodeQeuryResponse *response) {
  if (FLAGS_enable_node_verbose_log) {
    QLOG(INFO) << "Node Server: " << GetFullNodeName()
               << " received request from client " << request.full_node_name;
  }
  if (request.query_cmd == NodeQueryCmdType::QUERY_NODE_INPUTS) {
    response->node_inputs.clear();
    for (const auto &[channel_domain, field_name] :
         GetHostNodeInputsCollection()) {
      NodeInputMsgDecl msg_decl;
      msg_decl.field_name = field_name;
      msg_decl.channel = GetChannel(channel_domain);
      msg_decl.domain = GetDomain(channel_domain);
      response->node_inputs.push_back(msg_decl);
    }
    return true;
  }
  return false;
}

}  // namespace qcraft
