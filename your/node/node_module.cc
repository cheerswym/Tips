#include "onboard/node/node_module.h"

#include <string>

DECLARE_string(node_name);
DECLARE_string(name_space);

namespace qcraft {

NodeModule::NodeModule(LiteClientBase *lite_client) : LiteModule(lite_client) {}

NodeModule::~NodeModule() { nodes_.clear(); }

void NodeModule::OnInit() {
  nodes_ =
      Node::CreateNodes(FLAGS_node_name, FLAGS_name_space, param_manager());
  for (auto &node : nodes_) {
    node->OnInit();
  }
}

void NodeModule::OnSubscribeChannels() {
  DisableInputOutputChecker();
  for (auto &node : nodes_) {
    node->OnSubscribeChannels(
        [this](const std::string &channel, const std::string &domain,
               const std::string &field_name,
               std::function<void(std::shared_ptr<const LiteMsgWrapper>)>
                   lite_callback,
               std::function<void(std::shared_ptr<const ShmMessage>)>
                   shm_callback) {
          this->SubscribeMsgWithCallback(channel, domain, field_name,
                                         lite_callback, shm_callback);
        },
        [this](std::shared_ptr<LiteMsgWrapper> lite_msg) {
          QLOG_IF_NOT_OK(WARNING, ForwardLiteMsg(lite_msg.get()));
        },
        [this](std::shared_ptr<ShmMessage> shm_message) {
          QLOG_IF_NOT_OK(WARNING, PublishShmMsg(shm_message.get()));
        });
  }
}

void NodeModule::OnSetUpTimers() {
  for (auto &node : nodes_) {
    node->OnSetUpTimers();
  }
}
}  // namespace qcraft
