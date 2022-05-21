#include "onboard/lite/qissue_trans.h"

#include <mutex>

namespace qcraft {

QIssueTrans::QIssueTrans() {}

void QIssueTrans::CreateClient(MessageHub* hub, folly::Executor* executor,
                               const std::string& domain) {
  static std::once_flag issue_once;
  std::call_once(issue_once, [this, hub, executor, domain] {
    client_ = std::make_unique<LiteClient>(
        QISSUE_TRANS, std::make_unique<LiteTransport>(hub), executor, domain);
  });
}

void QIssueTrans::CreateClient(const SimClientOptions& options,
                               SimMessageQueue* sim_msg_queue,
                               const std::string& domain) {
  static std::once_flag issue_once;
  std::call_once(issue_once, [this, options, sim_msg_queue, domain] {
    client_ = std::make_unique<SimLiteClient>(options, QISSUE_TRANS, domain,
                                              sim_msg_queue);
  });
}

void QIssueTrans::CreateClient(LiteClientBase* lite_client) {
  if (lite_client == nullptr) {
    return;
  }

  if (lite_client->IsSimClient()) {
    auto client = dynamic_cast<SimLiteClient*>(lite_client);
    CreateClient(client->GetOptions(), client->GetQueue());
  } else {
    auto client = dynamic_cast<LiteClient*>(lite_client);
    CreateClient(client->GetMessageHub(), client->GetExecutor());
  }
}

void QIssueTrans::Publish(QIssueSeverity severity, QIssueType type,
                          QIssueSubType sub_type, const std::string& message,
                          const std::string& args_message) {
  if (client_ == nullptr) {
    QLOG(ERROR) << "client null: " << sub_type << "," << message << ","
                << args_message;
    return;
  }

  ExecutionIssueProto execution_issue;
  auto* issue = execution_issue.mutable_issue();
  issue->set_severity(severity);
  issue->set_type(type);
  issue->set_sub_type(sub_type);
  issue->set_message(message);
  if (!args_message.empty()) {
    issue->set_args_message(args_message);
  }

  QLOG_IF_NOT_OK(WARNING, client_->Publish(execution_issue));
}

}  // namespace qcraft
