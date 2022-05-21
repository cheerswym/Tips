#ifndef ONBOARD_LITE_QISSUE_TRANS_H_
#define ONBOARD_LITE_QISSUE_TRANS_H_

#include <memory>
#include <string>

#include "offboard/simulation/lite/sim_lite_client.h"
#include "onboard/autonomy_state/handlers/q_issue_util.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/logging.h"

namespace qcraft {

// qissue global api
#define QISSUEX(severity, type, sub_type, msg)              \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, 0.1) \
  QIssueTrans::Instance()->Publish(severity, type, sub_type, msg);

#define QISSUEX_WITH_ARGS(severity, type, sub_type, msg, args_msg) \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, 0.1)        \
  QIssueTrans::Instance()->Publish(severity, type, sub_type, msg, args_msg);

class QIssueTrans {
 public:
  void CreateClient(MessageHub* hub, folly::Executor* executor = nullptr,
                    const std::string& domain = "");

  void CreateClient(const SimClientOptions& options,
                    SimMessageQueue* sim_msg_queue,
                    const std::string& domain = "");

  void CreateClient(LiteClientBase* lite_client);

  void Publish(QIssueSeverity severity, QIssueType type, QIssueSubType sub_type,
               const std::string& message,
               const std::string& args_message = "");

 private:
  DECLARE_SINGLETON(QIssueTrans)

 private:
  std::unique_ptr<LiteClientBase> client_ = nullptr;
};
}  // namespace qcraft

#endif
