#ifndef ONBOARD_NODE_NODE_STATE_MODULE_H_
#define ONBOARD_NODE_NODE_STATE_MODULE_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "onboard/lite/lite_module.h"
#include "onboard/node/proto/node_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/execution_issue.pb.h"
#include "onboard/utils/periodic_runner.h"

namespace qcraft {

class NodeStateModule : public LiteModule {
 public:
  explicit NodeStateModule(LiteClientBase *lite_client);

  ~NodeStateModule();

  void OnInit() override;

  void OnSubscribeChannels() override;

  void OnSetUpTimers() override;

 private:
  void HandleNodeState(std::shared_ptr<const NodeStateProto> node_state_proto);

#ifdef __x86_64__
  void HandleSystemInfo(
      std::shared_ptr<const SystemInfoProto> system_info_proto);
#endif

 private:
  std::shared_ptr<const AutonomyStateProto> last_autonomy_state_;

  std::unique_ptr<PeriodicRunner> time_sync_runner_;
  std::unique_ptr<PeriodicRunner> system_info_runner_;

  std::unordered_map<std::string, std::chrono::steady_clock::time_point>
      node_time_points_;
};

REGISTER_LITE_MODULE(NodeStateModule);

}  // namespace qcraft

#endif  // ONBOARD_NODE_NODE_STATE_MODULE_H_
