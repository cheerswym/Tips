#ifndef ONBOARD_NODE_NODE_MODULE_H_
#define ONBOARD_NODE_NODE_MODULE_H_

#include <memory>
#include <vector>

#include "onboard/lite/lite_module.h"
#include "onboard/node/node.h"

namespace qcraft {

class NodeModule : public LiteModule {
 public:
  explicit NodeModule(LiteClientBase *lite_client);

  virtual ~NodeModule();

  void OnInit() override;

  void OnSubscribeChannels() override;

  void OnSetUpTimers() override;

 private:
  std::vector<std::unique_ptr<Node>> nodes_;
};

REGISTER_LITE_MODULE(NodeModule);

}  // namespace qcraft

#endif  // ONBOARD_NODE_NODE_MODULE_H_
