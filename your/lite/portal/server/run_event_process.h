#ifndef ONBOARD_LITE_PORTAL_RUN_EVENT_FUNCS_H_
#define ONBOARD_LITE_PORTAL_RUN_EVENT_FUNCS_H_

#include <functional>
#include <unordered_map>

#include "onboard/lite/portal/proto/portal_service.pb.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
namespace lite {

using LiteMsgProcFunc = std::function<void(LiteMsgWrapper *lite_msg)>;

class RunEventProcess {
 public:
  RunEventProcess();

  void OnProcess(const RunEventRequest &request,
                 const LiteMsgProcFunc func = nullptr);

 private:
  void RegisterProcessFunc();

 private:
  // process  define
  using RunEventProcFunc =
      std::function<bool(const uint64_t &timestamp, const QRunEvent &event,
                         LiteMsgWrapper *lite_msg)>;
  using RunEventProcFuncMap =
      std::unordered_map<QRunEvent::Key, RunEventProcFunc>;
  RunEventProcFuncMap proc_funcs_;
};

}  // namespace lite
}  // namespace qcraft

#endif
