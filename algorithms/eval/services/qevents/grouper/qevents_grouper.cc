#include "offboard/eval/services/qevents/grouper/qevents_grouper.h"

#include "onboard/global/registry.h"

namespace qcraft {
namespace eval {
std::unique_ptr<QEventsGrouperBase> GetQEventGrouper(
    const std::string& qevent_grouper_or_alias, const GrouperContext& context) {
  return std::unique_ptr<QEventsGrouperBase>(
      Registry<QEventsGrouperBase, const GrouperContext&>::CreateOrDie(
          qevent_grouper_or_alias, context));
}
}  // namespace eval
}  // namespace qcraft
