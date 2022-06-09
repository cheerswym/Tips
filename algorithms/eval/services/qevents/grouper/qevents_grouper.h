#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_H_

#include <memory>
#include <string>

#include "offboard/eval/services/qevents/grouper/qevents_grouper_base.h"
#include "offboard/eval/services/qevents/grouper/timestamp_grouper.h"

namespace qcraft {
namespace eval {
std::unique_ptr<QEventsGrouperBase> GetQEventGrouper(
    const std::string& qevent_grouper_or_alias, const GrouperContext& context);
}
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_H_
