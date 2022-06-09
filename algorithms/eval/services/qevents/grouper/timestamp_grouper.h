#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_TIMESTAMP_GROUPER_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_TIMESTAMP_GROUPER_H_

#include <vector>

#include "offboard/eval/services/qevents/grouper/qevents_grouper_base.h"
#include "onboard/base/base.h"

namespace qcraft {
namespace eval {
class TimestampGrouper : public QEventsGrouperBase {
 public:
  explicit TimestampGrouper(const GrouperContext &context)
      : QEventsGrouperBase(context),
        interval_(context.grouper.timestamp_grouper().interval()) {}

  std::vector<DiffEventsResponse::EventsGroup> Group(
      const std::vector<std::vector<QEventProto>> &qevents) override;

 private:
  int64 interval_ = 0;
};
REGISTER_QEVENTS_GROUPER(TimestampGrouper, timestamp_grouper);
}  // namespace eval
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_TIMESTAMP_GROUPER_H_
