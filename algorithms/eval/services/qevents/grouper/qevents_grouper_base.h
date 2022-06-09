#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_BASE_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_BASE_H_

#include <vector>

#include "offboard/eval/proto/qevents_service.pb.h"
#include "onboard/global/registry.h"

namespace qcraft {
namespace eval {

struct GrouperContext {
  QEventGrouper grouper;
};

class QEventsGrouperBase {
 public:
  explicit QEventsGrouperBase(const GrouperContext &context)
      : context_(context) {}
  virtual ~QEventsGrouperBase() {}

  virtual std::vector<DiffEventsResponse::EventsGroup> Group(
      const std::vector<std::vector<QEventProto>> &qevents) {
    return {};
  }

 private:
  GrouperContext context_;
};

// Registers an event grouper with the `class_name` and `alias`.
#define REGISTER_QEVENTS_GROUPER(class_name, alias)             \
  REGISTER_SUBCLASS_NAME(QEventsGrouperBase, class_name, alias, \
                         const GrouperContext &);               \
  REGISTER_SUBCLASS(QEventsGrouperBase, class_name, const GrouperContext &)

}  // namespace eval
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_GROUPER_QEVENTS_GROUPER_BASE_H_
