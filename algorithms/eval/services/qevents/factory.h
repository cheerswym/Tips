#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_FACTORY_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_FACTORY_H_

#include "offboard/db/cassandra/cass_table_store.h"
#include "offboard/eval/services/qevents/qevents.h"

namespace qcraft {
namespace eval {
QEventsServiceImpl QEventsServerWithCassTableOptions(
    const qcraft::cassandra::CassTableOptions &qevent_options,
    const qcraft::cassandra::CassTableOptions &qevent_lite_options,
    const qcraft::cassandra::CassTableOptions &stats_options);
}
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_FACTORY_H_
