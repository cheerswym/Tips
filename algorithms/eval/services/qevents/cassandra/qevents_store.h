#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_CASSANDRA_QEVENTS_STORE_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_CASSANDRA_QEVENTS_STORE_H_

#include "offboard/db/cassandra/cass_table_store.h"

namespace qcraft {
namespace eval {
qcraft::ProtoStore *NewQEventsStoreFromCassTableOptions(
    const qcraft::cassandra::CassTableOptions &options);
}
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_CASSANDRA_QEVENTS_STORE_H_
