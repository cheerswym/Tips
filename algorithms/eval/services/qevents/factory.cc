#include "offboard/eval/services/qevents/factory.h"

#include "offboard/eval/services/qevents/cassandra/qevents_store.h"
#include "offboard/eval/services/qevents/cassandra/stats_store.h"

namespace qcraft {
namespace eval {
QEventsServiceImpl QEventsServerWithCassTableOptions(
    const qcraft::cassandra::CassTableOptions &qevent_options,
    const qcraft::cassandra::CassTableOptions &qevent_lite_options,
    const qcraft::cassandra::CassTableOptions &stats_options) {
  return QEventsServiceImpl(
      NewQEventsStoreFromCassTableOptions(qevent_options),
      NewQEventsStoreFromCassTableOptions(qevent_lite_options),
      NewStatsStoreFromCassTableOptions(stats_options));
}
}  // namespace eval
}  // namespace qcraft
