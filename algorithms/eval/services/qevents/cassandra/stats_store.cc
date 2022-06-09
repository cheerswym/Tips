#include "offboard/eval/services/qevents/cassandra/stats_store.h"

#include "offboard/eval/proto/qevents_service.pb.h"

namespace qcraft {
namespace eval {
qcraft::ProtoStore *NewStatsStoreFromCassTableOptions(
    const qcraft::cassandra::CassTableOptions &options) {
  return qcraft::cassandra::CassTableStore::New<qcraft::QEventProto>(
      options,
      {qcraft::cassandra::BasicStringMapper("source_id", "SOURCE_ID"),
       qcraft::cassandra::BasicProtoMapper("", "PROTO")},
      {"SOURCE_ID"}, "PROTO");
}

}  // namespace eval
}  // namespace qcraft
