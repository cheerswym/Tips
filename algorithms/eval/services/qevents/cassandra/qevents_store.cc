#include "offboard/eval/services/qevents/cassandra/qevents_store.h"

#include "offboard/eval/proto/qevents_service.pb.h"

namespace qcraft {
namespace eval {
qcraft::ProtoStore *NewQEventsStoreFromCassTableOptions(
    const qcraft::cassandra::CassTableOptions &options) {
  return qcraft::cassandra::CassTableStore::New<qcraft::QEventProto>(
      options,
      {qcraft::cassandra::BasicStringMapper("name", "NAME"),
       qcraft::cassandra::BasicStringMapper("source_id", "SOURCE_ID"),
       qcraft::cassandra::BasicInt64Mapper("timestamp", "TIMESTAMP"),
       qcraft::cassandra::BasicStringMapper("uid", "UID"),
       qcraft::cassandra::BasicProtoMapper("", "PROTO")},
      {"SOURCE_ID", "NAME", "TIMESTAMP", "UID"}, "PROTO");
}

}  // namespace eval
}  // namespace qcraft
