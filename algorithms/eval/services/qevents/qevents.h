#ifndef OFFBOARD_EVAL_SERVICES_QEVENTS_QEVENTS_H_
#define OFFBOARD_EVAL_SERVICES_QEVENTS_QEVENTS_H_

#include <grpc++/grpc++.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "offboard/db/proto_store.h"
#include "offboard/eval/proto/qevents_service.grpc.pb.h"
#include "offboard/eval/proto/qevents_service.pb.h"

namespace qcraft {
namespace eval {

class QEventsServiceImpl final : public QEventsService::Service {
 public:
  QEventsServiceImpl(qcraft::ProtoStore *qevent_store,
                     qcraft::ProtoStore *qevent_lite_store,
                     qcraft::ProtoStore *stats_store)
      : qevent_store_(qevent_store),
        qevent_lite_store_(qevent_lite_store),
        stats_store_(stats_store) {}

  grpc::Status InsertQEvents(grpc::ServerContext *context,
                             const InsertQEventsRequest *request,
                             InsertQEventsResponse *response) override;

  grpc::Status ListEvents(grpc::ServerContext *context,
                          const ListEventsRequest *request,
                          ListEventsResponse *response) override;

  grpc::Status DiffEvents(grpc::ServerContext *context,
                          const DiffEventsRequest *request,
                          DiffEventsResponse *response) override;

  grpc::Status InsertStats(grpc::ServerContext *context,
                           const InsertStatsRequest *request,
                           InsertStatsResponse *response) override;

  grpc::Status GetStats(grpc::ServerContext *context,
                        const GetStatsRequest *request,
                        QEventStats *response) override;

 private:
  absl::Status FindInAllStore(
      const std::vector<ProtoStore::Condition> &conditions,
      std::vector<std::shared_ptr<google::protobuf::Message>> *results);

  std::unique_ptr<qcraft::ProtoStore> qevent_store_, qevent_lite_store_,
      stats_store_;
};

}  // namespace eval
}  // namespace qcraft
#endif  // OFFBOARD_EVAL_SERVICES_QEVENTS_QEVENTS_H_
