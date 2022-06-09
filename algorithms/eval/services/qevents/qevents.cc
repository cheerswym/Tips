#include "offboard/eval/services/qevents/qevents.h"

#include <memory>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "offboard/dashboard/services/utils/filter.h"
#include "offboard/dashboard/services/utils/status.h"
#include "offboard/eval/services/qevents/grouper/qevents_grouper.h"

namespace qcraft {
namespace eval {

grpc::Status QEventsServiceImpl::InsertQEvents(
    grpc::ServerContext *context, const InsertQEventsRequest *request,
    InsertQEventsResponse *response) {
  LOG(INFO) << "Inserting events " << request->qevents_size();
  for (auto qevent : request->qevents()) {
    auto status = qevent_lite_store_->insert(&qevent);
    if (!status.ok()) {
      LOG(ERROR) << "Insert error: " << status.code() << " "
                 << status.message();
    }
  }
  return grpc::Status::OK;
}

grpc::Status QEventsServiceImpl::ListEvents(grpc::ServerContext *context,
                                            const ListEventsRequest *request,
                                            ListEventsResponse *response) {
  LOG(INFO) << "list events " << request->DebugString();
  std::vector<std::shared_ptr<google::protobuf::Message>> results;
  std::vector<ProtoStore::Condition> conditions;

  if (request->has_filter()) {
    STATUS_OK_OR_GRPC_ERROR(dashboard::filter::ParseToConditions<QEventProto>(
                                request->filter(), &conditions),
                            grpc::StatusCode::INVALID_ARGUMENT);
  } else {
    auto proto = std::make_shared<QEventProto>();
    proto->set_source_id(request->source_id());
    conditions.push_back(
        std::make_tuple<>("source_id", ProtoStore::Predicate::EQ, proto));
  }

  STATUS_OK_OR_GRPC_ERROR(FindInAllStore(conditions, &results),
                          grpc::StatusCode::INTERNAL);

  for (auto proto : results) {
    QEventProto *qevent = static_cast<QEventProto *>(proto.get());
    response->add_events()->CopyFrom(*qevent);
  }
  LOG(INFO) << results.size() << "list events";
  return grpc::Status::OK;
}

grpc::Status QEventsServiceImpl::DiffEvents(grpc::ServerContext *context,
                                            const DiffEventsRequest *request,
                                            DiffEventsResponse *response) {
  LOG(INFO) << "diff events " << request->DebugString();
  std::unordered_map<std::string, std::vector<std::vector<QEventProto>>>
      qevents;

  for (const auto &filter : request->filters()) {
    std::vector<ProtoStore::Condition> conditions;
    std::vector<std::shared_ptr<google::protobuf::Message>> results;
    STATUS_OK_OR_GRPC_ERROR(
        dashboard::filter::ParseToConditions<QEventProto>(filter, &conditions),
        grpc::StatusCode::INVALID_ARGUMENT);
    STATUS_OK_OR_GRPC_ERROR(FindInAllStore(conditions, &results),
                            grpc::StatusCode::INTERNAL);
    std::unordered_map<std::string, std::vector<QEventProto>> current_selection;
    for (auto proto : results) {
      QEventProto *qevent = static_cast<QEventProto *>(proto.get());
      current_selection[qevent->name()].push_back(*qevent);
    }

    for (auto &entry : current_selection) {
      qevents[entry.first].push_back(std::move(entry.second));
    }
  }

  const auto &grouper = request->grouper();
  for (const auto &entry : qevents) {
    auto result =
        GetQEventGrouper(grouper.name(), {grouper})->Group(entry.second);
    for (auto &event_group : result) {
      event_group.set_event_name(entry.first);
      *response->mutable_events_groups()->Add() = event_group;
    }
  }

  return grpc::Status::OK;
}

bool CheckRequestValid(const InsertStatsRequest *request) {
  if (!request->has_source_id()) return false;
  return true;
}

grpc::Status QEventsServiceImpl::InsertStats(grpc::ServerContext *context,
                                             const InsertStatsRequest *request,
                                             InsertStatsResponse *response) {
  LOG(INFO) << "Insert stats " << request->DebugString();
  TRUE_OR_GRPC_ERROR(CheckRequestValid(request),
                     grpc::StatusCode::INVALID_ARGUMENT);
  QEventStats stats = request->stats();
  stats.set_source_id(request->source_id());
  STATUS_OK_OR_GRPC_ERROR(stats_store_->insert(&stats),
                          grpc::StatusCode::INTERNAL);
  return grpc::Status::OK;
}

bool CheckRequestValid(const GetStatsRequest *request) {
  if (!request->has_source_id()) return false;
  return true;
}

grpc::Status QEventsServiceImpl::GetStats(grpc::ServerContext *context,
                                          const GetStatsRequest *request,
                                          QEventStats *response) {
  LOG(INFO) << "Get stats " << request->source_id();
  TRUE_OR_GRPC_ERROR(CheckRequestValid(request),
                     grpc::StatusCode::INVALID_ARGUMENT);
  QEventStats stats;
  stats.set_source_id(request->source_id());
  STATUS_OK_OR_GRPC_ERROR(stats_store_->read(&stats),
                          grpc::StatusCode::INTERNAL);
  response->CopyFrom(stats);
  return grpc::Status::OK;
}

absl::Status QEventsServiceImpl::FindInAllStore(
    const std::vector<ProtoStore::Condition> &conditions,
    std::vector<std::shared_ptr<google::protobuf::Message>> *results) {
  auto status = qevent_lite_store_->find(conditions, results);
  if (results->empty()) {
    status = qevent_store_->find(conditions, results);
  }
  return status;
}

}  // namespace eval
}  // namespace qcraft
