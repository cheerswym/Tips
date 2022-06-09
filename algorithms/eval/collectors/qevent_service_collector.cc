#include "offboard/eval/collectors/qevent_service_collector.h"

#include <grpc++/grpc++.h>

#include "absl/cleanup/cleanup.h"
#include "absl/time/time.h"
#include "offboard/dashboard/utils/retry.h"
#include "onboard/async/async_util.h"

namespace qcraft {

QEventServiceCollector::QEventServiceCollector(
    const std::string &qevents_service_addr)
    : send_qevents_thread_pool_(std::make_unique<ThreadPool>(1)),
      qevent_protos_(),
      stub_(nullptr) {
  if (!qevents_service_addr.empty()) {
    stub_ = QEventsService::NewStub(grpc::CreateChannel(
        qevents_service_addr, grpc::InsecureChannelCredentials()));
  }
  qevent_protos_.reserve(kQEventBatchSize);
}

void QEventServiceCollector::AddQEventProtoInternal(QEventProto *qevent_proto) {
  absl::MutexLock lock(&mutex_);
  qevent_protos_.push_back(*qevent_proto);
  qevents_stats_[qevent_proto->name()]++;
  if (qevent_protos_.size() >= kQEventBatchSize) {
    std::vector<QEventProto> qevent_protos;
    qevent_protos.swap(qevent_protos_);
    ScheduleFuture(send_qevents_thread_pool_.get(),
                   [this, qevent_protos = std::move(qevent_protos)] {
                     SendQEventsToServer(qevent_protos);
                   });
  }
}

void QEventServiceCollector::FlushQEventsToServer() {
  absl::MutexLock lock(&mutex_);
  std::vector<QEventProto> qevent_protos;
  qevent_protos.swap(qevent_protos_);
  ScheduleFuture(send_qevents_thread_pool_.get(),
                 [this, qevent_protos = std::move(qevent_protos)] {
                   SendQEventsToServer(qevent_protos);
                 });
}

QEventServiceCollector::~QEventServiceCollector() {
  absl::MutexLock lock(&mutex_);
  if (!qevent_protos_.empty()) SendQEventsToServer(qevent_protos_);
  send_qevents_thread_pool_.reset();
  LOG(INFO) << "Uploading qevents took " << update_duration_ * 1e-6
            << " seconds.";
}

void QEventServiceCollector::SendQEventsToServer(
    const std::vector<QEventProto> &qevent_protos) {
  if (stub_ == nullptr) return;
  const auto start = absl::Now();
  const absl::Cleanup clean_up = [this, start] {
    update_duration_.fetch_add(absl::ToInt64Microseconds(absl::Now() - start),
                               std::memory_order_relaxed);
  };
  grpc::ClientContext context;
  InsertQEventsRequest req;
  InsertQEventsResponse resp;
  for (auto &qevent_proto : qevent_protos) {
    auto *new_qevent = req.add_qevents();
    *new_qevent = qevent_proto;
  }
  auto status = dashboard::RetryUntilLimit([this, &req, &resp]() {
    grpc::ClientContext context;
    return stub_->InsertQEvents(&context, req, &resp);
  });
  LOG_IF(ERROR, !status.ok())
      << "Error Inserting QEvents " << status.error_code() << ", "
      << status.error_message();
}

std::unordered_map<std::string, int> QEventServiceCollector::GetStats() {
  absl::ReaderMutexLock lock(&mutex_);
  return qevents_stats_;
}

}  // namespace qcraft
