#include "offboard/eval/collectors/qevent_file_collector.h"

#include <fstream>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "onboard/async/async_util.h"
#include "onboard/utils/file_util.h"

DEFINE_string(qevents_dir, "", "folder putting qevents");

namespace qcraft {

QEventFileCollector::QEventFileCollector()
    : qevent_protos_(), dir_(FLAGS_qevents_dir) {
  qevent_protos_.reserve(kQEventBatchSize);
}

void QEventFileCollector::AddQEventProtoInternal(QEventProto *qevent_proto) {
  absl::MutexLock lock(&mutex_);
  qevent_protos_.push_back(*qevent_proto);
  if (qevent_protos_.size() >= kQEventBatchSize) {
    std::vector<QEventProto> qevent_protos;
    qevent_protos.swap(qevent_protos_);
    ScheduleFuture(&send_qevents_thread_pool_,
                   [this, qevent_protos = std::move(qevent_protos)] {
                     DumpQEvents(qevent_protos);
                   });
  }
}

QEventFileCollector::~QEventFileCollector() {
  absl::MutexLock lock(&mutex_);
  if (!qevent_protos_.empty()) DumpQEvents(qevent_protos_);
}

void QEventFileCollector::DumpQEvents(
    const std::vector<QEventProto> &qevent_protos) {
  if (dir_.empty()) return;

  for (const auto &qevent_proto : qevent_protos) {
    CHECK(file_util::ProtoToTextFile(
        qevent_proto,
        absl::StrCat(dir_, "/", boost::filesystem::unique_path().c_str())));
  }
}
}  // namespace qcraft
