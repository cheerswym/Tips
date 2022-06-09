#ifndef OFFBOARD_EVAL_COLLECTORS_QEVENT_FILE_COLLECTOR_H_
#define OFFBOARD_EVAL_COLLECTORS_QEVENT_FILE_COLLECTOR_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/eval/collectors/qevent_collector.h"

namespace qcraft {

class QEventFileCollector : public QEventCollector {
 public:
  QEventFileCollector();
  ~QEventFileCollector() override;
  void AddQEventProtoInternal(QEventProto *qevent_proto) override
      LOCKS_EXCLUDED(mutex_);

 private:
  void DumpQEvents(const std::vector<QEventProto> &qevent_protos);

  ThreadPool send_qevents_thread_pool_{1};

  absl::Mutex mutex_;
  std::vector<QEventProto> qevent_protos_ GUARDED_BY(mutex_);
  std::string dir_;
};
}  // namespace qcraft
#endif
