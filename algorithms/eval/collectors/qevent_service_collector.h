#ifndef OFFBOARD_EVAL_COLLECTORS_QEVENT_SERVICE_COLLECTOR_H_
#define OFFBOARD_EVAL_COLLECTORS_QEVENT_SERVICE_COLLECTOR_H_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "offboard/eval/proto/qevents_service.grpc.pb.h"
#include "onboard/async/thread_pool.h"
#include "onboard/base/integral_types.h"
#include "onboard/eval/collectors/qevent_collector.h"

namespace qcraft {
class QEventServiceCollector : public QEventCollector {
 public:
  explicit QEventServiceCollector(const std::string &qevents_service_addr);
  ~QEventServiceCollector() override;
  void AddQEventProtoInternal(QEventProto *qevent_proto) override
      LOCKS_EXCLUDED(mutex_);
  void FlushQEventsToServer() LOCKS_EXCLUDED(mutex_);

  std::unordered_map<std::string, int> GetStats() LOCKS_EXCLUDED(mutex_);

 private:
  void SendQEventsToServer(const std::vector<QEventProto> &qevent_protos);

  std::unique_ptr<ThreadPool> send_qevents_thread_pool_;

  absl::Mutex mutex_;
  std::vector<QEventProto> qevent_protos_ GUARDED_BY(mutex_);
  std::unique_ptr<QEventsService::Stub> stub_;

  std::unordered_map<std::string, int> qevents_stats_ GUARDED_BY(mutex_);

  std::atomic<uint64> update_duration_ = 0;
};
}  // namespace qcraft
#endif
