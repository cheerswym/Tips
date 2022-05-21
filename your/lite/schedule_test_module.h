#ifndef ONBOARD_LITE_SCHEDULE_TEST_MODULE_H_
#define ONBOARD_LITE_SCHEDULE_TEST_MODULE_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "onboard/global/clock.h"
#include "onboard/lite/lite_module.h"

namespace qcraft {

class ScheduleTestModule : public LiteModule {
 public:
  explicit ScheduleTestModule(LiteClientBase *lite_client)
      : LiteModule(lite_client) {}
  void OnInit() override {}
  void OnSubscribeChannels() override {}
  void OnSetUpTimers() override {
    AddTimerOrDie("do_something", &ScheduleTestModule::DoSomething, this,
                  absl::Milliseconds(10), absl::Milliseconds(10), false);
  }

 private:
  void DoSomething() {}
  std::map<std::string, int> issue_count_;
};

REGISTER_LITE_MODULE(ScheduleTestModule);

class IssueCalculate : public LiteModule {
 public:
  explicit IssueCalculate(LiteClientBase *lite_client)
      : LiteModule(lite_client) {}

  void OnInit() override {}

  void OnSubscribeChannels() override {
    Subscribe(&IssueCalculate::HandleExecutionIssue, this);
  }

  void OnSetUpTimers() override {}

  ~IssueCalculate() {
    if (issue_count_.empty()) {
      QLOG(ERROR) << "No rate checker issues";
    }
    for (const auto &iter : issue_count_) {
      QLOG(ERROR) << "Received " << iter.first << ", " << iter.second;
    }
  }

 private:
  void HandleExecutionIssue(
      std::shared_ptr<const ExecutionIssueProto> execution_issue) {
    if (execution_issue->has_issue()) {
      issue_count_[execution_issue->issue().message()]++;
    }
    QLOG_EVERY_N(ERROR, 1) << execution_issue->DebugString();
  }

  std::map<std::string, int> issue_count_;
};

REGISTER_LITE_MODULE(IssueCalculate);

}  // namespace qcraft

#endif  // ONBOARD_LITE_SCHEDULE_TEST_MODULE_H_
