#include "onboard/lite/sidecar/periodic_sidecar.h"

#include <csignal>

#include "onboard/eval/collectors/qevent_log_collector.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/counter.h"
#include "onboard/global/system_info.h"

namespace qcraft {
void PerodicSideCar::Start(LiteModule *lite_module) {
  // set up qlog publisher
  BufferedLogger::Instance()->SetFatalHandler(
      [lite_module](const qcraft::LogItem &log_item) {
        std::stringstream ss;
        const auto time = absl::FromUnixMillis(log_item.capture_timestamp_ms);
        ss << "QLOG(FATAL) "
           << absl::FormatTime("%E4Y/%m/%d %H:%M:%E3S ", time,
                               absl::LocalTimeZone())
           << log_item.thread_id << " " << log_item.file_name << ":"
           << log_item.file_line << "] " << log_item.msg << std::endl;

        const auto message = absl::StrCat("Check module state: CRASHED(",
                                          lite_module->GetModuleName(), ")");
        QISSUEX_WITH_ARGS(
            QIssueSeverity::QIS_ERROR, QIssueType::QIT_CRASH,
            lite_module->GetQIssueSubType(lite_module->GetModuleName()),
            message, ss.str());

        // publish before exit.
        auto output =
            BufferedLogger::Instance()->DumpLogProto(lite_module->ModuleName());
        if (output.log_item_size() > 0) {
          QLOG_IF_NOT_OK(WARNING, lite_module->Publish(output));
        }

        std::cerr << ss.str();
        raise(SIGQUIT);
      });

  BufferedLogger::Instance()->SetDumpHandler([lite_module]() {
    auto output =
        BufferedLogger::Instance()->DumpLogProto(lite_module->ModuleName());
    if (output.log_item_size() > 0) {
      QLOG_IF_NOT_OK(WARNING, lite_module->Publish(output));
    }
  });

  runner_ = std::make_unique<PeriodicRunner>(absl::Seconds(1));
  runner_->Start([lite_module]() {
    absl::Time start_time = Clock::Now();
    auto log_output =
        BufferedLogger::Instance()->DumpLogProto(lite_module->ModuleName());
    if (log_output.log_item_size() > 0) {
      QLOG_IF_NOT_OK(WARNING, lite_module->Publish(log_output));
    }

    CounterProto counter_output = Counter::Instance()->GetCounterOutput();
    if (counter_output.item_size() > 0) {
      QLOG_IF_NOT_OK(WARNING, lite_module->Publish(counter_output));
    }

    QEventLogCollector *log_collector =
        static_cast<QEventLogCollector *>(GlobalQEventCollector::Instance());
    if (log_collector != nullptr) {
      QEventsProto qevent_output = log_collector->GetQEventsOutput();
      if (qevent_output.items_size() > 0) {
        LOG(INFO) << qevent_output.DebugString();
        QLOG_IF_NOT_OK(WARNING, lite_module->Publish(qevent_output));
      }
    }

    SystemInfoProto proto =
        qcraft::SystemInfo::Instance()->GetModuleSystemInfoProto(
            lite_module->ModuleName());
    QLOG_IF_NOT_OK(WARNING, lite_module->Publish(proto));
    lite_module->CheckSystemInfo(proto);
    absl::Time stop_time = Clock::Now();
    Counter::Instance()->AddCounterEvent(
        "PeriodicSideCar", absl::ToInt64Microseconds(stop_time - start_time));
  });
}

void PerodicSideCar::Stop(LiteModule *lite_module) {
  runner_->Stop();
  LOG(INFO) << "PerodicSideCar Stoped";
}

SideCar::Type PerodicSideCar::GetType() { return SideCar::Type::PERODIC; }

}  // namespace qcraft
