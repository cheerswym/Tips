#include "onboard/lite/runner/mock_runner.h"

#include <folly/Executor.h>
#include <folly/executors/CPUThreadPoolExecutor.h>
#include <signal.h>
#include <stdint.h>
#include <sys/prctl.h>

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "common/run/release.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/base/base_flags.h"
#include "onboard/eval/collectors/qevent_collector.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/car_common.h"
#include "onboard/global/ftrace.h"
#include "onboard/global/logging.h"
#include "onboard/global/registry.h"
#include "onboard/lite/check.h"
#include "onboard/lite/lite2_flags.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_transport.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/mock_modules.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/runner/runner_flags.h"
#include "onboard/lite/sidecar/ondemand_sidecar.h"
#include "onboard/lite/sidecar/periodic_sidecar.h"
#include "onboard/lite/transport/shm/shm_manager.h"
#include "onboard/logging/proto/lite_run.pb.h"
#include "onboard/maps/map_selector.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace lite {
bool MockRunner::Init() {
  if (OnTestBenchForRsim()) {
    LOG(INFO) << "FLAGS_rsim_clock_offset: " << FLAGS_rsim_clock_offset;
    clock_ = std::make_shared<RsimClock>(
        absl::Microseconds(FLAGS_rsim_clock_offset));
  } else if (IsOnboardMode()) {
    if (FLAGS_chrono_clock == 1) {
      LOG(ERROR) << "Use chrono system clock.";
      clock_ = std::make_shared<ChronoSystemClock>();
    } else if (FLAGS_chrono_clock == 2) {
      LOG(ERROR) << "Use chrono steady clock.";
      clock_ = std::make_shared<ChronoSteadyClock>();
    }
  }
  if (clock_ != nullptr) {
    SetGlobalClock(clock_);
  }

  qevent_collector_ = std::make_unique<QEventLogCollector>();
  if (FLAGS_enable_qevent_collection) {
    GlobalQEventCollector::SetCollector(qevent_collector_.get());
  }

  auto param_manager =
      CreateParamManager(FLAGS_run_conf, FLAGS_car_name, FLAGS_lite_run_dir);
  QCHECK(param_manager != nullptr);

  // Update the home coordinate based on the current locale.
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  if (run_params.locale_params().has_locale()) {
    locale_util::SetLocale(run_params.locale_params().locale());
  } else {
    locale_util::SetLocale(GetMapLocale(GetMap()));
  }
  return true;
}

int MockRunner::Run() {
  // init modules
  std::vector<qcraft::LiteModuleConfig> module_configs =
      LoadModuleConfigsFromCurrentLaunchFile();
  const int size = module_configs.size();

  int target = -1;
  for (int i = 0; i < size; ++i) {
    if (LiteModuleName_Name(module_configs[i].module_name()) ==
        FLAGS_lite2_module_name) {
      target = i;
      break;
    }

    if (FLAGS_lite_pressure_test) {
      if ((FLAGS_lite2_module_name.find("MOCK_LITE_PRESSURE_TEST_MODULE") !=
           std::string::npos) &&
          (FLAGS_lite2_module_name.find(LiteModuleName_Name(
               module_configs[i].module_name())) != std::string::npos)) {
        target = i;
        break;
      }
    }
  }
  if (target == -1) {
    LOG(ERROR) << "not found " << FLAGS_lite2_module_name << " in the config";
    return 1;
  }

  LOG(INFO) << "Starting :" << FLAGS_lite2_module_name;

  qcraft::shm::ShmManager::Instance()->AttachShm();
  auto module_config = module_configs[target];

  // change process name to a Q-<ModuleName>
  prctl(
      PR_SET_NAME,
      (int64_t)(absl::StrCat("Q-", module_config.module_class_name()).c_str()),
      0, 0, 0);

  int module_id = module_config.module_name();
  if (FLAGS_lite_pressure_test) {
    if (FLAGS_lite2_module_name.find("MOCK_LITE_PRESSURE_TEST_MODULE") !=
        std::string::npos) {
      module_id = std::stoi(FLAGS_lite2_module_name.substr(
          FLAGS_lite2_module_name.find_last_of("_") + 1));
      LOG(INFO) << "module_id:" << module_id;
    }
  }

  executor_ = std::make_unique<folly::CPUThreadPoolExecutor>(4);
  hub_ = std::make_unique<ShmMessageHub>(module_config.module_name());
  lite_client_ = std::make_unique<LiteClient>(
      static_cast<LiteModuleName>(module_id),
      std::make_unique<LiteTransport>(hub_.get()), executor_.get());
  QIssueTrans::Instance()->CreateClient(hub_.get(), executor_.get());
  lite_module_.reset(Registry<LiteModule, LiteClientBase *>::CreateOrDie(
      module_config.module_class_name(), lite_client_.get()));
  // On Lite 2.0, we run perodic sidecar on each module (Counter, Qlog)
  lite_module_->AddSideCar(SideCar::Type::PERODIC,
                           std::make_unique<PerodicSideCar>());
  lite_module_->AddSideCar(SideCar::Type::ONDEMAND,
                           std::make_unique<OnDemandSideCar>());
  lite_module_->Init();
  hub_->StartListening();
  lite_module_->Start(absl::Milliseconds(20));
  lite_module_->Join();
  return 0;
}

int MockRunner::Release() {
  auto ret = lite_module_->GetRetValue();
  lite_module_.reset();
  lite_client_.reset();
  hub_.reset();
  executor_.reset();
  clock_.reset();
  return ret;
}

void MockRunner::SignalHandler(int signum) {
  if (lite_module_ != nullptr) {
    if (SigNumToString(signum) == "SIGINT") {
      QLOG(WARNING) << "SIGINT will be ignored.";
    }
  }
}
}  // namespace lite
}  // namespace qcraft
