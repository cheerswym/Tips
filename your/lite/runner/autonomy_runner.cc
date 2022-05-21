#include "onboard/lite/runner/autonomy_runner.h"

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
#include "onboard/autonomy/autonomy_module.h"
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
#include "onboard/rsim/common/utils.h"
#include "onboard/rsim/config/config.h"
#include "onboard/rsim/config/proto/config.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace lite {
bool AutonomyRunner::Init() {
  // set clock
  if (OnTestBenchForRsim()) {
    SetRsimFlags();
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
  // init shm
  qcraft::shm::ShmManager::Instance()->DestroyAndCreateShm();

  // init ftrace
  QCHECK(qcraft::FTrace::Instance()->Init());
  return true;
}

int AutonomyRunner::Run() {
  std::vector<qcraft::LiteModuleConfig> module_configs =
      LoadModuleConfigsFromCurrentLaunchFile();
  int target = -1;
  for (int i = 0; i < module_configs.size(); ++i) {
    if (module_configs[i].module_name() == LiteModuleName::AUTONOMY_MODULE) {
      target = i;
      break;
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

  executor_ = std::make_unique<folly::CPUThreadPoolExecutor>(1);
  hub_ = std::make_unique<ShmMessageHub>(module_config.module_name());
  lite_client_ = std::make_unique<LiteClient>(
      module_config.module_name(), std::make_unique<LiteTransport>(hub_.get()),
      executor_.get());
  QIssueTrans::Instance()->CreateClient(hub_.get(), executor_.get());
  lite_module_.reset(Registry<LiteModule, LiteClientBase *>::CreateOrDie(
      module_config.module_class_name(), lite_client_.get()));
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

int AutonomyRunner::Release() {
  auto ret = lite_module_->GetRetValue();
  lite_module_.reset();
  lite_client_.reset();
  hub_.reset();
  executor_.reset();
  clock_.reset();
  return ret;
}

void AutonomyRunner::SignalHandler(int signum) {
  if (lite_module_ != nullptr) {
    if (SigNumToString(signum) == "SIGINT") {
      (static_cast<AutonomyModule *>(lite_module_.get()))->StopAll();
    }
  }
}

void AutonomyRunner::SetRsimFlags() {
  std::unordered_set<std::string> kWhiteList = {
      "route_recover_destinations_from_log", "multi_stops_route",
      "map_commit_sha", "map"};
  auto lite_run_dir = rsim::GetRunLiteRunPathOrDie();
  QLOG(INFO) << "rsim lite_run_dir: " << lite_run_dir;
  LiteRun lite_run;
  QCHECK(file_util::BinaryFileToProto(lite_run_dir, &lite_run));
  for (const auto &[key, value] : lite_run.release_info().commandline_flag()) {
    if (kWhiteList.count(key) == 0) {
      continue;
    }
    gflags::SetCommandLineOption(key.data(), value.data());
    kWhiteList.erase(key);
  }
  gflags::SetCommandLineOption("route_recover_destinations_from_log", "1");
  {
    const auto &config = rsim::RsimConfig::GetConfigProto();
    if (config.map().specified_commit().empty()) {
      std::string local_map = rsim::GetLocalMapPath(config.run());
      QLOG(ERROR) << "Set semantic_map_dir: " << local_map;
      gflags::SetCommandLineOption("semantic_map_dir", local_map.data());
    } else {
      QLOG(ERROR) << "No Set: " << config.map().specified_commit().empty();
    }
  }

  for (const auto &s : kWhiteList) {
    QLOG(ERROR) << "Not find key: " << s;
  }
  {
    const auto &rsim_start_time = rsim::GetRunStartTime();
    QLOG(ERROR) << "START TIME: " << FetchTimeString(rsim_start_time);

    constexpr int64_t kBiasInSeconds = 30;
    int64_t time_bias = FLAGS_rsim_clock_timepoint -
                        absl::ToUnixMicros(rsim_start_time) +
                        kBiasInSeconds * 1000 * 1000;
    QLOG(ERROR) << "time_bias: " << time_bias;
    std::string rsim_clock_offset_key = "rsim_clock_offset";
    std::string rsim_clock_offset_value = std::to_string(time_bias);
    gflags::SetCommandLineOption(rsim_clock_offset_key.data(),
                                 rsim_clock_offset_value.data());
  }
  {
    std::string lite_run_dir_key = "lite_run_dir";
    std::string lite_run_dir_value = lite_run_dir;
    gflags::SetCommandLineOption(lite_run_dir_key.data(),
                                 lite_run_dir_value.data());
  }
}
}  // namespace lite
}  // namespace qcraft
