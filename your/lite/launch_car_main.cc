// TODO(kun): Add run commands:
// bazel run -c opt onboard/lite/launch_car_main -- \
// --launch_file="launch_run_with_logger_64.pb.txt"
// "launch_run_with_logger_64.pb.txt" is the default value of flag launch_file,
// you can run without specify it explicitly.
//
// More launch files, please check out onboard/lite/launch_config directory.

#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "absl/debugging/stacktrace.h"
#include "absl/strings/str_split.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "folly/Executor.h"
#include "folly/executors/CPUThreadPoolExecutor.h"
#include "folly/executors/IOThreadPoolExecutor.h"
#include "glog/logging.h"
#include "onboard/async/parallel_for.h"
#include "onboard/base/base_flags.h"
#include "onboard/base/signal_handler.h"
#include "onboard/conf/run_conf_man.h"
#include "onboard/eval/collectors/qevent_log_collector.h"
#include "onboard/global/car_common.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/offboard_modules.h"
#include "onboard/lite/onboard_modules.h"
#include "onboard/lite/sidecar/ondemand_sidecar.h"
#include "onboard/lite/sidecar/periodic_sidecar.h"
#include "onboard/lite/transport/message/inner_message_hub.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/params/param_manager.h"

DEFINE_string(
    disable_prefixes, "",
    "A lite of disable modules prefixes, seperated by comma. It will match "
    "launch config file's module name's prefix. Like 'Perception' "
    "can disable all module's class name started with 'Perception'");

DEFINE_string(online_simulation_scenario_path, "",
              "If path is not empty, will run online simulation mode with "
              "configured virtual agents.");
DEFINE_bool(
    seq_init, true,
    "If seq_init is set, the module will initialize in sequential order.");

DEFINE_bool(enable_qevent_collection, true, "Enable onboard qevent collection");
DECLARE_string(lite_run_dir);

namespace {

constexpr char kCrashLogFile[] = "/tmp/qcraft_crash.txt";
absl::Notification exit_notification;

std::vector<std::unique_ptr<qcraft::LiteClientBase>> lite_clients;
std::vector<std::unique_ptr<qcraft::LiteModule>> lite_modules;

// A hook to write failure data indicated by `buf` to the crash log file.
// See Ref: @com_google_absl//absl/debugging/failure_signal_handler.h
void AbslSigWriterFunc(const char *buf) {
  std::ofstream outf(kCrashLogFile, std::ios::out | std::ios::app);
  outf << buf;
  outf.close();
}

void SignalHandler(int sig) {
  LOG(WARNING) << "Received signo=" << sig << "(" << strsignal(sig)
               << "), exiting...";

  if (!exit_notification.HasBeenNotified()) {
    exit_notification.Notify();
  }
  const auto status = qcraft::signal::ResetToDefaultHandler(sig);
  if (!status.ok()) {
    LOG(WARNING) << "ResetToDefaultHandler for " << sig
                 << " failed, msg: " << status.message();
  }

  (void)raise(sig);
}

}  // namespace

namespace qcraft {

void InitModule(const std::vector<LiteModuleConfig> &module_configs,
                const ThreadPool &thread_pool,
                const std::unique_ptr<folly::Executor> &executor,
                std::vector<std::unique_ptr<LiteClientBase>> *lite_clients_ptr,
                std::vector<std::unique_ptr<LiteModule>> *lite_modules_ptr,
                int i) {
  // clang lint only allow const ref or pointer, just create the ref here so we
  // don't modify the reset of the code.
  std::vector<std::unique_ptr<LiteClientBase>> &lite_clients =
      *lite_clients_ptr;
  std::vector<std::unique_ptr<LiteModule>> &lite_modules = *lite_modules_ptr;

  auto &module_config = module_configs[i];
  LOG(INFO) << "Initialize module: " << module_config.DebugString();

  // TODO(mike): enable timer rate checker.
  lite_clients[i] = std::make_unique<LiteClient>(
      module_config.module_name(),
      std::make_unique<LiteTransport>(GlobalInnerMessageHub()), executor.get());

  {
    lite_modules[i].reset(Registry<LiteModule, LiteClientBase *>::CreateOrDie(
        module_config.module_class_name(), lite_clients[i].get()));
    lite_modules[i]->AddSideCar(SideCar::Type::ONDEMAND,
                                std::make_unique<OnDemandSideCar>());
    // Lite 1.0, we do Counter, QLog only in the logging module.
    if (lite_modules[i]->ModuleName() == "LOGGING_MODULE") {
      lite_modules[i]->AddSideCar(SideCar::Type::PERODIC,
                                  std::make_unique<PerodicSideCar>());
    }

    if (!FLAGS_online_simulation_scenario_path.empty() &&
        module_config.module_name() == ONLINE_SIMULATION_MODULE) {
      ScenarioProto scenario;
      QCHECK(file_util::TextFileToProto(FLAGS_online_simulation_scenario_path,
                                        &scenario));
      QCHECK(scenario.execution()
                 .sim_modules()
                 .online_simulation_module()
                 .has_module_conf())
          << "online simulation module is not set.";
      const auto &sim_conf = scenario.execution()
                                 .sim_modules()
                                 .online_simulation_module()
                                 .module_conf();
      lite_modules[i]->SetSimModuleConf(sim_conf);
    }
    lite_modules[i]->Init();
  }
}

int Main() {
  // try to remove stale crash file.
  std::remove(kCrashLogFile);
  // init ftrace
  QCHECK(qcraft::FTrace::Instance()->Init());

  QEventLogCollector log_collector;
  if (FLAGS_enable_qevent_collection) {
    GlobalQEventCollector::SetCollector(&log_collector);
  }

  // Set up param manager with run config.
  QCHECK(!FLAGS_run_conf.empty()) << "--run_conf must be specified.";
  auto param_manager =
      CreateParamManager(FLAGS_run_conf, FLAGS_car_name, FLAGS_lite_run_dir);
  CHECK(param_manager != nullptr);

  // Update the home coordinate based on the current locale.
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  if (run_params.locale_params().has_locale()) {
    locale_util::SetLocale(run_params.locale_params().locale());
  } else {
    locale_util::SetLocale(GetMapLocale(GetMap()));
  }

  LOG(INFO) << "Start main initialize modules.";
  std::vector<std::string> disable_module_prefixes =
      absl::StrSplit(FLAGS_disable_prefixes, ",", absl::SkipEmpty());
  std::vector<LiteModuleConfig> module_configs =
      LoadModuleConfigsFromCurrentLaunchFile();
  std::unique_ptr<folly::Executor> executor =
      std::make_unique<folly::CPUThreadPoolExecutor>(8);
  std::vector<LiteModuleName> module_names;
  for (auto iter = module_configs.begin(); iter != module_configs.end();) {
    bool disable_module = false;
    for (const auto &prefix : disable_module_prefixes) {
      if (absl::StartsWith(iter->module_class_name(), prefix)) {
        disable_module = true;
        break;
      }
    }
    if (disable_module) {
      VLOG(1) << "Going to disable module: " << iter->DebugString();
      iter = module_configs.erase(iter);
    } else {
      iter++;
    }
  }

  const int num_modules = module_configs.size();
  for (const auto &conf : module_configs) {
    module_names.push_back(conf.module_name());
  }

  lite_clients.resize(num_modules);
  lite_modules.resize(num_modules);

  LOG(INFO) << "Going to launch " << num_modules << " modules.";
  QIssueTrans::Instance()->CreateClient(GlobalInnerMessageHub(),
                                        executor.get());
  {
    ThreadPool thread_pool(num_modules);
    ParallelFor(0, num_modules, FLAGS_seq_init ? nullptr : &thread_pool,
                [&](int i) {
                  InitModule(module_configs, thread_pool, executor,
                             &lite_clients, &lite_modules, i);
                });
  }

  for (auto &module : lite_modules) {
    LOG(INFO) << "Start module: " << module->ModuleName();
    module->Start(absl::Milliseconds(20));
  }
  // wait for module's main loop to start running.
  sleep(2);
  exit_notification.WaitForNotification();
  LOG(INFO) << "exit_notification notified, begin to shutdown";
  std::cout << "===  Shutting Down =====" << std::endl;
  GlobalInnerMessageHub()->Reset();
  std::cout << "===  All Messaging Stopped =====" << std::endl;

  for (auto &module : lite_modules) {
    std::cout << "NotifyStop " << module->ModuleName() << " ... " << std::endl;
    module->NotifyStop();
  }

  // destruct modules in backward order
  std::cout << "===  Destruct Modules =====" << std::endl;
  for (auto it = lite_modules.rbegin(); it != lite_modules.rend(); ++it) {
    auto &module = *it;
    const std::string module_name = module->ModuleName();
    std::cout << "Joining " << module_name << " ... " << std::endl;
    module->Join();
    std::cout << "Destructing " << module_name << " ... " << std::endl;
    module.reset();
    std::cout << "---------- " << std::endl;
  }
  lite_clients.clear();
  std::cout << "ALL CLEAR! " << std::endl;
  return 0;
}

}  // namespace qcraft

int main(int argc, char *argv[]) {
  absl::FailureSignalHandlerOptions options;
  options.writerfn = AbslSigWriterFunc;
  qcraft::InitQCraft(&argc, &argv, options);

  std::vector<gflags::CommandLineFlagInfo> all_flags;
  gflags::GetAllFlags(&all_flags);
  for (const auto &flag : all_flags) {
    LOG(INFO) << "Flag " << flag.name << ": \"" << flag.current_value << "\"";
  }

  // Register SIGINT
  const auto status = qcraft::signal::SetupHandler(SIGINT, SignalHandler);
  if (!status.ok()) {
    return -1;
  }

  return qcraft::Main();
}
