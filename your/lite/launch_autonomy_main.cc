#include <csignal>
#include <iostream>

#include "absl/debugging/failure_signal_handler.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/global/logging.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/runner/autonomy_runner.h"
#include "onboard/lite/runner/module_runner.h"
namespace qcraft {
class LiteRunner {
 public:
  bool Init() { return impl_->Init(); }
  int Run() { return impl_->Run(); }
  int Release() { return impl_->Release(); }
  void SignalHandler(int signum) { impl_->SignalHandler(signum); }

  virtual ~LiteRunner() { impl_.reset(); }

 private:
  DECLARE_SINGLETON(LiteRunner);

 private:
  std::unique_ptr<lite::Runner> impl_;
};

LiteRunner::LiteRunner() {
  QCHECK(FLAGS_lite2_module_name != "");
  QCHECK(FLAGS_lite2_module_name.find("MOCK") == std::string::npos);
  if (FLAGS_lite2_module_name == "AUTONOMY_MODULE") {
    impl_ = std::make_unique<lite::AutonomyRunner>();
  } else {
    impl_ = std::make_unique<lite::ModuleRunner>();
  }
}

namespace {
void AbslSignalHandler(const char *error) {
  std::cerr << std::endl << error << std::endl;
}

void SignalHandler(int signum) {
  LiteRunner::Instance()->SignalHandler(signum);
}

}  // namespace

int Run(int argc, char *argv[]) {
  std::signal(SIGINT, SignalHandler);

  if (!LiteRunner::Instance()->Init()) {
    QLOG(ERROR) << "Init failed.";
  }

  if (LiteRunner::Instance()->Run() != 0) {
    QLOG(ERROR) << "Run exception.";
  }
  auto ret = LiteRunner::Instance()->Release();
  if (ret != 0) {
    QLOG(ERROR) << "Exit exception.";
  }
  return ret;
}
}  // namespace qcraft

int main(int argc, char *argv[]) {
  absl::FailureSignalHandlerOptions options;
  options.writerfn = qcraft::AbslSignalHandler;
  qcraft::InitQCraft(&argc, &argv, options);
  return qcraft::Run(argc, argv);
}
