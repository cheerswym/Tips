#include "onboard/global/init_qcraft.h"

#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "folly/Singleton.h"
#include "folly/init/Init.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/maps/map_selector.h"

namespace qcraft {

void InitQCraft(int* argc, char*** argv, bool gflags_remove_flags,
                bool gflags_setlocale) {
  absl::FailureSignalHandlerOptions options;
  InitQCraft(argc, argv, options, gflags_remove_flags, gflags_setlocale);
}

void InitQCraft(int* argc, char*** argv,
                const absl::FailureSignalHandlerOptions& options,
                bool gflags_remove_flags, bool gflags_setlocale) {
  // Initialize the symbolizer to get a human-readable stack trace.
  absl::InitializeSymbolizer((*argv)[0]);
  absl::InstallFailureSignalHandler(options);

  gflags::ParseCommandLineFlags(argc, argv, gflags_remove_flags);

  // Do not call folly::init(argc, argv), since it will override the absl's
  // failure handlers. Call this function only which is the most interesting
  // part of the folly init.
  folly::SingletonVault::singleton()->registrationComplete();

  // Set locale.
  if (gflags_setlocale) {
    locale_util::SetLocale(GetMapLocale(GetMap()));
  }
}

}  // namespace qcraft
