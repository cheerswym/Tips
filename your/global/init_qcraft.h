#ifndef ONBOARD_GLOBAL_INIT_QCRAFT_H_
#define ONBOARD_GLOBAL_INIT_QCRAFT_H_

#include "absl/debugging/failure_signal_handler.h"

namespace qcraft {

// Initialize misc qcraft-related things in the binary. This includes
// initializing services such as command line flags, logging directories,
// stacktrace print, etc.. Typically called early on in main() and must be
// called before other threads start using functions from this file.
void InitQCraft(int* argc, char*** argv, bool gflags_remove_flags = true,
                bool gflags_setlocale = true);

// Same as above with providing FailureSignalHandlerOptions.
void InitQCraft(int* argc, char*** argv,
                const absl::FailureSignalHandlerOptions& options,
                bool gflags_remove_flags = true, bool gflags_setlocale = true);

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_INIT_QCRAFT_H_
