#ifndef ONBOARD_LITE_RUNNER_FLAGS_H_
#define ONBOARD_LITE_RUNNER_FLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(disable_prefixes);

DECLARE_string(online_simulation_scenario_path);

DECLARE_string(lite_run_dir);

DECLARE_bool(enable_qevent_collection);

DECLARE_int32(chrono_clock);

#endif  // ONBOARD_LITE_RUNNER_FLAGS_H_
