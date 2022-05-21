#include "onboard/lite/runner/runner_flags.h"

DEFINE_string(
    disable_prefixes, "",
    "A lite of disable modules prefixes, seperated by comma. It will match "
    "launch config file's module name's prefix. Like 'Perception' "
    "can disable all module's class name started with 'Perception'");

DEFINE_string(online_simulation_scenario_path, "",
              "If path is not empty, will run online simulation mode with "
              "configured virtual agents.");

DECLARE_string(lite_run_dir);

DEFINE_bool(enable_qevent_collection, true, "Enable onboard qevent collection");

DEFINE_int32(
    chrono_clock, 0,
    "If chrono_clock is not 0, we will use chrono clock in lite system, "
    "1:system time; 2:steady time.");
