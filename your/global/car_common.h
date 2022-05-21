#ifndef ONBOARD_GLOBAL_CAR_COMMON_H_
#define ONBOARD_GLOBAL_CAR_COMMON_H_

#include "gflags/gflags.h"

DECLARE_string(q_run_mode);

namespace qcraft {

// The qcraft car run mode.
enum QRunMode {
  // Running in real car on road.
  Q_ONBOARD = 1,
  // Running in deterministic simulation mode.
  Q_DSIM = 2,
  // Running in realtime/reproducible simulation mode.
  Q_RSIM = 3,
};

// Returns defined qcraft car run mode defined in flag q_run_mode.
QRunMode QCraftRunMode();

bool IsOnboardMode();

bool IsDSimMode();

bool IsRSimMode();

bool OnTestBench();

bool OnTestBenchForRsim();

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_CAR_COMMON_H_
