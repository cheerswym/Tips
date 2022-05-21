#ifndef ONBOARD_LITE_LOGGING_H_
#define ONBOARD_LITE_LOGGING_H_

#include "glog/logging.h"  // IWYU pragma: keep
#include "onboard/lite/check.h"

#define Q_ONBOARD_BUILD
#ifdef Q_ONBOARD_BUILD

#define QLOG(severity) QLOG_WITH_MODULE(severity, false)
#define QLOG_NOW(severity) QLOG_WITH_MODULE(severity, true)

#define QLOG_IF(severity, condition) QLOG_WITH_MODULE_IF(severity, condition)

#define QLOG_IF_NOT_OK(severity, status) \
  QLOG_WITH_MODULE_IF(severity, !status.ok()) << #status " is not ok"

#define QLOG_NOW_IF(severity, condition) \
  QLOG_WITH_MODULE_NOW_IF(severity, condition)

#define QLOG_EVERY_N_SEC(severity, n_seconds)                     \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, n_seconds) \
  QLOG(severity)

#define QLOG_EVERY_N(severity, n)                      \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryN, true, n) \
  QLOG(severity)

#else

#define QLOG(severity) LOG(severity)

#define QLOG_EVERY_N(severity, n) LOG_EVERY_N(severity, n)

#define QLOG_EVERY_N_SEC(severity, n_seconds)                     \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, n_seconds) \
  LOG(severity)

#define QLOG_EVERY_N_SEC(severity, n)                  \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryN, true, n) \
  LOG(severity)
#endif  // Q_ONBOARD_BUILD

#endif  // ONBOARD_LITE_LOGGING_H_
