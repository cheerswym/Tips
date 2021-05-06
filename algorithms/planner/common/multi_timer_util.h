#ifndef ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
#define ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_

#include <string>

#include "onboard/global/timer.h"
#include "onboard/proto/timer_report.pb.h"
namespace qcraft::planner {

void GetMultiTimerReport(const ScopedMultiTimer &timer,
                         MultiTimerReportProto *report_proto);

std::string PrintMultiTimerReport(const MultiTimerReportProto &report_proto);

void PrintMultiTimerReportStat(const ScopedMultiTimer &timer);
void PrintMultiTimerReportStat(const MultiTimerReportProto &report_proto);

void CombineMultiTimerProtos(MultiTimerReportProto *proto_1,
                             const MultiTimerReportProto &proto_2);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
