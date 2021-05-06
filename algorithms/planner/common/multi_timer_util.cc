#include "onboard/planner/common/multi_timer_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "onboard/lite/logging.h"

namespace qcraft::planner {

void GetMultiTimerReport(const ScopedMultiTimer &timer,
                         MultiTimerReportProto *report_proto) {
  report_proto->Clear();
  report_proto->set_start_ts(absl::ToUnixMicros(timer.start()));
  absl::Time prev_time = timer.start();
  for (const auto &timer_mark : timer.marks()) {
    MultiTimerReportProto::MarkProto *mark = report_proto->add_marks();
    mark->set_message(timer_mark.msg);
    mark->set_end_ts(absl::ToUnixMicros(timer_mark.time));
    mark->set_duration(absl::ToDoubleSeconds(timer_mark.time - prev_time));
    mark->set_id(timer_mark.id);
    prev_time = timer_mark.time;
  }

  const absl::Time now = absl::Now();
  MultiTimerReportProto::MarkProto *mark = report_proto->add_marks();
  mark->set_message("End (may contain overhead due to timing)");
  mark->set_end_ts(absl::ToUnixMicros(now));
  mark->set_duration(absl::ToDoubleSeconds(now - prev_time));
  mark->set_id(0);

  report_proto->set_total_duration(absl::ToDoubleSeconds(now - timer.start()));
}

std::string PrintMultiTimerReport(const MultiTimerReportProto &report_proto) {
  return absl::StrFormat(
      "start ts = %d, total_duratioin = %6.3f ms, marks: %s",
      report_proto.start_ts(), 1e3 * report_proto.total_duration(),
      absl::StrJoin(report_proto.marks().begin(), report_proto.marks().end(),
                    ", ", [](std::string *out, const auto &mark) {
                      absl::StrAppend(
                          out,
                          absl::StrFormat("(%s-%03d, %6.4f ms)", mark.message(),
                                          mark.id(), 1e3 * mark.duration()));
                    }));
}

void PrintMultiTimerReportStat(const ScopedMultiTimer &timer) {
  MultiTimerReportProto timer_report;
  GetMultiTimerReport(timer, &timer_report);
  PrintMultiTimerReportStat(timer_report);
}

void PrintMultiTimerReportStat(const MultiTimerReportProto &report_proto) {
  absl::flat_hash_map<std::string, std::pair<int, double>> accumul_time_map;
  for (const auto &marker : report_proto.marks()) {
    if (accumul_time_map.find(marker.message()) == accumul_time_map.end()) {
      accumul_time_map.emplace(marker.message(),
                               std::make_pair(1, marker.duration()));
    } else {
      accumul_time_map[marker.message()].first++;
      accumul_time_map[marker.message()].second += marker.duration();
    }
  }
  using marker_pair = std::pair<std::string, std::pair<int, double>>;
  auto accumul_time = std::vector<marker_pair>(accumul_time_map.begin(),
                                               accumul_time_map.end());
  std::sort(accumul_time.begin(), accumul_time.end(),
            [](const marker_pair &a, const marker_pair &b) {
              return a.second.second > b.second.second;
            });

  const int name_width =
      std::max_element(accumul_time.begin(), accumul_time.end(),
                       [](const marker_pair &a, const marker_pair &b) {
                         return a.first.size() < b.first.size();
                       })
          ->first.size();
  QLOG(INFO) << absl::StrFormat("\trank\t%-*s\tcount\tduration (ms)\tratio",
                                name_width, "name");
  for (int i = 0; i < accumul_time.size(); ++i) {
    QLOG(INFO) << absl::StrFormat(
        "\t%d\t%-*s\t%d\t%6.3f\t\t%6.2f%%", i, name_width,
        accumul_time[i].first, accumul_time[i].second.first,
        1e3 * accumul_time[i].second.second,
        100 * accumul_time[i].second.second / report_proto.total_duration());
  }
  QLOG(INFO) << absl::StrFormat("\t\t%-*s\t \t%6.3f\t\t100.00%%", name_width,
                                "total", 1e3 * report_proto.total_duration());
}

void CombineMultiTimerProtos(MultiTimerReportProto *proto_1,
                             const MultiTimerReportProto &proto_2) {
  proto_1->set_start_ts(std::min(proto_1->start_ts(), proto_2.start_ts()));

  for (const auto &timer_mark : proto_2.marks()) {
    *proto_1->add_marks() = timer_mark;
  }

  proto_1->set_total_duration(
      std::max(proto_1->total_duration(), proto_2.total_duration()));
}

}  // namespace qcraft::planner
