#include "offboard/eval/services/qevents/grouper/timestamp_grouper.h"

#include <queue>
#include <utility>

#include "absl/time/time.h"
#include "glog/logging.h"

namespace qcraft {
namespace eval {
std::vector<DiffEventsResponse::EventsGroup> TimestampGrouper::Group(
    const std::vector<std::vector<QEventProto>>& qevents) {
  LOG(INFO) << "grouping";
  auto timestamp_comp = [](const auto& p1, const auto& p2) {
    return p1.first->timestamp() < p2.first->timestamp();
  };

  std::priority_queue<
      std::pair<std::vector<QEventProto>::const_iterator, int>,
      std::vector<std::pair<std::vector<QEventProto>::const_iterator, int>>,
      decltype(timestamp_comp)>
      next_events(timestamp_comp);
  for (int i = 0; i < qevents.size(); i++) {
    if (qevents[i].size() > 0)
      next_events.push(std::make_pair(qevents[i].begin(), i));
  }

  std::vector<DiffEventsResponse::EventsGroup> result;
  int64 start_timestamp = absl::ToUnixMicros(absl::InfiniteFuture());

  while (!next_events.empty()) {
    auto next_event_pair = next_events.top();
    next_events.pop();
    auto& next_event = next_event_pair.first;

    if (start_timestamp - next_event->timestamp() > interval_) {
      start_timestamp = next_event->timestamp();
      DiffEventsResponse::EventsGroup event_groups;
      event_groups.mutable_selections()->Reserve(qevents.size());
      for (int i = 0; i < qevents.size(); i++) {
        event_groups.add_selections();
      }
      result.push_back(std::move(event_groups));
    }

    DiffEventsResponse::EventsGroup& events_group = result.back();
    *((*events_group.mutable_selections())[next_event_pair.second]
          .add_events()) = *next_event;

    if (++next_event != qevents[next_event_pair.second].end())
      next_events.emplace(next_event, next_event_pair.second);
  }
  return result;
}
}  // namespace eval
}  // namespace qcraft
