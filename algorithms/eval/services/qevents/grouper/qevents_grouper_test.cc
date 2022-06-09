#include "offboard/eval/services/qevents/grouper/qevents_grouper.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace eval {
TEST(QEventsGrouperUnitTest, timestamp_grouper) {
  std::vector<std::vector<QEventProto>> qevents(2, std::vector<QEventProto>());
  QEventProto event1, event2, event3;
  *event1.mutable_uid() = "1";
  event1.set_timestamp(123456789);

  *event2.mutable_uid() = "2";
  event2.set_timestamp(123456795);

  *event3.mutable_uid() = "3";
  event3.set_timestamp(123456800);

  qevents[0].push_back(event3);
  qevents[0].push_back(event1);
  qevents[1].push_back(event2);

  QEventGrouper grouper;
  grouper.set_name("timestamp_grouper");
  grouper.mutable_timestamp_grouper()->set_interval(10);
  const auto &result =
      GetQEventGrouper(grouper.name(), {grouper})->Group(qevents);
  ASSERT_EQ(result.size(), 2);
  ASSERT_EQ(result[0].selections(0).events(0).uid(), "3");
  ASSERT_EQ(result[0].selections(1).events(0).uid(), "2");
  ASSERT_EQ(result[1].selections(0).events(0).uid(), "1");
  ASSERT_EQ(result[1].selections(1).events_size(), 0);
}
}  // namespace eval
}  // namespace qcraft
