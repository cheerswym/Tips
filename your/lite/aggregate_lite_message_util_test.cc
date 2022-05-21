#include "onboard/lite/aggregate_lite_message_util.h"

#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "gtest/gtest.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

TEST(AggregateAggregateLiteMsgConverterTest, SetAggregateLiteMsgByTag) {
  AggregateLiteMsgWrapper agg_lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  QCHECK_OK(AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByTag(
      pose, 2, &agg_lite_msg));
  ASSERT_TRUE(agg_lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), agg_lite_msg.pose_proto().DebugString());

  EXPECT_EQ(absl::NotFoundError(absl::StrCat("tag ", 0, " doesn't exist.")),
            AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByTag(
                pose, 0, &agg_lite_msg));

  std::string buffer;
  ASSERT_TRUE(pose.SerializeToString(&buffer));
  agg_lite_msg.Clear();
  QCHECK_OK(AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByTag(
      buffer, 2, &agg_lite_msg));
  ASSERT_TRUE(agg_lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), agg_lite_msg.pose_proto().DebugString());
}

TEST(AggregateLiteMsgConverterTest, SetAggregateLiteMessageByName) {
  AggregateLiteMsgWrapper agg_lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  QCHECK_OK(AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByName(
      pose, "pose_proto", &agg_lite_msg));
  ASSERT_TRUE(agg_lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), agg_lite_msg.pose_proto().DebugString());

  EXPECT_EQ(absl::NotFoundError(
                absl::StrCat("field ", "fake_proto", " doesn't exist.")),
            AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByName(
                pose, "fake_proto", &agg_lite_msg));

  std::string buffer;
  ASSERT_TRUE(pose.SerializeToString(&buffer));
  agg_lite_msg.Clear();
  QCHECK_OK(AggregateLiteMsgConverter::Get().SetAggregateLiteMsgByName(
      buffer, "pose_proto", &agg_lite_msg));
  ASSERT_TRUE(agg_lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), agg_lite_msg.pose_proto().DebugString());
}

}  // namespace qcraft
