#include "onboard/lite/lite_message_util.h"

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

TEST(LiteMessageUtilTest, ProtoBothDefine) {
  std::set<std::string> proto_names;
  LiteMsgWrapper lite_msg;
  const auto* descriptor = lite_msg.GetDescriptor();
  QCHECK_EQ(1, descriptor->oneof_decl_count());
  auto* oneof_descriptor = descriptor->oneof_decl(0);
  for (int i = 0; i < oneof_descriptor->field_count(); ++i) {
    proto_names.insert(oneof_descriptor->field(i)->message_type()->full_name());
  }

  std::set<std::string> aggregate_proto_names;
  AggregateLiteMsgWrapper aggregate_lite_msg;
  const auto* aggregate_descriptor = aggregate_lite_msg.GetDescriptor();

  for (int i = 0; i < aggregate_descriptor->field_count(); ++i) {
    const auto& full_name =
        aggregate_descriptor->field(i)->message_type()->full_name();
    aggregate_proto_names.insert(full_name);
    const bool has = proto_names.count(full_name);
    if (!has) {
      LOG(ERROR) << "LiteMsgWrapper does not define:" << full_name;
    }
    EXPECT_TRUE(has);
  }

  for (const auto& full_name : proto_names) {
    const bool has = aggregate_proto_names.count(full_name);
    if (!has) {
      LOG(ERROR) << "AggregateLiteMsgWrapper does not defined:" << full_name;
    }
    EXPECT_TRUE(has);
  }
}

TEST(LiteMessageUtilTest, ReservedProtoInLiteMsgWrapper) {
  std::set<int> reserved_whitelist{4,  12, 26, 31,  36,  38,  63,  64, 65,
                                   76, 78, 99, 111, 112, 118, 123, 128};
  LiteMsgWrapper lite_msg;
  const auto* descriptor = lite_msg.GetDescriptor();

  const int32 reserved_range_cnt = descriptor->reserved_range_count();
  for (int i = 0; i < reserved_range_cnt; i++) {
    auto* range = descriptor->reserved_range(i);
    for (int j = range->start; j <= range->end; j++) {
      if (descriptor->IsReservedNumber(j)) {
        const bool has = reserved_whitelist.count(j);
        if (!has) {
          LOG(ERROR) << "Field number: " << j
                     << " is not allowed to be reserved in LiteMsgWrapper, "
                        "mark it deprecated instead.([deprecated = true]).";
        }
        EXPECT_TRUE(has);
      }
    }
  }
}

TEST(LiteMessageUtilTest, ReservedProtoInAggregateLiteMsgWrapper) {
  std::set<int> reserved_whitelist{4,   12,  26,  31,  38, 76,
                                   101, 113, 122, 129, 140};
  AggregateLiteMsgWrapper lite_msg;
  const auto* descriptor = lite_msg.GetDescriptor();

  const int32 reserved_range_cnt = descriptor->reserved_range_count();
  for (int i = 0; i < reserved_range_cnt; i++) {
    auto* range = descriptor->reserved_range(i);
    for (int j = range->start; j <= range->end; j++) {
      if (descriptor->IsReservedNumber(j)) {
        const bool has = reserved_whitelist.count(j);
        if (!has) {
          LOG(ERROR)
              << "Field number: " << j
              << " is not allowed to be reserved in AggregateLiteMsgWrapper "
                 "mark it deprecated instead.([deprecated = true]).";
        }
        EXPECT_TRUE(has);
      }
    }
  }
}

TEST(LiteMessageUtilTest, UniqueProtoDefineInLiteMsgWrapper) {
  std::set<std::string> proto_names;
  LiteMsgWrapper lite_msg;
  const auto* descriptor = lite_msg.GetDescriptor();
  QCHECK_EQ(1, descriptor->oneof_decl_count());
  auto* oneof_descriptor = descriptor->oneof_decl(0);
  for (int i = 0; i < oneof_descriptor->field_count(); ++i) {
    EXPECT_FALSE(ContainsKey(
        proto_names, oneof_descriptor->field(i)->message_type()->full_name()));
    proto_names.insert(oneof_descriptor->field(i)->message_type()->full_name());
    std::vector<std::string> names = absl::StrSplit(
        oneof_descriptor->field(i)->message_type()->full_name(), '.');
    ASSERT_GE(names.size(), 1);
    LOG(ERROR) << oneof_descriptor->field(i)->name() << " vs "
               << oneof_descriptor->field(i)->message_type()->full_name();
    EXPECT_EQ(ConvertToLowerUnderscore(names.back()),
              oneof_descriptor->field(i)->name());
  }
}

TEST(LiteMessageUtilTest, GetChannelDomain) {
  LiteMsgWrapper lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  pose.mutable_header()->set_channel("fake_pose");
  pose.mutable_header()->set_domain("fake_domain");
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByTag(pose, 2, &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());

  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(lite_msg);
  EXPECT_EQ(channel_domain.first, "fake_pose");
  EXPECT_EQ(channel_domain.second, "fake_domain");
}

TEST(LiteMessageUtilTest, SetLiteMessageByTag) {
  LiteMsgWrapper lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByTag(pose, 2, &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());

  EXPECT_EQ(absl::NotFoundError(absl::StrCat("tag ", 0, " doesn't exist.")),
            LiteMsgConverter::Get().SetLiteMsgByTag(pose, 0, &lite_msg));

  std::string buffer;
  ASSERT_TRUE(pose.SerializeToString(&buffer));
  lite_msg.Clear();
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByTag(buffer, 2, &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  EXPECT_EQ(lite_msg.tag_number(), 2);
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());
}

TEST(LiteMessageUtilTest, SetLiteMessageByName) {
  LiteMsgWrapper lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  QCHECK_OK(
      LiteMsgConverter::Get().SetLiteMsgByName(pose, "pose_proto", &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  EXPECT_EQ(lite_msg.tag_number(), 2);
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());

  EXPECT_EQ(
      absl::NotFoundError(
          absl::StrCat("field ", "fake_proto", " doesn't exist.")),
      LiteMsgConverter::Get().SetLiteMsgByName(pose, "fake_proto", &lite_msg));

  std::string buffer;
  ASSERT_TRUE(pose.SerializeToString(&buffer));
  lite_msg.Clear();
  QCHECK_OK(LiteMsgConverter::Get().SetLiteMsgByName(buffer, "pose_proto",
                                                     &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  EXPECT_EQ(lite_msg.tag_number(), 2);
  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());
}

TEST(LiteMessageUtilTest, GetFieldFromLiteMsg) {
  LiteMsgWrapper lite_msg;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(3.4);
  pose.mutable_pos_smooth()->set_y(4.3);
  QCHECK_OK(
      LiteMsgConverter::Get().SetLiteMsgByName(pose, "pose_proto", &lite_msg));
  ASSERT_TRUE(lite_msg.has_pose_proto());
  EXPECT_EQ(lite_msg.tag_number(), 2);

  // Fix this by using ProtoEqual.
  EXPECT_EQ(pose.DebugString(), lite_msg.pose_proto().DebugString());
  lite_msg.set_tag_number(2);

  EXPECT_EQ(LiteMsgConverter::Get().GetLiteMsgField(lite_msg).DebugString(),
            pose.DebugString());
  PoseProto* msg_pose = dynamic_cast<PoseProto*>(
      LiteMsgConverter::Get().MutableLiteMsgField(&lite_msg));
  msg_pose->mutable_pos_smooth()->set_x(5);
  EXPECT_NE(LiteMsgConverter::Get().GetLiteMsgField(lite_msg).DebugString(),
            pose.DebugString());
}

TEST(LiteMessageUtilTest, GetLiteMessageHeader) {
  PoseProto pose;
  pose.mutable_header()->set_timestamp(123);
  pose.mutable_header()->set_channel("pose");
  const auto& header = LiteMsgConverter::Get().GetLiteMessageHeader(pose);
  EXPECT_EQ(header.timestamp(), 123);
  EXPECT_EQ(header.channel(), "pose");
}

TEST(LiteMessageUtilTest, GetTagNumberByMsg) {
  EXPECT_EQ(3, LiteMsgConverter::Get().GetTagNumber<TrajectoryProto>());
  EXPECT_EQ(5, LiteMsgConverter::Get().GetTagNumber<SystemStatusProto>());
  EXPECT_EQ(15,
            LiteMsgConverter::Get().GetTagNumber<VehicleControlStatusProto>());
  EXPECT_EQ(-1, LiteMsgConverter::Get().GetTagNumber<LiteMsgWrapper>());
}
}  // namespace qcraft
