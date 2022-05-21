
#include "onboard/lite/aggregate_lite_message_util.h"

#include <functional>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

AggregateLiteMsgConverter::AggregateLiteMsgConverter() {
  const auto* descriptor = AggregateLiteMsgWrapper::descriptor();
  const int32 reserved_range_cnt = descriptor->reserved_range_count();
  for (int i = 0; i < reserved_range_cnt; i++) {
    auto* range = descriptor->reserved_range(i);
    for (int j = range->start; j <= range->end; j++) {
      if (!descriptor->IsReservedNumber(j)) {
        continue;
      }
      reserved_numbers_.insert(j);
    }
  }

  for (int i = 0; i < descriptor->field_count(); i++) {
    const google::protobuf::FieldDescriptor* fd = descriptor->field(i);
    VLOG(1) << "AggregateLiteMsgConverter field:" << fd->name();
    tag_number_to_fdr_[fd->number()] = fd;
    tag_number_to_dr_[fd->number()] = fd->message_type();
    field_name_to_fdr_[fd->name()] = fd;
    field_name_to_tag_number_[fd->name()] = fd->number();
    tag_number_to_field_name_[fd->number()] = fd->name();
    full_name_to_tag_[fd->message_type()->full_name()] = fd->number();
  }
}

absl::Status AggregateLiteMsgConverter::SetAggregateLiteMsgByTag(
    const google::protobuf::Message& src_msg, int32 tag_number,
    AggregateLiteMsgWrapper* agg_lite_msg) {
  if (ContainsKey(reserved_numbers_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " is marked as reserved."));
  }

  if (!ContainsKey(tag_number_to_fdr_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " doesn't exist."));
  }
  auto* fdr = FindOrDie(tag_number_to_fdr_, tag_number);
  auto* reflection = agg_lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(agg_lite_msg, fdr))
      ->CopyFrom(src_msg);
  return absl::OkStatus();
}

int32 AggregateLiteMsgConverter::GetAggregateLiteMsgTagNumber(
    const google::protobuf::Message& agg_lite_msg) {
  const std::string& full_name = agg_lite_msg.GetDescriptor()->full_name();
  return FindOrDie(full_name_to_tag_, full_name);
}

absl::Status AggregateLiteMsgConverter::SetAggregateLiteMsgByTag(
    std::string_view data, int32 tag_number,
    AggregateLiteMsgWrapper* agg_lite_msg) {
  if (ContainsKey(reserved_numbers_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " is marked as reserved."));
  }
  if (!ContainsKey(tag_number_to_fdr_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " doesn't exist."));
  }
  auto* fdr = FindOrDie(tag_number_to_fdr_, tag_number);
  auto* reflection = agg_lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(agg_lite_msg, fdr))
      ->ParseFromArray(data.data(), data.size());
  return absl::OkStatus();
}

absl::Status AggregateLiteMsgConverter::SetAggregateLiteMsgByName(
    const google::protobuf::Message& src_msg, const std::string& field_name,
    AggregateLiteMsgWrapper* agg_lite_msg) {
  if (!ContainsKey(field_name_to_fdr_, field_name)) {
    return absl::NotFoundError(
        absl::StrCat("field ", field_name, " doesn't exist."));
  }
  auto* fdr = FindOrDie(field_name_to_fdr_, field_name);
  auto* reflection = agg_lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(agg_lite_msg, fdr))
      ->CopyFrom(src_msg);
  return absl::OkStatus();
}

absl::Status AggregateLiteMsgConverter::SetAggregateLiteMsgByName(
    std::string_view data, const std::string& field_name,
    AggregateLiteMsgWrapper* lite_msg) {
  if (!ContainsKey(field_name_to_fdr_, field_name)) {
    return absl::NotFoundError(
        absl::StrCat("field ", field_name, " doesn't exist."));
  }
  auto* fdr = FindOrDie(field_name_to_fdr_, field_name);
  auto* reflection = lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(lite_msg, fdr))
      ->ParseFromArray(data.data(), data.size());
  return absl::OkStatus();
}
}  // namespace qcraft
