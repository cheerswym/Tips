#ifndef ONBOARD_LITE_AGGREGATE_LITE_MESSAGE_UTIL_H_
#define ONBOARD_LITE_AGGREGATE_LITE_MESSAGE_UTIL_H_

#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "google/protobuf/dynamic_message.h"
#include "google/protobuf/message.h"
#include "onboard/base/base.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

class AggregateLiteMsgConverter {
 public:
  static AggregateLiteMsgConverter &Get() {
    static AggregateLiteMsgConverter *converter =
        new AggregateLiteMsgConverter();
    return *converter;
  }

  absl::Status SetAggregateLiteMsgByTag(
      const google::protobuf::Message &src_msg, int32 tag_number,
      AggregateLiteMsgWrapper *agg_lite_msg);

  absl::Status SetAggregateLiteMsgByTag(std::string_view data, int32 tag_number,
                                        AggregateLiteMsgWrapper *agg_lite_msg);

  absl::Status SetAggregateLiteMsgByName(
      const google::protobuf::Message &src_msg, const std::string &field_name,
      AggregateLiteMsgWrapper *agg_lite_msg);

  absl::Status SetAggregateLiteMsgByName(std::string_view data,
                                         const std::string &field_name,
                                         AggregateLiteMsgWrapper *agg_lite_msg);

  int32 GetAggregateLiteMsgTagNumber(
      const google::protobuf::Message &agg_lite_msg);

  int32 GetTagNumber(const std::string &field_name) {
    return FindWithDefault(field_name_to_tag_number_, field_name, -1);
  }

 private:
  AggregateLiteMsgConverter();

  google::protobuf::DynamicMessageFactory dmf_;
  std::unordered_map<int32, const google::protobuf::FieldDescriptor *>
      tag_number_to_fdr_;
  std::unordered_map<int32, const google::protobuf::Descriptor *>
      tag_number_to_dr_;
  std::unordered_map<std::string, const google::protobuf::FieldDescriptor *>
      field_name_to_fdr_;
  std::unordered_map<std::string, int32> field_name_to_tag_number_;
  std::unordered_map<int32, std::string> tag_number_to_field_name_;
  std::unordered_map<std::string, int32> full_name_to_tag_;
  // tag numbers get reserved in agg_lite_msg proto definition.
  std::set<int32> reserved_numbers_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_AGGREGATE_LITE_MESSAGE_UTIL_H_
