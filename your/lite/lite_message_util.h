#ifndef ONBOARD_LITE_LITE_MESSAGE_UTIL_H_
#define ONBOARD_LITE_LITE_MESSAGE_UTIL_H_

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

std::string ConvertToLowerUnderscore(const std::string &proto_name);

std::string CombineDomainChannel(const std::string &domain,
                                 const std::string &channel);

size_t StringHash(const std::string &str);

template <typename ProtoType>
std::string ProtoMessageToName() {
  ProtoType proto;
  const google::protobuf::Descriptor *descriptor = proto.GetDescriptor();
  return ConvertToLowerUnderscore(descriptor->name());
}

class LiteMsgConverter {
 public:
  static LiteMsgConverter &Get() {
    static LiteMsgConverter *converter = new LiteMsgConverter();
    return *converter;
  }

  absl::Status SetLiteMsgByTag(const google::protobuf::Message &src_msg,
                               int32 tag_number, LiteMsgWrapper *lite_msg);

  absl::Status SetLiteMsgByTag(std::string_view data, int32 tag_number,
                               LiteMsgWrapper *lite_msg);

  absl::Status SetLiteMsgByName(const google::protobuf::Message &src_msg,
                                const std::string &field_name,
                                LiteMsgWrapper *lite_msg);

  absl::Status SetLiteMsgByName(std::string_view data,
                                const std::string &field_name,
                                LiteMsgWrapper *lite_msg);

  bool IsReservedField(const LiteMsgWrapper &lite_msg);

  // Returns true if the field set in the `lite_msg` is undefined in the
  // LiteMsgWrapper proto.
  bool IsUndefinedField(const LiteMsgWrapper &lite_msg);

  int32 GetLiteMsgTagNumber(const google::protobuf::Message &lite_msg);

  const google::protobuf::Message &GetLiteMsgField(
      const LiteMsgWrapper &lite_msg) const;

  std::pair<std::string, std::string> GetLiteMsgChannelDomain(
      const LiteMsgWrapper &lite_msg) const;

  std::pair<std::string, std::string> GetLiteMsgChannelDomain(
      const LiteHeader &header) const;

  std::string GetLiteMsgChannelDomain(
      const google::protobuf::Message &lite_msg) const;

  LiteModuleName GetLiteModuleName(const LiteMsgWrapper &lite_msg) const;

  // Get lite message's header from lite message wrapper.
  // q header from lite message wrapper.
  const LiteHeader &GetLiteMsgWrapperHeader(
      const LiteMsgWrapper &lite_msg) const;

  LiteHeader *GetMutableLiteMsgWrapperHeader(LiteMsgWrapper *lite_msg);
  LiteHeader *GetMutableLiteMessageHeader(google::protobuf::Message *lite_msg);

  // Get lite message's header from lite message.
  const LiteHeader &GetLiteMessageHeader(
      const google::protobuf::Message &lite_msg) const;

  google::protobuf::Message *MutableLiteMsgField(LiteMsgWrapper *lite_msg);

  std::string GetFieldName(const LiteMsgWrapper &lite_msg);

  std::string GetFieldName(int32 tag_number);

  template <typename T>
  int32 GetTagNumber() {
    T lite_msg;
    const std::string &full_name = lite_msg.GetDescriptor()->full_name();
    return FindWithDefault(full_name_to_tag_, full_name, -1);
  }

  int32 GetTagNumber(const std::string &field_name) {
    return FindWithDefault(field_name_to_tag_number_, field_name, -1);
  }

 private:
  LiteMsgConverter();

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
  // tag numbers get reserved in lite_msg proto definition.
  std::set<int32> reserved_numbers_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_MESSAGE_UTIL_H_
