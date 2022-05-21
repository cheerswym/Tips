#include "onboard/lite/lite_message_util.h"

#include <functional>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace {
class StringHasher {
 public:
  static StringHasher& Get() {
    static StringHasher* converter = new StringHasher();
    return *converter;
  }

  size_t Hash(const std::string& str) { return str_hash_(str); }

 private:
  StringHasher() {}
  std::hash<std::string> str_hash_;
};

}  // namespace

size_t StringHash(const std::string& str) {
  return StringHasher::Get().Hash(str);
}

std::string ConvertToLowerUnderscore(const std::string& proto_name) {
  std::string result;
  for (int i = 0; i < proto_name.size(); ++i) {
    if (i != 0 && absl::ascii_isupper(proto_name[i])) {
      result.push_back('_');
    }
    result.push_back(absl::ascii_tolower(proto_name[i]));
  }
  return result;
}

std::string CombineDomainChannel(const std::string& domain,
                                 const std::string& channel) {
  return absl::StrCat("/", domain, "/", channel);
}

LiteMsgConverter::LiteMsgConverter() {
  const auto* descriptor = LiteMsgWrapper::descriptor();
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
  QCHECK_EQ(1, descriptor->oneof_decl_count());
  auto* oneof_descriptor = descriptor->oneof_decl(0);
  for (int i = 0; i < oneof_descriptor->field_count(); ++i) {
    tag_number_to_fdr_[oneof_descriptor->field(i)->number()] =
        oneof_descriptor->field(i);
    tag_number_to_dr_[oneof_descriptor->field(i)->number()] =
        oneof_descriptor->field(i)->message_type();

    field_name_to_fdr_[oneof_descriptor->field(i)->name()] =
        oneof_descriptor->field(i);
    field_name_to_tag_number_[oneof_descriptor->field(i)->name()] =
        oneof_descriptor->field(i)->number();
    tag_number_to_field_name_[oneof_descriptor->field(i)->number()] =
        oneof_descriptor->field(i)->name();
    full_name_to_tag_[oneof_descriptor->field(i)->message_type()->full_name()] =
        oneof_descriptor->field(i)->number();
  }
}

std::string LiteMsgConverter::GetFieldName(const LiteMsgWrapper& lite_msg) {
  return FindWithDefault(tag_number_to_field_name_, lite_msg.tag_number(), "");
}

std::string LiteMsgConverter::GetFieldName(int32 tag_number) {
  return FindWithDefault(tag_number_to_field_name_, tag_number, "");
}

absl::Status LiteMsgConverter::SetLiteMsgByTag(
    const google::protobuf::Message& src_msg, int32 tag_number,
    LiteMsgWrapper* lite_msg) {
  if (ContainsKey(reserved_numbers_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " is marked as reserved."));
  }

  if (!ContainsKey(tag_number_to_fdr_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " doesn't exist."));
  }
  lite_msg->set_tag_number(tag_number);
  auto* fdr = FindOrDie(tag_number_to_fdr_, tag_number);
  auto* reflection = lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(lite_msg, fdr))->CopyFrom(src_msg);
  return absl::OkStatus();
}

int32 LiteMsgConverter::GetLiteMsgTagNumber(
    const google::protobuf::Message& lite_msg) {
  const std::string& full_name = lite_msg.GetDescriptor()->full_name();
  return FindOrDie(full_name_to_tag_, full_name);
}

absl::Status LiteMsgConverter::SetLiteMsgByTag(std::string_view data,
                                               int32 tag_number,
                                               LiteMsgWrapper* lite_msg) {
  if (ContainsKey(reserved_numbers_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " is marked as reserved."));
  }
  if (!ContainsKey(tag_number_to_fdr_, tag_number)) {
    return absl::NotFoundError(
        absl::StrCat("tag ", tag_number, " doesn't exist."));
  }
  lite_msg->set_tag_number(tag_number);
  auto* fdr = FindOrDie(tag_number_to_fdr_, tag_number);
  auto* reflection = lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(lite_msg, fdr))
      ->ParseFromArray(data.data(), data.size());
  return absl::OkStatus();
}

absl::Status LiteMsgConverter::SetLiteMsgByName(
    const google::protobuf::Message& src_msg, const std::string& field_name,
    LiteMsgWrapper* lite_msg) {
  if (!ContainsKey(field_name_to_fdr_, field_name)) {
    return absl::NotFoundError(
        absl::StrCat("field ", field_name, " doesn't exist."));
  }
  auto* fdr = FindOrDie(field_name_to_fdr_, field_name);
  lite_msg->set_tag_number(FindOrDie(field_name_to_tag_number_, field_name));
  auto* reflection = lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(lite_msg, fdr))->CopyFrom(src_msg);
  return absl::OkStatus();
}

absl::Status LiteMsgConverter::SetLiteMsgByName(std::string_view data,
                                                const std::string& field_name,
                                                LiteMsgWrapper* lite_msg) {
  if (!ContainsKey(field_name_to_fdr_, field_name)) {
    return absl::NotFoundError(
        absl::StrCat("field ", field_name, " doesn't exist."));
  }
  auto* fdr = FindOrDie(field_name_to_fdr_, field_name);
  lite_msg->set_tag_number(FindOrDie(field_name_to_tag_number_, field_name));
  auto* reflection = lite_msg->GetReflection();
  QCHECK_NOTNULL(reflection->MutableMessage(lite_msg, fdr))
      ->ParseFromArray(data.data(), data.size());
  return absl::OkStatus();
}

bool LiteMsgConverter::IsReservedField(const LiteMsgWrapper& lite_msg) {
  const int32 tag_number = lite_msg.tag_number();
  return ContainsKey(reserved_numbers_, tag_number);
}

bool LiteMsgConverter::IsUndefinedField(const LiteMsgWrapper& lite_msg) {
  return !ContainsKey(tag_number_to_fdr_, lite_msg.tag_number());
}

const google::protobuf::Message& LiteMsgConverter::GetLiteMsgField(
    const LiteMsgWrapper& lite_msg) const {
  int32 tag_number = lite_msg.tag_number();
  if (ContainsKey(reserved_numbers_, tag_number)) {
    LOG(FATAL) << tag_number << " is a reserved field. Should ignore it.";
  }
  const google::protobuf::FieldDescriptor* fdr =
      FindWithDefault(tag_number_to_fdr_, tag_number, nullptr);
  if (fdr == nullptr) {
    LOG(FATAL) << "Cannot find the tag number " << tag_number
               << " from message: " << lite_msg.ShortDebugString();
  }
  auto* reflection = lite_msg.GetReflection();
  return reflection->GetMessage(lite_msg, fdr);
}

LiteModuleName LiteMsgConverter::GetLiteModuleName(
    const LiteMsgWrapper& lite_msg) const {
  const auto& msg = this->GetLiteMsgField(lite_msg);
  const auto* descriptor = msg.GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  const auto& header = static_cast<const LiteHeader&>(
      msg.GetReflection()->GetMessage(msg, field_desc));
  return static_cast<LiteModuleName>(header.module_id());
}

std::pair<std::string, std::string> LiteMsgConverter::GetLiteMsgChannelDomain(
    const LiteMsgWrapper& lite_msg) const {
  int32 tag_number = lite_msg.tag_number();
  if (ContainsKey(reserved_numbers_, tag_number)) {
    LOG(FATAL) << tag_number << " is a reserved field. Should ignore it.";
  }
  const auto& msg = this->GetLiteMsgField(lite_msg);
  const auto* descriptor = msg.GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  const auto& header = static_cast<const LiteHeader&>(
      msg.GetReflection()->GetMessage(msg, field_desc));

  std::pair<std::string, std::string> result(header.channel(), header.domain());
  if (result.first.empty()) {
    result.first = FindOrDie(tag_number_to_field_name_, tag_number);
  }
  return result;
}

std::pair<std::string, std::string> LiteMsgConverter::GetLiteMsgChannelDomain(
    const LiteHeader& header) const {
  int32 tag_number = header.tag_number();
  if (ContainsKey(reserved_numbers_, tag_number)) {
    LOG(FATAL) << tag_number << " is a reserved field. Should ignore it.";
  }

  std::pair<std::string, std::string> result(header.channel(), header.domain());
  if (result.first.empty()) {
    result.first = FindOrDie(tag_number_to_field_name_, tag_number);
  }
  return result;
}

std::string LiteMsgConverter::GetLiteMsgChannelDomain(
    const google::protobuf::Message& lite_msg) const {
  const auto header = GetLiteMessageHeader(lite_msg);
  auto channel = header.channel();
  if (channel.empty()) {
    channel = FindOrDie(tag_number_to_field_name_, header.tag_number());
  }
  return absl::StrCat("/", header.domain(), "/", channel);
}

const LiteHeader& LiteMsgConverter::GetLiteMsgWrapperHeader(
    const LiteMsgWrapper& lite_msg) const {
  const auto& msg = this->GetLiteMsgField(lite_msg);
  const auto* descriptor = msg.GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  const auto& header = static_cast<const LiteHeader&>(
      msg.GetReflection()->GetMessage(msg, field_desc));
  return header;
}

LiteHeader* LiteMsgConverter::GetMutableLiteMsgWrapperHeader(
    LiteMsgWrapper* lite_msg) {
  auto* msg = this->MutableLiteMsgField(lite_msg);
  const auto* descriptor = msg->GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  return static_cast<LiteHeader*>(
      msg->GetReflection()->MutableMessage(msg, field_desc));
}

const LiteHeader& LiteMsgConverter::GetLiteMessageHeader(
    const google::protobuf::Message& lite_msg) const {
  const auto* descriptor = lite_msg.GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  const auto& header = static_cast<const LiteHeader&>(
      lite_msg.GetReflection()->GetMessage(lite_msg, field_desc));
  return header;
}

LiteHeader* LiteMsgConverter::GetMutableLiteMessageHeader(
    google::protobuf::Message* lite_msg) {
  const auto* descriptor = lite_msg->GetDescriptor();
  const auto* field_desc = descriptor->FindFieldByNumber(1);
  return static_cast<LiteHeader*>(
      lite_msg->GetReflection()->MutableMessage(lite_msg, field_desc));
}

google::protobuf::Message* LiteMsgConverter::MutableLiteMsgField(
    LiteMsgWrapper* lite_msg) {
  int32 tag_number = lite_msg->tag_number();
  if (ContainsKey(reserved_numbers_, tag_number)) {
    LOG(FATAL) << tag_number << " is a reserved field. Should ignore it.";
  }
  auto* fdr = FindOrDie(tag_number_to_fdr_, tag_number);
  auto* reflection = lite_msg->GetReflection();
  return reflection->MutableMessage(lite_msg, fdr);
}

}  // namespace qcraft
