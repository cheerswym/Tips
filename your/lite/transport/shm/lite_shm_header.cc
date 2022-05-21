#include "onboard/lite/transport/shm/lite_shm_header.h"

#include <arpa/inet.h>

#include "glog/logging.h"
#include "onboard/base/base.h"
#include "onboard/global/singleton.h"

namespace qcraft {
namespace shm {
LiteShmHeader::LiteShmHeader(const std::string& domain_channel,
                             int32_t tag_number, const LiteHeader& header)
    : module_id_(header.module_id()),
      seq_num_(header.seq_number()),
      tag_number_(tag_number),
      timestamp_(header.timestamp()),
      domain_channel_(domain_channel) {
  CHECK_LE(ContentByteSize() + sizeof(uint32_t), kMaxSizeLimit)
      << "LiteShmHeader is too large.";
}

LiteShmHeader& LiteShmHeader::operator=(const LiteShmHeader& another) {
  if (this != &another) {
    domain_channel_ = another.domain_channel_;
    module_id_ = another.module_id_;
    seq_num_ = another.seq_num_;
    tag_number_ = another.tag_number_;
    timestamp_ = another.timestamp_;
  }
  return *this;
}

bool LiteShmHeader::operator==(const LiteShmHeader& another) const {
  return domain_channel_ == another.domain_channel_ &&
         module_id_ == another.module_id_ && seq_num_ == another.seq_num_ &&
         tag_number_ == another.tag_number_ && timestamp_ == another.timestamp_;
}

bool LiteShmHeader::operator!=(const LiteShmHeader& another) const {
  return !(*this == another);
}

uint32_t LiteShmHeader::ContentByteSize() const {
  size_t byte_size = domain_channel_.size();
  byte_size += sizeof(module_id_);
  byte_size += sizeof(seq_num_);
  byte_size += sizeof(timestamp_);
  byte_size += sizeof(tag_number_);
  return byte_size;
}

uint32_t LiteShmHeader::ByteSize() const {
  const uint32_t content_byte_size = ContentByteSize();
  return content_byte_size + sizeof(content_byte_size);
}

bool LiteShmHeader::SerializeTo(char* dst) const {
  if (dst == nullptr) return false;
  const uint32_t content_byte_size = ContentByteSize();
  char* ptr = dst;
  memcpy(ptr,
         reinterpret_cast<char*>(const_cast<uint32_t*>(&content_byte_size)),
         sizeof(content_byte_size));
  ptr += sizeof(content_byte_size);
  memcpy(ptr, reinterpret_cast<char*>(const_cast<uint32_t*>(&module_id_)),
         sizeof(module_id_));
  ptr += sizeof(module_id_);
  memcpy(ptr, reinterpret_cast<char*>(const_cast<uint64_t*>(&seq_num_)),
         sizeof(seq_num_));
  ptr += sizeof(seq_num_);
  memcpy(ptr, reinterpret_cast<char*>(const_cast<int32_t*>(&tag_number_)),
         sizeof(tag_number_));
  ptr += sizeof(tag_number_);
  memcpy(ptr, reinterpret_cast<char*>(const_cast<int64_t*>(&timestamp_)),
         sizeof(timestamp_));
  ptr += sizeof(timestamp_);
  memcpy(ptr, domain_channel_.data(), domain_channel_.size());

  return true;
}

bool LiteShmHeader::DeserializeFrom(const char* src) {
  if (src == nullptr) return false;
  const char* ptr = src;
  uint32_t content_byte_size = 0;
  uint32_t string_byte_size = 0;
  memcpy(reinterpret_cast<char*>(&content_byte_size), ptr,
         sizeof(content_byte_size));
  string_byte_size = content_byte_size;
  ptr += sizeof(content_byte_size);

  memcpy(reinterpret_cast<char*>(&module_id_), ptr, sizeof(module_id_));
  ptr += sizeof(module_id_);
  string_byte_size -= sizeof(module_id_);

  memcpy(reinterpret_cast<char*>(&seq_num_), ptr, sizeof(seq_num_));
  ptr += sizeof(seq_num_);
  string_byte_size -= sizeof(seq_num_);

  memcpy(reinterpret_cast<char*>(&tag_number_), ptr, sizeof(tag_number_));
  ptr += sizeof(tag_number_);
  string_byte_size -= sizeof(tag_number_);

  memcpy(reinterpret_cast<char*>(&timestamp_), ptr, sizeof(timestamp_));
  ptr += sizeof(timestamp_);
  string_byte_size -= sizeof(timestamp_);

  domain_channel_.resize(string_byte_size);
  memcpy(domain_channel_.data(), ptr, string_byte_size);

  return true;
}
}  // namespace shm
}  // namespace qcraft
