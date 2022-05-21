#ifndef ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_HEADER_H_
#define ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_HEADER_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/strings/str_cat.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
namespace shm {
class LiteShmHeader {
 public:
  LiteShmHeader(const std::string& domain_channel, int32_t tag_number,
                const LiteHeader& header);
  LiteShmHeader() {}

  LiteShmHeader& operator=(const LiteShmHeader& another);
  bool operator==(const LiteShmHeader& another) const;
  bool operator!=(const LiteShmHeader& another) const;

  // Serialize to dst with the format:
  // content byte size(uint32_t) | module_id | seq_num_ |
  // tag_number|domain_channel_
  bool SerializeTo(char* dst) const;
  bool DeserializeFrom(const char* src);

  uint32_t ByteSize() const;
  uint32_t module_id() const { return module_id_; }

  uint64_t seq_num() const { return seq_num_; }
  uint64_t tag_number() const { return tag_number_; }

  int64_t timestamp() const { return timestamp_; }

  const std::string& domain_channel() { return domain_channel_; }

  std::string DebugString() {
    return absl::StrCat("Module_id=", module_id_, " seq_num=", seq_num_,
                        " tag_num=", tag_number_,
                        " domain_channel=", domain_channel_);
  }

 private:
  uint32_t ContentByteSize() const;
  const int kMaxSizeLimit = 1024;

  uint32_t module_id_ = 0;
  uint64_t seq_num_ = 0;
  int32_t tag_number_ = 0;
  int64_t timestamp_ = 0;
  std::string domain_channel_;
};
}  // namespace shm
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_HEADER_H_
