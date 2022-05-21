#include "onboard/utils/time_util.h"

#include "google/protobuf/timestamp.pb.h"

namespace qcraft {

absl::Time FromProto(const google::protobuf::Timestamp& proto) {
  return absl::FromUnixSeconds(proto.seconds()) +
         absl::Nanoseconds(proto.nanos());
}

void ToProto(absl::Time time, google::protobuf::Timestamp* proto) {
  const int64_t s = absl::ToUnixSeconds(time);
  proto->set_seconds(s);
  proto->set_nanos((time - absl::FromUnixSeconds(s)) / absl::Nanoseconds(1));
}

}  // namespace qcraft
