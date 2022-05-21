#ifndef ONBOARD_LITE_TRANSPORT_SHM_SHM_DEBUG_UTIL_H_
#define ONBOARD_LITE_TRANSPORT_SHM_SHM_DEBUG_UTIL_H_

#include <string>
#include <unordered_map>

#include "onboard/base/base.h"
#include "onboard/global/singleton.h"

namespace qcraft {
namespace shm {

class DebugUtil {
  // Using sha to find channel name, if not found just return hash as string.
 public:
  std::string TryFindChannelNameWithHash(size_t hash);

 private:
  std::unordered_map<size_t, std::string> hash_to_channel_name_;

  DECLARE_SINGLETON(DebugUtil);
};
}  // namespace shm
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_SHM_DEBUG_UTIL_H_
