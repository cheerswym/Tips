#include "onboard/lite/transport/shm/shm_debug_util.h"

#include <unordered_map>

#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/utils/map_util.h"
namespace qcraft {
namespace shm {

namespace {
std::unordered_map<size_t, std::string> HashToChannelName() {
  const auto channel_domain_to_field_name =
      GetOutputsFromModuleConfigs(LoadModuleConfigsFromAllDeclared());
  std::unordered_map<size_t, std::string> output;
  for (const auto& kv : channel_domain_to_field_name) {
    auto channel = kv.first.first;
    auto domain = kv.first.second;

    auto domain_channel = CombineDomainChannel(domain, channel);
    auto hash = StringHash(domain_channel);
    output[hash] = domain_channel;
    std::cout << hash << "-> " << domain_channel << std::endl;
  }
  output[StringHash("SHM_LIDAR_FRAME")] = "SHM_LIDAR_FRAME";
  output[StringHash("SHM_ENCODED_IMAGE")] = "SHM_ENCODED_IMAGE";
  output[StringHash("SHM_DECODED_IMAGE")] = "SHM_DECODED_IMAGE";
  return output;
}
}  // namespace

DebugUtil::DebugUtil() : hash_to_channel_name_(HashToChannelName()) {}

std::string DebugUtil::TryFindChannelNameWithHash(size_t hash) {
  auto* name = FindOrNull(hash_to_channel_name_, hash);
  if (name) {
    return *name;
  } else {
    return std::to_string(hash);
  }
}
}  // namespace shm
}  // namespace qcraft