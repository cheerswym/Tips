#ifndef ONBOARD_UTILS_ID_GENERATOR_H_
#define ONBOARD_UTILS_ID_GENERATOR_H_

#include <cstdint>
#include <string>

namespace qcraft {

uint64_t GetUId();
uint64_t GetAutoIncrementId(const std::string& bucket = "");

}  // namespace qcraft
#endif  // ONBOARD_UTILS_ID_GENERATOR_H_
