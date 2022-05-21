#ifndef ONBOARD_PERCEPTION_PERCEPTION_UTIL_H_
#define ONBOARD_PERCEPTION_PERCEPTION_UTIL_H_

#include <map>

#include "onboard/params/utils/param_util.h"

namespace qcraft {
namespace perception_util {

LidarParametersProto SelectLidarParams(
    const std::map<LidarId, LidarParametersProto> &lidar_params,
    bool is_ignore_solid_lidar = false);

LidarParametersProto SelectLidarParams(const RunParamsProtoV2 &run_params,
                                       bool is_ignore_solid_lidar = false);
}  // namespace perception_util
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_PERCEPTION_UTIL_H_
