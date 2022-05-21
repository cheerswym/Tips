#include "onboard/perception/perception_util.h"

#include <set>

#include "onboard/lite/logging.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace perception_util {

LidarParametersProto SelectLidarParams(
    const std::map<LidarId, LidarParametersProto>& lidar_params,
    const bool is_ignore_solid_lidar) {
  for (const auto& [lidar_id, _] : lidar_params) {
    QLOG(ERROR) << "lidar_id:" << LidarId_Name(lidar_id);
  }
  if (const auto* lidar_param = FindOrNull(lidar_params, LDR_CENTER)) {
    return *lidar_param;
  } else if (const auto* lidar_param = FindOrNull(lidar_params, LDR_FRONT);
             lidar_param && !is_ignore_solid_lidar) {
    return *lidar_param;
  } else if (const auto* lidar_param =
                 FindOrNull(lidar_params, LDR_FRONT_LEFT)) {
    return *lidar_param;
  } else {
    // If there has no LDR_CENTER, LDR_FRONT and LDR_FRONT_LEFT, we select first
    // lidar param.
    QCHECK(!lidar_params.empty());
    return lidar_params.begin()->second;
  }
}

LidarParametersProto SelectLidarParams(const RunParamsProtoV2& run_params,
                                       bool is_ignore_solid_lidar) {
  std::map<LidarId, LidarParametersProto> lidar_params;
  for (const auto& lidar_params_proto :
       run_params.vehicle_params().lidar_params()) {
    lidar_params[lidar_params_proto.installation().lidar_id()] =
        lidar_params_proto;
  }
  return SelectLidarParams(lidar_params, is_ignore_solid_lidar);
}

}  // namespace perception_util
}  // namespace qcraft
