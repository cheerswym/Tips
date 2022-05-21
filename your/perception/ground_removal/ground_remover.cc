#include "onboard/perception/ground_removal/ground_remover.h"

#include <vector>

#include "onboard/utils/map_util.h"

namespace qcraft::ground_removal {

GroundRemover::GroundRemover(const std::vector<LidarFrame>& lidar_frames,
                             const RunParamsProtoV2& run_params) {
  for (const auto& lidar_frame : lidar_frames) {
    InsertOrDie(&lidar_frames_, lidar_frame.lidar_id(), lidar_frame);
  }
  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    InsertOrDie(&lidar_params_, lidar_param.installation().lidar_id(),
                lidar_param);
  }
}

}  // namespace qcraft::ground_removal
