#ifndef ONBOARD_PERCEPTION_POSE_CORRECTION_H_
#define ONBOARD_PERCEPTION_POSE_CORRECTION_H_

#include <optional>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/perception/laser_point.h"
#include "onboard/perception/obstacle_grid.h"
#include "onboard/perception/registration/point_matcher_structs.h"

namespace qcraft {

struct PoseCorrectionResult {
  double timestamp = 0.0;
  PointMatchResult match_result;
  LocalizationTransformProto localization_transform;
};

using PoseCorrectionResultHistory =
    boost::circular_buffer<PoseCorrectionResult>;
// NOTE(dong): Pose correction is a method that aims to correct roll, pitch and
// z offset between smooth coordinate and global coordinate. We always keep pose
// correction estimation in the vehicle frame.
class PoseCorrection {
 public:
  std::optional<PointMatchResult> CorrectPose(
      const VehiclePose& pose, const std::vector<LaserPoint>& points,
      const mapping::SemanticMapManager& semantic_map_manager,
      const CoordinateConverter& coordinate_converter,
      const ObstacleGrid& obstacle_grid, const LocalImagery& local_imagery,
      ThreadPool* thread_pool);

  const PoseCorrectionDebugProto& pose_correction_debug_proto() const {
    return pose_correction_debug_proto_;
  }

 private:
  std::optional<PointMatchResult> UpdateAndGetMatchResultFromHistory(
      const PointMatchResult& match_result,
      const LocalizationTransformProto& localization_transform);

 private:
  PoseCorrectionResultHistory pose_correction_result_history_{5};
  PoseCorrectionDebugProto pose_correction_debug_proto_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_POSE_CORRECTION_H_
