#ifndef ONBOARD_PERCEPTION_PILLAR_OBSTACLE_CONVERTER_H_
#define ONBOARD_PERCEPTION_PILLAR_OBSTACLE_CONVERTER_H_

#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/obstacle_grid.h"
#include "onboard/perception/pillar_rc_coord_converter.h"
#include "onboard/utils/map_util.h"
#include "opencv2/core.hpp"

namespace qcraft {

struct RowColHash {
  size_t operator()(std::pair<uint16_t, uint16_t> rc) const {
    return std::hash<uint32_t>()((rc.first << 16) + rc.second);
  }
};

// Note(zhangtao): This class is designed for pillar semantic correspond to
// obstacle. The pillar semantic is in the vehicle coordinate system and the
// obstacle is in the smooth coordinate system，a pillar and an obstacle do not
// correspond one-to-one，therefore, if you need to get the pillar type of an
// obstacle through its row index and column index, you need to convert an
// obstacle to the vehicle coordinate system, and then convert it to the row and
// col in pillar coordinate system to get pillar type
class PillarObstacleConverter {
 public:
  PillarObstacleConverter(const int width, const int height,
                          const std::vector<Obstacle> &obstacles,
                          const VehiclePose &pose,
                          const cv::Mat &pillar_semantic_result);

  std::pair<int, int> ConvertObstacleRCToPillarRC(const int obstacle_row,
                                                  const int obstacle_col) {
    return FindWithDefault(obstacle_rc_to_pillar_rc_,
                           std::pair<int, int>({obstacle_row, obstacle_col}),
                           std::pair<int, int>({-1, -1}));
  }

  std::pair<int, int> ConvertPillarRCToObstacleRC(const int pillar_row,
                                                  const int pillar_col);
  PillarType GetPillarTypeWithObstacleRC(const int obstacle_row,
                                         const int obstacle_col) {
    const auto [row, col] =
        ConvertObstacleRCToPillarRC(obstacle_row, obstacle_col);
    if (row >= pillar_semantic_mat_.rows || row < 0 ||
        col >= pillar_semantic_mat_.cols || col < 0) {
      return PLT_NONE;
    }
    return static_cast<PillarType>(pillar_semantic_mat_.at<uchar>(row, col));
  }

 private:
  void ComputeObstaclePillarType(const Obstacle &obs);

 private:
  // Pillar semantic result
  cv::Mat pillar_semantic_mat_;
  // Obstacle rc to coordinate converter
  ObstacleRCCoordConverter obstacle_rc_coord_converter_;

  // pillar rc and coordinate converter
  PillarRCCoordConverter pillar_rc_coord_converter_;
  // Pillar's RC correspond to obstacle's RC
  std::vector<std::pair<int, int>> pillar_rc_to_obstacle_rc_;
  // Obstacle'RC(obstacle in obsacle manager) correspond to pillar'RC
  absl::flat_hash_map<std::pair<int, int>, std::pair<int, int>, RowColHash>
      obstacle_rc_to_pillar_rc_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_PILLAR_OBSTACLE_CONVERTER_H_
