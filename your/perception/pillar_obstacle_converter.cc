#include "onboard/perception/pillar_obstacle_converter.h"

#include "onboard/nets/fiery_eye_net_classifier_constants.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

PillarObstacleConverter::PillarObstacleConverter(
    const int width, const int height, const std::vector<Obstacle> &obstacles,
    const VehiclePose &pose, const cv::Mat &pillar_semantic_result)
    : pillar_semantic_mat_(pillar_semantic_result),
      obstacle_rc_coord_converter_(width, height),
      pillar_rc_coord_converter_(fen::kDetectionRegionFront,
                                 fen::kDetectionRegionLeft,
                                 fen::kPillarResolution) {
  obstacle_rc_to_pillar_rc_.reserve(obstacles.size());
  std::vector<std::pair<int, int>>(
      pillar_semantic_result.rows * pillar_semantic_result.cols,
      std::pair<int, int>({-1, -1}))
      .swap(pillar_rc_to_obstacle_rc_);
  obstacle_rc_coord_converter_.InitializeWithPose(pose);
  pillar_rc_coord_converter_.InitializeWithPose(pose);
  for (const auto &obstacle : obstacles) {
    ComputeObstaclePillarType(obstacle);
  }
}

std::pair<int, int> PillarObstacleConverter::ConvertPillarRCToObstacleRC(
    const int pillar_row, const int pillar_col) {
  const int index = pillar_row * pillar_semantic_mat_.cols + pillar_col;
  QCHECK_LT(index, pillar_rc_to_obstacle_rc_.size());
  std::pair<int, int> &obstacle_rc = pillar_rc_to_obstacle_rc_[index];
  if (obstacle_rc == std::pair<int, int>({-1, -1})) {
    const Vec2d coord =
        pillar_rc_coord_converter_.RCToSmoothCoord({pillar_row, pillar_col});
    obstacle_rc = obstacle_rc_coord_converter_.CoordToRC(
        {static_cast<float>(coord.x()), static_cast<float>(coord.y())});
  }
  return {obstacle_rc.first, obstacle_rc.second};
}

void PillarObstacleConverter::ComputeObstaclePillarType(const Obstacle &obs) {
  const auto [row, col] =
      pillar_rc_coord_converter_.SmoothCoordToRC(obs.coord());
  obstacle_rc_to_pillar_rc_[{obs.row, obs.col}] = {row, col};
  if (row >= 0 && col >= 0 && row < pillar_semantic_mat_.rows &&
      col < pillar_semantic_mat_.cols) {
    const int index = row * pillar_semantic_mat_.cols + col;
    QCHECK_LT(index, pillar_rc_to_obstacle_rc_.size());
    pillar_rc_to_obstacle_rc_[index] = {obs.row, obs.col};
  }
}

}  // namespace qcraft
