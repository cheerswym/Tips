#ifndef ONBOARD_PERCEPTION_OBSTACLE_CLUSTERER_H_
#define ONBOARD_PERCEPTION_OBSTACLE_CLUSTERER_H_

#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/obstacle_grid.h"
#include "onboard/utils/time_util.h"
#include "opencv2/core.hpp"

namespace qcraft {

class ObstacleClusterer {
 public:
  enum class FloodFilling { kNaive, kScanline };

  ObstacleClusterer(int width, int height, ThreadPool* thread_pool);

  ~ObstacleClusterer() { WaitForFuture(reset_future_); }

  // Generate clusters from the input obstacles.
  ClusterVector ClusterObstacles(
      const ObstaclePtrs& obstacles,
      const ObstacleRCCoordConverter& rc_coord_converter,
      const VehiclePose& pose,
      const FloodFilling flood_filling = FloodFilling::kNaive);

 private:
  void NaiveFloodFilling(const ObstaclePtrs& obstacles,
                         ClusterVector* clusters) const;
  void ScanlineFloodFilling(ClusterVector* clusters) const;

  const int width_;
  const int height_;

  // Index obstacles in col/row for fast obstacle locating.
  ObstaclePtrs obstacles_in_grid_;

  // Each pixel represents an obstacle, and we use this image to cluster
  // obstacles.
  cv::Mat obstacle_image_;

  // Used to binarize the obstacle image.
  cv::Mat threshold_image_;

  // A future used to reset obstacles_in_grid_ and obstacle_image_.
  Future<void> reset_future_;

  ThreadPool* const thread_pool_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_CLUSTERER_H_
