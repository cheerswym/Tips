#ifndef ONBOARD_PERCEPTION_OBSTACLE_DETECTOR_H_
#define ONBOARD_PERCEPTION_OBSTACLE_DETECTOR_H_

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "offboard/labeling/proto/filtering.pb.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/lidar_util.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/maps/imagery_manager.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/obstacle_grid.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/pose_correction.h"
#include "onboard/perception/registration/point_matcher_structs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

class ObstacleDetector {
 public:
  ObstacleDetector(const RunParamsProtoV2& run_params,
                   ImageryManager* imagery_manager, ThreadPool* thread_pool);

  ~ObstacleDetector();

  std::unique_ptr<ObstacleManager> DetectObstacles(
      const std::vector<LidarFrame>& lidar_frames, const VehiclePose& pose,
      const CoordinateConverter& coordinate_converter,
      const labeling::LabelFrameProto* latest_label_frame,
      const mapping::SemanticMapManager& semantic_map_manager,
      const VehicleParamApi& vehicle_params);

  // Return nullptr if the latest pose correction fails.
  const std::optional<VehiclePose>& pose_difference() const {
    return pose_difference_;
  }

  const std::optional<VehiclePose>& pose_correction_result() const {
    return pose_correction_result_;
  }

  double detection_region_width() const { return detection_region_width_; }
  double detection_region_height() const { return detection_region_height_; }

  int obstacle_grid_width() const { return obstacle_grid_.width(); }
  int obstacle_grid_height() const { return obstacle_grid_.height(); }

  PoseCorrectionDebugProto pose_correction_debug_proto() const {
    return pose_correction_.pose_correction_debug_proto();
  }

 private:
  void UpdateRunParams(const RunParamsProtoV2& run_params);
  ObstacleRefVector CreateObstaclesFromGrid(
      const VehiclePose& pose, const std::vector<LaserPoint>& points,
      const Vec2i row_col_offset, const AffineTransformation& pose_correction,
      const mapping::SemanticMapManager& semantic_map_manager);

  ObstacleRefVector PropagateGroundObstacles(
      const std::vector<LaserPoint>& points, ObstacleRefVector obstacles);

  // Extracts the distance to curb for the given obstacle grid cell.
  float ComputeDistToCurbForObstacleAt(int row, int col) const;

  // Return true if the point of the given index is added to an obstacle; false
  // otherwise.
  bool UpdateObstacleGrid(const std::pair<int, int>& rc, int point_index);

  std::optional<std::pair<int, int>> ComputeObstacleRC(
      const Box2d& av_box, const Box2d& detection_region, Vec2d obstacle_pos,
      float range) const;

  // Obstacles mist filter training data generation.
  std::optional<filtering::ObstacleDataProto::Type> MatchObstacleLabel(
      const Obstacle* obstacles, const labeling::LabelFrameProto& label_frame);
  filtering::PointDataProto ToPointDataProto(const VehiclePose& pose,
                                             const LaserPoint& point);
  void GenerateObstacleTrainingData(
      const VehiclePose& pose, const ObstaclePtrs& obstacles,
      const labeling::LabelFrameProto* label_frame,
      filtering::ObstaclesDataProto* obstacles_proto);

  const double detection_region_width_;
  const double detection_region_height_;

  ObstacleGrid obstacle_grid_;

  // This vector is used to hold laser points from all spins, and is reused in
  // each iteration in order to avoid repeated memory allocation.
  std::vector<LaserPoint> points_;

  PoseCorrection pose_correction_;

  std::optional<VehiclePose> pose_difference_;
  std::optional<VehiclePose> pose_correction_result_;

  ImageryManager* imagery_manager_;
  LocalImagery local_imagery_;
  // From run params
  CameraParamsMap camera_params_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;

  std::map<LidarId, lidar_util::LidarIgnoreTable> lidar_ignore_tables_;

  float av_height_ = -1.0f;
  // Not owned.
  ThreadPool* const thread_pool_;

  filtering::ObstaclesDataProto obstacles_proto_;
  double pre_label_time_ = 0.0;

  // Transform pose between smooth and [local or global]
  std::optional<CoordinateConverter> coordinate_converter_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_DETECTOR_H_
