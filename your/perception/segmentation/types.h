#ifndef ONBOARD_PERCEPTION_SEGMENTATION_TYPES_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_TYPES_H_

#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/human_pipeline_manager.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/perception/retroreflector.h"
#include "onboard/proto/camera.pb.h"
#include "onboard/proto/lidar.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::segmentation {
struct Context {
  FieryEyeNetClassifier::DetectionResult fiery_eye_net_result;
  HumanPipelineManager::PedFilteringResult ped_filtering_result;
  SemanticSegmentationResults ll_net_result;
  absl::flat_hash_map<LidarId, RangeImage> range_images;
  // Only save front radar measurements.
  std::vector<RadarMeasurementProto> front_radar_measurements;
  Retroreflectors retroreflectors;
};

inline std::ostream& operator<<(std::ostream& os, const ProposerType type) {
  return os << ProposerType_Name(type);
}

inline std::ostream& operator<<(std::ostream& os,
                                const ProposedProperty property) {
  return os << ProposedProperty_Name(property);
}

class ProposerEnvInfo {
 public:
  ProposerEnvInfo(const VehiclePose& pose,
                  const ObstacleManager& obstacle_manager,
                  const SemanticMapManager& semantic_map_manager,
                  const LocalImagery& local_imagery,
                  const CoordinateConverter& coordinate_converter,
                  const RunParamsProtoV2& run_params, const Context& context);

  ProposerEnvInfo() = delete;

  const VehiclePose& pose() const { return pose_.get(); }
  const ObstacleManager& obstacle_manager() const {
    return obstacle_manager_.get();
  }
  const SemanticMapManager& semantic_map_manager() const {
    return semantic_map_manager_.get();
  }
  const LocalImagery& local_imagery() const { return local_imagery_; }
  const CoordinateConverter& coordinate_converter() const {
    return coordinate_converter_;
  }
  const RunParamsProtoV2& run_params() const { return run_params_.get(); }
  const VehicleParamApi vehicle_params() const {
    return run_params_.get().vehicle_params();
  }
  const Context& context() const { return context_.get(); }

  const std::map<LidarId, LidarParametersProto>& lidar_params_map() const {
    return lidar_params_map_;
  }
  const CameraParamsMap& camera_params_map() const { return camera_params_; }

  const Box2d& av_box() const { return av_box_; }
  float av_height() const { return av_height_; }

  const LidarParametersProto* GetLidarParams(const LidarId lidar_id) const {
    return FindOrNull(lidar_params_map_, lidar_id);
  }

  const CameraParams* GetCameraParams(const CameraId camera_id) const {
    return FindOrNull(camera_params_, camera_id);
  }

  const std::vector<Polygon2d>& GetNearPerceptionZones(
      mapping::PerceptionZoneProto::Type type) const {
    return perception_zones_array_[static_cast<int>(type)];
  }

  bool PointInZones(const Vec2d& point,
                    mapping::PerceptionZoneProto::Type type) const;

 private:
  void CollectNearPerceptionZones();

  void ComputeAvBox();
  void ComputeAvHeight();

 private:
  std::reference_wrapper<const VehiclePose> pose_;
  std::reference_wrapper<const ObstacleManager> obstacle_manager_;
  std::reference_wrapper<const SemanticMapManager> semantic_map_manager_;
  std::reference_wrapper<const LocalImagery> local_imagery_;
  std::reference_wrapper<const CoordinateConverter> coordinate_converter_;
  std::reference_wrapper<const RunParamsProtoV2> run_params_;
  std::reference_wrapper<const Context> context_;

  std::map<LidarId, LidarParametersProto> lidar_params_map_;
  CameraParamsMap camera_params_;
  std::array<std::vector<Polygon2d>,
             mapping::PerceptionZoneProto::Type_ARRAYSIZE>
      perception_zones_array_;
  Box2d av_box_;
  float av_height_ = -1.0f;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_TYPES_H_
