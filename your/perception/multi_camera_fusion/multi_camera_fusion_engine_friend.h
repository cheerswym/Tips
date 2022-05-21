#ifndef MULTI_CAMERA_FUSION_MULTI_CAMERA_FUSION_ENGINE_FRIEND_H_
#define MULTI_CAMERA_FUSION_MULTI_CAMERA_FUSION_ENGINE_FRIEND_H_

#include <memory>
#include <vector>

#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine.h"

namespace qcraft::multi_camera_fusion {

class MultiCameraFusionEngineFriend {
 public:
  explicit MultiCameraFusionEngineFriend(
      MultiCameraFusionEngine* multi_camera_fusion_engine) {
    fusion_engine_ = multi_camera_fusion_engine;
  }
  ~MultiCameraFusionEngineFriend() = default;

 public:
  void TrackSingleCameraObjects(
      const CameraId camera_id, const VehiclePose& pose, const double timestamp,
      std::shared_ptr<MeasurementsProto> measurement_group) {
    fusion_engine_->TrackSingleCameraObjects(camera_id, pose, timestamp,
                                             measurement_group);
  }

  void TrackMultiCameraObjects(const std::vector<CameraId>& processed_cameras,
                               const CoordinateConverter& coordinate_converter,
                               const double timestamp,
                               const VehiclePose& pose) {
    fusion_engine_->TrackMultiCameraObjects(
        processed_cameras, coordinate_converter, timestamp, pose);
  }

  void FuseMultiCameraTracks(const std::vector<CameraId>& processed_cameras,
                             const double timestamp, const VehiclePose& pose,
                             MeasurementsProto* measurements_proto) {
    fusion_engine_->FuseMultiCameraTracks(processed_cameras, timestamp, pose,
                                          measurements_proto);
  }
  uint32_t GenerateNewTrackId() { return fusion_engine_->GenerateNewTrackId(); }

  void RemoveExpiredTracks(double timestamp) {
    return fusion_engine_->RemoveExpiredTracks(timestamp);
  }

  const std::vector<TrackRef>& SingleCameraTracks(const CameraId camera_id) {
    return fusion_engine_->multi_camera_fusion_engine_[camera_id]->tracks();
  }

 private:
  MultiCameraFusionEngine* fusion_engine_;
};
}  // namespace qcraft::multi_camera_fusion

#endif  // MULTI_CAMERA_FUSION_MULTI_CAMERA_FUSION_ENGINE_FRIEND_H_
