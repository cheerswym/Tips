#ifndef ONBOARD_PERCEPTION_TRACKER_MODULE_H_
#define ONBOARD_PERCEPTION_TRACKER_MODULE_H_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "boost/circular_buffer.hpp"
#include "offboard/labeling/label_frame_util.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/lite/lite_module.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/tracker/tracker.h"

namespace qcraft {
struct TrackerInputVariables {
  // Tracker input variables.
  VehiclePose pose;
  CoordinateConverter coordinate_converter;
  std::shared_ptr<const MeasurementsProto> measurements;
  std::shared_ptr<const labeling::LabelFrameProto> label_frame;
  double min_timestamp;
  double max_timestamp;
  std::shared_ptr<const SensorFovsProto> sensor_fovs_proto;
  std::map<CameraId, CameraImageWithTransform> camera_images;
};

class TrackerModule : public LiteModule {
 public:
  explicit TrackerModule(LiteClientBase* lite_client);

  void OnInit() override{};
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override{};

 private:
  void UpdateSensorFov(
      std::shared_ptr<const SensorFovsProto> sensor_fovs_proto);

  void UpdatePose(std::shared_ptr<const PoseProto> pose);

  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);

  void UpdateLabels(
      std::shared_ptr<const labeling::LabelFrameProto> label_frame);

  void UpdateMeasurementsProto(
      std::shared_ptr<const MeasurementsProto> measurements_proto);

  void UpdateImage(const CameraImage& camera_image);

  void UpdateCameraLidarParams();

  void ProcessAllMeasurementGroupsInQueue(
      std::deque<TrackerInputVariables> tracker_input_queue,
      double trigger_time, std::string trigger_type);
  std::map<CameraId, CameraImageWithTransform> GetBestMatchImageForCluster(
      std::shared_ptr<const MeasurementsProto> measurements_proto);
  std::unique_ptr<tracker::Tracker> tracker_;

  // Pose runs 100Hz and for now keep 0.2s history.
  boost::circular_buffer<std::pair<double, VehiclePose>> pose_history_{20};

  boost::circular_buffer<std::shared_ptr<const SensorFovsProto>>
      sensor_fovs_proto_history_{10};

  // Camera images with corresponding smooth to camera transforms.
  std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>
      camera_images_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;

  CoordinateConverter coordinate_converter_;

  std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame_;
  SemanticMapManager semantic_map_manager_;

  std::deque<TrackerInputVariables> measurement_group_queue_;
  double latest_trigger_time_ = 0.0;

  std::unique_ptr<ThreadPool> tracker_thread_pool_;
  Future<void> future_tracker_pipeline_;
};

REGISTER_LITE_MODULE(TrackerModule);
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TRACKER_MODULE_H_
