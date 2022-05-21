#ifndef ONBOARD_PERCEPTION_VISION_MODULE_H_
#define ONBOARD_PERCEPTION_VISION_MODULE_H_

#include <map>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "boost/circular_buffer.hpp"
#include "glog/logging.h"
#include "onboard/camera/camera_params.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/maps/imagery_manager.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/nets/depthnet.h"
#include "onboard/nets/panonet.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/lane_detector/lane_detector_adapter.h"
#include "onboard/perception/mono3d/mono3d_adapter.h"
#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine.h"
#include "onboard/perception/multi_camera_fusion/single_camera_tracker.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/tracker/track_classifier/camera_embedding_manager.h"
#include "onboard/perception/tracker/track_classifier/visual_feature_extractor.h"
#include "onboard/perception/traffic_light/traffic_light_classifier.h"
#include "onboard/perception/traffic_light/traffic_light_decider.h"

namespace qcraft {

class VisionModule : public LiteModule {
 public:
  explicit VisionModule(LiteClientBase* lite_client);
  ~VisionModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  // The main logic to process each image upon arrival.
  void OnImage(const CameraImage& image);

  bool SetImagePose(CameraImage* image);

  void CollectCamerasForMultiCameraPipelines();
  void DetectAndClassifyTrafficLights(
      const std::map<CameraId, CameraImage>& camera_images_for_tl,
      const std::map<CameraId, SemanticSegmentationResultProto>&
          semantic_segmentation_results_map);

  void RunDepthNetOnImage(const CameraImage& image);
  void RunPanoNetOnImage(const CameraImage& image);
  std::optional<multi_camera_fusion::TrackerInputVariable> RunMono3dNetOnImage(
      const CameraImage& image);
  void RunLaneNetOnImage(const CameraImage& image);
  void DetectTrafficLight(const CameraImage& image);
  void ComputePanopticSegmentation(const CameraImage& image);
  void EstimateDepth(const CameraImage& image);
  void DetectAndTrackObjects(const CameraImage& image);
  void TrackObjects(const CameraImage& image,
                    std::optional<multi_camera_fusion::TrackerInputVariable>
                        curr_mono3d_input);
  void DetectLanes(const CameraImage& image);

  void UpdatePose(std::shared_ptr<const PoseProto> pose);

  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);

  void UpdateLidarHostTimeDiff(
      std::shared_ptr<const LidarHostTimeDiffProto> lidar_host_time_diff_proto);

  void UpdatePoseCorrection(
      std::shared_ptr<const PoseCorrectionProto> pose_correction_proto);

  // Publish traffic light regions to render them on images.
  void ComputeAndPublishTrafficLightRegion(const Spin& spin,
                                           const VehiclePose& pose);

  // Classify traffic lights.
  void ClassfiyTrafficLights(const VehiclePose& localized_pose,
                             double timestamp);

  void UpdateCameraLidarParams();

  void ExtractAndSetCameraFeatureEmbeddings(
      const std::unordered_map<CameraId, CameraImage>& images,
      VisualNetDetectionsProto* visual_net_detections);

  void CreateCameraMeasurementsFromMono3d(
      const MultiCameraMono3dMeasurementsProto& mono3d_measurements,
      MeasurementsProto* measurements_proto);
  double SetCriticalRegionHeading(const Mono3dMeasurementProto& m);
  bool HasOverlapWithCautiousRegion(const Mono3dMeasurementProto& m);
  bool IsOnRoadPoint(const Vec2d& pos_smooth);
  void PreloadImagery();
  // Camera params.
  CameraParamsMap camera_params_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;

  std::unique_ptr<TrafficLightClassifier> tl_classifier_;
  std::unique_ptr<TrafficLightDecider> tl_decider_;
  std::unique_ptr<panonet::PanoNet> pano_net_;
  std::unique_ptr<depthnet::DepthNet> depth_net_;
  std::unique_ptr<Mono3DAdapter> mono3d_adapter_ = nullptr;
  std::unique_ptr<LaneDetectorAdapter> lane_detector_adapter_ = nullptr;
  std::unique_ptr<tracker::VisualFeatureExtractor> visual_feature_extractor_ =
      nullptr;
  uint8_t* pano_net_input_image_data_pinned_ = nullptr;
  uint8_t* depth_net_input_image_data_pinned_ = nullptr;
  uint8_t* depth_net_output_image_data_pinned_ = nullptr;

  // Pose runs 100Hz and for now keep 2s history.
  boost::circular_buffer<std::pair<double, VehiclePose>> pose_history_{200};
  // Subscribed time difference from spin_publisher.
  double lidar_host_time_diff_ = 0.0;
  // Pose correction published from perception module at 10Hz.
  boost::circular_buffer<std::pair<double, PoseCorrectionProto>>
      pose_correction_history_{10};

  // ThreadPool with 1 thread each for each components in each camera.
  std::map<CameraId, ThreadPool> visual_net_loop_executor_thread_pool_;
  ThreadPool segmentation_loop_executor_thread_pool_;
  ThreadPool traffic_light_loop_executor_thread_pool_;
  ThreadPool depth_estimation_loop_executor_thread_pool_;
  ThreadPool mono3d_thread_pool_;
  ThreadPool lane_detector_loop_executor_thread_pool_;

  // Note(zheng): ordering marks time of arrival in ascending order.
  std::vector<CameraId> cameras_for_mf_in_yaw_order_;

  std::set<CameraId> cameras_for_tl_;
  std::set<CameraId> cameras_for_segmentation_;
  std::set<CameraId> cameras_for_mono3d_;
  std::set<CameraId> cameras_for_lane_;
  std::set<CameraId> cameras_for_depth_estimation_;

  // 10 mono3d result buffers for frame descriptor 0 to 9, storing camera ids
  // corresponding to that frame descriptor.
  absl::Mutex mono3d_mutex_;
  std::vector<std::map<
      CameraId, std::optional<multi_camera_fusion::TrackerInputVariable>>>
      mono3d_result_frames_for_mf_ GUARDED_BY(mono3d_mutex_){10};

  // 10 camera image buffers for frame descriptor 0 to 9, storing image frames
  // corresponding to that frame descriptor.
  std::vector<std::map<CameraId, CameraImage>> camera_image_frames_for_tl_{10};

  // Used to join the futures when exit.
  Future<void> future_tl_;
  std::map<CameraId, Future<void>> future_pool_segmentation_;
  std::map<CameraId, Future<void>> future_pool_mono3d_;
  std::map<CameraId, Future<void>> future_pool_lane_;
  std::map<CameraId, Future<void>> future_pool_depth_estimation_;

  // The latest pano net result.
  absl::Mutex panonet_results_mutex_;
  std::map<CameraId, SemanticSegmentationResultProto> latest_panonet_results_
      GUARDED_BY(panonet_results_mutex_);

  tracker::CameraEmbeddingManager camera_embedding_manager_;

  // Localization transform between smooth and [local or global]
  CoordinateConverter coordinate_converter_;

  SemanticMapManager semantic_map_manager_;
  ThreadPool semantic_map_update_thread_pool_;

  RunParamsProtoV2 run_params_;

  ThreadPool multi_camera_fusion_thread_pool_;
  std::unique_ptr<multi_camera_fusion::MultiCameraFusionEngine>
      multi_camera_fusion_engine_;
  // A thread pool to preload imagery patches.
  ThreadPool imagery_preloading_thread_pool_;
  qcraft::ImageryManager imagery_manager_;
  qcraft::LocalImagery local_imagery_;
};

REGISTER_LITE_MODULE(VisionModule);

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_VISION_MODULE_H_
