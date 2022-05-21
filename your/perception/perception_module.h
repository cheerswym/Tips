#ifndef ONBOARD_PERCEPTION_PERCEPTION_MODULE_H_
#define ONBOARD_PERCEPTION_PERCEPTION_MODULE_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "glog/logging.h"
#include "offboard/labeling/label_frame_util.h"
#include "offboard/labeling/proto/filtering.pb.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/nets/ace_associator.h"
#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/nets/image_patch_classifier.h"
#include "onboard/nets/mist_obstacle_net.h"
#include "onboard/nets/visual_net_classifier.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/cluster_filter.h"
#include "onboard/perception/cluster_observer.h"
#include "onboard/perception/human_pipeline_manager.h"
#include "onboard/perception/obstacle_clusterer.h"
#include "onboard/perception/obstacle_detector.h"
#include "onboard/perception/obstacle_semantic_manager.h"
#include "onboard/perception/registration/icp_tracker.h"
#include "onboard/perception/retroreflector_detector.h"
#include "onboard/perception/segmentation/segmenter.h"
#include "onboard/perception/semantic_segmentation_result.h"
#include "onboard/perception/sensor_fov/sensor_fov_builder.h"
#include "onboard/perception/tracker/track_classifier/laser_embedding_manager.h"
#include "onboard/perception/traffic_light/traffic_light_classifier.h"
#include "onboard/proto/radar.pb.h"

namespace qcraft {

class PerceptionModule : public LiteModule {
 public:
  explicit PerceptionModule(LiteClientBase* lite_client);
  ~PerceptionModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

  void ProcessSpins(const std::vector<LidarFrame>& lidar_frames);

 private:
  using BoxWithScore = std::pair<Box2d, float>;

  // Run a work loop in which the latest spin is fetched and processed. When
  // try_once is true (used in simulation), at most one spin is processed.
  void RunWorkLoop(bool try_once);

  void PreloadImagery();

  // Push a task into the task queue, which will be run in sequential with
  // ProcessSpin().
  void AppendTaskToQueue(std::function<void()>&& task,
                         const std::string& task_field,
                         const std::string& task_channel = "");
  void RunAllTasksInQueue();

  std::map<LidarId, LidarParametersProto> GetCurrentLidarParams() const;

  void UpdatePose(std::shared_ptr<const PoseProto> pose);

  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);

  void UpdateLidarHostTimeDiff(
      std::shared_ptr<const LidarHostTimeDiffProto> lidar_host_time_diff_proto);

  void UpdateSemanticSegmentationResults(
      std::shared_ptr<const SemanticSegmentationResultsProto> ss_results);

  void UpdateLabels(
      std::shared_ptr<const labeling::LabelFrameProto> label_frame);
  // For backward compatible, only used in offboard.
  // NOTE(zheng): The subscribe callback function do not support overload,
  // so we name two different func name.
  void UpdateRadarMeasurementsDeprecated(
      std::shared_ptr<const RadarMeasurementsProto> radar_measurements_proto);
  void UpdateRadarMeasurements(
      std::shared_ptr<const MeasurementsProto> measurements_proto);

  std::unordered_map<RadarId, std::vector<RadarMeasurementProto>>
  CollectRadarMeasurements(const double timestamp);

  void PublishObstacles(const ObstaclePtrs& obstacles);

  // Compute the transformation from vehicle to the given camera.
  AffineTransformation ComputeVehicleToCameraTransform(
      CameraId camera_id) const;

  SemanticSegmentationResults CollectSyncedSemanticSegmentationResults(
      const std::vector<LidarFrame>& lidar_frames,
      const double obsolete_threshold) const;
  // For backward compatible, only used in offboard simualtion.
  void AddMeasurementsFromRadar(
      const RadarMeasurementsProto& radar_measurements, double* min_timestamp,
      double* max_timestamp, MeasurementsProto* measurements);

  void PublishDetectionResult(
      const FieryEyeNetClassifier::DetectionResult& fen_result,
      const HumanPipelineManager::PedFilteringResult& ped_filtering_result,
      const double lidar_timestamp, const VehiclePose& pose);

  void FilterReflectionsBySensorFov(SegmentedClusters* segmented_clusters,
                                    SegmentedClusters* ignored_clusters);
  void FilterReflections(SegmentedClusters* segmented_clusters,
                         SegmentedClusters* ignored_clusters);

  void CreateMeasurementsFromCamera(
      const VisualNetDetectionProto& vn_result, const ObstaclePtrs& obstacles,
      const std::map<CameraId,
                     boost::circular_buffer<CameraImageWithTransform>>&
          camera_images,
      MeasurementsProto* measurements) const;

  void CreateMeasurementsFromCamera(const SemanticSegmentationResult& ss_result,
                                    const ObstaclePtrs& obstacles,
                                    MeasurementsProto* measurements) const;

  void CreateMeasurementsFromLaser(
      const VehiclePose& pose, const SegmentedClusters& segmented_clusters,
      const std::vector<tracker::LaserEmbeddingManager::LaserFeatureEmbedding>&
          laser_feature_embeddings,
      const std::vector<std::vector<float>>& association_feature_embeddings,
      MeasurementsProto* measurements) const;

  void GetAssociationFeaturesFromLaser(
      std::vector<std::vector<float>>* association_feature_embeddings);

  void EmitLidarFrameBacklogIssue(
      const LidarFrame& lidar_frame,
      const boost::circular_buffer<LidarFrame>& lidar_frame_buffer)
      EXCLUSIVE_LOCKS_REQUIRED(lidar_frame_mutex_);

  void PublishCanvas();

  std::unique_ptr<IcpTracker> icp_tracker_;

  absl::Mutex lidar_frame_mutex_;
  absl::CondVar lidar_frame_cond_var_ GUARDED_BY(lidar_frame_mutex_);
  bool lidar_frame_cond_var_is_waiting_ = false;
  // Stores latest lidar frame from each lidar.
  std::map<LidarId, boost::circular_buffer<LidarFrame>> lidar_frame_buffer_
      GUARDED_BY(lidar_frame_mutex_);
  struct LidarInfo {
    enum Status {
      kInactive = 0,
      kHealthy,
      kUnhealthy,
    };
    std::string StatusString() const;

    Status status = kInactive;
    double latest_start_timestamp = 0.0;
  };
  std::map<LidarId, LidarInfo> lidar_info_map_ GUARDED_BY(lidar_frame_mutex_);

  std::unique_ptr<ObstacleDetector> obstacle_detector_;
  std::unique_ptr<ObstacleClusterer> obstacle_clusterer_;
  std::unique_ptr<ObstacleSemanticManager> obstacle_semantic_manager_;
  std::unique_ptr<RetroreflectorDetector> retroreflector_detector_;
  std::unique_ptr<segmentation::Segmenter> segmenter_;

  RunParamsProtoV2 run_params_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;
  CameraParamsMap camera_params_;

  // Camera images with corresponding smooth to camera transforms.
  std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>
      camera_images_;

  SemanticMapManager semantic_map_manager_;

  std::unique_ptr<HumanPipelineManager> human_pipeline_manager_;

  std::unique_ptr<FieryEyeNetClassifier> fen_classifier_;
  std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame_;

  std::unique_ptr<ClusterFilter> cluster_filter_;
  std::unique_ptr<sensor_fov::SensorFovBuilder> sensor_fov_builder_;
  std::unique_ptr<ClusterObserver> cluster_observer_;
  std::unique_ptr<MistObstacleNet> mof_net_;

  // Transformation interface to turn between smooth and global.
  CoordinateConverter coordinate_converter_;

  // Used to stop the perception thread.
  absl::Notification stop_notification_;

  ThreadPool serial_executor_thread_pool_;
  ThreadPool* serial_executor_ = nullptr;
  mutable ThreadPool thread_pool_;
  // A thread pool to run DNN classifiers.
  ThreadPool nn_thread_pool_;
  // A thread pool to preload imagery patches.
  ThreadPool imagery_preloading_thread_pool_;

  Future<void> serial_executor_future_;

  // Tasks that will be run in the serial executor (in sequential with
  // ProcessSpin).
  absl::Mutex queued_tasks_mutex_;
  std::map<std::string, std::queue<std::pair<double, std::function<void()>>>>
      queued_tasks_of_msgs_ GUARDED_BY(queued_tasks_mutex_);

  PoseProto latest_pose_;
  // Pose runs 100Hz and for now keep 1s history.
  boost::circular_buffer<std::pair<double, VehiclePose>> pose_history_{100};
  // Subscribed time difference from spin_publisher
  double lidar_host_time_diff_ = 0.0;

  // Radar measurements proto.
  static constexpr double kRadarMeasurementsBufferLength = 1.0;  // s
  std::map<RadarId, HistoryBuffer<std::vector<RadarMeasurementProto>>>
      radar_measurements_buffer_;

  ImageryManager imagery_manager_;
  LocalImagery local_imagery_{&imagery_manager_};

  std::unique_ptr<tracker::LaserEmbeddingManager> laser_embedding_manager_;

  static constexpr double kSemanticSegmentationResultBufferLength = 1.0;  // s
  std::map<CameraId, HistoryBuffer<SemanticSegmentationResultRef>>
      semantic_segmentation_results_buffer_;

  std::unique_ptr<AceAssociator> ace_associator_;
};

REGISTER_LITE_MODULE(PerceptionModule);

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_PERCEPTION_MODULE_H_
