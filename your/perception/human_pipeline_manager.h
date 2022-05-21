#ifndef ONBOARD_PERCEPTION_HUMAN_PIPELINE_MANAGER_H_
#define ONBOARD_PERCEPTION_HUMAN_PIPELINE_MANAGER_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "leveldb/db.h"
#include "offboard/labeling/proto/patch_classifier_data.pb.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/image_util.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_module.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/nets/image_patch_classifier.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/nets/trt/pcn_net.h"
#include "onboard/params/utils/param_util.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/human_pipeline_tracker.h"
#include "onboard/perception/laser_point.h"
#include "onboard/perception/obstacle.h"
#include "opencv2/core.hpp"

namespace qcraft {

class HumanPipelineManager {
 public:
  struct PedFilteringResult {
    std::vector<std::pair<FieryEyeNetClassifier::DetectionBox, float>>
        deleted_ped_boxes;
    // The following includes cyclists and motorcyclists.
    std::vector<std::pair<FieryEyeNetClassifier::DetectionBox, float>>
        deleted_cyc_boxes;

    std::vector<std::string> ped_skip_infos;
    std::vector<std::string> cyc_skip_infos;
    std::vector<std::string> ped_track_infos;
    std::vector<std::string> cyc_track_infos;
  };

  explicit HumanPipelineManager(LiteClientBase* lite_client,
                                ThreadPool* thread_pool);

  void InitImagePatchClassifier(LiteClientBase* lite_client,
                                const RunParamsProtoV2& run_params,
                                const NetParam& image_patch_net_param);

  // Filter FEN pedestrian & cyclist detections by image patch classifier.
  // Return the deleted pedestrian & cyclist boxes for visualization.
  PedFilteringResult FilterPedBoxes(
      const ObstaclePtrs& obstacles,
      const std::vector<LidarFrame>& lidar_frames,
      const std::map<CameraId,
                     boost::circular_buffer<CameraImageWithTransform>>&
          camera_images,
      double lidar_host_time_diff,
      std::vector<FieryEyeNetClassifier::DetectionBox>* ped_boxes,
      std::vector<FieryEyeNetClassifier::DetectionBox>* cyc_boxes) const;

 private:
  void FilterOneBox(
      const ObstaclePtrs& obstacles,
      const std::vector<LidarFrame>& lidar_frames,
      const std::unordered_map<LidarId, const LidarFrame*>& lidar_frame_map,
      const std::map<CameraId,
                     boost::circular_buffer<CameraImageWithTransform>>&
          camera_images,
      double lidar_host_time_diff, const Box2d& box, int index,
      std::vector<std::string>* pcn_skip_infos,
      std::vector<std::tuple<cv::Mat, PcnPoints, PcnRegionProto>>*
          ped_cyc_patches) const;

  void SampleBoxPoints(
      const Box2d& box, const ObstaclePtrs& obstacles, int index,
      const std::unordered_map<LidarId, const LidarFrame*>& lidar_frame_map,
      float* real_ground_z, float* max_ped_distance,
      PcnPoints* sampled_points_in_box,
      std::vector<std::string>* pcn_skip_infos) const;

  void GetBestImagePatch(
      const std::vector<LidarFrame>& lidar_frames,
      const std::map<CameraId,
                     boost::circular_buffer<CameraImageWithTransform>>&
          camera_images,
      double lidar_host_time_diff, const Box2d& box, int index,
      const PcnPoints& sampled_points_in_box, const float real_ground_z,
      const float max_ped_distance,
      std::tuple<CameraId, double, cv::Mat, cv::Rect>*
          expand_patch_for_labeling,
      std::vector<std::string>* pcn_skip_infos,
      std::vector<std::tuple<cv::Mat, PcnPoints, PcnRegionProto>>*
          ped_cyc_patches) const;

  void SetPatchResult(
      const std::vector<std::pair<ImagePatchClassifier::ClassificationResult,
                                  float>>& results,
      int num_ped_boxes, int num_boxes,
      const std::vector<int>& ped_cyc_patch_indices,
      std::vector<ImagePatchClassifier::ClassificationResult>*
          ped_cyc_box_types,
      std::vector<float>* ped_cyc_scores) const;

  void MaybeSaveDebugImage(
      const std::string& image_name,
      const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
      const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
          expand_patch_for_labeling) const;

  void MaybeSavePatchDataProto(
      int index, const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
      const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
          expand_patch_for_labeling) const;

  patch_classifier::PcnLabelingData AddPatchData(
      const std::string& db_key, int index,
      const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
      const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
          expand_patch_for_labeling) const;

  [[maybe_unused]] LiteClientBase* const lite_client_;
  ThreadPool* const thread_pool_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;
  std::unique_ptr<ImagePatchClassifier> image_patch_classifier_ = nullptr;
  std::unique_ptr<human_tracking::HumanTracker> human_tracker_ = nullptr;
  // LevelDB database to store training data.
  std::unique_ptr<leveldb::DB> data_db_ = nullptr;

  DISALLOW_COPY_AND_ASSIGN(HumanPipelineManager);
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_HUMAN_PIPELINE_MANAGER_H_
