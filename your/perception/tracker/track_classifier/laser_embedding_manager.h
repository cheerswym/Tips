#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_EMBEDDING_MANAGER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_EMBEDDING_MANAGER_H_

#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "leveldb/db.h"
#include "onboard/base/macros.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/global/trace.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/nets/tcn_net_classifier.h"
#include "onboard/nets/trt/tcn_net.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/tracker/track_classifier/laser_data_storage.h"
#include "onboard/perception/utils/timer.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::tracker {

// This class is used for extracting embeddings from clusters for track
// classifier, which is also going to be used for offboard data extraction.
class LaserEmbeddingManager {
 public:
  explicit LaserEmbeddingManager(const RunParamsProtoV2& run_params,
                                 const ParamManager& param_manager);

  // Embedding size for camera embeddings.
  struct LaserFeatureEmbedding {
    std::vector<float> point_feature;
    std::vector<float> image_feature;
  };

  using CameraImages =
      std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>;
  std::vector<LaserFeatureEmbedding> ExtractLaserFeatureEmbeddings(
      const SegmentedClusters& segmented_clusters,
      const CameraImages& camera_images, const VehiclePose& pose,
      const double lidar_host_time_diff);

  std::vector<std::pair<std::vector<const LaserPoint*>, ImagePatch>>&
  GetClusterDataVec() {
    return cluster_data_vec_;
  }

 private:
  void GetImagePatchForCluster(
      const std::vector<const LaserPoint*>& cluster_points,
      const CameraImages& camera_images, Polygon2d contour,
      const double cluster_ts, ImagePatch* image_patch);
  std::vector<LaserFeatureEmbedding> ExtractLaserFeatureEmbeddingsByTcn(
      const SegmentedClusters& segmented_clusters, const VehiclePose& pose,
      const std::vector<Polygon2d>& contours,
      const std::vector<std::pair<std::vector<const LaserPoint*>, ImagePatch>>&
          cluster_data_vec);
  std::vector<std::pair<float, int>> GetClusterRank(
      const SegmentedClusters& segmented_clusters, const VehiclePose& pose,
      const std::vector<Polygon2d>& contours);

  void RunImageModel(const std::vector<cv::Mat>& batch_patch,
                     const std::vector<int> object_index,
                     std::vector<LaserFeatureEmbedding>* feature_embeddings);
  void RunPointModel(
      const std::vector<std::vector<const LaserPoint*>>& input_laser_points,
      const std::vector<int> object_index,
      std::vector<LaserFeatureEmbedding>* feature_embeddings);

  static constexpr int kMaxBatchSize = 64;
  static constexpr int kMinPoint = 10;
  static constexpr int kImageMinSize = 10;

  std::unique_ptr<LaserDataStorage> laser_data_storage_;
  std::unique_ptr<TcnImageNetClassifier> tcn_image_net_classifier_;
  std::unique_ptr<TcnPointNetClassifier> tcn_point_net_classifier_;
  std::vector<std::pair<std::vector<const LaserPoint*>, ImagePatch>>
      cluster_data_vec_;
  DISALLOW_COPY_AND_ASSIGN(LaserEmbeddingManager);
};

}  // namespace qcraft::tracker

// NOLINTNEXTLINE
#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_EMBEDDING_MANAGER_H_
