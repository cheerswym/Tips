#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "leveldb/db.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "offboard/labeling/proto/track_classifier_label.pb.h"
#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/nets/tcn_net_classifier.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/perception/tracker/track_classifier/track_data_storage.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::tracker {

class TrackClassifier {
 public:
  explicit TrackClassifier(const SemanticMapManager* semantic_map_manager,
                           const ParamManager& param_manager,
                           ThreadPool* thread_pool);

  void ClassifyTracks(
      const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
      TrackClassifierDebugProto* track_classifier_debug_proto,
      std::vector<TrackRef>* tracks);

 private:
  using Feature = std::vector<float>;
  void MaybeSaveTrackData(
      const Track<TrackState>& track,
      const TrackClassifierDebugProto::TrackClassifierInfo&
          track_classifier_info,
      const VehiclePose& pose,
      std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame);
  void ClassifyTracksByModel(
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      TrackClassifierDebugProto* track_classifier_debug_proto,
      std::vector<TrackRef>* tracks);
  void ClassifyTracksByVoting(
      const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
      std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
      TrackClassifierDebugProto* track_classifier_debug_proto,
      std::vector<TrackRef>* tracks);
  void ClassifyWarningTriangle(
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      std::vector<TrackRef>* tracks);
  bool IsPossiblyWarningTriangle(
      const LaserMeasurementProto::ClusterMeasurement& cluster_m,
      MeasurementType type);
  void RunWarningTriangleClassifyModel(const std::vector<cv::Mat>& images,
                                       const std::vector<int>& index,
                                       std::vector<TrackRef>* tracks);
  MeasurementType GetTypeForUnknownObjectMeasurement(
      const Track<TrackState>& track, const MeasurementProto& measurement,
      const VehiclePose& pose);
  void RunSequenceModel(const std::vector<std::vector<Feature>>& input_feature,
                        const std::vector<int>& track_index,
                        std::vector<float>* out_cls_ptr,
                        TrackClassifierDebugProto* track_classifier_debug_proto,
                        std::vector<TrackRef>* tracks);
  void ApplyTcnLabel(
      const int label, const float max_score,
      TrackClassifierDebugProto::TrackClassifierInfo* track_classifier_info,
      Track<TrackState>* tracks);
  bool IsInTcnScope(const Track<TrackState>& track,
                    const MeasurementType& tcn_label, const float max_score);
  void SetDebugInfoFromPreviousResult(
      const Track<TrackState>& track,
      TrackClassifierDebugProto::TrackClassifierInfo* track_classifier_info);
  bool IsFlyingBird(const Track<TrackState>& track);
  bool IsFence(const Track<TrackState>& track);
  void CollectSequenceFeature(
      const HistoryBuffer<const MeasurementProto*>& m_history,
      std::vector<Feature>* feature_seq);
  track_classifier_data::TrackData UpdateTrackData(
      const Track<TrackState>& track, const std::string& db_key,
      const TrackClassifierDebugProto::TrackClassifierInfo&
          track_classifier_info,
      const VehiclePose& pose, track_classifier_label::Category category);
  const SemanticMapManager* const semantic_map_manager_;
  ThreadPool* thread_pool_ = nullptr;

  // LevelDB database to store training data.
  std::unique_ptr<TrackDataStorage> track_data_storage_;
  std::unique_ptr<TcnSequenceNetClassifier> tcn_sequence_net_classifier_;

  std::unique_ptr<TcnImageNetClassifier> warning_triangle_classifier_;
  static constexpr int kMaxBatchSize = 64;
  static constexpr int kWarningTriangleMaxBatchSize = 32;

  DISALLOW_COPY_AND_ASSIGN(TrackClassifier);
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_H_
