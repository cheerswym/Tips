#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_SIMILARITY_MATRIX_BUILDER_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_SIMILARITY_MATRIX_BUILDER_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/tracker/association/gate.h"
#include "onboard/perception/tracker/association/metric.h"
#include "onboard/perception/tracker/association/proto/associator_config.pb.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {
namespace association {

template <typename TrackType, typename MeasurementType>
class SimilarityMatrixBuilder {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit SimilarityMatrixBuilder(ThreadPool* thread_pool)
      : thread_pool_(thread_pool) {}

  bool Init(const association::SimilarityMatrixBuilderProto& config);

  /*
   Row: measurements
   Column: tracks
  */
  bool BuildSimilarityMatrix(
      const double timestamp, const std::vector<const M*>& laser_measurements,
      const std::vector<std::shared_ptr<T>>& tracks,
      const CoordinateConverter* coordinate_converter,
      const SemanticMapManager* semantic_map_manager,
      Eigen::MatrixXd* similarity_matrix,
      TrackerDebugProto_AssociationDebugInfoProto* debug_proto) const;

 private:
  struct SimilarityInfo {
    // Squared distance between track centers.
    std::optional<double> centroid_dist_sqr;
    // Mahalanobis distance.
    std::optional<double> mahalanobis_distance;
    // IoU between two contours.
    std::optional<double> contour_iou;
    std::optional<double> contour_iop;
    // IoU between two bboxes.
    std::optional<double> bbox_iou;
    // IoU between two min_bboxes.
    std::optional<double> min_bbox_iou;
    // The elapsed time between two tracks.
    std::optional<double> time_diff;
    // Whether the measurement is onroad or offroad.
    std::optional<bool> onroad;
    // ICP match info.
    std::optional<double> mse;
  };

  void SafeSaveDebugInfo(
      TrackerDebugProto_AssociationDebugInfoProto_SimilarityInfoProto* log_info,
      TrackerDebugProto_AssociationDebugInfoProto* debug_info) const;

  double CalculateSimilarity(
      const int measurement_id, const M& measurement, const T& track,
      const mapping::LaneInfo* lane_info, SimilarityInfo* similarity_info,
      TrackerDebugProto_AssociationDebugInfoProto* debug_info) const;

  ThreadPool* const thread_pool_;

  static constexpr double kIgnoreValue = -1.0;
  std::optional<association::SimilarityMatrixBuilderProto> config_;

  using GatePtr = std::shared_ptr<Gate<T, M>>;
  std::vector<GatePtr> priority_gates_;
  using MetricPtr = std::shared_ptr<Metric<T, M>>;
  using WeightedMetric = std::pair<double, MetricPtr>;
  std::vector<WeightedMetric> metrics_;

  mutable std::mutex debug_info_mutex_;
};

}  // namespace association
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_SIMILARITY_MATRIX_BUILDER_H_
