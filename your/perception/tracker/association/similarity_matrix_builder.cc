#include "onboard/perception/tracker/association/similarity_matrix_builder.h"

#include <limits>
#include <utility>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "unsupported/Eigen/MatrixFunctions"

namespace qcraft::tracker {
namespace association {

double ComputeIoU(const Polygon2d& contour1, const Polygon2d& contour2) {
  Polygon2d intersection;
  if (!contour1.ComputeOverlap(contour2, &intersection)) {
    return 0.0;
  }
  const double intersection_area = intersection.area();
  return intersection_area /
         (contour1.area() + contour2.area() - intersection_area);
}

template <typename T, typename M>
bool SimilarityMatrixBuilder<T, M>::Init(
    const association::SimilarityMatrixBuilderProto& config) {
  // Gate
  QCHECK(config.has_gates());
  const auto& gates_proto = config.gates();
  const int gate_num = gates_proto.gate_params_size();
  priority_gates_.reserve(gate_num);
  for (size_t i = 0; i < gate_num; ++i) {
    priority_gates_.push_back(
        BuildGateFromProto<T, M>(gates_proto.gate_params(i)));
  }
  // Metric
  QCHECK(config.has_metrics());
  const auto& metric_proto = config.metrics();
  const int metric_num = metric_proto.metric_params_size();
  metrics_.reserve(metric_num);
  double metric_weight_sum = 0.0;
  for (int i = 0; i < metric_num; ++i)
    metric_weight_sum += metric_proto.metric_params(i).metric_weight();
  for (size_t i = 0; i < metric_num; ++i) {
    metrics_.push_back(
        {metric_proto.metric_params(i).metric_weight() / metric_weight_sum,
         BuildMetricFromType<T, M>(metric_proto.metric_params(i))});
  }

  config_ = config;
  return true;
}

template <typename T, typename M>
void SimilarityMatrixBuilder<T, M>::SafeSaveDebugInfo(
    TrackerDebugProto_AssociationDebugInfoProto_SimilarityInfoProto* log_info,
    TrackerDebugProto_AssociationDebugInfoProto* debug_info) const {
  std::lock_guard<std::mutex> lock(debug_info_mutex_);
  *debug_info->add_similarity_infos() = std::move(*log_info);
}

template <typename T, typename M>
double SimilarityMatrixBuilder<T, M>::CalculateSimilarity(
    const int measurement_id, const M& measurement_proto, const T& track,
    const mapping::LaneInfo* lane_info, SimilarityInfo* similarity_info,
    TrackerDebugProto_AssociationDebugInfoProto* debug_info) const {
  if (nullptr == similarity_info || nullptr == debug_info) return kIgnoreValue;

  const auto track_pos = track.track_state.estimator_3d.GetStatePos();
  Vec2d mea_pos;
  if (measurement_proto.has_laser_measurement()) {
    mea_pos = tracker_util::ComputeWeightedObstacleCentroid(
        measurement_proto.laser_measurement().cluster_measurement());
  } else if (measurement_proto.has_radar_measurement()) {
    mea_pos = Vec2dFromProto(measurement_proto.radar_measurement().pos());
  } else if (measurement_proto.has_camera3d_measurement()) {
    const Vec3d mea_pos_3d =
        Vec3dFromProto(measurement_proto.camera3d_measurement().pos());
    mea_pos = mea_pos_3d.block<2, 1>(0, 0);
  } else {
    QLOG(ERROR) << "Unknown measurement type";
  }

  constexpr double kDebugDist2 = 255.;
  bool add_debug_info =
      (track_pos - mea_pos).squaredNorm() < kDebugDist2 ? true : false;
  TrackerDebugProto_AssociationDebugInfoProto_SimilarityInfoProto info;

  if (add_debug_info) {
    info.set_track_id(track.track_state.id);
    info.set_measurement_id(measurement_id);
    Vec2dToProto(track_pos, info.mutable_track_centroid());
    Vec2dToProto(mea_pos, info.mutable_measurement_centroid());
    const auto predict_state =
        tracker_util::SafePredictPos(track, measurement_proto.timestamp());
    const auto predict_offset =
        predict_state.has_value()
            ? predict_state->GetStatePos() -
                  track.track_state.estimator_3d.GetStatePos()
            : Vec2d(0., 0.);
    Vec2dToProto(predict_offset, info.mutable_predicted_vector());
  }

  // Gating
  for (const auto& gate : priority_gates_) {
    const auto [pass_gate, gate_val] = gate->Gating(measurement_proto, track);
    if (add_debug_info) {
      info.add_gate_vals(gate_val);
    }
    if (!pass_gate) {
      if (add_debug_info) {
        info.set_pair_type(TrackerDebugProto_AssociationDebugInfoProto::NOBODY);
        SafeSaveDebugInfo(&info, debug_info);
      }
      return kIgnoreValue;
    }
  }
  // Metric
  double similarity = 0.0;
  for (const auto& metric : metrics_) {
    const double metric_val =
        metric.second->Similarity(measurement_proto, track);
    info.add_metric_vals(metric_val);
    similarity += metric.first * metric_val;
  }

  // Reject
  // TODO(jingwei) Carefully adjust reject strategy after fused metric.
  if (lane_info && measurement_proto.has_laser_measurement()) {
    if (lane_info->speed_limit < 70.0 / 3.6 ||
        track.track_state.life_state == TrackLifeState::kConfirmed) {
      // Reject distance is 3m.
      constexpr double k3mSimilarity = 0.91 * 0.5;
      if (similarity < k3mSimilarity) {
        info.set_pair_type(TrackerDebugProto_AssociationDebugInfoProto::NOBODY);
        SafeSaveDebugInfo(&info, debug_info);
        return kIgnoreValue;
      }
    }
  }

  // {T, M} pair whose similairty > 0 is real candidate.
  if (similarity > 0.0) {
    info.set_pair_type(TrackerDebugProto_AssociationDebugInfoProto::CANDIDATE);
    // We set track priority by adding different positive values to similarity.
    // For LVR, LV, LR, LO, we set priority as 1st. (Add kPriority1st)
    // For others, we add 0.
    // TODO(jingwei) Improve priority association.
    constexpr double kPriority1st = 50.0;
    if (TMST_LO == track.track_state.measurement_source_type ||
        TMST_LV == track.track_state.measurement_source_type ||
        TMST_LR == track.track_state.measurement_source_type ||
        TMST_LVR == track.track_state.measurement_source_type)
      similarity += kPriority1st;
  }
  SafeSaveDebugInfo(&info, debug_info);

  return similarity;
}

template <typename T, typename M>
bool SimilarityMatrixBuilder<T, M>::BuildSimilarityMatrix(
    const double timestamp, const std::vector<const M*>& measurement_protos,
    const std::vector<std::shared_ptr<T>>& tracks,
    const CoordinateConverter* coordinate_converter,
    const SemanticMapManager* semantic_map_manager,
    Eigen::MatrixXd* similarity_matrix,
    TrackerDebugProto_AssociationDebugInfoProto* debug_proto) const {
  SCOPED_QTRACE("SimilarityMatrixBuilder::BuildSimilarityMatrix");

  if (nullptr == similarity_matrix || nullptr == debug_proto) return false;
  for (const auto& ptr : measurement_protos)
    if (!ptr) return false;

  const int row = measurement_protos.size();
  const int col = tracks.size();
  similarity_matrix->resize(row, col);
  similarity_matrix->setZero();

  ParallelFor(0, tracks.size(), thread_pool_, [&](int i) {
    const auto& track = *tracks[i];
    const auto track_pos = track.track_state.estimator_3d.GetStatePos();
    const auto* lane_ptr =
        coordinate_converter && semantic_map_manager
            ? semantic_map_manager->GetNearestLaneInfoAtLevel(
                  coordinate_converter->GetLevel(), track_pos)
            : nullptr;

    const int num_measurements = measurement_protos.size();
    for (int j = 0; j < num_measurements; ++j) {
      const auto* measurement_proto = measurement_protos[j];
      QCHECK(!measurement_proto->has_camera_measurement());

      SimilarityInfo similarity_info;
      (*similarity_matrix)(j, i) =
          CalculateSimilarity(j, *measurement_proto, track, lane_ptr,
                              &similarity_info, debug_proto);
    }
  });

  return true;
}

template class SimilarityMatrixBuilder<Track<TrackState>, MeasurementProto>;
template class SimilarityMatrixBuilder<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;

}  // namespace association
}  // namespace qcraft::tracker
