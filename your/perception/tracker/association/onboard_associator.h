#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_ONBOARD_ASSOCIATOR_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_ONBOARD_ASSOCIATOR_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/tracker/association/match_solver.h"
#include "onboard/perception/tracker/association/proto/associator_config.pb.h"
#include "onboard/perception/tracker/association/similarity_matrix_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {
namespace association {

template <typename TrackType, typename MeasurementType>
class OnboardAssociator {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit OnboardAssociator(ThreadPool* thread_pool)
      : similarity_matrix_builder_(thread_pool) {}

  bool Init(std::string config_file, std::string msg);

  /* Association 1-measurement vs 1-track:
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  template <typename AssociationDebugInfoProto>
  std::vector<int> Association1v1(
      const double timestamp, const std::vector<const M*>& measurements,
      const std::vector<std::shared_ptr<T>>& tracks,
      const CoordinateConverter* coordinate_converter,
      const SemanticMapManager* semantic_map_manager,
      AssociationDebugInfoProto* asso_debug_proto) const {
    SCOPED_QTRACE("OnboardAssociator::Association1v1");

    if (nullptr == asso_debug_proto || measurements.empty() || tracks.empty()) {
      return {};
    }

    Eigen::MatrixXd similarity_matrix;
    similarity_matrix_builder_.BuildSimilarityMatrix(
        timestamp, measurements, tracks, coordinate_converter,
        semantic_map_manager, &similarity_matrix, asso_debug_proto);
    const std::vector<int> result = match_solver_.Solve(similarity_matrix);
    AddDebugMessage(measurements, tracks, result, asso_debug_proto);

    return result;
  }

  /* Association multi-measurement vs 1-track:
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  template <typename AssociationDebugInfoProto>
  std::vector<int> AssociationMv1(
      const double timestamp, const std::vector<const M*>& measurements,
      const std::vector<std::shared_ptr<T>>& tracks,
      const CoordinateConverter* coordinate_converter,
      const SemanticMapManager* semantic_map_manager,
      AssociationDebugInfoProto* asso_debug_proto) const {
    SCOPED_QTRACE("OnboardAssociator::AssociationMv1");

    if (nullptr == asso_debug_proto || measurements.empty() || tracks.empty()) {
      return {};
    }

    Eigen::MatrixXd similarity_matrix;
    similarity_matrix_builder_.BuildSimilarityMatrix(
        timestamp, measurements, tracks, coordinate_converter,
        semantic_map_manager, &similarity_matrix, asso_debug_proto);
    const std::vector<int> result = match_solver_.Solve(similarity_matrix);
    AddDebugMessage(measurements, tracks, result, asso_debug_proto);

    return result;
  }

  const association::AssociationProto& Config() const { return config_; }

 private:
  template <typename AssociationDebugInfoProto>
  void AddDebugMessage(
      const std::vector<const M*>& measurements,
      const std::vector<std::shared_ptr<T>>& tracks,
      const std::vector<int>& match_result,
      AssociationDebugInfoProto* association_debug_info) const {
    *association_debug_info->mutable_config() = config_;

    // Add track debug info.

    for (const auto& track : tracks) {
      auto added_track = association_debug_info->add_track_infos();
      added_track->set_track_id(track->track_state.id);
      added_track->set_track_type(track->track_state.type);
      // Do not publish vegetation or barrier contour for saving debug proto
      // memory.
      if (track->track_state.type == MT_VEGETATION ||
          track->track_state.type == MT_BARRIER) {
        continue;
      }
      for (const auto& pt : track->track_state.contour.points()) {
        Vec2dToProto(pt, added_track->add_contour());
      }
    }

    // Add measurement debug info.
    for (int i = 0; i < measurements.size(); ++i) {
      const auto& measurement = measurements[i];
      auto added_measurement = association_debug_info->add_m_infos();

      added_measurement->set_measurement_id(i);
      added_measurement->set_measurement_type(measurement->type());
      added_measurement->set_timestamp(measurement->timestamp());

      if (measurement->has_laser_measurement()) {
        auto* m_laser = added_measurement->mutable_m_laser();
        Vec2dToProto(
            tracker_util::ComputeWeightedObstacleCentroid(
                measurement->laser_measurement().cluster_measurement()),
            m_laser->mutable_pos());
        // Do not publish vegetation or barrier contour for saving debug proto
        // memory.
        if (measurement->type() == MT_VEGETATION ||
            measurement->type() == MT_BARRIER) {
          continue;
        }
        for (const auto& pt : measurement->laser_measurement().contour()) {
          *m_laser->add_contour() = pt;
        }
      } else if (measurement->has_radar_measurement()) {
        auto* m_radar = added_measurement->mutable_m_radar();
        *m_radar->mutable_pos() = measurement->radar_measurement().pos();
        *m_radar->mutable_vel() = measurement->radar_measurement().vel();
        *m_radar->mutable_pos_cov() =
            measurement->radar_measurement().pos_cov();
        *m_radar->mutable_vel_cov() =
            measurement->radar_measurement().vel_cov();
        m_radar->set_orientation_variance(
            measurement->radar_measurement().orientation_variance());
      } else if (measurement->has_camera3d_measurement()) {
        auto* m_camera = added_measurement->mutable_m_camera();
        m_camera->mutable_pos()->set_x(
            measurement->camera3d_measurement().pos().x());
        m_camera->mutable_pos()->set_y(
            measurement->camera3d_measurement().pos().y());
        m_camera->set_heading(measurement->camera3d_measurement().heading());
        m_camera->set_width(measurement->camera3d_measurement().width());
        m_camera->set_length(measurement->camera3d_measurement().length());
        m_camera->set_height(measurement->camera3d_measurement().height());
      }
    }

    // Add association final result.
    for (int mea_id = 0; mea_id < match_result.size(); ++mea_id) {
      if (match_result[mea_id] == -1) continue;
      const int track_id = tracks[match_result[mea_id]]->track_state.id;
      for (int i = 0; i < association_debug_info->similarity_infos_size();
           ++i) {
        auto info = association_debug_info->mutable_similarity_infos(i);
        if (info->track_id() == track_id && info->measurement_id() == mea_id) {
          info->set_pair_type(
              TrackerDebugProto_AssociationDebugInfoProto::MATE);
        }
      }
    }
  }

  SimilarityMatrixBuilder<T, M> similarity_matrix_builder_;
  MatchSolver match_solver_;
  association::AssociationProto config_;
};

}  // namespace association
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_ONBOARD_ASSOCIATOR_H_
