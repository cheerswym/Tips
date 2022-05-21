#include "onboard/perception/tracker/association/metric.h"

#include <algorithm>
#include <limits>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/association/association_util.h"

namespace qcraft::tracker {
namespace association {

template <typename PointsT>
Polygon2d ToPolygon2d(const PointsT& points_proto) {
  std::vector<Vec2d> points;
  points.reserve(points_proto.size());
  for (const auto& p : points_proto) {
    points.push_back(Vec2dFromProto(p));
  }
  return Polygon2d(points);
}

Vec2d Vec2dFromNearestContourCornerToStatePos(const Vec2d& radar_pos,
                                              const TrackState& track_state) {
  const auto& corners = track_state.contour.points();
  if (corners.empty()) {
    QLOG(ERROR) << "Track " << track_state.id << " has no corners.";
    return Vec2d(0., 0.);
  }

  int min_idx = 0;
  double min_dist2 = (corners[0] - radar_pos).squaredNorm();
  for (int i = 1; i < corners.size(); ++i) {
    const float dist2 = (corners[i] - radar_pos).squaredNorm();
    if (dist2 < min_dist2) {
      min_idx = i;
      min_dist2 = dist2;
    }
  }

  const auto& state_pos = track_state.estimator_3d.GetStatePos();
  return state_pos - corners[min_idx];
}

// DBQ
template class Metric<Track<TrackState>, MeasurementProto>;
template class EuclideanMetric<Track<TrackState>, MeasurementProto>;
template class PValueMetric<Track<TrackState>, MeasurementProto>;
template class RadarMahalanobisMetric<Track<TrackState>, MeasurementProto>;
template class VehicleIouMetric<Track<TrackState>, MeasurementProto>;
template class ImageIouMetric<Track<TrackState>, MeasurementProto>;
template class ImageAppearanceFeatureMetric<Track<TrackState>,
                                            MeasurementProto>;
// PBQ
template class Metric<Track<multi_camera_fusion::MultiCameraTrackState>,
                      MeasurementProto>;
template class EuclideanMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;
template class PValueMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                            MeasurementProto>;
template class RadarMahalanobisMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;
template class VehicleIouMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;
template class ImageIouMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                              MeasurementProto>;
template class ImageAppearanceFeatureMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;

// DBQ
template <>
double EuclideanMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  const auto predict_state =
      tracker_util::SafePredictPos(track, measurement.timestamp());
  const Vec2d predict_offset =
      predict_state.has_value()
          ? predict_state->GetStatePos() -
                track.track_state.estimator_3d.GetStatePos()
          : Vec2d(0., 0.);

  const double m_ts = measurement.timestamp();
  const auto* ckpt_ldr = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, m_ts, MeasurementsProto::LIDAR);
  const auto* ckpt_cam = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, m_ts, MeasurementsProto::CAMERA);
  const auto* ckpt_radar = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, m_ts, MeasurementsProto::RADAR);

  double dist2 = std::numeric_limits<double>::max();
  constexpr double kMaxDist2 = 1e6;
  QCHECK_GT(kMaxDist2, thresholds_[0]);
  if (measurement.has_laser_measurement()) {
    const auto m_weighted_obstacle_centroid =
        tracker_util::ComputeWeightedObstacleCentroid(
            measurement.laser_measurement().cluster_measurement());
    if (ckpt_ldr) {
      // L-L: use weighted obstacle centroid to calculate dist2.
      const auto predict_state = ckpt_ldr->estimator_3d.ComputePrediction(m_ts);
      const Vec2d offset =
          predict_state.GetStatePos() - ckpt_ldr->estimator_3d.GetStatePos();
      const auto ldr_ckpt_weighted_obstacle_centroid =
          tracker_util::ComputeWeightedObstacleCentroid(
              ckpt_ldr->measurement->laser_measurement().cluster_measurement());
      dist2 = (ldr_ckpt_weighted_obstacle_centroid + offset -
               m_weighted_obstacle_centroid)
                  .squaredNorm();
    } else if (ckpt_cam) {
      // L-C: use lidar weighted obstacle centroid to camera box to caculate
      // dist2.
      const auto predict_state = ckpt_cam->estimator_3d.ComputePrediction(m_ts);
      const Vec2d offset =
          predict_state.GetStatePos() - ckpt_cam->estimator_3d.GetStatePos();
      auto bbox = association_util::GetBox2dFromCamera3dMeasurementProto(
          ckpt_cam->measurement->camera3d_measurement());
      bbox.Shift(offset);

      dist2 = bbox.DistanceSquareTo(m_weighted_obstacle_centroid);
    } else if (ckpt_radar) {
      // L-R: use lidar polygon to radar position to caculate dist2.
      const auto predict_state =
          ckpt_radar->estimator_3d.ComputePrediction(m_ts);
      const Vec2d offset =
          predict_state.GetStatePos() - ckpt_radar->estimator_3d.GetStatePos();
      const auto radar_ckpt_pos =
          Vec2dFromProto(ckpt_radar->measurement->radar_measurement().pos());
      dist2 = (radar_ckpt_pos + offset - m_weighted_obstacle_centroid)
                  .squaredNorm();
    } else {
      QLOG(ERROR) << "No checkpoint found before time " << m_ts;
      dist2 = kMaxDist2;
    }
  } else if (measurement.has_camera3d_measurement()) {
    const auto track_pos = track.track_state.estimator_3d.GetStatePos();
    const auto measurement_pos =
        Vec3dFromProto(measurement.camera3d_measurement().pos());
    dist2 = (track_pos + predict_offset - measurement_pos.block<2, 1>(0, 0))
                .squaredNorm();
  } else {
    QCHECK(false) << "Invalid sensor type.";
  }

  return dist2 < this->thresholds_[0] ? 1.0 - dist2 / this->thresholds_[0]
                                      : 0.0;
}

// FIXME(jingwei) should return p_value
template <>
double PValueMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_laser_measurement());
  const double mea_timestamp = measurement.timestamp();
  const auto& laser_measurement = measurement.laser_measurement();
  const auto m_weighted_obstacle_centroid =
      tracker_util::ComputeWeightedObstacleCentroid(
          laser_measurement.cluster_measurement());
  const TrackState* query_ckpt =
      tracker_util::FindLatestCheckPointBeforeTime(track, mea_timestamp);
  if (nullptr == query_ckpt) return -1.0;
  if (laser_measurement.has_icp_measurement()) {
    // Use mahalanobis distance of position and speed when both track and
    // measurement has speed.
    const auto icp_vel =
        Vec2dFromProto(laser_measurement.icp_measurement().vel());
    const tracker::PosSpeedMeasurement pos_speed_mea(
        m_weighted_obstacle_centroid.x(), m_weighted_obstacle_centroid.y(),
        icp_vel.x(), icp_vel.y());
    const double m_distance =
        query_ckpt->estimator_3d
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                pos_speed_mea, mea_timestamp);
    return m_distance;
  } else {
    // Use mahalanobis distance of position when track or measurement has no
    // speed.
    const tracker::PositionMeasurement pos_mea(
        m_weighted_obstacle_centroid.x(), m_weighted_obstacle_centroid.y());
    const double m_distance =
        query_ckpt->estimator_3d
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                pos_mea, mea_timestamp);
    return m_distance;
  }
}

template <>
double RadarMahalanobisMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_radar_measurement());
  const double mea_timestamp = measurement.timestamp();
  const TrackState* query_ckpt =
      tracker_util::FindLatestCheckPointBeforeTime(track, mea_timestamp);
  if (nullptr == query_ckpt) return 0.0;

  const auto& radar_measurement = measurement.radar_measurement();
  const auto radar_pos = Vec2dFromProto(radar_measurement.pos());
  const auto radar_vel = Vec2dFromProto(radar_measurement.vel());
  const auto offset_radar_pos =
      radar_pos +
      Vec2dFromNearestContourCornerToStatePos(radar_pos, *query_ckpt);

  double m_distance = 0.0;
  if (query_ckpt->estimator_3d.IsPointModel()) {
    const tracker::PosSpeedMeasurement pos_speed_mea(
        offset_radar_pos.x(), offset_radar_pos.y(), radar_vel.x(),
        radar_vel.y());
    m_distance =
        query_ckpt->estimator_3d
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                pos_speed_mea, mea_timestamp);
  } else {
    const tracker::PosHeadingVelocityMeasurement pos_heading_velocity_mea(
        offset_radar_pos.x(), offset_radar_pos.y(),
        NormalizeAngle(radar_vel.FastAngle()), radar_vel.norm());
    m_distance =
        query_ckpt->estimator_3d
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                pos_heading_velocity_mea, mea_timestamp);
  }
  constexpr double kPvalue4Dof95Percent = 9.49;
  constexpr double kPvalue4Dof95PercentInv = 1.0 / kPvalue4Dof95Percent;
  return m_distance > kPvalue4Dof95Percent
             ? 0.0
             : 1.0 - m_distance * kPvalue4Dof95PercentInv;
}

template <>
double VehicleIouMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_type());
  QCHECK(measurement.has_laser_measurement() ||
         measurement.has_camera3d_measurement());

  if (MT_VEHICLE == track.track_state.type &&
      track.track_state.type == measurement.type() &&
      track.track_state.bounding_box) {
    if (measurement.has_laser_measurement() &&
        measurement.laser_measurement().has_detection_bounding_box()) {
      const Box2d measurement_box2d(
          measurement.laser_measurement().detection_bounding_box());

      const double box_iou = association_util::IoU(
          measurement_box2d, track.track_state.bounding_box.value());

      return box_iou > this->thresholds_[0] ? kMaxSimilarityValue : 0.0;
    } else if (measurement.has_camera3d_measurement()) {
      const auto& m_cam = measurement.camera3d_measurement();
      const Box2d measurement_box2d({m_cam.pos().x(), m_cam.pos().y()},
                                    m_cam.heading(), m_cam.length(),
                                    m_cam.width());

      const double box_iou = association_util::IoU(
          measurement_box2d, track.track_state.bounding_box.value());

      return box_iou > this->thresholds_[0] ? box_iou : 0.0;
    }
  }

  return 0.0;
}

template <>
double ImageIouMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QLOG(ERROR) << "ImageIouMetric is not implemented yet.";
  return 0.0;
}

// PBQ
template <>
double EuclideanMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                       MeasurementProto>::
    Similarity(const MeasurementProto& measurement,
               const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QCHECK(measurement.has_camera3d_measurement());
  const auto predict_state =
      tracker_util::SafePredictPos(track, measurement.timestamp());
  const Vec2d predict_offset =
      predict_state.has_value()
          ? predict_state->GetStatePos() -
                track.track_state.estimator_3d.GetStatePos()
          : Vec2d(0., 0.);

  const auto track_pos = track.track_state.estimator_3d.GetStatePos();
  const auto measurement_pos =
      Vec3dFromProto(measurement.camera3d_measurement().pos());
  const double dist2 =
      (track_pos + predict_offset - measurement_pos.block<2, 1>(0, 0))
          .squaredNorm();

  return dist2 < this->thresholds_[0] ? 1.0 - dist2 / this->thresholds_[0]
                                      : 0.0;
}

template <>
double PValueMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                    MeasurementProto>::
    Similarity(const MeasurementProto& measurement,
               const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented";
  return 0.0;
}

template <>
double RadarMahalanobisMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                              MeasurementProto>::
    Similarity(const MeasurementProto& measurement,
               const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented";
  return 0.0;
}

template <>
double VehicleIouMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                        MeasurementProto>::
    Similarity(const MeasurementProto& measurement,
               const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented";
  return 0.0;
}

template <>
double ImageIouMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                      MeasurementProto>::
    Similarity(const MeasurementProto& measurement,
               const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QCHECK(measurement.has_camera3d_measurement() &&
         measurement.camera3d_measurement().has_bbox_2d() &&
         !track.measurement_history.empty())
      << " measurement.has_camera3d_measurement()="
      << measurement.has_camera3d_measurement()
      << " measurement.camera3d_measurement().has_bbox_2d()="
      << measurement.camera3d_measurement().has_bbox_2d()
      << " track.measurement_history.size()="
      << track.measurement_history.size();

  // TODO(jingwei) Use prediction.
  // const auto predict_state =
  //     track.track_state.estimator_2d.ComputePrediction(measurement.timestamp());
  // const Vec2d offset = predict_state.GetStatePos() -
  //                      track.track_state.estimator_2d.GetStatePos();
  auto track_box2d =
      track.measurement_history.back_value()->camera3d_measurement().bbox_2d();
  // track_box2d.set_x(track_box2d.x() + offset.x());
  // track_box2d.set_y(track_box2d.y() + offset.y());

  const auto& mea_box2d = measurement.camera3d_measurement().bbox_2d();
  // The kminiou is not included in gating, this is to avoid wrong gating when
  // reid matches but low IOU occurs.
  const double iou = association_util::IoU(mea_box2d, track_box2d);
  return iou > this->thresholds_[0] ? iou : 0.0;
}

// DBQ & PBQ share the same implementation
template <typename TrackType, typename MeasurementType>
double ImageAppearanceFeatureMetric<TrackType, MeasurementType>::Similarity(
    const MeasurementType& measurement, const TrackType& track) {
  QCHECK(measurement.has_camera3d_measurement() &&
         !track.measurement_history.empty())
      << " measurement.has_camera3d_measurement()="
      << measurement.has_camera3d_measurement()
      << " track.measurement_history.size()="
      << track.measurement_history.size();

  // TODO(jingwei) Use the biggest similarity in the history.
  const auto& track_feature = track.measurement_history.back_value()
                                  ->camera3d_measurement()
                                  .reid_feature_embeddings();
  const auto& mea_feature =
      measurement.camera3d_measurement().reid_feature_embeddings();
  const double cos_sim =
      association_util::CosineSimilarity(track_feature, mea_feature);

  return cos_sim > this->thresholds_[0] ? cos_sim : 0.0;
}
template double
ImageAppearanceFeatureMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto&, const Track<TrackState>&);
template double ImageAppearanceFeatureMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>::
    Similarity(const MeasurementProto&,
               const Track<multi_camera_fusion::MultiCameraTrackState>&);

template <typename TrackType, typename MeasurementType>
double CameraTrackerIdMetric<TrackType, MeasurementType>::Similarity(
    const MeasurementType& measurement, const TrackType& track) {
  QCHECK(measurement.has_camera3d_measurement() &&
         !track.measurement_history.empty())
      << " measurement.has_camera3d_measurement()="
      << measurement.has_camera3d_measurement()
      << " track.measurement_history.size()="
      << track.measurement_history.size();

  const int history_size = track.measurement_history.size();
  constexpr int kMaxHistorySize = 10;
  const int end = std::max(0, history_size - kMaxHistorySize);

  for (int i = history_size - 1; i >= end; --i) {
    const auto& history_mea = track.measurement_history.value(i);
    if (history_mea->has_camera3d_measurement()) {
      if (measurement.camera3d_measurement().track_id() ==
          history_mea->camera3d_measurement().track_id())
        return 1.0;
    }
  }

  return 0.0;
}
template double
CameraTrackerIdMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto&, const Track<TrackState>&);
template double CameraTrackerIdMetric<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>::
    Similarity(const MeasurementProto&,
               const Track<multi_camera_fusion::MultiCameraTrackState>&);

template <typename TrackType, typename MeasurementType>
double BevIouMetric<TrackType, MeasurementType>::Similarity(
    const MeasurementType& measurement, const TrackType& track) {
  // TODO(jingwei) Use prediction when anchor point is available.
  const Vec2d predict_offset = {0., 0.};
  const auto predict_polygon = track.track_state.contour.Transform(
      track.track_state.contour.centroid(), 1.0, 0., predict_offset);

  if (measurement.has_laser_measurement()) {
    const auto& laser = measurement.laser_measurement();
    std::vector<Vec2d> points;
    points.reserve(laser.contour_size());
    for (int i = 0; i < laser.contour_size(); ++i) {
      points.emplace_back(Vec2dFromProto(laser.contour(i)));
    }
    const Polygon2d mea_polygon(points);
    const double iop = association_util::IoP(mea_polygon, predict_polygon);
    return iop > this->thresholds_[0] ? iop : 0.0;
  } else if (measurement.has_camera3d_measurement()) {
    const auto& cam = measurement.camera3d_measurement();
    const Polygon2d mea_polygon({{cam.pos().x(), cam.pos().y()},
                                 cam.heading(),
                                 cam.length(),
                                 cam.width()});
    const double iop = association_util::IoP(mea_polygon, predict_polygon);
    return iop > this->thresholds_[1] ? iop : 0.0;
  } else if (measurement.has_radar_measurement()) {
    // Note: because radar has no shape, we use overlap and distance to polygon
    // to compute similarity.
    const auto& radar = measurement.radar_measurement();
    return predict_polygon.IsPointIn({radar.pos().x(), radar.pos().y()}) ? 1.0
                                                                         : 0.0;
  } else if (measurement.has_camera_measurement()) {
    // NOTE: We do not use camera measurement anymore.
    QLOG(WARNING) << "Measurement has_camera_measurement, data may be too old.";
    return 0.0;
  } else {
    QLOG(FATAL) << "Unknown measurement type.";
  }

  return 0.0;
}
template double BevIouMetric<Track<TrackState>, MeasurementProto>::Similarity(
    const MeasurementProto&, const Track<TrackState>&);
template double BevIouMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                             MeasurementProto>::
    Similarity(const MeasurementProto&,
               const Track<multi_camera_fusion::MultiCameraTrackState>&);

}  // namespace association
}  // namespace qcraft::tracker
