#include "onboard/perception/tracker/association/gate.h"

#include <algorithm>
#include <limits>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/association/association_util.h"

namespace qcraft::tracker {
namespace association {

inline const Box2d ProtoToBox2d(const Box2dProto& proto) {
  return Box2d({proto.x(), proto.y()}, proto.heading(), proto.length(),
               proto.width());
}

inline const Polygon2d Box2dToPolygon2d(const Box2d& box2d) {
  return Polygon2d(box2d.GetCornersCounterClockwise());
}

// DBQ
template class CircleGate<Track<TrackState>, MeasurementProto>;
template class TypeCircleGate<Track<TrackState>, MeasurementProto>;
template class ElipseGate<Track<TrackState>, MeasurementProto>;
template class SpeedGate<Track<TrackState>, MeasurementProto>;
template class BevIouGate<Track<TrackState>, MeasurementProto>;
template class TypeGate<Track<TrackState>, MeasurementProto>;
template class ImageIouGate<Track<TrackState>, MeasurementProto>;
// PBQ
template class CircleGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                          MeasurementProto>;
template class TypeCircleGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                              MeasurementProto>;
template class ElipseGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                          MeasurementProto>;
template class SpeedGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                         MeasurementProto>;
template class BevIouGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                          MeasurementProto>;
template class TypeGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                        MeasurementProto>;
template class ImageIouGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                            MeasurementProto>;

// DBQ
template <>
GateResult CircleGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(!measurement.has_camera_measurement());
  const auto measurement_contour_centroid =
      measurement.has_laser_measurement()
          ? tracker_util::ToPolygon2d(measurement.laser_measurement().contour())
                .centroid()
          : Vec2dFromProto(measurement.radar_measurement().pos());
  const double dist2 =
      (measurement_contour_centroid - track.track_state.contour.centroid())
          .squaredNorm();
  return {dist2 < thresholds_[0], dist2};
}

template <>
GateResult TypeCircleGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  double dist2 = std::numeric_limits<double>::max();
  if (measurement.has_laser_measurement()) {
    const auto laser_contour_centroid =
        tracker_util::ToPolygon2d(measurement.laser_measurement().contour())
            .centroid();
    if (track.track_state.track_shape_source_type ==
        TrackShapeSourceType::TSST_CAMERA) {
      dist2 =
          track.track_state.contour.DistanceSquareTo(laser_contour_centroid);
    } else {
      dist2 = (laser_contour_centroid - track.track_state.contour.centroid())
                  .squaredNorm();
    }
  } else if (measurement.has_radar_measurement()) {
    const Vec2d radar_pos =
        Vec2dFromProto(measurement.radar_measurement().pos());
    for (const auto& corner : track.track_state.contour.points()) {
      dist2 = std::min(dist2, (radar_pos - corner).squaredNorm());
    }
  } else if (measurement.has_camera3d_measurement()) {
    const auto mea_pos =
        Vec3dFromProto(measurement.camera3d_measurement().pos());
    dist2 = (track.track_state.estimator_3d.GetStatePos() -
             mea_pos.block<2, 1>(0, 0))
                .squaredNorm();
  } else {
    QCHECK(false) << "Invalid sensor type.";
  }
  // TypeCircleGate deals with two types {Pedestrian, Others}
  // Others:     thresholds_[1]
  if (MT_PEDESTRIAN == measurement.type() ||
      MT_PEDESTRIAN == track.track_state.type) {
    return {dist2 < thresholds_[0], dist2};
  } else {
    return {dist2 < thresholds_[1], dist2};
  }
}

// DBQ
template <>
GateResult SpeedGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_type());
  QCHECK(measurement.has_laser_measurement());

  // No history measurement, no gating.
  if (track.measurement_history.empty()) return {true, -2.0};

  // Find the latest laser measurement before input measurement timestamp.
  constexpr double kMaxTimeDuration = 0.3;
  const double earliest_time = measurement.timestamp() - kMaxTimeDuration;
  const MeasurementProto* history_lidar_mea_before_input = nullptr;
  for (int i = track.measurement_history.size() - 1; i >= 0; --i) {
    const auto& [ts, m] = track.measurement_history[i];
    if (ts > measurement.timestamp()) continue;
    if (ts < earliest_time) break;
    if (m->has_laser_measurement()) {
      history_lidar_mea_before_input = m;
      break;
    }
  }
  // No valid measurement found, no gating.
  if (!history_lidar_mea_before_input) return {true, -3.0};

  const double delta_time =
      measurement.timestamp() - history_lidar_mea_before_input->timestamp();
  constexpr double kTooShortTimeDuration = 0.05;
  // When delta_time is too short, noise may be too big, no gating.
  if (delta_time < kTooShortTimeDuration) return {true, -4.0};

  // Calculate speed by diff.
  const auto history_measurement_centroid =
      tracker_util::ToPolygon2d(
          history_lidar_mea_before_input->laser_measurement().contour())
          .centroid();
  const auto measurement_centroid =
      tracker_util::ToPolygon2d(measurement.laser_measurement().contour())
          .centroid();
  const double centroid_speed_sqr =
      ((measurement_centroid - history_measurement_centroid) / delta_time)
          .squaredNorm();

  constexpr double kPedestrianMaxVelocitySqr = Sqr(10);  // 10 m/s
  constexpr double kCyclistMaxVelocitySqr = Sqr(40);     // 40 m/s
  if (MT_PEDESTRIAN == track.track_state.type &&
      MT_PEDESTRIAN == measurement.type()) {
    return {centroid_speed_sqr < kPedestrianMaxVelocitySqr, centroid_speed_sqr};
  }
  if (MT_CYCLIST == track.track_state.type &&
      MT_CYCLIST == measurement.type()) {
    return {centroid_speed_sqr < kCyclistMaxVelocitySqr, centroid_speed_sqr};
  }
  if ((MT_PEDESTRIAN == track.track_state.type &&
       MT_CYCLIST == measurement.type()) ||
      (MT_CYCLIST == track.track_state.type &&
       MT_PEDESTRIAN == measurement.type())) {
    return {centroid_speed_sqr < kCyclistMaxVelocitySqr, centroid_speed_sqr};
  }

  // No valid type, no gating.
  return {true, -1.0};
}

template <>
GateResult BevIouGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_type());
  QCHECK(measurement.has_laser_measurement());
  const auto& laser_measurement = measurement.laser_measurement();
  /* Vehicle has big bounding box, so if bounding box and bounding box(contour)
   * has no overlap, gate it.
   * This [BANDAGE] do not gate MT_VEHICLE without bounding box.
   */
  if (MT_VEHICLE == track.track_state.type ||
      MT_VEHICLE == measurement.type()) {
    if (track.track_state.bounding_box ||
        laser_measurement.has_detection_bounding_box()) {
      const Polygon2d measurement_shape =
          laser_measurement.has_detection_bounding_box()
              ? Box2dToPolygon2d(
                    ProtoToBox2d(laser_measurement.detection_bounding_box()))
              : tracker_util::ToPolygon2d(laser_measurement.contour());

      const double m_ts = measurement.timestamp();
      const auto* ckpt_ldr =
          tracker_util::FindLatestCheckPointBeforeTimeWithType(
              track, m_ts, MeasurementsProto::LIDAR);
      if (nullptr == ckpt_ldr) return {true, -1.0};
      if (ckpt_ldr->track_shape_source_type ==
          TrackShapeSourceType::TSST_CAMERA)
        return {true, -2.0};

      const auto track_shape =
          ckpt_ldr->bounding_box
              ? Box2dToPolygon2d(ckpt_ldr->bounding_box.value())
              : ckpt_ldr->contour;
      const auto predict_state = ckpt_ldr->estimator_3d.ComputePrediction(m_ts);
      const Vec2d predict_offset =
          predict_state.GetStatePos() - ckpt_ldr->estimator_3d.GetStatePos();

      const auto predict_shape =
          tracker_util::ShiftPoints(predict_offset, track_shape.points());

      const bool has_overlap = predict_shape.HasOverlap(measurement_shape);
      return {has_overlap, has_overlap};
    }
  }
  return {true, -3.0};
}

template <>
GateResult ImageIouGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QLOG(FATAL) << "Not implemented.";
  return {false, -1.0};
}

template <>
GateResult ElipseGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto& measurement, const Track<TrackState>& track) {
  QCHECK(measurement.has_laser_measurement());
  if (track.track_state.measurement_source_type !=
          TrackMeasurementSourceType::TMST_RO &&
      !track.track_state.estimator_3d.IsCarModel()) {
    const double mea_timestamp = measurement.timestamp();
    const TrackState* query_ckpt =
        tracker_util::FindLatestCheckPointBeforeTime(track, mea_timestamp);
    if (nullptr == query_ckpt) return {true, -2.0};

    const double delta_t = abs(mea_timestamp - query_ckpt->state_timestamp);
    constexpr double kTooShortTimeDuration = 0.08;
    if (delta_t < kTooShortTimeDuration) return {true, -3.0};

    const auto measurement_contour =
        tracker_util::ToPolygon2d(measurement.laser_measurement().contour());
    const auto predict_state =
        tracker_util::SafePredictPos(track, measurement.timestamp());
    const Vec2d predict_offset =
        predict_state.has_value()
            ? predict_state->GetStatePos() -
                  track.track_state.estimator_3d.GetStatePos()
            : Vec2d(0., 0.);
    const auto predict_contour = tracker_util::ShiftPoints(
        predict_offset, track.track_state.contour.points());
    const bool has_overlap = predict_contour.HasOverlap(measurement_contour);
    if (has_overlap) return {true, -4.0};

    const auto measurement_centroid = measurement_contour.centroid();
    const Vec2d est_vel =
        measurement.laser_measurement().has_icp_measurement()
            ? Vec2dFromProto(
                  measurement.laser_measurement().icp_measurement().vel())
            : (measurement_centroid - query_ckpt->contour.centroid()) /
                  (measurement.timestamp() - query_ckpt->state_timestamp +
                   std::numeric_limits<double>::epsilon());
    const tracker::PosSpeedMeasurement pos_speed_mea{measurement_centroid.x(),
                                                     measurement_centroid.y(),
                                                     est_vel.x(), est_vel.y()};
    const double m_distance =
        query_ckpt->estimator_3d
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                pos_speed_mea, mea_timestamp);

    if (m_distance >= 0.0) {
      const double p_value =
          tracker_util::ComputePValueFromChiSquareDistr(m_distance, /*dof*/ 4);
      return {(1 - p_value) <= thresholds_[0], (1 - p_value)};
    }
  }

  return {true, -1.0};
}

// PBQ
template <>
GateResult ImageIouGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                        MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QCHECK(measurement.has_camera3d_measurement() &&
         measurement.camera3d_measurement().has_bbox_2d() &&
         !track.measurement_history.empty());

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
  const double iou = association_util::IoU(mea_box2d, track_box2d);
  return {iou >= thresholds_[0], iou};
}

template <>
GateResult CircleGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                      MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented.";
  return {false, -1.0};
}

template <>
GateResult TypeCircleGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                          MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  double dist2 = std::numeric_limits<double>::max();
  if (measurement.has_camera3d_measurement()) {
    const auto mea_pos =
        Vec3dFromProto(measurement.camera3d_measurement().pos());
    dist2 = (track.track_state.estimator_3d.GetStatePos() -
             mea_pos.block<2, 1>(0, 0))
                .squaredNorm();
  } else {
    QCHECK(false) << "Invalid sensor type.";
  }

  // TypeCircleGate deals with two types {Pedestrian, Others}
  // Others:     thresholds_[1]
  if (MT_PEDESTRIAN == measurement.type() ||
      MT_PEDESTRIAN == track.track_state.type) {
    return {dist2 < thresholds_[0], dist2};
  } else if (MT_CYCLIST == measurement.type() ||
             MT_CYCLIST == track.track_state.type) {
    return {dist2 < thresholds_[1], dist2};
  } else {
    return {dist2 < thresholds_[2], dist2};
  }
}

template <>
GateResult ElipseGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                      MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented.";
  return {false, -1.0};
}

// PBQ
template <>
GateResult
SpeedGate<Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented.";
  return {false, -1.0};
}

template <>
GateResult BevIouGate<Track<multi_camera_fusion::MultiCameraTrackState>,
                      MeasurementProto>::
    Gating(const MeasurementProto& measurement,
           const Track<multi_camera_fusion::MultiCameraTrackState>& track) {
  QLOG(FATAL) << "Not implemented.";
  return {false, -1.0};
}

// DBQ & PBQ
template <typename TrackType, typename MeasurementType>
GateResult TypeGate<TrackType, MeasurementType>::Gating(
    const MeasurementType& measurement, const TrackType& track) {
  QCHECK(measurement.has_type());
  QCHECK(measurement.has_type_source());

  // No association for "barrier" and "vegetation" from "semantic map zone".
  if (track.track_state.is_certain_static_track) return {false, -1.0};
  if (measurement.type_source() == MTS_SEMANTIC_MAP_ZONE &&
      (measurement.type() == MT_BARRIER ||
       measurement.type() == MT_VEGETATION)) {
    return {false, -2.0};
  }

  QCHECK(0 <= measurement.type() && measurement.type() < dim_ &&
         0 <= track.track_state.type && track.track_state.type < dim_);
  const double val =
      kTypeMatrix[measurement.type() * dim_ + track.track_state.type];

  return {val > this->thresholds_[0], val};
}
template GateResult TypeGate<Track<TrackState>, MeasurementProto>::Gating(
    const MeasurementProto&, const Track<TrackState>&);
template GateResult
TypeGate<Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>::
    Gating(const MeasurementProto&,
           const Track<multi_camera_fusion::MultiCameraTrackState>&);

}  // namespace association
}  // namespace qcraft::tracker
