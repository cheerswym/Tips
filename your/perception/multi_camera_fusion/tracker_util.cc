#include "onboard/perception/multi_camera_fusion/tracker_util.h"

namespace qcraft::multi_camera_fusion {

MeasurementProto TrackToMeasurementProto(
    const tracker::Track<MultiCameraTrackState>& track,
    bool sync_reid_feature) {
  MeasurementProto m;
  m.set_type(track.track_state.type);
  m.set_timestamp(track.track_state.state_timestamp);
  m.set_type_source(MeasurementTypeSource::MTS_MONO_3D);
  auto* camera3d_m = m.mutable_camera3d_measurement();

  // 2D Detection.
  camera3d_m->mutable_bbox_2d_detection()->set_x(
      track.measurement_history.back_value()
          ->camera3d_measurement()
          .bbox_2d()
          .x());
  camera3d_m->mutable_bbox_2d_detection()->set_y(
      track.measurement_history.back_value()
          ->camera3d_measurement()
          .bbox_2d()
          .y());
  camera3d_m->mutable_bbox_2d_detection()->set_width(
      track.measurement_history.back_value()
          ->camera3d_measurement()
          .bbox_2d()
          .width());
  camera3d_m->mutable_bbox_2d_detection()->set_height(
      track.measurement_history.back_value()
          ->camera3d_measurement()
          .bbox_2d()
          .height());
  // 2D state bbox
  camera3d_m->mutable_bbox_2d()->set_x(
      track.track_state.motion_filter_2d.GetState().x());
  camera3d_m->mutable_bbox_2d()->set_y(
      track.track_state.motion_filter_2d.GetState().y());
  camera3d_m->mutable_bbox_2d()->set_width(
      track.track_state.motion_filter_2d.GetState().w());
  camera3d_m->mutable_bbox_2d()->set_height(
      track.track_state.motion_filter_2d.GetState().h());
  // 3D State.
  // TODO(zheng): We will use the state output until we have a better
  // estimator.
  const auto track_bev_pos = track.track_state.pos;
  const auto track_bev_heading = track.track_state.heading;
  Vec3dToProto(track_bev_pos, camera3d_m->mutable_pos());
  camera3d_m->set_heading(track_bev_heading);

  Vec2dToProto(track.track_state.estimator_3d.GetStateData().GetStateSpeed(),
               camera3d_m->mutable_vel());
  // 3D Size.
  camera3d_m->set_width(
      track.track_state.estimator_3d.GetExtentStateData().state().width());
  camera3d_m->set_length(
      track.track_state.estimator_3d.GetExtentStateData().state().length());
  camera3d_m->set_height(
      track.track_state.estimator_3d.GetExtentStateData().state().height());

  camera3d_m->set_camera_id(track.track_state.camera_id);

  camera3d_m->set_timestamp(track.track_state.state_timestamp);

  camera3d_m->set_type(track.track_state.type);

  camera3d_m->set_existence_confidence(1.0);

  // Covariance.
  Mat2dToProto(track.track_state.estimator_3d.GetStateData().GetStatePosCov(),
               camera3d_m->mutable_pos_covariance());
  camera3d_m->set_heading_variance(
      track.track_state.estimator_3d.GetStateData().GetYawCov());
  Mat2dToProto(track.track_state.estimator_3d.GetStateData().GetSpeedCov(),
               camera3d_m->mutable_vel_covariance());
  Mat3dToProto(track.track_state.estimator_3d.GetExtentStateData().state_cov(),
               camera3d_m->mutable_size_covariance());
  if (sync_reid_feature) {
    // Feature embedding.
    *camera3d_m->mutable_reid_feature_embeddings() =
        track.measurement_history.back_value()
            ->camera3d_measurement()
            .reid_feature_embeddings();
  }
  camera3d_m->set_track_id(track.track_state.id);
  *camera3d_m->mutable_mono3d_pos_cov() =
      track.measurement_history.back_value()
          ->camera3d_measurement()
          .mono3d_pos_cov();

  return m;
}

}  // namespace qcraft::multi_camera_fusion
