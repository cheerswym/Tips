#ifndef ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACK_H_
#define ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACK_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/perception/tracker/motion_filter_2/estimator.h"
#include "onboard/perception/tracker/motion_filter_2/img_2d_kalman_filter.h"
#include "onboard/perception/tracker/motion_filter_2/img_2d_model.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::multi_camera_fusion {

struct MultiCameraTrackState {
  std::string DebugString() const;
  uint32_t id;
  CameraId camera_id;
  MeasurementType type;
  // TODO(zheng, keyan): Unify 2d motion filter to estimator.
  tracker::Img2DKalmanFilter<tracker::Img2DModelCV> motion_filter_2d;
  tracker::Estimator estimator_3d;

  double first_timestamp;  // timestamp corresponding to first measurement.
  double last_timestamp;   // timestamp corresponding to last measurement.
  double state_timestamp;  // timestamp corresponding to the current state.
  TrackLifeState life_state = TrackLifeState::kIdle;
  Polygon2d contour;
  Box2d bounding_box;
  bool is_certain_static_track = false;

  // NOTE(zheng): Because the motion fileter's result is verey bad so we
  // save the measurement pos and heading to publish for sensor fusion.
  Vec3d pos;
  double heading;

  TrackMeasurementSourceType measurement_source_type = TMST_VO;
  // Which track's bbox, contour comes from: lidar, camera or radar.
  TrackShapeSourceType track_shape_source_type = TSST_CAMERA;
};

using TrackRef = std::shared_ptr<tracker::Track<MultiCameraTrackState>>;
using TrackConstRef =
    std::shared_ptr<const tracker::Track<MultiCameraTrackState>>;

}  // namespace qcraft::multi_camera_fusion

#endif  // ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACK_H_
