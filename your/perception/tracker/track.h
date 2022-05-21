#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/perception/tracker/motion_filter_2/estimator.h"
#include "onboard/perception/tracker/tracker_constant.h"
#include "onboard/perception/tracker/voter.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/history_buffer.h"

// TODO(zheng): Use namespace qcraft::tracker.
namespace qcraft::tracker {

// A track that represents a tracked object for many frames.
struct TrackState {
  std::string DebugString() const;
  struct RefPoint {
    enum Type {
      kNone = 0,
      kBBCenter,
      kContourCentroid,
      kNearestObstacleCenter,
      kWeightedObstacleCentroid,
      // Corners.
      kTopRightCorner,
      kTopLeftCorner,
      kBottomLeftCorner,
      kBottomRightCorner,
      // Face center.
      kFrontFaceCenter,
      kLeftFaceCenter,
      kRearFaceCenter,
      kRightFaceCenter,
    };

    Type type = kNone;
    Vec2d pos;
  };

  enum MovingState {
    kUnKnown = 0,
    kStatic,
    kMoving,
  };

  struct ClassifierInfo {
    enum TypeSource {
      kVoting = 0,
      kTcnModel,
      kWarningTriangleModel,
    };
    struct ModelResult {
      float score = 0.0;
      int type = -1;
    };
    TypeSource adopted_method;
    ModelResult tcn_result;
    ModelResult warning_triangle_result;
    MeasurementType voting_type;
  };

  // NOTE(jiawei): Motion filter type is for object type switch. e.g. from
  // unknown to car or ped.
  enum MotionFilterType {
    kCarModel = 0,
    kPointModelCA,
    kPointModelCP,
  };

  struct ObstacleInfo {
    Vec2d center;
    double min_z;
    double max_z;
    int num_points;
  };

  MovingState moving_state = MovingState::kUnKnown;
  // TODO(zheng): Compute and publish moving state confidence.
  double moving_state_confidence = 1.0;

  uint32_t id;
  MeasurementType type;
  ClassifierInfo classifier_info;
  RefPoint ref_point;
  // TODO(zheng): Remove heading, vel, acc, yaw rate variables,
  // which we can get them from motion filter.
  double heading;
  double vel;
  Vec2d acc;
  double yaw_rate;
  double min_z;
  double max_z;
  double ground_z;
  // Velocity measurement.
  std::optional<Vec2d> icp_vel;
  std::optional<Vec2d> fen_vel;
  // The ref point vel is the velocity measurement that computed from
  // ref point.
  std::optional<Vec2d> ref_point_vel;
  double first_timestamp;       // timestamp corresponding to first measurement.
  double last_timestamp;        // timestamp corresponding to last measurement.
  double state_timestamp;       // timestamp corresponding to the current state.
  double last_laser_timestamp;  // timestamp corresponding to last laser
                                // measurement.

  double classification_timestamp;
  double last_update_radar_velocity_timestamp;
  // Bbox from fen.
  std::optional<Box2d> bounding_box;
  // The refined bbox originated from detection bounding box, it contains
  // the whole contour.
  std::optional<Box2d> refined_bounding_box;

  Polygon2d contour;
  // LiDAR: weighted_obstacle_centroid
  // Camera: box center
  // Radar: radar pos
  Vec2d anchor_point;

  // Key point has consistent semantics over time series and maintain
  // good motion consistency at the same time. To achieve this, we use motion
  // filter state pos as base, and reset history key points if motion filter
  // reset ref point(forward and rollback).
  Vec2d key_point;

  bool merged_by_other_track = false;
  bool split_from_other_track = false;

  // Indicate if this object is in parking area. When the speed is near zero, we
  // will classify this track as parked.
  bool in_parking_area = false;

  // Indicate if this object is an offroad object. Currently only for
  // pedestrians.
  bool offroad = false;

  // // Lane speed limit
  // std::optional<double> speed_limit;

  // If the track is barrier or vegetation and the type is from
  // semantic map zone, it's certain static track.
  bool is_certain_static_track = false;

  MotionFilterParamType motion_filter_param_type;
  MotionFilterType motion_filter_type = MotionFilterType::kPointModelCP;
  bool is_on_highway = false;

  TrackLifeState life_state = TrackLifeState::kIdle;
  // The IMM estimator.
  tracker::Estimator estimator_3d;

  HeadingVoter heading_voter;

  TrackMeasurementSourceType measurement_source_type;

  // Which track's bbox, contour comes from: lidar, camera or radar.
  TrackShapeSourceType track_shape_source_type;
  ObservationState observation_state;

  // Measurement that update the track, nullptr if not updated.
  // TODO(jingwei) make sure the measurement is nullptr when not updated.
  const MeasurementProto* measurement = nullptr;

  const CoordinateConverter* coordinate_converter = nullptr;
};

struct TrackDebugInfo {
  std::string DebugString() const;

  // If the state pos is reset, we save the offset in debug info.
  std::optional<Vec2d> state_pos_reset_offset = std::nullopt;
  // If the motion filter is update with icp velocity.
  bool updated_with_icp_velocity = false;
  // If the motion filter is update with icp velocity.
  bool updated_with_fen_velocity = false;
  // If the motion filter is update with radar velocity.
  bool updated_with_radar_velocity = false;
  // Update using camera.
  bool updated_with_camera = false;
};

inline std::string MeasurementSensorTypeToString(
    const MeasurementProto::MeasurementCase type) {
  switch (type) {
    case MeasurementProto::kLaserMeasurement:
      return "laser_m";
    case MeasurementProto::kCameraMeasurement:
    case MeasurementProto::kCamera3DMeasurement:
      return "camera_m";
    case MeasurementProto::kRadarMeasurement:
      return "radar_m";
    default:
      return "no_m";
  }
}

template <typename TrackState>
struct Track {
  using State = TrackState;

  std::string DebugString() const {
    std::string debug_string =
        absl::StrCat(track_state.DebugString(), debug_info.DebugString());
    constexpr double kValidTimeDuration = 0.5;  // s
    for (const auto& [ts, m] : measurement_history) {
      const double time_diff = track_state.state_timestamp - ts;
      if (time_diff > kValidTimeDuration) {
        continue;
      }
      debug_string += absl::StrFormat(
          " ts: %.4f type: %s %s\n", ts, MeasurementType_Name(m->type()),
          MeasurementSensorTypeToString(m->Measurement_case()));
    }
    return debug_string;
  }

  TrackState track_state;
  // TrackState checkpoints.
  HistoryBuffer<TrackState> checkpoints;
  // Measurement history.
  HistoryBuffer<const MeasurementProto*> measurement_history;
  TrackDebugInfo debug_info;
};

// NOTE(zheng): We may copy track ptr in MultiCameraFusionEngine, so
// we use shared_ptr here.
using TrackRef = std::shared_ptr<Track<TrackState>>;

// Enum type to string func, for debugging.
// TODO(zheng): Use map instead of func.
std::string RefPointTypeToString(const TrackState::RefPoint::Type type);
std::string MovingStateToString(const TrackState::MovingState moving_state);
std::string MotionFilterTypeToString(const TrackState::MotionFilterType type);
std::string GetMotionFilterParamPathByType(const MotionFilterParamType type);

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_H_
