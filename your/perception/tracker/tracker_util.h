#ifndef ONBOARD_PERCEPTION_TRACKER_TRACKER_UTIL_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACKER_UTIL_H_

#include <map>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/sensor_fov/sensor_fov.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/vis/canvas/canvas.h"

namespace qcraft::tracker {
// NOTE(youjiawei): generic way to output enum class.
template <typename T>
std::ostream& operator<<(
    typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& os,
    const T& e) {
  return os << static_cast<typename std::underlying_type<T>::type>(e);
}

}  // namespace qcraft::tracker

// This util should have no concept of Track, which is easy to be shared logic.
namespace qcraft::tracker::tracker_util {
bool IsInCautiousRegion(const VehiclePose& vehicle_pose, const Vec2d& vec);

bool IsCornerRadar(qcraft::RadarId radar_id);

// The input matrix has rows equal to # of measurements, and cols equal to # of
// tracks. Each element in the matrix should be >= 0.
std::vector<int> ComputeMatches(const Eigen::MatrixXd& cost_matrix);

bool IsInSpecialZone(const SemanticMapManager& semantic_map_manager,
                     const Polygon2d& contour,
                     const CoordinateConverter& coordinate_converter,
                     const mapping::PerceptionZoneProto::Type zone_type,
                     double min_overlap_requirement);

bool IsInIgnoranceZone(const SemanticMapManager& semantic_map_manager,
                       const Polygon2d& contour,
                       const CoordinateConverter& coordinate_converter);

bool IsInStaticObjectZone(const SemanticMapManager& semantic_map_manager,
                          const Polygon2d& contour,
                          const CoordinateConverter& coordinate_converter);

bool IsInCrosswalk(const SemanticMapManager& semantic_map_manager,
                   const Track<TrackState>& track,
                   const CoordinateConverter& coordinate_converter);

bool IsInParkingArea(const SemanticMapManager& semantic_map_manager,
                     const Track<TrackState>& track,
                     const CoordinateConverter& coordinate_converter);

// Check if the given track is in a vegetation zone.
bool IsInVegetationZone(const SemanticMapManager& semantic_map_manager,
                        const Track<TrackState>& track,
                        const CoordinateConverter& coordinate_converter);

bool IsInFenDetectionRange(const Vec3d& point, const VehiclePose& pose);

bool IsInLane(const SemanticMapManager& semantic_map_manager,
              const Track<TrackState>& track, double threshold);
bool IsInLane(const SemanticMapManager& semantic_map_manager,
              const Vec2d& centroid, double threshold);

float DistanceToCurb(const SemanticMapManager& semantic_map_manager,
                     const Vec2d& SmoothPos);

Polygon2d GetContour(const Track<TrackState>& track);

void RenderContourCvs(const Polygon2d& contour, double ground_z,
                      vis::Color color, int size, vis::Canvas* canvas,
                      vis::BorderStyleProto::LineStyle border_line_style =
                          vis::BorderStyleProto::SOLID);

Vec2d ComputeCentroid(const std::vector<Vec2d>& points);

Vec2d ComputeWeightedObstacleCentroid(
    const std::vector<TrackState::ObstacleInfo>& obstacle_infos);

Vec2d ComputeWeightedObstacleCentroid(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m);

Vec2d ComputeCentroid(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m);

// Compute squared Mahalanobis distance.
template <int kDim>
double ComputeMahalanobisSquare(const Eigen::Matrix<double, kDim, 1>& point,
                                const Eigen::Matrix<double, kDim, 1>& mean,
                                const Eigen::Matrix<double, kDim, kDim>& cov) {
  const auto offset = point - mean;
  QCHECK(!cov.isZero(1e-6)) << "\nZero-like matrix: \n" << cov;
  return offset.transpose() * cov.inverse() * offset;
}

std::optional<Vec2d> GetBboxCenter(
    const LaserMeasurementProto& laser_measurement,
    bool use_min_area_bbox = false);

// Velocity post processing function, mainly process some shooters.
void RefineMotion(const VehiclePose& pose, Track<TrackState>* track,
                  Vec2d* vel);

ObjectProto::MovingStateProto MovingStateToProto(
    const TrackState::MovingState& moving_state);

double ComputeCarAndCycHeadingMeasurementNoise(
    const Track<TrackState>& track, const MeasurementProto& curr_measurement,
    double init_heading_noise);

bool IsCertainStaticMeasurement(const MeasurementProto& measurement);

bool IsSuddenBreakHappening(const Track<TrackState>& track);

bool IsSlowMovingVehicle(const Track<TrackState>& track, double timestamp);

Polygon2d ShiftPoints(const Vec2d& shift,
                      const std::vector<Vec2d>& contour_points);

Polygon2d PredictContour(const double timestamp, const TrackState& track_state);

std::optional<qcraft::sensor_fov::SensorFov> ComputeLidarSensorFov(
    std::shared_ptr<const SensorFovsProto> sensor_fovs_proto);

bool IsTrackInLidarOccludedArea(
    const std::optional<qcraft::sensor_fov::SensorFov>& lidar_sensor_fov,
    const Track<TrackState>& track, const double timestamp);

template <typename PointsT>
Polygon2d ToPolygon2d(const PointsT& points_proto) {
  std::vector<Vec2d> points;
  points.reserve(points_proto.size());
  for (const auto& p : points_proto) {
    points.push_back(Vec2dFromProto(p));
  }
  return Polygon2d(points);
}

double ComputePValueFromChiSquareDistr(const double value, const int dof);

// Input timestamp smaller than state timestamp may cause undefined behavior in
// Motion filter 2.0. Use SafePredictPos to prevent ub.
template <typename State>
std::optional<tracker::StateData> SafePredictPos(const Track<State>& track,
                                                 double timestamp) {
  const State* checkpoint = nullptr;
  for (int i = track.checkpoints.size() - 1; i >= 0; --i) {
    if (track.checkpoints.value(i).state_timestamp < timestamp) {
      checkpoint = &track.checkpoints.value(i);
      break;
    }
  }
  if (nullptr == checkpoint) return std::nullopt;
  return checkpoint->estimator_3d.ComputePrediction(timestamp);
}

template <typename State>
const State* FindLatestCheckPointBeforeTime(const Track<State>& track,
                                            double timestamp) {
  for (int i = track.checkpoints.size() - 1; i >= 0; --i) {
    if (track.checkpoints.value(i).state_timestamp < timestamp) {
      return &track.checkpoints.value(i);
    }
  }
  return nullptr;
}

template <typename State>
const State* FindLatestLidarCheckPointBeforeTimeWithBbox(
    const Track<State>& track, double timestamp) {
  for (int i = track.checkpoints.size() - 1; i >= 0; --i) {
    const auto& ckpt = track.checkpoints.value(i);
    if (ckpt.state_timestamp < timestamp && ckpt.measurement) {
      if (ckpt.measurement->has_laser_measurement() &&
          ckpt.measurement->laser_measurement().has_detection_bounding_box()) {
        return &ckpt;
      }
    }
  }
  return nullptr;
}

template <typename State>
const State* FindLatestCheckPointBeforeTimeWithType(
    const Track<State>& track, double timestamp,
    const MeasurementsProto_GroupType type) {
  for (int i = track.checkpoints.size() - 1; i >= 0; --i) {
    const auto& ckpt = track.checkpoints.value(i);
    if (ckpt.state_timestamp < timestamp && ckpt.measurement) {
      if ((type == MeasurementsProto::LIDAR &&
           ckpt.measurement->has_laser_measurement()) ||
          (type == MeasurementsProto::CAMERA &&
           ckpt.measurement->has_camera3d_measurement()) ||
          (type == MeasurementsProto::RADAR &&
           ckpt.measurement->has_radar_measurement())) {
        return &ckpt;
      }
    }
  }
  return nullptr;
}

template <typename State>
double GetLatestLaserOrCameraMeasurementTimestamp(
    const Track<TrackState>& track) {
  double latest_timestamp = 0.0;
  const int m_num = track.measurement_history.size();
  for (int i = m_num - 1; i >= 0; --i) {
    const auto* m = track.measurement_history.value(i);
    if (m->has_laser_measurement() || m->has_camera3d_measurement()) {
      latest_timestamp = m->timestamp();
      break;
    }
  }
  return latest_timestamp;
}

template <typename TrackState, typename MotionStateProto>
bool TrackStateToMotionStateProto(const TrackState& track_state,
                                  const Vec2d& offset,
                                  const CoordinateConverter* target_coord,
                                  MotionStateProto* proto) {
  if (proto == nullptr) {
    return false;
  }

  proto->set_timestamp(track_state.state_timestamp);

  const auto s = track_state.estimator_3d.GetStateData();
  Vec2d pos = track_state.key_point + offset;

  // Convert to current smooth.
  if (target_coord && track_state.coordinate_converter) {
    Vec2dToProto(target_coord->GlobalToSmooth(
                     track_state.coordinate_converter->SmoothToGlobal(pos)),
                 proto->mutable_pos());
    proto->set_heading(target_coord->GlobalYawToSmooth(
        track_state.coordinate_converter->SmoothYawToGlobal(s.GetYaw())));

  } else {
    Vec2dToProto(pos, proto->mutable_pos());
    proto->set_heading(s.GetYaw());
  }
  // vel.
  proto->set_velocity(s.GetVel());

  return true;
}

template <typename TrackState>
bool ShiftHistoryKeyPoints(const Vec2d& offset, Track<TrackState>* track) {
  if (nullptr == track) return false;

  for (int i = track->checkpoints.size() - 1; i >= 0; --i) {
    track->checkpoints.value(i).key_point += offset;
  }

  return true;
}

template <typename TrackState>
bool ResetEstimatorStatePos(const tracker::StateData& state,
                            Track<TrackState>* track) {
  if (nullptr == track) return false;

  const auto cur_state = track->track_state.estimator_3d.GetStateData();
  track->track_state.estimator_3d.SetStateData(state);
  const Vec2d current_to_reset = state.GetStatePos() - cur_state.GetStatePos();
  // Shift history key points to keep consistent with the latest checkpoint if
  // imm has been resetted.
  QCHECK(ShiftHistoryKeyPoints(current_to_reset, track));

  return true;
}

template <typename TrackState>
Vec2d FindKeyPointToImmStateOffset(const Track<TrackState>& track, int idx) {
  if (idx < 0 || idx >= track.checkpoints.size()) {
    return Vec2d(0.0, 0.0);
  }
  return track.checkpoints.value(idx)
             .estimator_3d.GetStateData()
             .GetStatePos() -
         track.checkpoints.value(idx).key_point;
}

// Align all key points to the latest estimator state if they are not equal.
template <typename TrackState>
void AlignKeyPointsToLatestEstimator(Track<TrackState>* track) {
  if (track->checkpoints.empty()) return;
  const Vec2d offset =
      FindKeyPointToImmStateOffset(*track, track->checkpoints.size() - 1);
  ShiftHistoryKeyPoints(offset, track);
}
bool IsMeasurementGroupContainSpecificMeasurementType(
    const MeasurementsProto_GroupType group_type,
    const MeasurementProto::MeasurementCase measurement_type);

}  // namespace qcraft::tracker::tracker_util

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACKER_UTIL_H_
