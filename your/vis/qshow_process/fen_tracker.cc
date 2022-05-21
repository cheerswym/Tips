#include "onboard/vis/qshow_process/fen_tracker.h"

#include <algorithm>
#include <limits>
#include <map>
#include <set>

#include "absl/strings/str_format.h"
#include "onboard/async/parallel_for.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/perception/type_util.h"

namespace qcraft {
namespace {

const std::map<int, int> TypeFromTrackToFen = {
    {MT_VEHICLE, 1}, {MT_PEDESTRIAN, 2}, {MT_CYCLIST, 3}};

struct AssociationInfo {
  double ComputeWeight() const {
    if (dist2 > Sqr(0.1) * Sqr(kMaxMatchDistPerSec)) {
      return 0.0;
    }
    return 0.1 / (0.1 * (dist2 + 25.0 * Sqr(0.2) + 1.0));
  }
  // Squared distance between track centers.
  double dist2;
};

// Return if we should promote this track. A track is promoted if the latest
// kMaxNumUpdatesToPromote measurements are associated within
bool ShouldPromoteTrack(const FenTrack& track) {
  const int num_measurements = track.measurement_history.size();
  if (num_measurements < kMaxNumUpdatesToPromote) {
    return false;
  }
  return true;
}

std::vector<const MeasurementProto*> GetMeasurementsWithType(
    const MeasurementsProto& measurements_proto,
    const MeasurementProto::MeasurementCase& type) {
  const int num_measurements = measurements_proto.measurements_size();
  std::vector<const MeasurementProto*> results;
  results.reserve(num_measurements);
  for (const auto& measurement : measurements_proto.measurements()) {
    if (measurement.Measurement_case() == type) {
      results.push_back(&measurement);
    }
  }
  return results;
}

}  // namespace

Box2d FenTracker::PredictBbox(const double timestamp, FenTrack* track) {
  const int tracked_size = track->measurement_history.size();
  auto measurement_back_bbox_x = track->measurement_history.back()
                                     .second->laser_measurement()
                                     .detection_bounding_box()
                                     .x();
  auto measurement_front_bbox_x = track->measurement_history.front()
                                      .second->laser_measurement()
                                      .detection_bounding_box()
                                      .x();
  auto measurement_back_bbox_y = track->measurement_history.back()
                                     .second->laser_measurement()
                                     .detection_bounding_box()
                                     .y();
  auto measurement_front_bbox_y = track->measurement_history.front()
                                      .second->laser_measurement()
                                      .detection_bounding_box()
                                      .y();
  const Vec2d pos_shift =
      tracked_size <= 1
          ? Vec2d{0, 0}
          : Vec2d{(measurement_back_bbox_x - measurement_front_bbox_x) /
                      (tracked_size - 1),
                  (measurement_back_bbox_y - measurement_front_bbox_y) /
                      (tracked_size - 1)};
  auto box = track->bounding_box.value();
  box.Shift(pos_shift);
  return box;
}

std::vector<std::pair<Box2d, int>> FenTracker::GetTrackedObjects() {
  std::vector<ObjectProto> objects(tracks_.size());
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int track_index) {
    const auto& track = tracks_[track_index];
    if (!track.promoted) return;

    ObjectProto& object = objects[track_index];

    object.set_id(absl::StrFormat("%d", track.id));
    object.set_type(type_util::ToObjectType(track.type));

    const Vec2d object_pos = track.bounding_box->center();
    Vec2dToProto(object_pos, object.mutable_pos());

    auto* bb = object.mutable_bounding_box();
    bb->set_x(track.bounding_box->center_x());
    bb->set_y(track.bounding_box->center_y());
    bb->set_heading(track.bounding_box->heading());
    bb->set_width(track.bounding_box->width());
    bb->set_length(track.bounding_box->length());
    object.set_bounding_box_source(ObjectProto::FIERY_EYE_NET);
  });

  std::vector<std::pair<Box2d, int>> objects_vec;
  for (auto& object : objects) {
    if (object.has_id()) {
      Box2d box(Vec2d{object.mutable_bounding_box()->x(),
                      object.mutable_bounding_box()->y()},
                object.mutable_bounding_box()->heading(),
                object.mutable_bounding_box()->length(),
                object.mutable_bounding_box()->width());
      std::pair<Box2d, int> obj =
          std::make_pair(box, TypeFromTrackToFen.at(object.type()));
      objects_vec.emplace_back(obj);
    }
  }
  return objects_vec;
}

std::vector<std::pair<Box2d, int>> FenTracker::TrackObjects(
    double timestamp, MeasurementsProto measurements) {
  // If we jump to an earlier timestamp in playback, clear tracks to avoid
  // unexpected incorrect associations.
  if (timestamp < prev_timestamp_) {
    tracks_.clear();
  }
  prev_timestamp_ = timestamp;
  // Associate each measurements to current tracks.
  AssociateMeasurementsTracks(measurements);
  // Delete expired tracks.
  RemoveExpiredTracks(timestamp);
  // Get objects.
  return GetTrackedObjects();
}

void FenTracker::CreateNewTracksFromMeasurements(
    const std::vector<const MeasurementProto*>& measurements) {
  for (const auto* m : measurements) {
    if (!m->has_laser_measurement()) {
      return;
    }
    const auto& laser_measurement = m->laser_measurement();
    tracks_.emplace_back();
    auto& track = tracks_.back();
    track.id = GenerateNewTrackId();
    track.type = m->type();
    if (laser_measurement.has_detection_bounding_box()) {
      track.bounding_box = Box2d(laser_measurement.detection_bounding_box());
    } else {
      track.bounding_box = std::nullopt;
    }
    // Set initial heading if detection exists
    if (laser_measurement.has_detection_bounding_box()) {
      track.heading = laser_measurement.detection_bounding_box().heading();
    }
    UpdateTrackFromMeasurement(m, &track);
  }
}

void FenTracker::UpdateTracksFromMeasurements(
    const std::vector<std::vector<const MeasurementProto*>>&
        matched_measurements_per_track) {
  QCHECK_EQ(tracks_.size(), matched_measurements_per_track.size());
  double latest_measurement_timestamp = std::numeric_limits<double>::lowest();
  for (const auto& measurements : matched_measurements_per_track) {
    if (measurements.empty()) {
      continue;
    }
    for (const auto* m : measurements) {
      if (latest_measurement_timestamp < m->timestamp()) {
        latest_measurement_timestamp = m->timestamp();
      }
    }
  }
  // If no measurements associated with any track, return.
  if (latest_measurement_timestamp <= 0.) {
    return;
  }
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    auto& track = tracks_[i];
    const auto& measurements = matched_measurements_per_track[i];
    if (measurements.empty()) {
      return;
    }
    // Store measurements in ascending order in terms of measurement
    // timestamp. Using copy since measurement is very light weight.
    auto cmp = [](const MeasurementProto* left, const MeasurementProto* right) {
      return left->timestamp() < right->timestamp();
    };
    std::set<const MeasurementProto*, decltype(cmp)> measurements_to_process(
        cmp);
    for (const auto* measurement : measurements) {
      auto* to_process_m = measurement;
      measurements_to_process.insert(to_process_m);
    }
    for (const auto* measurement : measurements_to_process) {
      UpdateTrackFromMeasurement(measurement, &track);
    }
  });
}

void FenTracker::UpdateTrackFromMeasurement(const MeasurementProto* measurement,
                                            FenTrack* track) const {
  if (measurement->has_laser_measurement()) {
    UpdateTrackFromLaserMeasurement(measurement, track);
  }
}

void FenTracker::UpdateTrackFromLaserMeasurement(
    const MeasurementProto* measurement, FenTrack* track) const {
  QCHECK_NOTNULL(track);
  track->last_laser_timestamp = measurement->timestamp();

  QCHECK(measurement->has_laser_measurement());
  const auto& laser_measurement = measurement->laser_measurement();

  if (laser_measurement.has_detection_bounding_box()) {
    track->bounding_box = Box2d(laser_measurement.detection_bounding_box());
  } else {
    track->bounding_box = std::nullopt;
  }
  track->measurement_history.PushBackAndClearStale(
      measurement->timestamp(), measurement,
      kMaxMeasurementHistoryBufferLength);
  if (ShouldPromoteTrack(*track)) {
    track->promoted = true;
  }
}

std::vector<int> FenTracker::AssociateLaserMeasurements(
    const double timestamp,
    const std::vector<const MeasurementProto*>& laser_measurements) {
  if (laser_measurements.empty() || tracks_.empty()) return {};
  const int num_measurements = laser_measurements.size();
  Eigen::MatrixXd weight_matrix(num_measurements, tracks_.size());
  weight_matrix.setZero();
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    // Compute the predicted track position.
    auto& track = tracks_[i];
    for (int j = 0; j < num_measurements; ++j) {
      const auto* measurement = laser_measurements[j];
      QCHECK(measurement->has_laser_measurement());
      const auto& laser_measurement = measurement->laser_measurement();
      Vec2d choose_m_ref_point_pos(
          laser_measurement.detection_bounding_box().x(),
          laser_measurement.detection_bounding_box().y());
      // Association info.
      AssociationInfo association_info;
      Vec2d pos_pred_at_m_ref;
      // If the ref_point_offset returns nullopt, to unify
      // measurement distance for fair comparison, we use bbox center
      // or centroid for association.
      if (track.bounding_box &&
          laser_measurement.has_detection_bounding_box()) {
        pos_pred_at_m_ref =
            PredictBbox(measurement->timestamp(), &track).center();
        association_info.dist2 =
            (pos_pred_at_m_ref - choose_m_ref_point_pos).squaredNorm();
      }
      // Set weight to -std::numeric_limits<double>::max() to skip matching in
      // ComputeMatches function below.
      if (j < 0 || j >= weight_matrix.rows() || i < 0 ||
          i >= weight_matrix.cols()) {
        continue;
      }
      weight_matrix(j, i) = association_info.ComputeWeight();
    }
  });
  return ComputeMatches(weight_matrix);
}

std::vector<int> FenTracker::ComputeMatches(
    const Eigen::MatrixXd& weight_matrix) {
  const int rows = weight_matrix.rows();
  const int cols = weight_matrix.cols();
  std::vector<bool> matched_rows(rows);  // measurements
  std::vector<bool> matched_cols(cols);  // tracks
  std::vector<int> match_result(rows, -1);
  while (true) {
    double max_val = -std::numeric_limits<double>::max();
    int max_val_col, max_val_row;
    for (int i = 0; i < cols; ++i) {
      if (matched_cols[i]) continue;
      for (int j = 0; j < rows; ++j) {
        if (matched_rows[j]) continue;
        if (weight_matrix(j, i) > max_val) {
          max_val = weight_matrix(j, i);
          max_val_col = i;
          max_val_row = j;
        }
      }
    }
    if (max_val > 0.0) {
      matched_cols[max_val_col] = true;
      matched_rows[max_val_row] = true;
      match_result[max_val_row] = max_val_col;
    } else {
      break;
    }
  }
  return match_result;
}

void FenTracker::AssociateMeasurementsTracks(
    MeasurementsProto measurements_proto) {
  MeasurementsProto new_measurements_proto;
  new_measurements_proto = std::move(measurements_proto);
  if (new_measurements_proto.measurements_size() > 0) {
    // To make sure measurement_history_groups_ has a longer
    // history than measurement_history to avoid dangling pointer, the
    // measurement history buffer should be greater than
    // kMaxMeasurementHistoryBufferLength +
    // max(kMaxTrackLifeWithoutUpdate, kMaxOffroadTrackLifeWithoutUpdate),
    // besides we expand extra 0.5s to the measurement_history_groups_ buffer.
    const double buffer_time =
        0.5 + kMaxMeasurementHistoryBufferLength + kMaxTrackLifeWithoutUpdate;
    const double max_measurement_timestamp =
        new_measurements_proto.max_timestamp();
    measurement_history_groups_.PushBackAndClearStale(
        max_measurement_timestamp, std::move(new_measurements_proto),
        buffer_time);
    // Update pose and coordinate converter.
    const auto& to_process_measurements =
        measurement_history_groups_.back().second;
    AssociateMeasurementsAndUpdateTracks(
        to_process_measurements.max_timestamp(), to_process_measurements);
  }
}

void FenTracker::AssociateMeasurementsAndUpdateTracks(
    double timestamp, const MeasurementsProto& measurements_proto) {
  const auto laser_measurements = GetMeasurementsWithType(
      measurements_proto, MeasurementProto::kLaserMeasurement);
  // On the very first iteration when there are no tracks yet, create a
  // track from each measurement.
  if (tracks_.empty()) {
    CreateNewTracksFromMeasurements(laser_measurements);
    return;
  }
  const auto laser_m_matches =
      AssociateLaserMeasurements(timestamp, laser_measurements);
  QCHECK_EQ(laser_m_matches.size(), laser_measurements.size());

  std::vector<std::vector<const MeasurementProto*>>
      matched_measurements_per_track(tracks_.size());
  std::vector<const MeasurementProto*> unmatched_measurements_laser;
  unmatched_measurements_laser.reserve(laser_m_matches.size());
  for (int i = 0; i < laser_m_matches.size(); ++i) {
    const int track_ind = laser_m_matches[i];
    if (track_ind >= 0) {
      matched_measurements_per_track[track_ind].push_back(
          laser_measurements[i]);
    } else {
      unmatched_measurements_laser.push_back(laser_measurements[i]);
    }
  }
  UpdateTracksFromMeasurements(matched_measurements_per_track);
  // Create new tracks for laser measurements without associations.
  CreateNewTracksFromMeasurements(unmatched_measurements_laser);
}

void FenTracker::RemoveExpiredTracks(double timestamp) {
  // Remove dead tracks.
  std::set<uint32_t> track_ids_to_delete;
  for (const auto& track : tracks_) {
    if (timestamp - track.last_laser_timestamp > kMaxTrackLifeWithoutUpdate) {
      track_ids_to_delete.insert(track.id);
    }
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const auto& track) {
                                 return ContainsKey(track_ids_to_delete,
                                                    track.id);
                               }),
                tracks_.end());
}

}  // namespace qcraft
