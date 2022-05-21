#ifndef ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACKER_H_
#define ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACKER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/lite/lite_module.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/vis/qshow_process/fen_track.h"

namespace qcraft {

class FenTracker {
 public:
  explicit FenTracker(ThreadPool* thread_pool) : thread_pool_(thread_pool) {}

  FenTracker(const FenTracker&) = delete;
  FenTracker& operator=(const FenTracker&) const = delete;

  std::vector<std::pair<Box2d, int>> TrackObjects(
      double timestamp, MeasurementsProto measurements);

 private:
  void AssociateMeasurementsAndUpdateTracks(
      double timestamp, const MeasurementsProto& measurements_proto);
  void AssociateMeasurementsTracks(MeasurementsProto measurements_proto);
  std::vector<int> AssociateLaserMeasurements(
      const double timestamp,
      const std::vector<const MeasurementProto*>& laser_measurements);

  void RemoveExpiredTracks(double timestamp);
  uint32_t GenerateNewTrackId() { return id_counter_++; }

  // Create a new track from the input measurement, and append it to tracks_.
  void CreateNewTracksFromMeasurements(
      const std::vector<const MeasurementProto*>& measurements);

  void UpdateTracksFromMeasurements(
      const std::vector<std::vector<const MeasurementProto*>>&
          matched_measurements_per_track);

  // Update the given track from the input laser measurement.
  void UpdateTrackFromMeasurement(const MeasurementProto* measurement,
                                  FenTrack* track) const;

  // Update the given track from the input laser measurement.
  void UpdateTrackFromLaserMeasurement(const MeasurementProto* measurement,
                                       FenTrack* track) const;

  std::vector<std::pair<Box2d, int>> GetTrackedObjects();

  std::vector<int> ComputeMatches(const Eigen::MatrixXd& weight_matrix);
  Box2d PredictBbox(const double timestamp, FenTrack* track);

  std::vector<FenTrack> tracks_;
  uint32_t id_counter_ = 0;

  double prev_timestamp_ = 0.0;

  ThreadPool* const thread_pool_;

  static constexpr double kMaxMeasurementHistoryBufferLength = 1.0;  // 1.0s
  HistoryBuffer<MeasurementsProto> measurement_history_groups_;
};

}  // namespace qcraft

#endif  // ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACKER_H_
