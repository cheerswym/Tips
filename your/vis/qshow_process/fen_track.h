#ifndef ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACK_H_
#define ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACK_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft {

// The maximum duration of a track without any update. That is, a track can last
// this time before receiving any update.
constexpr double kMaxTrackLifeWithoutUpdate = 0.3;  // sec
// The minimum number of updates of each track before it is promoted.
constexpr int kMaxNumUpdatesToPromote = 3;
// The maximum distance between matched track and measurement.
const double kMaxMatchDistPerSec = 30.0;  // m/s

// A track that represents a tracked object for many frames.
struct FenTrack {
  struct RefPoint {
    enum Type {
      kNone = 0,
      kBBCenter,
    };
    Type type = kNone;
  };
  uint32_t id;
  MeasurementType type;
  double heading;
  double last_laser_timestamp;  // timestamp corresponding to last laser
                                // measurement.
  std::optional<Box2d> bounding_box;
  bool promoted = false;
  // Measurement history.
  HistoryBuffer<const MeasurementProto*> measurement_history;

  // // Construct and copy construct function.
  FenTrack() = default;
  FenTrack(const FenTrack& track) {
    id = track.id;
    type = track.type;
    heading = track.heading;
    last_laser_timestamp = track.last_laser_timestamp;
    bounding_box = track.bounding_box;
    promoted = track.promoted;
    measurement_history = track.measurement_history;
  }

  FenTrack& operator=(const FenTrack& track) {
    if (this == &track) {
      return *this;
    }
    id = track.id;
    type = track.type;
    heading = track.heading;
    last_laser_timestamp = track.last_laser_timestamp;
    bounding_box = track.bounding_box;
    promoted = track.promoted;
    measurement_history = track.measurement_history;
    return *this;
  }
};

}  // namespace qcraft

#endif  // ONBOARD_VIS_QSHOW_PROCESS_FEN_TRACK_H_
