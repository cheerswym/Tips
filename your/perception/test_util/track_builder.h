#ifndef ONBOARD_PERCEPTION_TEST_UTIL_TRACK_BUILDER_H_
#define ONBOARD_PERCEPTION_TEST_UTIL_TRACK_BUILDER_H_

#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/radar.pb.h"

namespace qcraft {

class TrackBuilder {
 public:
  // No good way to init a track with a default value since a track doesn't own
  // measurements and measurements are needed to provide a valid track.
  TrackBuilder() = default;

  TrackBuilder& set_id(uint32_t id);

  TrackBuilder& set_type(MeasurementType type);

  TrackBuilder& set_ref_point(tracker::TrackState::RefPoint ref_point);

  TrackBuilder& set_first_timestamp(double first_timestamp);

  TrackBuilder& set_contour(const Polygon2d& poly);

  TrackBuilder& set_offroad(bool is_offroad);

  TrackBuilder& set_estimator(const tracker::Estimator& extimator);
  TrackBuilder& set_life_state(TrackLifeState life_state);

  TrackBuilder& AddMeasurementToMeasurementHistory(
      const MeasurementProto& measurement_proto);

  // Adjust first_timestamp, last_timestamp and last_laser_timestamp according
  // to measurement history.
  TrackBuilder& AdjustFirstLastTimestampsGivenMeasurementHistory();

  tracker::Track<tracker::TrackState> Build();

 private:
  tracker::Track<tracker::TrackState> track_;
};
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TEST_UTIL_TRACK_BUILDER_H_
