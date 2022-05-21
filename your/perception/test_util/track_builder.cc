#include "onboard/perception/test_util/track_builder.h"

#include <algorithm>
#include <random>
#include <utility>

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"

namespace qcraft {

TrackBuilder& TrackBuilder::set_id(uint32_t id) {
  track_.track_state.id = id;
  return *this;
}

TrackBuilder& TrackBuilder::set_type(MeasurementType type) {
  track_.track_state.type = type;
  return *this;
}

TrackBuilder& TrackBuilder::set_ref_point(
    tracker::TrackState::RefPoint ref_point) {
  track_.track_state.ref_point = ref_point;
  return *this;
}

TrackBuilder& TrackBuilder::set_first_timestamp(double first_timestamp) {
  track_.track_state.first_timestamp = first_timestamp;
  return *this;
}

TrackBuilder& TrackBuilder::set_contour(const Polygon2d& poly) {
  track_.track_state.contour = poly;
  return *this;
}

TrackBuilder& TrackBuilder::set_offroad(bool is_offroad) {
  track_.track_state.offroad = is_offroad;
  return *this;
}

TrackBuilder& TrackBuilder::set_estimator(const tracker::Estimator& estimator) {
  track_.track_state.estimator_3d = estimator;
  return *this;
}

TrackBuilder& TrackBuilder::set_life_state(TrackLifeState life_state) {
  track_.track_state.life_state = life_state;
  return *this;
}

TrackBuilder& TrackBuilder::AddMeasurementToMeasurementHistory(
    const MeasurementProto& measurement_proto) {
  const double timestamp = measurement_proto.timestamp();
  if (!track_.measurement_history.empty()) {
    QCHECK_GT(timestamp, track_.measurement_history.back_time());
  }
  track_.measurement_history.push_back(timestamp, &measurement_proto);
  return *this;
}

TrackBuilder& TrackBuilder::AdjustFirstLastTimestampsGivenMeasurementHistory() {
  const auto& m_history = track_.measurement_history;
  QCHECK(!m_history.empty());
  track_.track_state.first_timestamp = m_history.front_value()->timestamp();
  track_.track_state.last_timestamp = m_history.back_value()->timestamp();
  for (auto m = m_history.rbegin(); m != m_history.rend(); ++m) {
    if (m->second->has_laser_measurement()) {
      track_.track_state.last_laser_timestamp = m->second->timestamp();
      break;
    }
  }
  return *this;
}

tracker::Track<tracker::TrackState> TrackBuilder::Build() { return track_; }

}  // namespace qcraft
