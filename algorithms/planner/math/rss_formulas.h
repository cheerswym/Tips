#ifndef ONBOARD_PLANNER_RSS_FORMULAS_H_
#define ONBOARD_PLANNER_RSS_FORMULAS_H_

#include <string>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/proto/perception.pb.h"

// These formulas below are refered to Mobileye's rss
// https://arxiv.org/pdf/1708.06374.pdf
// https://github.com/intel/ad-rss-lib#release_4

namespace qcraft {
namespace planner {

namespace RssLongitudialFormulas {

constexpr double kMinSafeDistance = 3.0;

struct VehicleState {
  double response_time = 1.0;  // s.
  double current_v;            // m/s.   longitudial speed
  double max_v;
  double max_brake = 3.0;  // m/s^2
  double min_brake = 3.0;
  double max_accel = 1.0;
  ObjectType type;
  AABox2d vehicle_box;
  std::string id;

  std::string DebugString() const {
    return absl::StrFormat(
        "id:%s, response_time:%.2f, v:%.2f, max_brake:%.2f, "
        "min_brake:%.2f, max_accel:%.2f",
        id, response_time, current_v, max_brake, min_brake, max_accel);
  }
};

double SafeLongitudinalDistanceSameDirection(const VehicleState& front_vehicle,
                                             const VehicleState& rear_vehicle);

}  // namespace RssLongitudialFormulas
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_RSS_FORMULAS_H_
