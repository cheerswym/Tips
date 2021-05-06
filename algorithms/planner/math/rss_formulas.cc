#include "onboard/planner/math/rss_formulas.h"

#include <algorithm>
#include <cmath>

#include "onboard/lite/check.h"

namespace qcraft {
namespace planner {
namespace RssLongitudialFormulas {

namespace {

double StoppingDistance(double v, double brake) {
  return v * v / (2.0 * brake);
}

double TravelDistanceToStop(double current_v, double max_v,
                            double response_time, double accl, double brake) {
  QCHECK(current_v >= 0.0 && max_v >= 0.0);
  QCHECK_GE(response_time, 0.0);
  QCHECK(accl >= 0.0 && brake >= 0.0);

  double travel_dis = 0.0;
  max_v = std::fmax(max_v, current_v);

  // accelerate before reponse time
  const double v_after_accel =
      std::fmin(current_v + response_time * accl, max_v);
  const double accl_duration =
      accl == 0.0 ? 0.0 : (v_after_accel - current_v) / accl;
  const double max_v_duration = std::max(0.0, response_time - accl_duration);

  travel_dis += current_v * accl_duration;
  travel_dis += 0.5 * accl * accl_duration * accl_duration;
  travel_dis += v_after_accel * max_v_duration;

  // brake after reponse time
  travel_dis += StoppingDistance(v_after_accel, brake);

  return travel_dis;
}

}  // namespace

double SafeLongitudinalDistanceSameDirection(const VehicleState& front_vehicle,
                                             const VehicleState& rear_vehicle) {
  const double rear_dis_still_stop = TravelDistanceToStop(
      rear_vehicle.current_v, rear_vehicle.max_v, rear_vehicle.response_time,
      rear_vehicle.max_accel, rear_vehicle.min_brake);

  const double front_stop_dis =
      StoppingDistance(front_vehicle.current_v, front_vehicle.max_brake);

  return rear_dis_still_stop - front_stop_dis;
}

}  // namespace RssLongitudialFormulas
}  // namespace planner
}  // namespace qcraft
