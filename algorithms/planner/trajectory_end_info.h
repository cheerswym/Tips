#ifndef ONBOARD_PLANNER_TRAJECTORY_END_INFO_H_
#define ONBOARD_PLANNER_TRAJECTORY_END_INFO_H_

#include <string>

namespace qcraft::planner {

struct TrajectoryEndInfo {
  // The nearest stationary spacetime object that slow the trajectory.
  std::string st_traj_id;
  double end_s = 0.0;
  // The value of trajectory entering following standstill distance.
  double intrusion_value = 0.0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_TRAJECTORY_END_INFO_H_
