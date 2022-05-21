#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_BASE_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_BASE_H_

#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"

namespace qcraft::tracker {

template <typename MotionModelType>
class KalmanFilterBase {
 public:
  using StateType = typename MotionModelType::StateType;
  using T = typename StateType::Scalar;
  using CovarianceType = Covariance<StateType>;

  KalmanFilterBase() {}
  StateType state() const { return x_; }
  CovarianceType state_covariance() const { return P_; }
  void set_state(const StateType& x) { x_ = x; }
  void set_state_covariance(const CovarianceType& P) { P_ = P; }
  static bool DebugIsEnabled() { return enable_debug_; }

 protected:
  StateType x_;
  CovarianceType P_;
  static bool enable_debug_;
};

template <typename T>
bool KalmanFilterBase<T>::enable_debug_ = false;

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_BASE_H_
