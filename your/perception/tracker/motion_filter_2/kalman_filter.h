#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_H_

#include <algorithm>
#include <limits>
#include <typeinfo>

#include "onboard/perception/tracker/motion_filter_2/kalman_filter_base.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"
#include "onboard/perception/tracker/motion_filter_2/normal_distribution.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
#include "onboard/perception/tracker/motion_filter_2/state_data.h"
#include "onboard/perception/tracker/motion_filter_2/utils.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {

template <typename MotionModelType>
class KalmanFilter : public KalmanFilterBase<MotionModelType> {
 public:
  static_assert(
      std::is_same<typename MotionModelType::StateType, PointState>::value,
      "State type of Kalman Filter must be Point State");
  using Base = KalmanFilterBase<MotionModelType>;
  using typename Base::CovarianceType;
  using typename Base::StateType;
  using typename Base::T;
  using S = StateType;

  // KalmanFilter() = default;
  explicit KalmanFilter(const MotionFilterParamProto& params,
                        bool enable_debug = false)
      : params_(params) {
    P_.setIdentity();
    InitParams(params);
    enable_debug_ = enable_debug;
  }

  void Init(const StateType& x_in) { x_ = x_in; }

  void InitParams(const MotionFilterParamProto& params) {
    CheckParams(params);
    P_(S::X, S::X) = Sqr(params.state_x_init_std());
    P_(S::Y, S::Y) = Sqr(params.state_y_init_std());
    P_(S::VEL_X, S::VEL_X) = Sqr(params.state_vel_x_init_std());
    P_(S::VEL_Y, S::VEL_Y) = Sqr(params.state_vel_y_init_std());
    P_(S::ACC_X, S::ACC_X) = Sqr(params.state_acc_x_init_std());
    P_(S::ACC_Y, S::ACC_Y) = Sqr(params.state_acc_y_init_std());

    motion_model_.InitParams(params);
  }

  PointStateData ComputePrediction(const StateType& x_in,
                                   const CovarianceType& P_in, T dt) const {
    const auto x = motion_model_.ComputeStateTransition(x_in, dt);
    const auto Q = motion_model_.ComputeProcessNoise(dt);
    const auto F = motion_model_.ComputeF(dt);
    const auto P = F * P_in * F.transpose() + Q;
    return PointStateData(x, P);
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
      const MeasurementType& z, const StateType& x_in,
      const CovarianceType& P_in, T dt,
      const std::optional<MotionFilterParamProto>& params) const {
    const PointStateData predict_state_data = ComputePrediction(x_in, P_in, dt);
    return ComputeMahalanobisDistance(predict_state_data.state(),
                                      predict_state_data.state_cov(), z,
                                      params ? *params : params_);
  }

  S Predict(T dt) {
    const auto state_data = ComputePrediction(x_, P_, dt);
    x_ = state_data.state();
    P_ = state_data.state_cov();
    return x_;
  }

  template <typename MeasurementType>
  S Update(const MeasurementType& z,
           const std::optional<MotionFilterParamProto>& params = std::nullopt,
           bool likelihood = false) {
    using MeasModelType =
        typename MeasModelTypeSelector<StateType, MeasurementType>::type;
    using M = MeasurementType;
    MeasModelType meas_model;

    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params ? *params : params_, &R));

    const auto H = meas_model.ComputeH();

    const Covariance<M> S = H * P_ * H.transpose() + R;

    const M hx = meas_model.ComputeExpectedMeasurement(x_);

    if (likelihood) {
      NormalDistribution<M> norm(S);
      likelihood_ = std::max(norm.pdf(z - hx), 1e-6);
    }

    const KalmanGain<StateType, M> K = P_ * H.transpose() * S.inverse();

    const M y = z - hx;

    x_ += K * y;
    P_ -= K * H * P_;

    if constexpr (std::is_same<MotionModelType, PointModelCP>::value) {
      x_.vx() = 0.0;
      x_.vy() = 0.0;
      x_.ax() = 0.0;
      x_.ay() = 0.0;
    } else if constexpr (std::is_same<MotionModelType, PointModelCV>::value) {
      x_.ax() = 0.0;
      x_.ay() = 0.0;
    }

    if (enable_debug_) {
      GenerateMeasModelProto(z, R, filter_proto_.add_meas_model());
    }

    return x_;
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const MeasurementType& z,
                               const std::optional<MotionFilterParamProto>&
                                   params = std::nullopt) const {
    return ComputeMahalanobisDistance(x_, P_, z, params ? *params : params_);
  }

  T likelihood() const { return likelihood_; }

  FilterProto filter_proto() const { return filter_proto_; }

  void ClearFilterInfo() { filter_proto_.Clear(); }

 protected:
  using Base::enable_debug_;
  using Base::P_;
  using Base::x_;

 private:
  // Check if input params valid.
  void CheckParams(const MotionFilterParamProto& params) {
    QCHECK(params.has_state_x_init_std());
    QCHECK(params.has_state_y_init_std());
    QCHECK(params.has_state_vel_x_init_std());
    QCHECK(params.has_state_vel_y_init_std());
    QCHECK(params.has_state_acc_x_init_std());
    QCHECK(params.has_state_acc_y_init_std());

    QCHECK(params.has_state_x_process_noise_std());
    QCHECK(params.has_state_y_process_noise_std());
    QCHECK(params.has_state_vel_x_process_noise_std());
    QCHECK(params.has_state_vel_y_process_noise_std());
    QCHECK(params.has_state_acc_x_process_noise_std());
    QCHECK(params.has_state_acc_y_process_noise_std());

    QCHECK(params.has_state_x_measurement_noise_std());
    QCHECK(params.has_state_y_measurement_noise_std());
    QCHECK(params.has_state_vel_x_measurement_noise_std());
    QCHECK(params.has_state_vel_y_measurement_noise_std());
    QCHECK(params.has_state_acc_x_measurement_noise_std());
    QCHECK(params.has_state_acc_y_measurement_noise_std());
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const StateType& x, const CovarianceType& P,
                               const MeasurementType& z,
                               const MotionFilterParamProto& params) const {
    using MeasModelType =
        typename MeasModelTypeSelector<StateType, MeasurementType>::type;
    using M = MeasurementType;
    MeasModelType meas_model;

    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params, &R));

    const auto H = meas_model.ComputeH();

    const Covariance<M> S = H * P * H.transpose() + R;

    const M hx = meas_model.ComputeExpectedMeasurement(x);

    const M y = z - hx;

    return ComputeMDistance(y, S);
  }

 private:
  MotionModelType motion_model_;
  MotionFilterParamProto params_;
  T likelihood_{};
  FilterProto filter_proto_;
};
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_KALMAN_FILTER_H_
