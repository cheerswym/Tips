#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_FILTER_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_FILTER_H_

#include <algorithm>
#include <limits>
#include <typeinfo>

#include "onboard/perception/tracker/motion_filter_2/extent_model.h"
#include "onboard/perception/tracker/motion_filter_2/kalman_filter_base.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"
#include "onboard/perception/tracker/motion_filter_2/state_data.h"
#include "onboard/perception/tracker/motion_filter_2/utils.h"

namespace qcraft::tracker {

template <typename MotionModelType>
class ExtentFilter : public KalmanFilterBase<MotionModelType> {
 public:
  static_assert(
      std::is_same<typename MotionModelType::StateType, BBoxState>::value,
      "State type of Extent Filter must be BBox State");
  using Base = KalmanFilterBase<MotionModelType>;
  using typename Base::CovarianceType;
  using typename Base::StateType;
  using typename Base::T;
  using S = StateType;

  ExtentFilter() = default;
  explicit ExtentFilter(const MotionFilterParamProto& params)
      : params_(params) {
    P_.setIdentity();
    InitParams(params);
  }

  void Init(const StateType& x_in) { x_ = x_in; }

  void InitParams(const MotionFilterParamProto& params) {
    CheckParams(params);
    P_(S::LENGTH, S::LENGTH) = Sqr(params.state_length_init_std());
    P_(S::WIDTH, S::WIDTH) = Sqr(params.state_width_init_std());
    P_(S::HEIGHT, S::HEIGHT) = Sqr(params.state_height_init_std());

    motion_model_.InitParams(params);
  }

  ExtentStateData ComputePrediction(const StateType& x_in,
                                    const CovarianceType& P_in, T dt) const {
    const auto x = motion_model_.ComputeStateTransition(x_in, dt);
    const auto Q = motion_model_.ComputeProcessNoise(dt);
    const auto F = motion_model_.ComputeF(dt);
    const auto P = F * P_in * F.transpose() + Q;
    return ExtentStateData(x, P);
  }

  S Predict(T dt) {
    const auto state_data = ComputePrediction(x_, P_, dt);
    x_ = state_data.state();
    P_ = state_data.state_cov();
    return x_;
  }

  template <typename MeasurementType>
  S Update(const MeasurementType& z,
           const std::optional<MotionFilterParamProto>& params = std::nullopt) {
    using MeasModelType =
        typename MeasModelTypeSelector<StateType, MeasurementType>::type;
    using M = MeasurementType;
    MeasModelType meas_model;

    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params ? *params : params_, &R));

    const auto H = meas_model.ComputeH();

    const Covariance<M> S = H * P_ * H.transpose() + R;

    const M hx = meas_model.ComputeExpectedMeasurement(x_);

    const KalmanGain<StateType, M> K = P_ * H.transpose() * S.inverse();

    const M y = z - hx;

    x_ += K * y;
    P_ -= K * H * P_;

    extent_filter_proto_ = GenerateExtentFilterProto(*this, z, R);

    return x_;
  }

  ExtentFilterProto GetExtentFilterProto() const {
    return extent_filter_proto_;
  }

 protected:
  using Base::P_;
  using Base::x_;

 private:
  // Check if input params valid.
  void CheckParams(const MotionFilterParamProto& params) {
    QCHECK(params.has_state_length_init_std());
    QCHECK(params.has_state_width_init_std());
    QCHECK(params.has_state_height_init_std());

    QCHECK(params.has_state_length_process_noise_std());
    QCHECK(params.has_state_width_process_noise_std());
    QCHECK(params.has_state_height_process_noise_std());

    QCHECK(params.has_state_length_measurement_noise_std());
    QCHECK(params.has_state_width_measurement_noise_std());
    QCHECK(params.has_state_height_measurement_noise_std());
  }

 private:
  MotionModelType motion_model_;
  MotionFilterParamProto params_;
  // Extent filter debug proto
  ExtentFilterProto extent_filter_proto_;
};
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_FILTER_H_
