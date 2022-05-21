#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_KALMAN_FILTER_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_KALMAN_FILTER_H_

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
class Img2DKalmanFilter : public KalmanFilterBase<MotionModelType> {
 public:
  static_assert(
      std::is_same<typename MotionModelType::StateType, Img2DState>::value,
      "State type of Img 2D Kalman FIlter must be Img2DState");
  using Base = KalmanFilterBase<MotionModelType>;
  using typename Base::CovarianceType;
  using typename Base::StateType;
  using typename Base::T;
  using S = StateType;

  Img2DKalmanFilter() = default;
  // the constructor here is different from Kalman Filter
  // for point model as this initiate x and params together
  // cause noise needs aspect information
  // TODO(Yan): delete init_time_stamp here and deal this in estimator
  explicit Img2DKalmanFilter(const MotionFilterParamProto& params,
                             const StateType& x_in, T init_time_stamp)
      : params_(params), last_timestamp_(init_time_stamp) {
    P_.setIdentity();
    Init(x_in);
    InitParams(params);
  }

  void Init(const StateType& x_in) { x_ = x_in; }

  void InitParams(const MotionFilterParamProto& params) {
    CheckParams(params);
    P_(S::X, S::X) =
        Sqr(2 * params.state_process_noise_position_weight_std() * x_.h());
    P_(S::Y, S::Y) =
        Sqr(2 * params.state_process_noise_position_weight_std() * x_.h());
    P_(S::A, S::A) = Sqr(1e-2);
    P_(S::H, S::H) =
        Sqr(2 * params.state_process_noise_position_weight_std() * x_.h());
    P_(S::VEL_X, S::VEL_X) =
        Sqr(1000 * params.state_process_noise_velocity_weight_std() * x_.h());
    P_(S::VEL_Y, S::VEL_Y) =
        Sqr(1000 * params.state_process_noise_velocity_weight_std() * x_.h());
    P_(S::VEL_A, S::VEL_A) = 1e-2;
    P_(S::VEL_H, S::VEL_H) =
        Sqr(1000 * params.state_process_noise_velocity_weight_std() * x_.h());

    motion_model_.InitParams(params);
  }

  Img2DStateData ComputePrediction(const StateType& x_in,
                                   const CovarianceType& P_in,
                                   T new_time_stamp) const {
    // TODO(Yan): creat estimator to utilize time_stamp in outer layer
    T dt = new_time_stamp - last_timestamp_;
    if (dt < 0) {
      QLOG(ERROR) << "time_stamp is brefore last update";
      return Img2DStateData(x_in, P_in);
    }
    const auto x = motion_model_.ComputeStateTransition(x_in, dt);
    const auto Q = motion_model_.ComputeProcessNoise(x_in, dt);
    const auto F = motion_model_.ComputeF(dt);
    const auto P = F * P_in * F.transpose() + Q;
    return Img2DStateData(x, P);
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
      const MeasurementType& z, const StateType& x_in,
      const CovarianceType& P_in, T new_time_stamp,
      const std::optional<MotionFilterParamProto>& params) const {
    // TODO(Yan): creat estimator to utilize time_stamp in outer layer
    const Img2DStateData predict_state_data =
        ComputePrediction(x_in, P_in, new_time_stamp);
    return ComputeMahalanobisDistance(predict_state_data.state(),
                                      predict_state_data.state_cov(), z,
                                      params ? *params : params_);
  }

  S Predict(T new_time_stamp) {
    if (new_time_stamp < last_timestamp_) {
      return x_;
    }
    // TODO(Yan): creat estimator to utilize time_stamp in outer layer
    const auto state_data = ComputePrediction(x_, P_, new_time_stamp);
    x_ = state_data.state();
    P_ = state_data.state_cov();
    last_timestamp_ = new_time_stamp;
    return x_;
  }

  S GetState() const { return x_; }

  // for prediction the measurement timestamp should be greater than last
  // timestamp
  template <typename MeasurementType>
  S Update(const MeasurementType& z,
           const std::optional<MotionFilterParamProto>& params = std::nullopt) {
    using MeasModelType = Img2DMeasModel<Img2DState>;
    using M = MeasurementType;
    MeasModelType meas_model;

    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params ? *params : params_, &R, x_));

    const auto H = meas_model.ComputeH();

    const Covariance<M> S = H * P_ * H.transpose() + R;

    const M hx = meas_model.ComputeExpectedMeasurement(x_);

    const KalmanGain<StateType, M> K = P_ * H.transpose() * S.inverse();

    const M y = z - hx;

    x_ += K * y;
    P_ -= K * H * P_;

    // GenerateMeasModelProto(z, R, filter_proto_.add_meas_model());

    return x_;
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const MeasurementType& z,
                               const std::optional<MotionFilterParamProto>&
                                   params = std::nullopt) const {
    return ComputeMahalanobisDistance(x_, P_, z, params ? *params : params_);
  }

  T last_timestamp() const { return last_timestamp_; }

  FilterProto filter_proto() const { return filter_proto_; }

  // TODO(Yan): delete init_time_stamp here and deal this in estimator
  Img2DStateData GetStateData() const { return Img2DStateData(x_, P_); }

  void ClearFilterInfo() { filter_proto_.Clear(); }

 protected:
  using Base::P_;
  using Base::x_;

 private:
  // Check if input params valid
  void CheckParams(const MotionFilterParamProto& params) {
    // Note a hack solution used the same weight for both
    // state process noise and init noise
    QCHECK(params.has_state_process_noise_position_weight_std());
    QCHECK(params.has_state_process_noise_velocity_weight_std());
    QCHECK(params.has_state_measurement_noise_std());
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const StateType& x, const CovarianceType& P,
                               const MeasurementType& z,
                               const MotionFilterParamProto& params) const {
    using MeasModelType = Img2DMeasModel<Img2DState>;
    using M = MeasurementType;
    MeasModelType meas_model;

    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params, &R, x));

    const auto H = meas_model.ComputeH();

    const Covariance<M> S = H * P * H.transpose() + R;

    const M hx = meas_model.ComputeExpectedMeasurement(x);

    const M y = z - hx;

    return ComputeMDistance(y, S);
  }

 private:
  MotionModelType motion_model_;
  MotionFilterParamProto params_;
  // TODO(Yan): delete init_time_stamp here and deal this in estimator
  T last_timestamp_{};
  FilterProto filter_proto_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_KALMAN_FILTER_H_
