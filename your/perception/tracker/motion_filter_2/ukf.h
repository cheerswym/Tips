#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UKF_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UKF_H_

#include <algorithm>
#include <limits>

#include "onboard/perception/tracker/motion_filter_2/kalman_filter_base.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"
#include "onboard/perception/tracker/motion_filter_2/normal_distribution.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
#include "onboard/perception/tracker/motion_filter_2/state_data.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/motion_filter_2/utils.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {

template <typename MotionModelType>
class UKF : public KalmanFilterBase<MotionModelType> {
 public:
  static_assert(
      std::is_same<typename MotionModelType::StateType, CarState>::value,
      "State type of UKF must be Car State");
  using Base = KalmanFilterBase<MotionModelType>;
  using typename Base::CovarianceType;
  using typename Base::StateType;
  using typename Base::T;
  using S = StateType;
  using WeightsType = Weights<StateType>;

  // State dimension: L
  static constexpr int L = StateType::RowsAtCompileTime;
  static constexpr int kSigmaPointsNum = 2 * L + 1;
  static constexpr int lambda = 3 - L;

  // Sigma points for state or measurement
  template <typename VectorType>
  using SigmaPointsMatrix = SigmaPointsMatrix<VectorType, kSigmaPointsNum>;

  // UKF() = default;
  explicit UKF(const MotionFilterParamProto& params, bool enable_debug = false)
      : params_(params) {
    P_.setIdentity();
    InitParams(params);
    ComputeWeights();
    enable_debug_ = enable_debug;

    if (enable_debug_) {
      *filter_proto_.mutable_param() = params;
    }
  }

  void Init(const S& x_in) { x_ = x_in; }

  void InitParams(const MotionFilterParamProto& params) {
    CheckParams(params);
    P_(S::X, S::X) = Sqr(params.state_x_init_std());
    P_(S::Y, S::Y) = Sqr(params.state_y_init_std());
    P_(S::VEL, S::VEL) = Sqr(params.state_vel_init_std());
    P_(S::YAW, S::YAW) = Sqr(d2r(params.state_heading_init_std()));
    P_(S::YAWD, S::YAWD) = Sqr(d2r(params.state_yawd_init_std()));
    P_(S::ACC, S::ACC) = Sqr(params.state_acc_init_std());

    motion_model_.InitParams(params);
  }

  static void ComputeWeights() {
    weights_.setConstant(1.0 / (2 * (L + lambda)));
    weights_(0) = lambda / (L + lambda);
  }

  CarStateData ComputePrediction(const StateType& x_in,
                                 const CovarianceType& P_in, T dt) const {
    SigmaPointsMatrix<StateType> sigma_state =
        ComputeSigmaPointsForState(x_in, P_in);
    PredictSigmaStatePoints(&sigma_state, dt);
    return PredictStateAndCovariance(sigma_state, x_in, dt);
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
      const MeasurementType& z, const StateType& x_in,
      const CovarianceType& P_in, T dt,
      const std::optional<MotionFilterParamProto>& params =
          std::nullopt) const {
    // Predict
    SigmaPointsMatrix<StateType> sigma_state =
        ComputeSigmaPointsForState(x_in, P_in);
    PredictSigmaStatePoints(&sigma_state, dt);
    const CarStateData state_data =
        PredictStateAndCovariance(sigma_state, x_in, dt);
    SigmaPointsMatrix<StateType> sigma_predict =
        ComputeSigmaPointsForState(state_data.state(), state_data.state_cov());

    return ComputeMahalanobisDistance(sigma_predict, z,
                                      params ? *params : params_);
  }

  S Predict(T dt) {
    sigma_state_ = ComputeSigmaPointsForState(x_, P_);
    PredictSigmaStatePoints(&sigma_state_, dt);
    const CarStateData state_data =
        PredictStateAndCovariance(sigma_state_, x_, dt);
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
    // Compute sigma points for measurement.
    SigmaPointsMatrix<M> sigma_meas;
    sigma_meas.setZero();
    for (int i = 0; i < kSigmaPointsNum; ++i) {
      sigma_meas.col(i) =
          meas_model.ComputeExpectedMeasurement(sigma_state_.col(i));
    }

    // Expected measurement
    M z_pred;
    z_pred.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      z_pred += weights_(i) * sigma_meas.col(i);
    }
    z_pred.NormalizeYaw();

    // Get measurement noise.
    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params ? *params : params_, &R));

    // Compute innovation covariance.
    Covariance<M> S;
    S.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      M z_diff = sigma_meas.col(i) - z_pred;
      z_diff.NormalizeYaw();
      S += weights_(i) * z_diff * z_diff.transpose();
    }
    S += R;

    // Compute cross correlation matrix.
    CrossCorelationMatrix<StateType, M> Tc;
    Tc.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      M z_diff = sigma_meas.col(i) - z_pred;

      // Angle normalization
      z_diff.NormalizeYaw();

      // State difference.
      StateType x_diff = sigma_state_.col(i) - x_;
      x_diff.yaw() = NormalizeAngle(x_diff.yaw());
      Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // Compute kalman gain
    KalmanGain<StateType, M> K = Tc * S.inverse();

    M y = z - z_pred;
    y.NormalizeYaw();
    if (likelihood) {
      NormalDistribution<M> norm(S);
      likelihood_ = std::max(norm.pdf(y), 1e-6);
    }
    // Update state and covariance
    x_ += K * y;
    x_.NormalizeYaw();
    P_ -= K * S * K.transpose();

    if constexpr (std::is_same<MotionModelType, CarModelCP>::value) {
      x_.vel() = 0.0;
      x_.acc() = 0.0;
      x_.yawd() = 0.0;
    } else if constexpr (std::is_same<MotionModelType, CarModelCV>::value) {
      x_.acc() = 0.0;
      x_.yawd() = 0.0;
    } else if constexpr (std::is_same<MotionModelType, CarModelCA>::value) {
      x_.yawd() = 0.0;
    }

    if (enable_debug_) {
      GenerateMeasModelProto(z, R, filter_proto_.add_meas_model());
    }

    return x_;
  }

  SigmaPointsMatrix<StateType> ComputeSigmaPointsForState(
      const S& x_in, const CovarianceType& P_in) const {
    SigmaPointsMatrix<StateType> sigma_state;
    sigma_state.setZero();
    CovarianceType A = P_in.llt().matrixL();
    sigma_state.col(0) = x_in;

    for (int i = 0; i < L; ++i) {
      sigma_state.col(i + 1) = x_in + std::sqrt(lambda + L) * A.col(i);
      sigma_state.col(i + 1 + L) = x_in - std::sqrt(lambda + L) * A.col(i);
    }
    return sigma_state;
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const MeasurementType& z,
                               const std::optional<MotionFilterParamProto>&
                                   params = std::nullopt) const {
    return ComputeMahalanobisDistance(sigma_state_, z,
                                      params ? *params : params_);
  }

  T likelihood() const { return likelihood_; }

  FilterProto filter_proto() const { return filter_proto_; }

  void ClearFilterInfo() { filter_proto_.Clear(); }

 protected:
  using Base::enable_debug_;
  using Base::P_;
  using Base::x_;

 private:
  void CheckParams(const MotionFilterParamProto& params) {
    QCHECK(params.has_state_x_init_std());
    QCHECK(params.has_state_y_init_std());
    QCHECK(params.has_state_heading_init_std());
    QCHECK(params.has_state_vel_init_std());
    QCHECK(params.has_state_yawd_init_std());
    QCHECK(params.has_state_acc_init_std());

    QCHECK(params.has_state_x_process_noise_std());
    QCHECK(params.has_state_y_process_noise_std());
    QCHECK(params.has_state_heading_process_noise_std());
    QCHECK(params.has_state_vel_process_noise_std());
    QCHECK(params.has_state_yawd_process_noise_std());
    QCHECK(params.has_state_acc_process_noise_std());

    QCHECK(params.has_state_x_measurement_noise_std());
    QCHECK(params.has_state_y_measurement_noise_std());
    QCHECK(params.has_state_heading_measurement_noise_std());
    QCHECK(params.has_state_vel_measurement_noise_std());
    QCHECK(params.has_state_yawd_measurement_noise_std());
    QCHECK(params.has_state_acc_measurement_noise_std());
  }

  void PredictSigmaStatePoints(SigmaPointsMatrix<StateType>* sigma_state,
                               T dt) const {
    // Predict state of sigma points
    for (int i = 0; i < kSigmaPointsNum; ++i) {
      sigma_state->col(i) =
          motion_model_.ComputeStateTransition(sigma_state->col(i), dt);
    }
  }

  CarStateData PredictStateAndCovariance(
      const SigmaPointsMatrix<StateType>& sigma_state, const S& x_in,
      T dt) const {
    // Predict state
    S state_pred;
    state_pred.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      state_pred += weights_(i) * sigma_state.col(i);
    }
    state_pred.NormalizeYaw();

    // Predict state covariance
    CovarianceType state_pred_cov;
    state_pred_cov.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      S x_diff = sigma_state.col(i) - state_pred;
      x_diff.yaw() = NormalizeAngle(x_diff.yaw());
      state_pred_cov += weights_(i) * x_diff * x_diff.transpose();
    }

    // Add process noise
    state_pred_cov += motion_model_.ComputeProcessNoise(x_in, dt);

    return CarStateData(state_pred, state_pred_cov);
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(const SigmaPointsMatrix<StateType>& sigma_state,
                               const MeasurementType& z,
                               const MotionFilterParamProto& params) const {
    using MeasModelType =
        typename MeasModelTypeSelector<StateType, MeasurementType>::type;
    using M = MeasurementType;
    MeasModelType meas_model;
    // Compute sigma points for measurement.
    SigmaPointsMatrix<M> sigma_meas;
    sigma_meas.setZero();
    for (int i = 0; i < kSigmaPointsNum; ++i) {
      sigma_meas.col(i) =
          meas_model.ComputeExpectedMeasurement(sigma_state.col(i));
    }

    // Expected measurement
    M z_pred;
    z_pred.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      z_pred += weights_(i) * sigma_meas.col(i);
    }
    z_pred.NormalizeYaw();

    // Get measurement noise.
    Covariance<M> R;
    QCHECK(meas_model.GetMeasurementNoise(params, &R));

    // Compute innovation covariance.
    Covariance<M> S;
    S.setZero();
    for (int i = 0; i < kSigmaPointsNum; i++) {
      M z_diff = sigma_meas.col(i) - z_pred;
      z_diff.NormalizeYaw();
      S += weights_(i) * z_diff * z_diff.transpose();
    }
    S += R;

    M y = z - z_pred;
    y.NormalizeYaw();
    return ComputeMDistance(y, S);
  }

 private:
  MotionModelType motion_model_;
  MotionFilterParamProto params_;
  SigmaPointsMatrix<StateType> sigma_state_;
  T likelihood_{};
  static WeightsType weights_;
  FilterProto filter_proto_;
};

template <typename MotionModelType>
typename UKF<MotionModelType>::WeightsType UKF<MotionModelType>::weights_ =
    UKF<MotionModelType>::WeightsType::Zero();
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UKF_H_
