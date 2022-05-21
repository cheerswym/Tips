#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MOTION_FILTER_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MOTION_FILTER_H_
#include <variant>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/kalman_filter.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/motion_filter_2/ukf.h"
#include "onboard/perception/tracker/motion_filter_2/utils.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft::tracker {

using KFVariant =
    std::variant<KalmanFilter<PointModelCP>, KalmanFilter<PointModelCV>,
                 KalmanFilter<PointModelCA>>;

using UKFVariant = std::variant<UKF<CarModelCP>, UKF<CarModelCV>,
                                UKF<CarModelCA>, UKF<CarModelCTRV>>;

template <typename>
struct FilterVariantSelector;

template <>
struct FilterVariantSelector<PointState> {
  using type = KFVariant;
};

template <>
struct FilterVariantSelector<CarState> {
  using type = UKFVariant;
};

template <typename StateType>
class MotionFilter {
 public:
  using FilterVariant = typename FilterVariantSelector<StateType>::type;
  using CovarianceType = Covariance<StateType>;
  using T = typename StateType::Scalar;
  using S = StateType;
  // Main diagonal value for transition matrix
  static constexpr T kMainDiagVal = 0.99;

  MotionFilter() = default;

  explicit MotionFilter(const IMMProto& proto, bool enable_debug = false) {
    num_of_model_ = proto.params_size();
    QCHECK(num_of_model_ > 0 && num_of_model_ < 4);
    if constexpr (std::is_same<StateType, PointState>::value) {
      for (const auto& param : proto.params()) {
        const auto& filter_param = param.filter_param();
        switch (filter_param.type()) {
          case MotionFilterParamProto::POINT_CP:
            filters_.push_back(
                KalmanFilter<PointModelCP>(filter_param, enable_debug));
            break;
          case MotionFilterParamProto::POINT_CV:
            filters_.push_back(
                KalmanFilter<PointModelCV>(filter_param, enable_debug));
            break;
          case MotionFilterParamProto::POINT_CA:
            filters_.push_back(
                KalmanFilter<PointModelCA>(filter_param, enable_debug));
            break;
          default:
            QLOG(FATAL) << "No match kalman filter from input params";
            break;
        }
      }
    } else if constexpr (std::is_same<StateType, CarState>::value) {
      for (const auto& param : proto.params()) {
        const auto& filter_param = param.filter_param();
        switch (filter_param.type()) {
          case MotionFilterParamProto::CAR_CP:
            filters_.push_back(UKF<CarModelCP>(filter_param, enable_debug));
            break;
          case MotionFilterParamProto::CAR_CV:
            filters_.push_back(UKF<CarModelCV>(filter_param, enable_debug));
            break;
          case MotionFilterParamProto::CAR_CA:
            filters_.push_back(UKF<CarModelCA>(filter_param, enable_debug));
            break;
          case MotionFilterParamProto::CAR_CTRV:
            filters_.push_back(UKF<CarModelCTRV>(filter_param, enable_debug));
            break;
          default:
            QLOG(FATAL) << "No match ukf from input params";
            break;
        }
      }
    } else {
      QLOG(FATAL) << "Unknown filter type from input params";
    }

    trans_ = Eigen::MatrixXd(num_of_model_, num_of_model_);
    lambda_ = Eigen::VectorXd(num_of_model_);
    lambda_.setZero();
    mu_ = Eigen::VectorXd(num_of_model_);
    for (int i = 0; i < num_of_model_; ++i) {
      mu_(i) = proto.params(i).filter_weight();
    }
    QCHECK_EQ(mu_.sum(), 1.0);

    if (num_of_model_ == 1) {
      trans_(0, 0) = 1.0;
    } else {  // num_of_model_ > 1
      T others = (1 - kMainDiagVal) / (num_of_model_ - 1);
      trans_.setConstant(others);
      for (int i = 0; i < num_of_model_; ++i) {
        trans_(i, i) = kMainDiagVal;
      }
    }
    GaussianMixture(mu_, &x_, &P_);

    // Add motion filter info
    if (DebugIsEnabled()) {
      *motion_filter_proto_.add_action_info() =
          GenerateActionInfo(ActionInfo::CONSTRUCT, *this);
    }
  }

  void Init(const S& x_in) {
    for (size_t i = 0; i < num_of_model_; ++i) {
      std::visit([&x_in](auto& f) { f.Init(x_in); }, filters_[i]);
    }

    GaussianMixture(mu_, &x_, &P_);

    if (DebugIsEnabled()) {
      *motion_filter_proto_.add_action_info() =
          GenerateActionInfo(ActionInfo::INIT, *this);
    }
  }

  StateData ComputePrediction(T dt) const {
    // Predicted mode probability
    Eigen::VectorXd pred_mu = trans_ * mu_;

    // Mixing steps
    std::vector<StateType> mix_states;
    mix_states.resize(num_of_model_);
    std::vector<CovarianceType> mix_states_cov;
    mix_states_cov.resize(num_of_model_);
    MixingStep(pred_mu, &mix_states, &mix_states_cov);

    // Compute prediction without changing state.
    std::vector<StateType> states;
    states.resize(num_of_model_);
    std::vector<CovarianceType> states_cov;
    states_cov.resize(num_of_model_);
    for (size_t i = 0; i < num_of_model_; ++i) {
      const S state = mix_states[i];
      const CovarianceType state_cov = mix_states_cov[i];
      // Return state and state covariance.
      auto state_data = std::visit(
          [&state, &state_cov, &dt](const auto& f) {
            return f.ComputePrediction(state, state_cov, dt);
          },
          filters_[i]);
      states[i] = state_data.state();
      states_cov[i] = state_data.state_cov();
    }

    StateType x;
    CovarianceType P;
    GaussianMixture(pred_mu, states, states_cov, &x, &P);
    if constexpr (std::is_same<StateType, PointState>::value) {
      return StateData(PointStateData(x, P));
    } else if constexpr (std::is_same<StateType, CarState>::value) {
      return StateData(CarStateData(x, P));
    }
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
      const MeasurementType& z, T dt,
      const std::optional<MotionFilterParamProto>& params =
          std::nullopt) const {
    auto mahalanobis_distances = Eigen::VectorXd(num_of_model_);
    // Predicted mode probability
    const Eigen::VectorXd prior_mu = trans_ * mu_;
    // Compute mahalanobis distance
    for (int i = 0; i < num_of_model_; ++i) {
      const S state =
          std::visit([](const auto& f) { return f.state(); }, filters_[i]);
      const CovarianceType state_cov = std::visit(
          [](const auto& f) { return f.state_covariance(); }, filters_[i]);
      mahalanobis_distances[i] = std::visit(
          [&](const auto& f) {
            return f.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
                z, state, state_cov, dt, params);
          },
          filters_[i]);
    }
    return mahalanobis_distances.dot(prior_mu);
  }

  S Predict(T dt) {
    // Predicted mode probability
    Eigen::VectorXd pred_mu = trans_ * mu_;

    // Mixing steps
    std::vector<StateType> mix_states;
    mix_states.resize(num_of_model_);
    std::vector<CovarianceType> mix_states_cov;
    mix_states_cov.resize(num_of_model_);
    MixingStep(pred_mu, &mix_states, &mix_states_cov);

    for (size_t i = 0; i < num_of_model_; ++i) {
      const S state = mix_states[i];
      std::visit([&state](auto& f) { f.set_state(state); }, filters_[i]);
      const CovarianceType state_cov = mix_states_cov[i];
      std::visit([&state_cov](auto& f) { f.set_state_covariance(state_cov); },
                 filters_[i]);
      // Filter predict
      std::visit([&dt](auto& f) { f.Predict(dt); }, filters_[i]);
    }

    mu_ = pred_mu;
    // Predict state and covariance
    GaussianMixture(mu_, &x_, &P_);

    if (DebugIsEnabled()) {
      *motion_filter_proto_.add_action_info() =
          GenerateActionInfo(ActionInfo::PREDICT, *this);
      motion_filter_proto_.set_dt(dt);
    }

    return x_;
  }

  template <typename MeasurementType>
  S Update(const MeasurementType& z,
           const std::optional<MotionFilterParamProto>& param = std::nullopt) {
    // Update each filters and likelihood
    for (size_t i = 0; i < num_of_model_; ++i) {
      std::visit([&z, &param](auto& f) { f.Update(z, param, true); },
                 filters_[i]);
      lambda_[i] =
          std::visit([](const auto& f) { return f.likelihood(); }, filters_[i]);
    }

    // Update model probability
    mu_ = NormalizeWithLikelihood(mu_, lambda_);

    // Update state and covariance
    GaussianMixture(mu_, &x_, &P_);

    // Add QEvent for motion filter divergence
    if (CheckNaN(x_) || CheckDivergence(P_)) {
      QEVENT("jiawei", "motion_filter_has_divergence", [=](QEvent* qevent) {
        qevent->AddField("IsCarModel", std::is_same<StateType, CarState>::value)
            .AddField("S[0]", x_[0])
            .AddField("S[1]", x_[1])
            .AddField("S[2]", x_[2])
            .AddField("S[3]", x_[3])
            .AddField("S[4]", x_[4])
            .AddField("S[5]", x_[5])
            .AddField("P[0,0]", P_(0, 0))
            .AddField("P[1,1]", P_(1, 1))
            .AddField("P[2,2]", P_(2, 2))
            .AddField("P[3,3]", P_(3, 3))
            .AddField("P[4,4]", P_(4, 4))
            .AddField("P[5,5]", P_(5, 5));
      });
    }

    if (DebugIsEnabled()) {
      *motion_filter_proto_.add_action_info() =
          GenerateActionInfo(ActionInfo::UPDATE, *this);
    }
    // Get each filter info
    motion_filter_proto_.clear_filters();
    for (size_t i = 0; i < num_of_model_; ++i) {
      *motion_filter_proto_.add_filters() = std::visit(
          [](const auto& f) { return f.filter_proto(); }, filters_[i]);
    }

    return x_;
  }

  S state() const { return x_; }

  CovarianceType state_covariance() const { return P_; }

  Eigen::VectorXd model_probability() const { return mu_; }

  void SetStateAndCov(const S& x_in, const CovarianceType& P_in) {
    for (size_t i = 0; i < num_of_model_; ++i) {
      std::visit([&x_in](auto& f) { f.set_state(x_in); }, filters_[i]);
      std::visit([&P_in](auto& f) { f.set_state_covariance(P_in); },
                 filters_[i]);
    }

    GaussianMixture(mu_, &x_, &P_);

    if (DebugIsEnabled()) {
      *motion_filter_proto_.add_action_info() =
          GenerateActionInfo(ActionInfo::RESET, *this);
    }
  }

  template <typename MeasurementType>
  T ComputeMahalanobisDistance(
      const MeasurementType& z,
      const std::optional<MotionFilterParamProto>& param = std::nullopt) const {
    auto m_dist_vec = Eigen::VectorXd(num_of_model_);
    for (size_t i = 0; i < num_of_model_; ++i) {
      m_dist_vec[i] = std::visit(
          [&z, &param](const auto& f) {
            return f.ComputeMahalanobisDistance(z, param);
          },
          filters_[i]);
    }
    return m_dist_vec.dot(mu_);
  }

  MotionFilterProto GetMotionFilterInfo() const { return motion_filter_proto_; }

  void ClearMotionFilterInfo() {
    motion_filter_proto_.Clear();
    for (size_t i = 0; i < num_of_model_; ++i) {
      std::visit([](auto& f) { f.ClearFilterInfo(); }, filters_[i]);
    }
  }

  bool DebugIsEnabled() const {
    QCHECK_GT(num_of_model_, 0);
    return std::visit([](const auto& f) { return f.DebugIsEnabled(); },
                      filters_[0]);
  }

 private:
  void MixingStep(const Eigen::VectorXd& pred_mu,
                  std::vector<StateType>* mix_states,
                  std::vector<CovarianceType>* mix_states_cov) const {
    QCHECK(pred_mu.rows() == num_of_model_);
    QCHECK(mix_states->size() == mix_states_cov->size());
    QCHECK(mix_states->size() == num_of_model_);
    for (size_t i = 0; i < num_of_model_; ++i) {
      // Mixing probability
      const Eigen::VectorXd omega =
          mu_.cwiseProduct(trans_.col(i)) / pred_mu(i);
      StateType mix_x;
      mix_x.setZero();
      Eigen::VectorXd angle_in(num_of_model_);
      for (size_t j = 0; j < num_of_model_; ++j) {
        S state =
            std::visit([](const auto& f) { return f.state(); }, filters_[j]);
        if constexpr (std::is_same<StateType, CarState>::value) {
          angle_in[j] = state.yaw();
        }
        mix_x += omega[j] * state;
      }
      if constexpr (std::is_same<StateType, CarState>::value) {
        mix_x.yaw() = MixAngles(omega, angle_in);
      }

      (*mix_states)[i] = mix_x;

      CovarianceType mix_p;
      mix_p.setZero();
      for (size_t j = 0; j < num_of_model_; ++j) {
        const S state =
            std::visit([](const auto& f) { return f.state(); }, filters_[j]);
        S x_diff = state - mix_x;
        x_diff.NormalizeYaw();
        const CovarianceType state_cov = std::visit(
            [](const auto& f) { return f.state_covariance(); }, filters_[j]);
        mix_p += omega[j] * (state_cov + x_diff * x_diff.transpose());
      }
      (*mix_states_cov)[i] = mix_p;
    }
  }
  void GaussianMixture(const Eigen::VectorXd& mu,
                       const std::vector<StateType>& mix_states,
                       const std::vector<CovarianceType>& mix_states_cov, S* x,
                       CovarianceType* P) const {
    QCHECK(mu.rows() == num_of_model_);
    QCHECK(mix_states.size() == mix_states_cov.size());
    QCHECK(mix_states.size() == num_of_model_);
    // Merged state
    x->setZero();
    Eigen::VectorXd angle_in(num_of_model_);
    for (size_t i = 0; i < num_of_model_; ++i) {
      *x += mu[i] * mix_states[i];
      if constexpr (std::is_same<StateType, CarState>::value) {
        angle_in[i] = mix_states[i].yaw();
      }
    }
    if constexpr (std::is_same<StateType, CarState>::value) {
      x->yaw() = MixAngles(mu, angle_in);
    }

    // Merged covariance
    P->setZero();
    for (size_t i = 0; i < num_of_model_; ++i) {
      S x_diff = mix_states[i] - *x;
      x_diff.NormalizeYaw();
      *P += mu[i] * (mix_states_cov[i] + x_diff * x_diff.transpose());
    }
  }

  void GaussianMixture(const Eigen::VectorXd& mu, S* x,
                       CovarianceType* P) const {
    // Merged state
    x->setZero();
    Eigen::VectorXd angle_in(num_of_model_);
    for (size_t i = 0; i < num_of_model_; ++i) {
      const S state =
          std::visit([](const auto& f) { return f.state(); }, filters_[i]);
      if constexpr (std::is_same<StateType, CarState>::value) {
        angle_in[i] = state.yaw();
      }
      *x += mu[i] * state;
    }
    if constexpr (std::is_same<StateType, CarState>::value) {
      x->yaw() = MixAngles(mu, angle_in);
    }

    // Merged covariance
    P->setZero();
    for (size_t i = 0; i < num_of_model_; ++i) {
      const S state =
          std::visit([](const auto& f) { return f.state(); }, filters_[i]);
      S x_diff = state - *x;
      x_diff.NormalizeYaw();
      const CovarianceType state_cov = std::visit(
          [](const auto& f) { return f.state_covariance(); }, filters_[i]);
      *P += mu[i] * (state_cov + x_diff * x_diff.transpose());
    }
  }

  // Normalize likelihood with log-sum-exp tricks
  Eigen::VectorXd NormalizeWithLikelihood(const Eigen::VectorXd& mu,
                                          const Eigen::VectorXd& l) {
    QCHECK(mu.rows() == l.rows());
    QCHECK(mu.rows() == num_of_model_);
    static T torelance = log(1e-16) - log(num_of_model_);
    Eigen::VectorXd a_log =
        mu.array().log().matrix() + l.array().log().matrix();
    T A = a_log.maxCoeff();
    Eigen::VectorXd norm = mu;
    for (size_t i = 0; i < num_of_model_; ++i) {
      if (a_log[i] - A <= torelance) {
        norm[i] = 0.0;
      } else {
        norm[i] = std::exp(a_log[i] - A);
      }
    }
    return norm / norm.sum();
  }

  double MixAngles(const Eigen::VectorXd& mu_in,
                   const Eigen::VectorXd& angles_in) const {
    QCHECK(mu_in.rows() == angles_in.rows());
    QCHECK(mu_in.rows() > 0);
    double yaw_mix = 0.0;
    if constexpr (std::is_same<StateType, CarState>::value) {
      double yaw_base = angles_in[0];
      for (size_t i = 0; i < mu_in.rows(); ++i) {
        yaw_mix += mu_in[i] * yaw_base;
        yaw_mix += mu_in[i] * AngleDifference(yaw_base, angles_in[i]);
      }
    } else {
      QLOG(FATAL) << "Should not use this function when using Point Model.";
    }

    return NormalizeAngle(yaw_mix);
  }

 private:
  std::vector<FilterVariant> filters_;
  S x_;
  CovarianceType P_;

  // Model probability (N, 1)
  Eigen::VectorXd mu_;

  // Likelihood: (N, 1)
  Eigen::VectorXd lambda_;

  // Transition matrix: (N, N)
  Eigen::MatrixXd trans_;

  // Number of models
  int num_of_model_;

  // Motion filter debug proto
  MotionFilterProto motion_filter_proto_;
};
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MOTION_FILTER_H_
