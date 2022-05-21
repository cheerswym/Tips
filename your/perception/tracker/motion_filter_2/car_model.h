#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_CAR_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_CAR_MODEL_H_

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {

class CarState : public Vector<double, 6> {
 public:
  using Base = Vector<double, 6>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr size_t X = 0;
  static constexpr size_t Y = 1;
  static constexpr size_t YAW = 2;
  static constexpr size_t VEL = 3;
  static constexpr size_t YAWD = 4;
  static constexpr size_t ACC = 5;

  CarState() { this->setZero(); }

  void NormalizeYaw() { (*this)[YAW] = NormalizeAngle((*this)[YAW]); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T yaw() const { return (*this)[YAW]; }
  T vel() const { return (*this)[VEL]; }
  T yawd() const { return (*this)[YAWD]; }
  T acc() const { return (*this)[ACC]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& yaw() { return (*this)[YAW]; }
  T& vel() { return (*this)[VEL]; }
  T& yawd() { return (*this)[YAWD]; }
  T& acc() { return (*this)[ACC]; }
};

class CarModelCP {
 public:
  using StateType = CarState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  CarModelCP() {}

  explicit CarModelCP(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    S x_out;
    x_out.x() = x_in.x();
    x_out.y() = x_in.y();
    x_out.yaw() = x_in.yaw();
    return x_out;
  }

  void InitParams(const MotionFilterParamProto& params) {
    pos_rate_var_ = Sqr(params.state_x_process_noise_std());
    yaw_rate_var_ = Sqr(d2r(params.state_heading_process_noise_std()));
  }

  CovarianceType ComputeProcessNoise(const S& x_in, T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    const T yaw = x_in.yaw();
    const T cos_yaw = std::cos(yaw);
    const T sin_yaw = std::sin(yaw);
    const T cos_yaw_2 = cos_yaw * cos_yaw;
    const T sin_yaw_2 = sin_yaw * sin_yaw;
    const T cos_yaw_sin_yaw = cos_yaw * sin_yaw;

    Q(S::X, S::X) = pos_rate_var_ * dt2 * cos_yaw_2;
    Q(S::X, S::Y) = pos_rate_var_ * dt2 * cos_yaw_sin_yaw;
    Q(S::X, S::VEL) = pos_rate_var_ * dt * cos_yaw;

    Q(S::Y, S::X) = pos_rate_var_ * dt2 * cos_yaw_sin_yaw;
    Q(S::Y, S::Y) = pos_rate_var_ * dt2 * sin_yaw_2;
    Q(S::Y, S::VEL) = pos_rate_var_ * dt * sin_yaw;

    Q(S::VEL, S::X) = pos_rate_var_ * dt * cos_yaw;
    Q(S::VEL, S::Y) = pos_rate_var_ * dt * sin_yaw;
    Q(S::VEL, S::VEL) = pos_rate_var_;

    Q(S::YAW, S::YAW) = yaw_rate_var_ * dt2;
    Q(S::YAW, S::YAWD) = yaw_rate_var_ * dt;

    Q(S::YAWD, S::YAW) = yaw_rate_var_ * dt;
    Q(S::YAWD, S::YAWD) = yaw_rate_var_;
    return Q;
  }

 private:
  T pos_rate_var_{};
  T yaw_rate_var_{};
};

class CarModelCV {
 public:
  using StateType = CarState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  CarModelCV() {}

  explicit CarModelCV(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const T x = x_in.x();
    const T y = x_in.y();
    const T yaw = x_in.yaw();
    const T vel = x_in.vel();
    S x_out;
    x_out.x() = x + vel * dt * std::cos(yaw);
    x_out.y() = y + vel * dt * std::sin(yaw);
    x_out.yaw() = yaw;
    x_out.vel() = vel;
    return x_out;
  }

  void InitParams(const MotionFilterParamProto& params) {
    vel_rate_var_ = Sqr(params.state_vel_process_noise_std());
    yaw_rate_var_ = Sqr(d2r(params.state_heading_process_noise_std()));
  }

  CovarianceType ComputeProcessNoise(const S& x_in, T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    const T dt3 = dt2 * dt;
    const T dt4 = dt3 * dt;
    const T yaw = x_in.yaw();
    const T cos_yaw = std::cos(yaw);
    const T sin_yaw = std::sin(yaw);
    const T cos_yaw_2 = cos_yaw * cos_yaw;
    const T sin_yaw_2 = sin_yaw * sin_yaw;
    const T cos_yaw_sin_yaw = cos_yaw * sin_yaw;

    Q(S::X, S::X) = vel_rate_var_ * dt4 * cos_yaw_2 / 4.0;
    Q(S::X, S::Y) = vel_rate_var_ * dt4 * cos_yaw_sin_yaw / 4.0;
    Q(S::X, S::VEL) = vel_rate_var_ * dt3 * cos_yaw / 2.0;
    Q(S::X, S::ACC) = vel_rate_var_ * dt2 * cos_yaw / 2.0;

    Q(S::Y, S::X) = vel_rate_var_ * dt4 * cos_yaw_sin_yaw / 4.0;
    Q(S::Y, S::Y) = vel_rate_var_ * dt4 * sin_yaw_2 / 4.0;
    Q(S::Y, S::VEL) = vel_rate_var_ * dt3 * sin_yaw / 2.0;
    Q(S::Y, S::ACC) = vel_rate_var_ * dt2 * sin_yaw / 2.0;

    Q(S::VEL, S::X) = vel_rate_var_ * dt3 * cos_yaw / 2.0;
    Q(S::VEL, S::Y) = vel_rate_var_ * dt3 * sin_yaw / 2.0;
    Q(S::VEL, S::VEL) = vel_rate_var_ * dt2;
    Q(S::VEL, S::ACC) = vel_rate_var_ * dt;

    Q(S::ACC, S::X) = vel_rate_var_ * dt2 * cos_yaw / 2.0;
    Q(S::ACC, S::Y) = vel_rate_var_ * dt2 * sin_yaw / 2.0;
    Q(S::ACC, S::VEL) = vel_rate_var_ * dt;
    Q(S::ACC, S::ACC) = vel_rate_var_;

    Q(S::YAW, S::YAW) = yaw_rate_var_ * dt2;
    Q(S::YAW, S::YAWD) = yaw_rate_var_ * dt;

    Q(S::YAWD, S::YAW) = yaw_rate_var_ * dt;
    Q(S::YAWD, S::YAWD) = yaw_rate_var_;
    return Q;
  }

 private:
  T vel_rate_var_{};
  T yaw_rate_var_{};
};

class CarModelCA {
 public:
  using StateType = CarState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  CarModelCA() {}

  explicit CarModelCA(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const T x = x_in.x();
    const T y = x_in.y();
    const T yaw = x_in.yaw();
    const T vel = x_in.vel();
    const T acc = x_in.acc();
    S x_out;
    x_out.x() =
        x + vel * dt * std::cos(yaw) + 0.5 * acc * dt * dt * std::cos(yaw);
    x_out.y() =
        y + vel * dt * std::sin(yaw) + 0.5 * acc * dt * dt * std::sin(yaw);
    x_out.yaw() = yaw;
    x_out.vel() = vel + acc * dt;
    x_out.acc() = acc;
    return x_out;
  }

  void InitParams(const MotionFilterParamProto& params) {
    acc_rate_var_ = Sqr(params.state_acc_process_noise_std());
    yaw_rate_var_ = Sqr(d2r(params.state_heading_process_noise_std()));
  }

  CovarianceType ComputeProcessNoise(const S& x_in, T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    const T dt3 = dt2 * dt;
    const T dt4 = dt3 * dt;
    const T dt5 = dt4 * dt;
    const T dt6 = dt5 * dt;
    const T yaw = x_in.yaw();
    const T cos_yaw = std::cos(yaw);
    const T sin_yaw = std::sin(yaw);
    const T cos_yaw_2 = cos_yaw * cos_yaw;
    const T sin_yaw_2 = sin_yaw * sin_yaw;
    const T cos_yaw_sin_yaw = cos_yaw * sin_yaw;

    Q(S::X, S::X) = acc_rate_var_ * dt6 * cos_yaw_2 / 36.0;
    Q(S::X, S::Y) = acc_rate_var_ * dt6 * cos_yaw_sin_yaw / 36.0;
    Q(S::X, S::VEL) = acc_rate_var_ * dt5 * cos_yaw / 12.0;
    Q(S::X, S::ACC) = acc_rate_var_ * dt4 * cos_yaw / 6.0;

    Q(S::Y, S::X) = acc_rate_var_ * dt6 * cos_yaw_sin_yaw / 36.0;
    Q(S::Y, S::Y) = acc_rate_var_ * dt6 * sin_yaw_2 / 36.0;
    Q(S::Y, S::VEL) = acc_rate_var_ * dt5 * sin_yaw / 12.0;
    Q(S::Y, S::ACC) = acc_rate_var_ * dt4 * sin_yaw / 6.0;

    Q(S::VEL, S::X) = acc_rate_var_ * dt5 * cos_yaw / 12.0;
    Q(S::VEL, S::Y) = acc_rate_var_ * dt5 * sin_yaw / 12.0;
    Q(S::VEL, S::VEL) = acc_rate_var_ * dt4 / 4.0;
    Q(S::VEL, S::ACC) = acc_rate_var_ * dt3 / 2.0;

    Q(S::ACC, S::X) = acc_rate_var_ * dt4 * cos_yaw / 6.0;
    Q(S::ACC, S::Y) = acc_rate_var_ * dt4 * sin_yaw / 6.0;
    Q(S::ACC, S::VEL) = acc_rate_var_ * dt3 / 2.0;
    Q(S::ACC, S::ACC) = acc_rate_var_ * dt2;

    Q(S::YAW, S::YAW) = yaw_rate_var_ * dt2;
    Q(S::YAW, S::YAWD) = yaw_rate_var_ * dt;

    Q(S::YAWD, S::YAW) = yaw_rate_var_ * dt;
    Q(S::YAWD, S::YAWD) = yaw_rate_var_;
    return Q;
  }

 private:
  T acc_rate_var_{};
  T yaw_rate_var_{};
};

class CarModelCTRV {
 public:
  using StateType = CarState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  CarModelCTRV() {}

  explicit CarModelCTRV(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const T x = x_in.x();
    const T y = x_in.y();
    const T yaw = x_in.yaw();
    const T vel = x_in.vel();
    const T yawd = x_in.yawd();
    S x_out;

    T x_pred, y_pred, yaw_pred;
    if (std::abs(yawd) > 0.001 /*std::numeric_limits<T>::epsilon()*/) {
      x_pred = x + vel / yawd * (std::sin(yaw + yawd * dt) - std::sin(yaw));
      y_pred = y - vel / yawd * (std::cos(yaw + yawd * dt) - std::cos(yaw));
      yaw_pred = yaw + yawd * dt;
    } else {
      x_pred = x + vel * dt * std::cos(yaw);
      y_pred = y + vel * dt * std::sin(yaw);
      yaw_pred = yaw;
    }

    x_out.x() = x_pred;
    x_out.y() = y_pred;
    x_out.yaw() = yaw_pred;
    x_out.vel() = vel;
    x_out.yawd() = yawd;
    return x_out;
  }

  void InitParams(const MotionFilterParamProto& params) {
    vel_rate_var_ = Sqr(params.state_vel_process_noise_std());
    yawd_rate_var_ = Sqr(d2r(params.state_yawd_process_noise_std()));
  }

  CovarianceType ComputeProcessNoise(const S& x_in, T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    const T dt3 = dt2 * dt;
    const T dt4 = dt3 * dt;
    const T yaw = x_in.yaw();
    const T cos_yaw = std::cos(yaw);
    const T sin_yaw = std::sin(yaw);
    const T cos_yaw_2 = cos_yaw * cos_yaw;
    const T sin_yaw_2 = sin_yaw * sin_yaw;
    const T cos_yaw_sin_yaw = cos_yaw * sin_yaw;

    Q(S::X, S::X) = vel_rate_var_ * dt4 * cos_yaw_2 / 4.0;
    Q(S::X, S::Y) = vel_rate_var_ * dt4 * cos_yaw_sin_yaw / 4.0;
    Q(S::X, S::VEL) = vel_rate_var_ * dt3 * cos_yaw / 2.0;
    Q(S::X, S::ACC) = vel_rate_var_ * dt2 * cos_yaw / 2.0;

    Q(S::Y, S::X) = vel_rate_var_ * dt4 * cos_yaw_sin_yaw / 4.0;
    Q(S::Y, S::Y) = vel_rate_var_ * dt4 * sin_yaw_2 / 4.0;
    Q(S::Y, S::VEL) = vel_rate_var_ * dt3 * sin_yaw / 2.0;
    Q(S::Y, S::ACC) = vel_rate_var_ * dt2 * sin_yaw / 2.0;

    Q(S::VEL, S::X) = vel_rate_var_ * dt3 * cos_yaw / 2.0;
    Q(S::VEL, S::Y) = vel_rate_var_ * dt3 * sin_yaw / 2.0;
    Q(S::VEL, S::VEL) = vel_rate_var_ * dt2;
    Q(S::VEL, S::ACC) = vel_rate_var_ * dt;

    Q(S::ACC, S::X) = vel_rate_var_ * dt2 * cos_yaw / 2.0;
    Q(S::ACC, S::Y) = vel_rate_var_ * dt2 * sin_yaw / 2.0;
    Q(S::ACC, S::VEL) = vel_rate_var_ * dt;
    Q(S::ACC, S::ACC) = vel_rate_var_;

    Q(S::YAW, S::YAW) = yawd_rate_var_ * dt4 / 4.0;
    Q(S::YAW, S::YAWD) = yawd_rate_var_ * dt3 / 2.0;

    Q(S::YAWD, S::YAW) = yawd_rate_var_ * dt3 / 2.0;
    Q(S::YAWD, S::YAWD) = yawd_rate_var_ * dt2;
    return Q;
  }

 private:
  T vel_rate_var_{};
  T yawd_rate_var_{};
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_CAR_MODEL_H_
