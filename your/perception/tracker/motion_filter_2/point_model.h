#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_POINT_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_POINT_MODEL_H_

#include "onboard/lite/logging.h"  // QCHECK()
#include "onboard/math/util.h"     // Sqr()
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {

class PointState : public Vector<double, 6> {
 public:
  using Base = Vector<double, 6>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VEL_X = 2;
  static constexpr int VEL_Y = 3;
  static constexpr int ACC_X = 4;
  static constexpr int ACC_Y = 5;

  PointState() { this->setZero(); }

  void NormalizeYaw() {}

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T vx() const { return (*this)[VEL_X]; }
  T vy() const { return (*this)[VEL_Y]; }
  T ax() const { return (*this)[ACC_X]; }
  T ay() const { return (*this)[ACC_Y]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& vx() { return (*this)[VEL_X]; }
  T& vy() { return (*this)[VEL_Y]; }
  T& ax() { return (*this)[ACC_X]; }
  T& ay() { return (*this)[ACC_Y]; }
};

class PointModelCP {
 public:
  using StateType = PointState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  PointModelCP() {}

  explicit PointModelCP(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const auto F = ComputeF(dt);
    return F * x_in;
  }

  CovarianceType ComputeF(T dt) const {
    CovarianceType F;
    F.setZero();
    F(S::X, S::X) = 1.0;
    F(S::Y, S::Y) = 1.0;
    return F;
  }

  void InitParams(const MotionFilterParamProto& params) {
    x_rate_var_ = Sqr(params.state_x_process_noise_std());
    y_rate_var_ = Sqr(params.state_y_process_noise_std());
  }

  CovarianceType ComputeProcessNoise(T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;

    Q(S::X, S::X) = x_rate_var_ * dt2;
    Q(S::X, S::VEL_X) = x_rate_var_ * dt;

    Q(S::Y, S::Y) = y_rate_var_ * dt2;
    Q(S::Y, S::VEL_Y) = y_rate_var_ * dt;

    Q(S::VEL_X, S::X) = x_rate_var_ * dt;
    Q(S::VEL_X, S::VEL_X) = x_rate_var_;

    Q(S::VEL_Y, S::Y) = y_rate_var_ * dt;
    Q(S::VEL_Y, S::VEL_Y) = y_rate_var_;

    return Q;
  }

 private:
  T x_rate_var_{};
  T y_rate_var_{};
};

class PointModelCV {
 public:
  using StateType = PointState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  PointModelCV() {}

  explicit PointModelCV(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const auto F = ComputeF(dt);
    return F * x_in;
  }

  CovarianceType ComputeF(T dt) const {
    CovarianceType F;
    F.setZero();
    F(S::X, S::X) = 1.0;
    F(S::X, S::VEL_X) = dt;
    F(S::Y, S::Y) = 1.0;
    F(S::Y, S::VEL_Y) = dt;
    F(S::VEL_X, S::VEL_X) = 1.0;
    F(S::VEL_Y, S::VEL_Y) = 1.0;
    return F;
  }

  void InitParams(const MotionFilterParamProto& params) {
    vx_rate_var_ = Sqr(params.state_vel_x_process_noise_std());
    vy_rate_var_ = Sqr(params.state_vel_y_process_noise_std());
  }

  CovarianceType ComputeProcessNoise(T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    const T dt3 = dt2 * dt;
    const T dt4 = dt3 * dt;

    Q(S::X, S::X) = vx_rate_var_ * dt4 / 4.0;
    Q(S::X, S::VEL_X) = vx_rate_var_ * dt3 / 2.0;
    Q(S::X, S::ACC_X) = vx_rate_var_ * dt2 / 2.0;

    Q(S::Y, S::Y) = vy_rate_var_ * dt4 / 4.0;
    Q(S::Y, S::VEL_Y) = vy_rate_var_ * dt3 / 2.0;
    Q(S::Y, S::ACC_Y) = vy_rate_var_ * dt2 / 2.0;

    Q(S::VEL_X, S::X) = vx_rate_var_ * dt3 / 2.0;
    Q(S::VEL_X, S::VEL_X) = vx_rate_var_ * dt2;
    Q(S::VEL_X, S::ACC_X) = vx_rate_var_ * dt;

    Q(S::VEL_Y, S::Y) = vy_rate_var_ * dt3 / 2.0;
    Q(S::VEL_Y, S::VEL_Y) = vy_rate_var_ * dt2;
    Q(S::VEL_Y, S::ACC_Y) = vy_rate_var_ * dt;

    Q(S::ACC_X, S::X) = vx_rate_var_ * dt2 / 2.0;
    Q(S::ACC_X, S::VEL_X) = vx_rate_var_ * dt;
    Q(S::ACC_X, S::ACC_X) = vx_rate_var_;

    Q(S::ACC_Y, S::Y) = vy_rate_var_ * dt2 / 2.0;
    Q(S::ACC_Y, S::VEL_Y) = vy_rate_var_ * dt;
    Q(S::ACC_Y, S::ACC_Y) = vy_rate_var_;

    return Q;
  }

 private:
  T vx_rate_var_{};
  T vy_rate_var_{};
};

class PointModelCA {
 public:
  using StateType = PointState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  PointModelCA() {}

  explicit PointModelCA(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const auto F = ComputeF(dt);
    return F * x_in;
  }

  CovarianceType ComputeF(T dt) const {
    CovarianceType F;
    F.setZero();
    F(S::X, S::X) = 1.0;
    F(S::X, S::VEL_X) = dt;
    F(S::X, S::ACC_X) = dt * dt / 2.0;
    F(S::Y, S::Y) = 1.0;
    F(S::Y, S::VEL_Y) = dt;
    F(S::Y, S::ACC_Y) = dt * dt / 2.0;
    F(S::VEL_X, S::VEL_X) = 1.0;
    F(S::VEL_X, S::ACC_X) = dt;
    F(S::VEL_Y, S::VEL_Y) = 1.0;
    F(S::VEL_Y, S::ACC_Y) = dt;
    F(S::ACC_X, S::ACC_X) = 1.0;
    F(S::ACC_Y, S::ACC_Y) = 1.0;
    return F;
  }

  void InitParams(const MotionFilterParamProto& params) {
    ax_rate_var_ = Sqr(params.state_acc_x_process_noise_std());
    ay_rate_var_ = Sqr(params.state_acc_y_process_noise_std());
  }

  CovarianceType ComputeProcessNoise(T dt) const {
    CovarianceType Q;
    Q.setIdentity();
    const T dt2 = dt * dt;
    const T dt3 = dt2 * dt;
    const T dt4 = dt3 * dt;
    const T dt5 = dt4 * dt;
    const T dt6 = dt5 * dt;

    Q(S::X, S::X) = ax_rate_var_ * dt6 / 36.0;
    Q(S::X, S::VEL_X) = ax_rate_var_ * dt5 / 12.0;
    Q(S::X, S::ACC_X) = ax_rate_var_ * dt4 / 6.0;

    Q(S::Y, S::Y) = ay_rate_var_ * dt6 / 36.0;
    Q(S::Y, S::VEL_Y) = ay_rate_var_ * dt5 / 12.0;
    Q(S::Y, S::ACC_Y) = ay_rate_var_ * dt4 / 6.0;

    Q(S::VEL_X, S::X) = ax_rate_var_ * dt5 / 12.0;
    Q(S::VEL_X, S::VEL_X) = ax_rate_var_ * dt4 / 4.0;
    Q(S::VEL_X, S::ACC_X) = ax_rate_var_ * dt3 / 2.0;

    Q(S::VEL_Y, S::Y) = ay_rate_var_ * dt5 / 12.0;
    Q(S::VEL_Y, S::VEL_Y) = ay_rate_var_ * dt4 / 4.0;
    Q(S::VEL_Y, S::ACC_Y) = ay_rate_var_ * dt3 / 2.0;

    Q(S::ACC_X, S::X) = ax_rate_var_ * dt4 / 6.0;
    Q(S::ACC_X, S::VEL_X) = ax_rate_var_ * dt3 / 2.0;
    Q(S::ACC_X, S::ACC_X) = ax_rate_var_ * dt2;

    Q(S::ACC_Y, S::Y) = ay_rate_var_ * dt4 / 6.0;
    Q(S::ACC_Y, S::VEL_Y) = ay_rate_var_ * dt3 / 2.0;
    Q(S::ACC_Y, S::ACC_Y) = ay_rate_var_ * dt2;

    return Q;
  }

 private:
  T ax_rate_var_{};
  T ay_rate_var_{};
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_POINT_MODEL_H_
