#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_MODEL_H_

#include "onboard/lite/logging.h"  // QCHECK()
#include "onboard/math/util.h"     // Sqr()
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {

class Img2DState : public Vector<double, 8> {
 public:
  using Base = Vector<double, 8>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  // A is the aspect ratio
  // H is the height of bbox
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int A = 2;
  static constexpr int H = 3;
  static constexpr int VEL_X = 4;
  static constexpr int VEL_Y = 5;
  static constexpr int VEL_A = 6;
  static constexpr int VEL_H = 7;

  Img2DState() { this->setZero(); }

  void NormalizeYaw() {}

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T a() const { return (*this)[A]; }
  T h() const { return (*this)[H]; }
  T w() const { return (*this)[H] * (*this)[A]; }
  T vx() const { return (*this)[VEL_X]; }
  T vy() const { return (*this)[VEL_Y]; }
  T vw() const {
    return (*this)[H] * (*this)[VEL_A] + (*this)[A] * (*this)[VEL_H];
  }
  T va() const { return (*this)[VEL_A]; }
  T vh() const { return (*this)[VEL_H]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& a() { return (*this)[A]; }
  T& h() { return (*this)[H]; }
  T& vx() { return (*this)[VEL_X]; }
  T& vy() { return (*this)[VEL_Y]; }
  T& va() { return (*this)[VEL_A]; }
  T& vh() { return (*this)[VEL_H]; }
};

class Img2DModelCV {
 public:
  using StateType = Img2DState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  Img2DModelCV() {}

  explicit Img2DModelCV(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  void InitParams(const MotionFilterParamProto& params) {
    weight_position_ = Sqr(params.state_process_noise_position_weight_std());
    weight_velocity_ = Sqr(params.state_process_noise_velocity_weight_std());
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    const auto F = ComputeF(dt);
    return F * x_in;
  }

  CovarianceType ComputeF(T dt) const {
    CovarianceType F;
    F.setZero();
    for (int i = 0; i < S::RowsAtCompileTime; ++i) {
      F(i, i) = 1.0;
    }
    F(S::X, S::VEL_X) = dt;
    F(S::Y, S::VEL_Y) = dt;
    F(S::A, S::VEL_A) = dt;
    F(S::H, S::VEL_H) = dt;
    return F;
  }

  CovarianceType ComputeProcessNoise(const S& x_in, T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T mean3_2 = x_in.h() * x_in.h();
    Q(S::X, S::X) = weight_position_ * mean3_2;
    Q(S::Y, S::Y) = weight_position_ * mean3_2;
    Q(S::A, S::A) = 1e-1 * 1e-1;
    Q(S::H, S::H) = weight_position_ * mean3_2;
    Q(S::VEL_X, S::VEL_X) = weight_velocity_ * mean3_2;
    Q(S::VEL_Y, S::VEL_Y) = weight_velocity_ * mean3_2;
    Q(S::VEL_A, S::VEL_A) = 1e-1 * 1e-1;
    Q(S::VEL_H, S::VEL_H) = weight_velocity_ * mean3_2;
    return Q;
  }

 private:
  T weight_position_{};
  T weight_velocity_{};
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_IMG_2D_MODEL_H_
