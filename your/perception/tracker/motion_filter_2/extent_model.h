#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MODEL_H_

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"

namespace qcraft::tracker {

class BBoxState : public Vector<double, 3> {
 public:
  using Base = Vector<double, 3>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr size_t LENGTH = 0;
  static constexpr size_t WIDTH = 1;
  static constexpr size_t HEIGHT = 2;

  BBoxState() { this->setZero(); }

  T length() const { return (*this)[LENGTH]; }
  T width() const { return (*this)[WIDTH]; }
  T height() const { return (*this)[HEIGHT]; }

  T& length() { return (*this)[LENGTH]; }
  T& width() { return (*this)[WIDTH]; }
  T& height() { return (*this)[HEIGHT]; }
};

class BBoxModel {
 public:
  using StateType = BBoxState;
  using CovarianceType = Covariance<StateType>;
  using T = StateType::Scalar;
  using S = StateType;

  BBoxModel() {}

  explicit BBoxModel(const MotionFilterParamProto& params) {
    InitParams(params);
  }

  S ComputeStateTransition(const S& x_in, T dt) const {
    S x_out;
    x_out.length() = x_in.length();
    x_out.width() = x_in.width();
    x_out.height() = x_in.height();
    return x_out;
  }

  CovarianceType ComputeF(T dt) const {
    CovarianceType F;
    F.setZero();
    F(S::LENGTH, S::LENGTH) = 1.0;
    F(S::WIDTH, S::WIDTH) = 1.0;
    F(S::HEIGHT, S::HEIGHT) = 1.0;
    return F;
  }

  void InitParams(const MotionFilterParamProto& params) {
    length_rate_var_ = Sqr(params.state_length_process_noise_std());
    width_rate_var_ = Sqr(params.state_width_process_noise_std());
    height_rate_var_ = Sqr(params.state_height_process_noise_std());
  }

  CovarianceType ComputeProcessNoise(T dt) const {
    CovarianceType Q;
    Q.setZero();
    const T dt2 = dt * dt;
    Q(S::LENGTH, S::LENGTH) = length_rate_var_ * dt2;
    Q(S::WIDTH, S::WIDTH) = width_rate_var_ * dt2;
    Q(S::HEIGHT, S::HEIGHT) = height_rate_var_ * dt2;
    return Q;
  }

 private:
  T length_rate_var_{};
  T width_rate_var_{};
  T height_rate_var_{};
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MODEL_H_
