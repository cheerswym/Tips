#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MEAS_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MEAS_MODEL_H_

#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/motion_filter_2/extent_model.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
namespace qcraft::tracker {

class BBoxMeasurement : public Vector<double, 3> {
 public:
  using Base = Vector<double, 3>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int LENGTH = 0;
  static constexpr int WIDTH = 1;
  static constexpr int HEIGHT = 2;

  BBoxMeasurement() { this->setZero(); }
  BBoxMeasurement(T length, T width) {
    this->setZero();
    this->length() = length;
    this->width() = width;
  }

  T length() const { return (*this)[LENGTH]; }
  T width() const { return (*this)[WIDTH]; }
  T height() const { return (*this)[HEIGHT]; }

  T& length() { return (*this)[LENGTH]; }
  T& width() { return (*this)[WIDTH]; }
  T& height() { return (*this)[HEIGHT]; }
};

class BBoxMeasModel {
 public:
  using StateType = BBoxState;
  using MeasurementType = BBoxMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  BBoxMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const auto H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::LENGTH, S::LENGTH) = 1.0;
    H(M::WIDTH, S::WIDTH) = 1.0;
    H(M::HEIGHT, S::HEIGHT) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_length_measurement_noise_std()) {
      (*R)(M::LENGTH, M::LENGTH) =
          Sqr(params.state_length_measurement_noise_std());
    } else {
      return false;
    }
    if (params.has_state_width_measurement_noise_std()) {
      (*R)(M::WIDTH, M::WIDTH) =
          Sqr(params.state_width_measurement_noise_std());
    } else {
      return false;
    }
    if (params.has_state_height_measurement_noise_std()) {
      (*R)(M::HEIGHT, M::HEIGHT) =
          Sqr(params.state_height_measurement_noise_std());
    } else {
      return false;
    }
    return true;
  }
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_EXTENT_MEAS_MODEL_H_
