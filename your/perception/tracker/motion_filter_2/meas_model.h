#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_H_

#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/img_2d_model.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft::tracker {

class Img2DMeasurement : public Vector<double, 4> {
 public:
  using Base = Vector<double, 4>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  // A is aspect ratio while h is height of bbox
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int A = 2;
  static constexpr int H = 3;

  Img2DMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T a() const { return (*this)[A]; }
  T h() const { return (*this)[H]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& a() { return (*this)[A]; }
  T& h() { return (*this)[H]; }
};

template <typename State_T>
class Img2DMeasModel {
 public:
  static_assert(std::is_same<State_T, Img2DState>::value,
                "State type of Img 2D Measurement must be Img2DState");
  using StateType = State_T;
  using MeasurementType = Img2DMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  Img2DMeasModel() {
    H_.setZero();
    H_(M::X, S::X) = 1.0;
    H_(M::Y, S::Y) = 1.0;
    H_(M::A, S::A) = 1.0;
    H_(M::H, S::H) = 1.0;
  }

  M ComputeExpectedMeasurement(const S& x_in) {
    const ObsMatType H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    H(M::A, S::A) = 1.0;
    H(M::H, S::H) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R, const State_T x_in) {
    R->setZero();
    if (params.has_state_measurement_noise_std()) {
      (*R)(M::X, M::X) = Sqr(params.state_measurement_noise_std() * x_in.h());
      (*R)(M::Y, M::Y) = Sqr(params.state_measurement_noise_std() * x_in.h());
      (*R)(M::A, M::A) = Sqr(1e-1);
      (*R)(M::H, M::H) = Sqr(params.state_measurement_noise_std() * x_in.h());
    } else {
      return false;
    }
    return true;
  }

 private:
  ObsMatType H_;
};

class PositionMeasurement : public Vector<double, 2> {
 public:
  using Base = Vector<double, 2>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;

  PositionMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }

  void NormalizeYaw() {}
};

template <typename State_T>
class PosMeasModel {
 public:
  using StateType = State_T;
  using MeasurementType = PositionMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  PosMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const ObsMatType H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_x_measurement_noise_std() &&
        params.has_state_y_measurement_noise_std() &&
        params.has_state_x_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_x_y_measurement_correlation_coefficient() >= -1.0 &&
             params.state_x_y_measurement_correlation_coefficient() <= 1.0);
      (*R)(M::X, M::X) = Sqr(params.state_x_measurement_noise_std());
      (*R)(M::Y, M::Y) = Sqr(params.state_y_measurement_noise_std());
      (*R)(M::X, M::Y) =
          params.state_x_y_measurement_correlation_coefficient() *
          params.state_x_measurement_noise_std() *
          params.state_y_measurement_noise_std();
      (*R)(M::Y, M::X) = (*R)(M::X, M::Y);
    } else {
      return false;
    }
    return true;
  }
};

class HeadingMeasurement : public Vector<double, 1> {
 public:
  using Base = Vector<double, 1>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int YAW = 0;

  HeadingMeasurement() { this->setZero(); }

  T yaw() const { return (*this)[YAW]; }

  T& yaw() { return (*this)[YAW]; }

  void NormalizeYaw() { (*this)[YAW] = NormalizeAngle((*this)[YAW]); }
};

class HeadingMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = HeadingMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  HeadingMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const ObsMatType H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::YAW, S::YAW) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_heading_measurement_noise_std()) {
      (*R)(M::YAW, M::YAW) =
          Sqr(d2r(params.state_heading_measurement_noise_std()));
    } else {
      return false;
    }
    return true;
  }
};

class SpeedMeasurement : public Vector<double, 2> {
 public:
  using Base = Vector<double, 2>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int VEL_X = 0;
  static constexpr int VEL_Y = 1;

  SpeedMeasurement() { this->setZero(); }

  T vx() const { return (*this)[VEL_X]; }
  T vy() const { return (*this)[VEL_Y]; }

  T& vx() { return (*this)[VEL_X]; }
  T& vy() { return (*this)[VEL_Y]; }

  void NormalizeYaw() {}
};

template <typename State_T>
class SpeedMeasModel {
 public:
  using StateType = State_T;
  using MeasurementType = SpeedMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  SpeedMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    if constexpr (std::is_same<StateType, PointState>::value) {
      const ObsMatType H = ComputeH();
      return H * x_in;
    } else if constexpr (std::is_same<StateType, CarState>::value) {
      M m;
      m.vx() = x_in.vel() * std::cos(x_in.yaw());
      m.vy() = x_in.vel() * std::sin(x_in.yaw());
      return m;
    } else {
      QLOG(FATAL) << "No match state type";
    }
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::VEL_X, S::VEL_X) = 1.0;
    H(M::VEL_Y, S::VEL_Y) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_vel_x_measurement_noise_std() &&
        params.has_state_vel_y_measurement_noise_std() &&
        params.has_state_vel_x_vel_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_vel_x_vel_y_measurement_correlation_coefficient() >=
                 -1.0 &&
             params.state_vel_x_vel_y_measurement_correlation_coefficient() <=
                 1.0);
      (*R)(M::VEL_X, M::VEL_X) =
          Sqr(params.state_vel_x_measurement_noise_std());
      (*R)(M::VEL_Y, M::VEL_Y) =
          Sqr(params.state_vel_y_measurement_noise_std());
      (*R)(M::VEL_X, M::VEL_Y) =
          params.state_vel_x_vel_y_measurement_correlation_coefficient() *
          params.state_vel_x_measurement_noise_std() *
          params.state_vel_y_measurement_noise_std();
      (*R)(M::VEL_Y, M::VEL_X) = (*R)(M::VEL_X, M::VEL_Y);
    } else {
      return false;
    }
    return true;
  }
};

class VelocityMeasurement : public Vector<double, 1> {
 public:
  using Base = Vector<double, 1>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int VEL = 0;

  VelocityMeasurement() { this->setZero(); }

  T vel() const { return (*this)[VEL]; }

  T& vel() { return (*this)[VEL]; }

  void NormalizeYaw() {}
};

class VelocityMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = VelocityMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  VelocityMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const ObsMatType H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::VEL, S::VEL) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_vel_measurement_noise_std()) {
      (*R)(M::VEL, M::VEL) = Sqr(params.state_vel_measurement_noise_std());
    } else {
      return false;
    }
    return true;
  }
};

class PosSpeedMeasurement : public Vector<double, 4> {
 public:
  using Base = Vector<double, 4>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VEL_X = 2;
  static constexpr int VEL_Y = 3;

  PosSpeedMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T vx() const { return (*this)[VEL_X]; }
  T vy() const { return (*this)[VEL_Y]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& vx() { return (*this)[VEL_X]; }
  T& vy() { return (*this)[VEL_Y]; }

  void NormalizeYaw() {}
};

template <typename State_T>
class PosSpeedMeasModel {
 public:
  using StateType = State_T;
  using MeasurementType = PosSpeedMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  PosSpeedMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    if constexpr (std::is_same<StateType, PointState>::value) {
      const ObsMatType H = ComputeH();
      return H * x_in;
    } else if constexpr (std::is_same<StateType, CarState>::value) {
      M m;
      m.x() = x_in.x();
      m.y() = x_in.y();
      m.vx() = x_in.vel() * std::cos(x_in.yaw());
      m.vy() = x_in.vel() * std::sin(x_in.yaw());
      return m;
    } else {
      QLOG(FATAL) << "No match state type";
    }
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    H(M::VEL_X, S::VEL_X) = 1.0;
    H(M::VEL_Y, S::VEL_Y) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_x_measurement_noise_std() &&
        params.has_state_y_measurement_noise_std() &&
        params.has_state_x_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_x_y_measurement_correlation_coefficient() >= -1.0 &&
             params.state_x_y_measurement_correlation_coefficient() <= 1.0);
      (*R)(M::X, M::X) = Sqr(params.state_x_measurement_noise_std());
      (*R)(M::Y, M::Y) = Sqr(params.state_y_measurement_noise_std());
      (*R)(M::X, M::Y) =
          params.state_x_y_measurement_correlation_coefficient() *
          params.state_x_measurement_noise_std() *
          params.state_y_measurement_noise_std();
      (*R)(M::Y, M::X) = (*R)(M::X, M::Y);
    } else {
      return false;
    }
    if (params.has_state_vel_x_measurement_noise_std() &&
        params.has_state_vel_y_measurement_noise_std() &&
        params.has_state_vel_x_vel_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_vel_x_vel_y_measurement_correlation_coefficient() >=
                 -1.0 &&
             params.state_vel_x_vel_y_measurement_correlation_coefficient() <=
                 1.0);
      (*R)(M::VEL_X, M::VEL_X) =
          Sqr(params.state_vel_x_measurement_noise_std());
      (*R)(M::VEL_Y, M::VEL_Y) =
          Sqr(params.state_vel_y_measurement_noise_std());
      (*R)(M::VEL_X, M::VEL_Y) =
          params.state_vel_x_vel_y_measurement_correlation_coefficient() *
          params.state_vel_x_measurement_noise_std() *
          params.state_vel_y_measurement_noise_std();
      (*R)(M::VEL_Y, M::VEL_X) = (*R)(M::VEL_X, M::VEL_Y);
    } else {
      return false;
    }
    return true;
  }
};

class PosVelocityMeasurement : public Vector<double, 3> {
 public:
  using Base = Vector<double, 3>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VEL = 2;

  PosVelocityMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T vel() const { return (*this)[VEL]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& vel() { return (*this)[VEL]; }

  void NormalizeYaw() {}
};

class PosVelocityMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = PosVelocityMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  PosVelocityMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const auto H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    H(M::VEL, S::VEL) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_x_measurement_noise_std() &&
        params.has_state_y_measurement_noise_std() &&
        params.has_state_x_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_x_y_measurement_correlation_coefficient() >= -1.0 &&
             params.state_x_y_measurement_correlation_coefficient() <= 1.0);
      (*R)(M::X, M::X) = Sqr(params.state_x_measurement_noise_std());
      (*R)(M::Y, M::Y) = Sqr(params.state_y_measurement_noise_std());
      (*R)(M::X, M::Y) =
          params.state_x_y_measurement_correlation_coefficient() *
          params.state_x_measurement_noise_std() *
          params.state_y_measurement_noise_std();
      (*R)(M::Y, M::X) = (*R)(M::X, M::Y);
    } else {
      return false;
    }
    if (params.has_state_vel_measurement_noise_std()) {
      (*R)(M::VEL, M::VEL) = Sqr(params.state_vel_measurement_noise_std());
    } else {
      return false;
    }
    return true;
  }
};

class PosHeadingMeasurement : public Vector<double, 3> {
 public:
  using Base = Vector<double, 3>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int YAW = 2;

  PosHeadingMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T yaw() const { return (*this)[YAW]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& yaw() { return (*this)[YAW]; }

  void NormalizeYaw() { (*this)[YAW] = NormalizeAngle((*this)[YAW]); }
};

class PosHeadingMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = PosHeadingMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  PosHeadingMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const auto H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    H(M::YAW, S::YAW) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_x_measurement_noise_std() &&
        params.has_state_y_measurement_noise_std() &&
        params.has_state_x_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_x_y_measurement_correlation_coefficient() >= -1.0 &&
             params.state_x_y_measurement_correlation_coefficient() <= 1.0);
      (*R)(M::X, M::X) = Sqr(params.state_x_measurement_noise_std());
      (*R)(M::Y, M::Y) = Sqr(params.state_y_measurement_noise_std());
      (*R)(M::X, M::Y) =
          params.state_x_y_measurement_correlation_coefficient() *
          params.state_x_measurement_noise_std() *
          params.state_y_measurement_noise_std();
      (*R)(M::Y, M::X) = (*R)(M::X, M::Y);
    } else {
      return false;
    }
    if (params.has_state_heading_measurement_noise_std()) {
      (*R)(M::YAW, M::YAW) =
          Sqr(d2r(params.state_heading_measurement_noise_std()));
    } else {
      return false;
    }
    return true;
  }
};

class PosHeadingVelocityMeasurement : public Vector<double, 4> {
 public:
  using Base = Vector<double, 4>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int YAW = 2;
  static constexpr int VEL = 3;

  PosHeadingVelocityMeasurement() { this->setZero(); }

  T x() const { return (*this)[X]; }
  T y() const { return (*this)[Y]; }
  T yaw() const { return (*this)[YAW]; }
  T vel() const { return (*this)[VEL]; }

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& yaw() { return (*this)[YAW]; }
  T& vel() { return (*this)[VEL]; }

  void NormalizeYaw() { (*this)[YAW] = NormalizeAngle((*this)[YAW]); }
};

class PosHeadingVelocityMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = PosHeadingVelocityMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  PosHeadingVelocityMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const auto H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::X, S::X) = 1.0;
    H(M::Y, S::Y) = 1.0;
    H(M::YAW, S::YAW) = 1.0;
    H(M::VEL, S::VEL) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_x_measurement_noise_std() &&
        params.has_state_y_measurement_noise_std() &&
        params.has_state_x_y_measurement_correlation_coefficient()) {
      QCHECK(params.state_x_y_measurement_correlation_coefficient() >= -1.0 &&
             params.state_x_y_measurement_correlation_coefficient() <= 1.0);
      (*R)(M::X, M::X) = Sqr(params.state_x_measurement_noise_std());
      (*R)(M::Y, M::Y) = Sqr(params.state_y_measurement_noise_std());
      (*R)(M::X, M::Y) =
          params.state_x_y_measurement_correlation_coefficient() *
          params.state_x_measurement_noise_std() *
          params.state_y_measurement_noise_std();
      (*R)(M::Y, M::X) = (*R)(M::X, M::Y);
    } else {
      return false;
    }
    if (params.has_state_heading_measurement_noise_std()) {
      (*R)(M::YAW, M::YAW) =
          Sqr(d2r(params.state_heading_measurement_noise_std()));
    } else {
      return false;
    }
    if (params.has_state_vel_measurement_noise_std()) {
      (*R)(M::VEL, M::VEL) = Sqr(params.state_vel_measurement_noise_std());
    } else {
      return false;
    }
    return true;
  }
};

class HeadingVelocityMeasurement : public Vector<double, 2> {
 public:
  using Base = Vector<double, 2>;
  using Base::operator=;
  using Base::Base;
  using T = Base::Scalar;
  static constexpr int YAW = 0;
  static constexpr int VEL = 1;

  HeadingVelocityMeasurement() { this->setZero(); }

  T yaw() const { return (*this)[YAW]; }
  T vel() const { return (*this)[VEL]; }

  T& yaw() { return (*this)[YAW]; }
  T& vel() { return (*this)[VEL]; }

  void NormalizeYaw() { (*this)[YAW] = NormalizeAngle((*this)[YAW]); }
};

class HeadingVelocityMeasModel {
 public:
  using StateType = CarState;
  using MeasurementType = HeadingVelocityMeasurement;
  using CovarianceType = Covariance<MeasurementType>;
  using M = MeasurementType;
  using S = StateType;
  using ObsMatType = ObservationMatrix<S, M>;

  HeadingVelocityMeasModel() {}

  M ComputeExpectedMeasurement(const S& x_in) {
    const auto H = ComputeH();
    return H * x_in;
  }

  ObsMatType ComputeH() const {
    ObsMatType H;
    H.setZero();
    H(M::YAW, S::YAW) = 1.0;
    H(M::VEL, S::VEL) = 1.0;
    return H;
  }

  bool GetMeasurementNoise(const MotionFilterParamProto& params,
                           CovarianceType* R) {
    R->setZero();
    if (params.has_state_heading_measurement_noise_std()) {
      (*R)(M::YAW, M::YAW) =
          Sqr(d2r(params.state_heading_measurement_noise_std()));
    } else {
      return false;
    }
    if (params.has_state_vel_measurement_noise_std()) {
      (*R)(M::VEL, M::VEL) = Sqr(params.state_vel_measurement_noise_std());
    } else {
      return false;
    }
    return true;
  }
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_H_
