#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_STATE_DATA_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_STATE_DATA_H_

#include <utility>

#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/extent_model.h"
#include "onboard/perception/tracker/motion_filter_2/img_2d_model.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
namespace qcraft::tracker {

class ExtentStateData {
 public:
  using S = BBoxState;
  using CovarianceType = Covariance<S>;

  ExtentStateData() { state_cov_.setIdentity(); }

  ExtentStateData(const S& state, const CovarianceType& state_cov)
      : state_(state), state_cov_(state_cov) {}
  S& state() { return state_; }

  S state() const { return state_; }

  CovarianceType& state_cov() { return state_cov_; }

  CovarianceType state_cov() const { return state_cov_; }

 private:
  S state_;
  CovarianceType state_cov_;
};

class Img2DStateData {
 public:
  using S = Img2DState;
  using CovarianceType = Covariance<Img2DState>;

  Img2DStateData() { state_cov_.setIdentity(); }

  Img2DStateData(const Img2DState& state, const CovarianceType& state_cov)
      : state_(state), state_cov_(state_cov) {}

  Img2DState& state() { return state_; }
  CovarianceType& state_cov() { return state_cov_; }

  Img2DState state() const { return state_; }
  CovarianceType state_cov() const { return state_cov_; }

  Eigen::Matrix4d GetStatePosCov() const {
    return state_cov_.block<4, 4>(S::X, S::X);
  }

 private:
  Img2DState state_;
  CovarianceType state_cov_;
};

class PointStateData {
 public:
  using S = PointState;
  using CovarianceType = Covariance<PointState>;

  PointStateData() { state_cov_.setIdentity(); }

  PointStateData(const PointState& state, const CovarianceType& state_cov)
      : state_(state), state_cov_(state_cov) {}
  PointState& state() { return state_; }

  PointState state() const { return state_; }

  CovarianceType& state_cov() { return state_cov_; }

  CovarianceType state_cov() const { return state_cov_; }

  Eigen::Matrix2d GetPosCov() const {
    return state_cov_.block<2, 2>(S::X, S::X);
  }
  void SetPosCov(const Eigen::Matrix2d& pos_cov) {
    state_cov_.block<2, 2>(S::X, S::X) = pos_cov;
  }
  Eigen::Matrix2d GetSpeedCov() const {
    return state_cov_.block<2, 2>(S::VEL_X, S::VEL_X);
  }

  Eigen::Matrix4d GetPosSpeedCov() const {
    return state_cov_.block<4, 4>(S::X, S::X);
  }

  Eigen::Matrix2d GetAxAyCov() const {
    return state_cov_.block<2, 2>(S::ACC_X, S::ACC_X);
  }

 private:
  PointState state_;
  CovarianceType state_cov_;
};

class CarStateData {
 public:
  using S = CarState;
  using T = typename CarState::Scalar;
  using CovarianceType = Covariance<PointState>;
  CarStateData() { state_cov_.setIdentity(); }

  CarStateData(const CarState& state, const CovarianceType& state_cov)
      : state_(state), state_cov_(state_cov) {}
  CarState& state() { return state_; }

  CarState state() const { return state_; }

  CovarianceType& state_cov() { return state_cov_; }

  CovarianceType state_cov() const { return state_cov_; }

  Eigen::Matrix2d GetPosCov() const {
    return state_cov_.block<2, 2>(S::X, S::X);
  }

  void SetPosCov(const Eigen::Matrix2d& pos_cov) {
    state_cov_.block<2, 2>(S::X, S::X) = pos_cov;
  }

  T GetVelCov() const { return state_cov_(S::VEL, S::VEL); }

  T GetAccCov() const { return state_cov_(S::ACC, S::ACC); }

  T GetYawCov() const { return state_cov_(S::YAW, S::YAW); }

  T GetYawDCov() const { return state_cov_(S::YAWD, S::YAWD); }

 public:
  CarState state_;
  CovarianceType state_cov_;
};

class StateData {
 public:
  using P = PointState;
  using C = CarState;
  using T = typename PointState::Scalar;

  template <typename StateType>
  explicit StateData(StateType&& state) {
    using StateT = typename std::decay<StateType>::type;
    if constexpr (std::is_same<StateT, PointState>::value) {
      point_.state() = std::forward<StateType>(state);
      PointToCar();
    } else if constexpr (std::is_same<StateT, CarState>::value) {
      car_.state() = std::forward<StateType>(state);
      CarToPoint();
    } else if constexpr (std::is_same<StateT, PointStateData>::value) {
      point_ = std::forward<StateType>(state);
      PointToCar();
    } else if constexpr (std::is_same<StateT, CarStateData>::value) {
      car_ = std::forward<StateType>(state);
      CarToPoint();
    } else {
      QLOG(FATAL) << "No match input type for StateData constrction.";
    }
  }

  // Get State Element
  Eigen::Vector2d GetStatePos() const { return point_.state().head<2>(); }

  Eigen::Vector2d GetStateSpeed() const {
    return point_.state().block<2, 1>(2, 0);
  }

  Eigen::Vector4d GetStatePosSpeed() const { return point_.state().head<4>(); }

  T x() const { return point_.state().x(); }

  T y() const { return point_.state().y(); }

  T GetVx() const { return point_.state().vx(); }

  T GetVy() const { return point_.state().vy(); }

  T GetAx() const { return point_.state().ax(); }

  T GetAy() const { return point_.state().ay(); }

  T GetVel() const { return car_.state().vel(); }

  T GetAcc() const { return car_.state().acc(); }

  T GetYaw() const { return car_.state().yaw(); }

  T GetYawD() const { return car_.state().yawd(); }

  void SetPosX(T x) {
    point_.state().x() = x;
    car_.state().x() = x;
  }

  void SetPosY(T y) {
    point_.state().y() = y;
    car_.state().y() = y;
  }

  void SetPos(T x, T y) {
    SetPosX(x);
    SetPosY(y);
  }

  void SetPosCov(const Eigen::Matrix2d& pos_cov) {
    point_.SetPosCov(pos_cov);
    car_.SetPosCov(pos_cov);
  }

  // Get state covariance.
  Eigen::Matrix2d GetStatePosCov() const { return point_.GetPosCov(); }

  Eigen::Matrix2d GetSpeedCov() const { return point_.GetSpeedCov(); }

  Eigen::Matrix2d GetAxAyCov() const { return point_.GetAxAyCov(); }

  Eigen::Matrix4d GetPosSpeedCov() const { return point_.GetPosSpeedCov(); }

  T GetVelCov() const { return car_.GetVelCov(); }

  T GetAccCov() const { return car_.GetAccCov(); }

  T GetYawCov() const { return car_.GetYawCov(); }

  T GetYawDCov() const { return car_.GetYawDCov(); }

  PointStateData& point() { return point_; }

  PointStateData point() const { return point_; }

  CarStateData& car() { return car_; }

  CarStateData car() const { return car_; }

  friend std::ostream& operator<<(std::ostream& out, const StateData& data) {
    out << "State Data: \n"
        << "Point State: \n"
        << data.point().state() << "\nPoint State Cov: \n"
        << data.point().state_cov() << "\nCar State: \n"
        << data.car().state() << "\nCar State Cov: \n"
        << data.car().state_cov();
    return out;
  }

  void PointToCar() {
    car_.state().x() = point_.state().x();
    car_.state().y() = point_.state().y();
    car_.state().yaw() = std::atan2(point_.state().vy(), point_.state().vx());
    car_.state().vel() = std::sqrt(point_.state().vx() * point_.state().vx() +
                                   point_.state().vy() * point_.state().vy());
    // Angular velocity cannot be derived from point state(). So we set it to 0.
    car_.state().yawd() = 0.0;
    car_.state().acc() = 0.0;
    // NOTE(Jiawei): atan2(y, x) returns angle between [-pi, pi], we have to
    // normalize it to [-pi, pi).
    car_.state().NormalizeYaw();
    car_.state_cov().block<2, 2>(C::X, C::X) =
        point_.state_cov().block<2, 2>(P::X, P::X);
  }

  void CarToPoint() {
    point_.state().x() = car_.state().x();
    point_.state().y() = car_.state().y();
    point_.state().vx() = car_.state().vel() * std::cos(car_.state().yaw());
    point_.state().vy() = car_.state().vel() * std::sin(car_.state().yaw());
    point_.state().ax() = car_.state().acc() * std::cos(car_.state().yaw());
    point_.state().ay() = car_.state().acc() * std::sin(car_.state().yaw());

    point_.state_cov().block<2, 2>(P::X, P::X) =
        car_.state_cov().block<2, 2>(C::X, C::X);
  }

 private:
  PointStateData point_;
  CarStateData car_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_STATE_DATA_H_
