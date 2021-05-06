#ifndef ONBOARD_MATH_KALMAN_FILTER_H_
#define ONBOARD_MATH_KALMAN_FILTER_H_

namespace qcraft {

// SIZE_X is dimension of state. SIZE_Z is dimension of measurement. SIZE_U is
// dimension of control.
template <typename T, unsigned int SIZE_X, unsigned int SIZE_Z,
          unsigned int SIZE_U>
class KalmanFilter {
 public:
  // State vector type.
  using SVec = Eigen::Matrix<T, SIZE_X, 1>;
  // Control vector type.
  using CVec = Eigen::Matrix<T, SIZE_U, 1>;
  // Measurement vector type.
  using MVec = Eigen::Matrix<T, SIZE_Z, 1>;

  // State covariance type.
  using SCov = Eigen::Matrix<T, SIZE_X, SIZE_X>;
  // Measurement covariance type.
  using MCov = Eigen::Matrix<T, SIZE_Z, SIZE_Z>;

  // State transition matrix type.
  using SMat = Eigen::Matrix<T, SIZE_X, SIZE_X>;
  // Control matrix type.
  using CMat = Eigen::Matrix<T, SIZE_X, SIZE_U>;
  // Measurement matrix type.
  using MMat = Eigen::Matrix<T, SIZE_Z, SIZE_X>;

  // Gain matrix type.
  using GMat = Eigen::Matrix<T, SIZE_X, SIZE_Z>;

  const SVec &x() const { return x_; }
  SVec *mutable_x() { return &x_; }
  const SCov &P() const { return P_; }
  SCov *mutable_P() { return &P_; }
  const SMat &A() const { return A_; }
  SMat *mutable_A() { return &A_; }
  const SMat &B() const { return B_; }
  SMat *mutable_B() { return &B_; }
  const SCov &Q() const { return Q_; }
  SCov *mutable_Q() { return &Q_; }
  const MMat &H() const { return H_; }
  MMat *mutable_H() { return &H_; }
  const MCov &R() const { return R_; }
  MCov *mutable_R() { return &R_; }
  const MVec &y() const { return y_; }
  MVec *mutable_y() { return &y_; }
  const MCov &S() const { return S_; }
  MCov *mutable_S() { return &S_; }
  const GMat &K() const { return K_; }
  GMat *mutable_K() { return &K_; }

  void Predict(const CVec &u = CVec::Zero());
  void Correct(const MVec &z);

 private:
  // Mean of the current state distribution.
  SVec x_;
  // Covariance of the current state distribution.
  SCov P_;
  // State transition matrix.
  SMat A_;
  // Control matrix.
  SMat B_;
  // Covariance of the state transition noise.
  SCov Q_;
  // Measurement matrix.
  MMat H_;
  // Covariance of measurement noise.
  MCov R_;
  // Difference between actual measurement and expected measurement from
  // prediction.
  MVec y_;
  // Covariance of measurement.
  MCov S_;
  // Kalman gain.
  GMat K_;
};

template <typename T, unsigned int SIZE_X, unsigned int SIZE_Z,
          unsigned int SIZE_U>
inline void KalmanFilter<T, SIZE_X, SIZE_Z, SIZE_U>::Predict(const CVec &u) {
  x_ = A_ * x_ + B_ * u;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

template <typename T, unsigned int SIZE_X, unsigned int SIZE_Z,
          unsigned int SIZE_U>
inline void KalmanFilter<T, SIZE_X, SIZE_Z, SIZE_U>::Correct(
    const Eigen::Matrix<T, SIZE_Z, 1> &z) {
  y_ = z - H_ * x_;
  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = S_.transpose().partialPivLu().solve(H_ * P_.transpose()).transpose();
  x_ = x_ + K_ * y_;
  P_ = (SCov::Identity() - K_ * H_) * P_;
}

}  // namespace qcraft

#endif  // ONBOARD_MATH_KALMAN_FILTER_H_
