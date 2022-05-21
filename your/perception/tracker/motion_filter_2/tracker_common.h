#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_TRACKER_COMMON_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_TRACKER_COMMON_H_

#include "onboard/math/eigen.h"

namespace qcraft::tracker {

template <typename T, size_t rows, size_t cols>
using Matrix = Eigen::Matrix<T, rows, cols>;

// Vector can be state or measurement
template <typename T, size_t N>
using Vector = Matrix<T, N, 1>;

template <typename T, size_t N>
using SquareMatrix = Matrix<T, N, N>;

template <typename VectorType>
using Covariance =
    SquareMatrix<typename VectorType::Scalar, VectorType::RowsAtCompileTime>;

// Common matrix type whose row/col is equal to State/Measurement dimension
template <typename State, typename Measurement>
using StateMeasMatrix = Matrix<typename State::Scalar, State::RowsAtCompileTime,
                               Measurement::RowsAtCompileTime>;

// Kalman gain
template <typename State, typename Measurement>
using KalmanGain = StateMeasMatrix<State, Measurement>;

// Cross corelation matrix in UKF.
template <typename State, typename Measurement>
using CrossCorelationMatrix = StateMeasMatrix<State, Measurement>;

template <typename State, typename Measurement>
using ObservationMatrix =
    Matrix<typename State::Scalar, Measurement::RowsAtCompileTime,
           State::RowsAtCompileTime>;

template <typename VectorType, size_t Nums>
using SigmaPointsMatrix =
    Matrix<typename VectorType::Scalar, VectorType::RowsAtCompileTime, Nums>;

template <typename State>
using Weights =
    Vector<typename State::Scalar, 2 * State::RowsAtCompileTime + 1>;

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_TRACKER_COMMON_H_
