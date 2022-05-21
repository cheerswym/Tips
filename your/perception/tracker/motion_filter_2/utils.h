
#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UTILS_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UTILS_H_
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/geometry/util.h"
#include "onboard/perception/tracker/motion_filter_2/extent_meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/proto/motion_filter.pb.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"

namespace qcraft::tracker {

using Vec6d = Vector<double, 6>;

// Using the fact that NaN is the only value that is not equal to itself
template <typename Derived>
inline bool CheckNaN(const Eigen::MatrixBase<Derived>& x) {
  return !((x.array() == x.array())).all();
}

// Check motion filter divergence
template <typename CovarianceType>
inline bool CheckDivergence(const CovarianceType& P) {
  QCHECK(CovarianceType::RowsAtCompileTime == 6);
  QCHECK(CovarianceType::ColsAtCompileTime == 6);

  for (int i = 0; i < CovarianceType::RowsAtCompileTime; ++i) {
    if (P(i, i) < 0.0) return true;
  }
  return false;
}

// Compute mahalanobis distance
template <typename M>
double ComputeMDistance(const M& meas_diff, const Covariance<M>& S) {
  if constexpr (M::RowsAtCompileTime == 1)
    return meas_diff.transpose() * 1.0 / S(0) * meas_diff;
  else
    return meas_diff.transpose() * S.inverse() * meas_diff;
}

inline bool IsInValidationGateByMahalanobisDistance(double m_dist, int dof) {
  if (m_dist < 0.0) return false;
  QCHECK(dof == 1 || dof == 2);
  // Build chi square table for 95% validation gate.
  static std::vector<double> chi_square_table{0.0,  3.84, 5.99,
                                              7.81, 9.49, 11.07};
  return m_dist < chi_square_table[dof];
}

inline void Vec6dToProto(const Vec6d& m, Vec6dProto* proto) {
  for (int i = 0; i < 6; ++i) proto->add_m(m.data()[i]);
}

inline void Vec6dFromProto(const Vec6dProto& proto, Vec6d* m) {
  QCHECK_EQ(proto.m_size(), 6);
  for (int i = 0; i < 6; ++i) m->data()[i] = proto.m(i);
}

template <typename MotionFilter>
ActionInfo GenerateActionInfo(const ActionInfo::Type type,
                              const MotionFilter& motion_filter) {
  ActionInfo action_info;
  action_info.set_type(type);
  auto* state_proto = action_info.mutable_state();
  Vec6dToProto(motion_filter.state(), state_proto);

  auto* state_cov_proto = action_info.mutable_state_cov();
  Vec6dToProto(motion_filter.state_covariance().diagonal(), state_cov_proto);

  return action_info;
}

template <typename ExtentFilter, typename MeasurementType>
ExtentFilterProto GenerateExtentFilterProto(
    const ExtentFilter& extent_filter, const MeasurementType& z,
    const Covariance<MeasurementType>& R) {
  ExtentFilterProto extent_filter_proto;
  auto* state_proto = extent_filter_proto.mutable_state();
  Vec3dToProto(extent_filter.state(), state_proto);
  auto* state_cov_proto = extent_filter_proto.mutable_state_cov();
  Vec3dToProto(extent_filter.state_covariance().diagonal(), state_cov_proto);
  auto* meas_model_proto = extent_filter_proto.mutable_meas_model();
  GenerateMeasModelProto(z, R, meas_model_proto);

  return extent_filter_proto;
}

template <typename MeasurementType>
void GenerateMeasModelProto(const MeasurementType& z,
                            const Covariance<MeasurementType>& R,
                            MeasModelProto* meas_model_proto) {
  meas_model_proto->Clear();
  meas_model_proto->set_meas_type(GetMeasTypeForProto(z));
  if constexpr (MeasurementType::RowsAtCompileTime == 1) {
    MeasModel1D meas_model_1d;
    meas_model_1d.set_measurement(z[0]);
    meas_model_1d.set_meas_noise(R[0]);
    *meas_model_proto->mutable_meas_model_1d() = std::move(meas_model_1d);
  } else if constexpr (MeasurementType::RowsAtCompileTime == 2) {
    MeasModel2D meas_model_2d;
    Vec2dToProto(z, meas_model_2d.mutable_measurement());
    Mat2dToProto(R, meas_model_2d.mutable_meas_noise());
    *meas_model_proto->mutable_meas_model_2d() = std::move(meas_model_2d);
  } else if constexpr (MeasurementType::RowsAtCompileTime == 3) {
    MeasModel3D meas_model_3d;
    Vec3dToProto(z, meas_model_3d.mutable_measurement());
    Mat3dToProto(R, meas_model_3d.mutable_meas_noise());
    *meas_model_proto->mutable_meas_model_3d() = std::move(meas_model_3d);
  } else if constexpr (MeasurementType::RowsAtCompileTime == 4) {
    MeasModel4D meas_model_4d;
    Vec4dToProto(z, meas_model_4d.mutable_measurement());
    Mat4dToProto(R, meas_model_4d.mutable_meas_noise());
    *meas_model_proto->mutable_meas_model_4d() = std::move(meas_model_4d);
  }
}

template <typename T>
MeasModelProto::Type GetMeasTypeForProto(const T& type) {
  if constexpr (std::is_same<T, PositionMeasurement>::value) {
    return MeasModelProto::POSITION;
  } else if constexpr (std::is_same<T, HeadingMeasurement>::value) {
    return MeasModelProto::HEADING;
  } else if constexpr (std::is_same<T, SpeedMeasurement>::value) {
    return MeasModelProto::SPEED;
  } else if constexpr (std::is_same<T, VelocityMeasurement>::value) {
    return MeasModelProto::VELOCITY;
  } else if constexpr (std::is_same<T, PosSpeedMeasurement>::value) {
    return MeasModelProto::POS_SPEED;
  } else if constexpr (std::is_same<T, PosVelocityMeasurement>::value) {
    return MeasModelProto::POS_VELOCITY;
  } else if constexpr (std::is_same<T, PosHeadingMeasurement>::value) {
    return MeasModelProto::POS_HEADING;
  } else if constexpr (std::is_same<T, PosHeadingVelocityMeasurement>::value) {
    return MeasModelProto::POS_HEADING_VELOCITY;
  } else if constexpr (std::is_same<T, HeadingVelocityMeasurement>::value) {
    return MeasModelProto::HEADING_VELOCITY;
  } else if constexpr (std::is_same<T, BBoxMeasurement>::value) {
    return MeasModelProto::BBOX;
  } else {
    return MeasModelProto::NONE;
  }
}

IMMProto ToImmProto(const std::vector<MotionFilterParamProto>& filter_params,
                    const std::vector<double>& weights);

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_UTILS_H_
