#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_ESTIMATOR_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_ESTIMATOR_H_

#include <vector>

#include "onboard/perception/tracker/motion_filter_2/extent_filter.h"
#include "onboard/perception/tracker/motion_filter_2/motion_filter.h"
#include "onboard/perception/tracker/motion_filter_2/state_data.h"

namespace qcraft::tracker {

class Estimator {
 public:
  Estimator() = default;
  explicit Estimator(const IMMProto& proto, bool enable_debug = false);

  void Init(const StateData& x, double timestamp) {
    prev_timestamp_ = timestamp;
    if (is_point_model_) {
      point_model_.Init(x.point().state());
    } else {
      car_model_.Init(x.car().state());
    }
  }

  void InitStateAndCov(const StateData& x, double timestamp) {
    prev_timestamp_ = timestamp;
    SetStateData(x);
  }

  StateData ComputePrediction(double timestamp) const {
    const double dt = timestamp - prev_timestamp_;
    if (dt < 0.0) {
      VLOG(2) << "  timestamp: " << timestamp
              << "  prev_timestamp_: " << prev_timestamp_ << "  dt: " << dt;
      return GetStateData();
    }
    if (is_point_model_) {
      return point_model_.ComputePrediction(dt);
    } else {
      return car_model_.ComputePrediction(dt);
    }
  }

  template <typename MeasurementType>
  double ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
      const MeasurementType& z, double timestamp,
      const std::optional<MotionFilterParamProto>& params =
          std::nullopt) const {
    const double dt = timestamp - prev_timestamp_;
    if (dt < 0.0) {
      VLOG(2) << "  timestamp: " << timestamp
              << "  prev_timestamp_: " << prev_timestamp_ << "  dt: " << dt;
      return -1.0;
    }
    if constexpr (std::is_same<
                      typename StateTypeFromMeasType<MeasurementType>::type,
                      PointState>::value) {
      QCHECK(is_point_model_);
      return point_model_
          .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(z, dt,
                                                                      params);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      CarState>::value) {
      QCHECK(!is_point_model_);
      return car_model_
          .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(z, dt,
                                                                      params);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      void>::value) {
      if (is_point_model_)
        return point_model_
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(z, dt,
                                                                        params);
      else
        return car_model_
            .ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(z, dt,
                                                                        params);
    }
  }

  void Predict(double timestamp) {
    const double dt = timestamp - prev_timestamp_;
    // NOTE(jiawei): This temporary solution will affect predict-update cycle.
    if (dt < 0.0) {
      VLOG(2) << "  timestamp: " << timestamp
              << "  prev_timestamp_: " << prev_timestamp_ << "  dt: " << dt;
      return;
    }

    prev_timestamp_ = timestamp;
    if (is_point_model_) {
      point_model_.Predict(dt);
    } else {
      car_model_.Predict(dt);
    }
  }

  template <typename MeasurementType>
  void Update(
      const MeasurementType& z,
      const std::optional<MotionFilterParamProto>& param = std::nullopt) {
    if constexpr (std::is_same<
                      typename StateTypeFromMeasType<MeasurementType>::type,
                      PointState>::value) {
      QCHECK(is_point_model_);
      point_model_.Update(z, param);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      CarState>::value) {
      QCHECK(!is_point_model_);
      car_model_.Update(z, param);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      void>::value) {
      if (is_point_model_) {
        point_model_.Update(z, param);
      } else {
        car_model_.Update(z, param);
      }
    }
  }

  template <typename MeasurementType>
  void PredictAndUpdate(const MeasurementType& z, double timestamp) {
    Predict(timestamp);
    Update(z);
  }

  template <typename MeasurementType>
  void PredictAndUpdate(const MeasurementType& z,
                        const MotionFilterParamProto& param, double timestamp) {
    Predict(timestamp);
    Update(z, param);
  }

  void InitExtent(const BBoxState& x) {
    extent_filter_.Init(x);
    extent_is_initialized_ = true;
  }

  void PredictExtent(double timestamp) {
    const double dt = timestamp - prev_timestamp_;
    if (dt < 0.0) {
      VLOG(2) << "  timestamp: " << timestamp
              << "  prev_timestamp_: " << prev_timestamp_ << "  dt: " << dt;
      return;
    }
    extent_filter_.Predict(dt);
  }

  template <typename MeasurementType>
  void UpdateExtent(
      const MeasurementType& z,
      const std::optional<MotionFilterParamProto>& param = std::nullopt) {
    extent_filter_.Update(z, param);
  }

  template <typename MeasurementType>
  double ComputeMahalanobisDistance(
      const MeasurementType& z,
      const std::optional<MotionFilterParamProto>& param = std::nullopt) const {
    if constexpr (std::is_same<
                      typename StateTypeFromMeasType<MeasurementType>::type,
                      PointState>::value) {
      QCHECK(is_point_model_);
      return point_model_.ComputeMahalanobisDistance(z, param);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      CarState>::value) {
      QCHECK(!is_point_model_);
      return car_model_.ComputeMahalanobisDistance(z, param);
    } else if constexpr (std::is_same<typename StateTypeFromMeasType<  // NOLINT
                                          MeasurementType>::type,
                                      void>::value) {
      if (is_point_model_) {
        return point_model_.ComputeMahalanobisDistance(z, param);
      } else {
        return car_model_.ComputeMahalanobisDistance(z, param);
      }
    }
  }

  // NOTE(jiawei): Essentially, the uncertainty can't be measured by
  // determinant. This function is only for adapting Motion Filter 1.0.
  double ComputeUncertainty() const {
    if (is_point_model_) {
      return point_model_.state_covariance().determinant();
    } else {
      return car_model_.state_covariance().determinant();
    }
  }

  StateData GetStateData() const {
    if (is_point_model_) {
      return StateData(PointStateData(point_model_.state(),
                                      point_model_.state_covariance()));
    } else {
      return StateData(
          CarStateData(car_model_.state(), car_model_.state_covariance()));
    }
  }

  ExtentStateData GetExtentStateData() const {
    return {extent_filter_.state(), extent_filter_.state_covariance()};
  }

  Eigen::Vector2d GetStatePos() const { return GetStateData().GetStatePos(); }

  Eigen::Matrix2d GetStatePosCov() const {
    return GetStateData().GetStatePosCov();
  }

  Eigen::VectorXd model_probability() {
    if (is_point_model_) {
      return point_model_.model_probability();
    } else {
      return car_model_.model_probability();
    }
  }

  // TODO(jiawei, zheng): Clarify if we really need to set state and state
  // covariance.
  void SetStateData(const StateData& s) {
    if (is_point_model_) {
      point_model_.SetStateAndCov(s.point().state(), s.point().state_cov());
    } else {
      car_model_.SetStateAndCov(s.car().state(), s.car().state_cov());
    }
  }

  MotionFilterParamProto GetMeasurementNoise() const {
    QCHECK(params_.params_size() > 0);
    return params_.params(0).filter_param();
  }

  MotionFilterProto GetMotionFilterInfo() const;

  void ClearMotionFilterInfo() {
    if (is_point_model_) {
      point_model_.ClearMotionFilterInfo();
    } else {
      car_model_.ClearMotionFilterInfo();
    }
  }

  bool IsPointModel() const { return is_point_model_; }
  bool IsCarModel() const { return !is_point_model_; }

  double prev_timestamp() const { return prev_timestamp_; }

  bool extent_is_initialized() const { return extent_is_initialized_; }

 private:
  MotionFilter<PointState> point_model_;
  MotionFilter<CarState> car_model_;
  ExtentFilter<BBoxModel> extent_filter_;
  IMMProto params_;
  bool is_point_model_;
  double prev_timestamp_;
  bool extent_is_initialized_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_ESTIMATOR_H_
