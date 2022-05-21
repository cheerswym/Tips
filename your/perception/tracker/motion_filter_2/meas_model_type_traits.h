#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_TYPE_TRAITS_H_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_TYPE_TRAITS_H_

#include "onboard/perception/tracker/motion_filter_2/extent_meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
namespace qcraft::tracker {

template <typename, typename>
struct MeasModelTypeSelector;

template <>
struct MeasModelTypeSelector<PointState, PositionMeasurement> {
  using type = PosMeasModel<PointState>;
};

template <>
struct MeasModelTypeSelector<CarState, PositionMeasurement> {
  using type = PosMeasModel<CarState>;
};

template <>
struct MeasModelTypeSelector<CarState, HeadingMeasurement> {
  using type = HeadingMeasModel;
};

template <>
struct MeasModelTypeSelector<PointState, SpeedMeasurement> {
  using type = SpeedMeasModel<PointState>;
};

template <>
struct MeasModelTypeSelector<CarState, SpeedMeasurement> {
  using type = SpeedMeasModel<CarState>;
};

template <>
struct MeasModelTypeSelector<CarState, VelocityMeasurement> {
  using type = VelocityMeasModel;
};

template <>
struct MeasModelTypeSelector<PointState, PosSpeedMeasurement> {
  using type = PosSpeedMeasModel<PointState>;
};

template <>
struct MeasModelTypeSelector<CarState, PosSpeedMeasurement> {
  using type = PosSpeedMeasModel<CarState>;
};

template <>
struct MeasModelTypeSelector<CarState, PosVelocityMeasurement> {
  using type = PosVelocityMeasModel;
};

template <>
struct MeasModelTypeSelector<CarState, PosHeadingMeasurement> {
  using type = PosHeadingMeasModel;
};

template <>
struct MeasModelTypeSelector<CarState, PosHeadingVelocityMeasurement> {
  using type = PosHeadingVelocityMeasModel;
};

template <>
struct MeasModelTypeSelector<CarState, HeadingVelocityMeasurement> {
  using type = HeadingVelocityMeasModel;
};

template <>
struct MeasModelTypeSelector<BBoxState, BBoxMeasurement> {
  using type = BBoxMeasModel;
};

template <typename>
struct StateTypeFromMeasType;

template <>
struct StateTypeFromMeasType<PositionMeasurement> {
  using type = void;
};
template <>
struct StateTypeFromMeasType<SpeedMeasurement> {
  using type = void;
};

template <>
struct StateTypeFromMeasType<PosSpeedMeasurement> {
  using type = void;
};

template <>
struct StateTypeFromMeasType<HeadingMeasurement> {
  using type = CarState;
};

template <>
struct StateTypeFromMeasType<VelocityMeasurement> {
  using type = CarState;
};

template <>
struct StateTypeFromMeasType<PosVelocityMeasurement> {
  using type = CarState;
};

template <>
struct StateTypeFromMeasType<PosHeadingMeasurement> {
  using type = CarState;
};

template <>
struct StateTypeFromMeasType<PosHeadingVelocityMeasurement> {
  using type = CarState;
};

template <>
struct StateTypeFromMeasType<HeadingVelocityMeasurement> {
  using type = CarState;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_FILTER_2_MEAS_MODEL_TYPE_TRAITS_H_
