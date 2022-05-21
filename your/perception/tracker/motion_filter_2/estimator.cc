#include "onboard/perception/tracker/motion_filter_2/estimator.h"

namespace qcraft::tracker {

Estimator::Estimator(const IMMProto& proto, bool enable_debug)
    : params_(proto), extent_is_initialized_(false) {
  // Check if the motion filter model is consistent (point or car model).
  bool is_point(false), is_car(false);
  for (const auto& param : proto.params()) {
    switch (param.filter_param().type()) {
      case MotionFilterParamProto::POINT_CP:
      case MotionFilterParamProto::POINT_CV:
      case MotionFilterParamProto::POINT_CA:
        is_point = true;
        break;
      case MotionFilterParamProto::CAR_CP:
      case MotionFilterParamProto::CAR_CV:
      case MotionFilterParamProto::CAR_CA:
      case MotionFilterParamProto::CAR_CTRV:
        is_car = true;
        break;
      default:
        QLOG(FATAL) << "Unknown motion filter type from input params";
    }
  }
  QCHECK(is_point ^ is_car) << "Motion Filter Model must be consistent.";

  is_point_model_ = is_point;
  if (is_point_model_) {
    point_model_ = MotionFilter<PointState>(proto, enable_debug);
  } else {
    car_model_ = MotionFilter<CarState>(proto, enable_debug);
  }

  QCHECK_GT(proto.params_size(), 0);
  extent_filter_ = ExtentFilter<BBoxModel>(proto.params(0).filter_param());
}

MotionFilterProto Estimator::GetMotionFilterInfo() const {
  MotionFilterProto motion_filter_proto;
  if (is_point_model_) {
    motion_filter_proto = point_model_.GetMotionFilterInfo();
  } else {
    motion_filter_proto = car_model_.GetMotionFilterInfo();
  }
  motion_filter_proto.set_is_car_model(IsCarModel());
  motion_filter_proto.set_timestamp(prev_timestamp_);
  *motion_filter_proto.mutable_extent_filter() =
      extent_filter_.GetExtentFilterProto();
  return motion_filter_proto;
}

}  // namespace qcraft::tracker
