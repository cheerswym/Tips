#ifndef ONBOARD_PERCEPTION_TYPE_UTIL_H_
#define ONBOARD_PERCEPTION_TYPE_UTIL_H_

#include "onboard/proto/perception.pb.h"

namespace qcraft::type_util {

// Converts from ObjectType to MeasurementType.
MeasurementType ToMeasurementType(ObjectType type);

// Converts from MeasurementType to ObjectType.
ObjectType ToObjectType(MeasurementType type);
}  // namespace qcraft::type_util

#endif  // ONBOARD_PERCEPTION_TYPE_UTIL_H_
