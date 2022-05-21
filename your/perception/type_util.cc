#include "onboard/perception/type_util.h"

#include "onboard/lite/logging.h"

namespace qcraft::type_util {

MeasurementType ToMeasurementType(ObjectType type) {
  switch (type) {
    case OT_UNKNOWN_STATIC:
      return MT_STATIC_OBJECT;
    case OT_VEHICLE:
      return MT_VEHICLE;
    case OT_MOTORCYCLIST:
      return MT_MOTORCYCLIST;
    case OT_PEDESTRIAN:
      return MT_PEDESTRIAN;
    case OT_CYCLIST:
      return MT_CYCLIST;
    case OT_FOD:
      return MT_FOD;
    case OT_UNKNOWN_MOVABLE:
      return MT_UNKNOWN;
    case OT_VEGETATION:
      return MT_VEGETATION;
    case OT_BARRIER:
      return MT_BARRIER;
    case OT_CONE:
      return MT_CONE;
    case OT_WARNING_TRIANGLE:
      return MT_WARNING_TRIANGLE;
  }
  QLOG(WARNING) << "Unknown ObjectType at " << static_cast<int>(type);
  return MT_UNKNOWN;
}

ObjectType ToObjectType(MeasurementType type) {
  switch (type) {
    case MT_UNKNOWN:
      return OT_UNKNOWN_MOVABLE;
    case MT_VEHICLE:
      return OT_VEHICLE;
    case MT_MOTORCYCLIST:
      return OT_MOTORCYCLIST;
    case MT_PEDESTRIAN:
      return OT_PEDESTRIAN;
    case MT_CYCLIST:
      return OT_CYCLIST;
    case MT_FOD:
      return OT_FOD;
    case MT_STATIC_OBJECT:
      return OT_UNKNOWN_STATIC;
    case MT_VEGETATION:
      return OT_VEGETATION;
    case MT_BARRIER:
      return OT_BARRIER;
    case MT_CONE:
      return OT_CONE;
    case MT_WARNING_TRIANGLE:
      return OT_WARNING_TRIANGLE;

    // The following types are internal use only, we should not publish them to
    // downstream, aka convert to ObjectType.
    case MT_ROAD:
    case MT_MIST:
    case MT_FLYING_BIRD:
      break;
  }
  QLOG(FATAL) << "Should not reach here at " << static_cast<int>(type);
  return OT_UNKNOWN_MOVABLE;
}
}  // namespace qcraft::type_util
