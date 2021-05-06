#ifndef ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_

#include <string>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

inline bool IsVehicle(ObjectType type) {
  return type == ObjectType::OT_VEHICLE;
}

//  Returns the contour of a perception object.
Polygon2d ComputeObjectContour(const ObjectProto &object);

// Returns object proto representation of AV.
ObjectProto AvPoseProtoToObjectProto(
    const std::string &object_id,
    const VehicleGeometryParamsProto &vehicle_geom, const PoseProto &pose,
    bool offroad);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
