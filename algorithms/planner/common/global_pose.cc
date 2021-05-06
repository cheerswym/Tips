#include "onboard/planner/common/global_pose.h"

namespace qcraft {
GlobalPose ConvertToGlobalPose(const GlobalPoseProto &proto) {
  return {.pos = Vec3d(proto.geo_location().longitude(),
                       proto.geo_location().latitude(),
                       proto.geo_location().altitude()),
          .heading = proto.yaw()};
}

GlobalPoseProto ConvertToGlobalPoseProto(const GlobalPose &global_pose) {
  GlobalPoseProto proto;
  proto.mutable_geo_location()->set_longitude(global_pose.pos.x());
  proto.mutable_geo_location()->set_latitude(global_pose.pos.y());
  proto.mutable_geo_location()->set_altitude(global_pose.pos.z());
  proto.set_yaw(global_pose.heading);
  return proto;
}

}  // namespace qcraft
