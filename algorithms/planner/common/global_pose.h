#ifndef ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_
#define ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_

#include "common/proto/global_pose.pb.h"
#include "onboard/math/vec.h"

namespace qcraft {

struct GlobalPose {
  Vec3d pos;
  double heading;
};

GlobalPose ConvertToGlobalPose(const GlobalPoseProto &proto);
GlobalPoseProto ConvertToGlobalPoseProto(const GlobalPose &global_pose);

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_
