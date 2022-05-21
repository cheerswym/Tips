#ifndef ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_H_
#define ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_H_

#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/registration/point_matcher_structs.h"

namespace qcraft {

class IcpWithNormal {
 public:
  PointMatchResult MatchCluster(const Cluster& ref_cluster,
                                const Cluster& src_cluster,
                                const PointMatcherOptions& options) const;
  PointMatchResult MatchPoints(const std::vector<Vec3d>& ref_points,
                               const std::vector<Vec3d>& src_points,
                               const PointMatcherOptions& options,
                               const AffineTransformation& transform_inv) const;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_NORMAL_H_
