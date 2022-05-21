#ifndef ONBOARD_PERCEPTION_CLUSTER_UTIL_H_
#define ONBOARD_PERCEPTION_CLUSTER_UTIL_H_

#include <memory>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/perception/cluster.h"

namespace qcraft::cluster_util {

// Compute the contour of the given cluster as a convex 2D polygon.
Polygon2d ComputeContour(const Cluster& cluster);

// Compute the contour of the given cluster in 3d points with the given z.
std::vector<Vec3d> ComputeContourWithZ(const Cluster& cluster, double z);

// Compute the contour of the given cluster from all its points. This is more
// precise than ComputeContour but also more expensive.
Polygon2d ComputeContourFromClusterPoints(const Cluster& cluster);

// For front nearby clusters, compute the contour to tightly bound all laser
// points as a more precise contour comparing to the contour from obstacles.
Polygon2d ComputeContourWithRefinement(const VehiclePose& pose,
                                       const Cluster& cluster);

// Collect all points from the given cluster.
std::vector<Vec3d> CollectPoints(const Cluster& cluster);

// Compute IoU of two clusters.
double ComputeIoU(const Cluster& cluster1, const Cluster& cluster2);

// Compute ground z.
double ComputeGroundZ(const Cluster& cluster);

// Collect points that are between min_z and max_z.
std::vector<Vec3d> CollectPointsInZRange(const Cluster& cluster, double min_z,
                                         double max_z);

}  // namespace qcraft::cluster_util

#endif  // ONBOARD_PERCEPTION_CLUSTER_UTIL_H_
