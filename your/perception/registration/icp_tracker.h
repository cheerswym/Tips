#ifndef ONBOARD_PERCEPTION_REGISTRATION_ICP_TRACKER_H_
#define ONBOARD_PERCEPTION_REGISTRATION_ICP_TRACKER_H_

#include <cfloat>
#include <limits>
#include <tuple>
#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/perception/registration/icp_with_normal_lls.h"
#include "onboard/perception/segmentation/segmenter.h"

namespace qcraft {

class IcpTracker {
 public:
  explicit IcpTracker(ThreadPool* thread_pool) : thread_pool_(thread_pool) {}

  struct TrackerResult {
    bool success = false;
    double mse = std::numeric_limits<double>::max();
    AffineTransformation transform;
    Eigen::Matrix6d transform_cov = Eigen::Matrix6d::Identity();
    Vec2d velocity;
    Eigen::Matrix2d velocity_cov;
    int prev_cluster_id = -1;
    double prev_cluster_timestamp = 0.0;
    // FIXME(jingwei) remove fields below.
    int64_t cost_time = 0;  // auxiliary measurement time-consuming(remove)
    int src_num = 0;
    int ref_num = 0;
    bool dir_prev2src = false;
  };

  std::vector<IcpTracker::TrackerResult> FastTrack(
      const VehiclePose& pose, const RunParamsProtoV2& run_params,
      const SegmentedClusters& curr_clusters);

  std::vector<IcpTracker::TrackerResult> Track(
      const VehiclePose& pose, const RunParamsProtoV2& run_params,
      const SegmentedClusters& curr_clusters);

 private:
  // RegPair consists of <prev_idx, curr_idx, who_has_normal>
  // About who_has_normal, 0->prev, 1->curr, -1->no_normal
  using RegPair = std::tuple<int, int, int>;
  using RegPairs = std::vector<RegPair>;

  std::vector<int> NaiveMatch(
      const std::vector<ClusterWithObstacles>& row_clusters,
      const std::vector<ClusterWithObstacles>& col_clusters) const;

  RegPairs CorrectMatchAndPrepareNormal(
      const RunParamsProtoV2& run_params, const VehiclePose& pose,
      const std::vector<int>& matches,
      std::vector<ClusterWithObstacles>* prev_cluster,
      std::vector<ClusterWithObstacles>* curr_cluster) const;

  bool Point2PointICP(const std::vector<ClusterWithObstacles>& curr_clusters,
                      const std::vector<ClusterWithObstacles>& prev_clusters,
                      const std::vector<int>& matches, const VehiclePose& pose,
                      std::vector<IcpTracker::TrackerResult>* results) const;

  bool Point2PlaneICP(const std::vector<ClusterWithObstacles>& prev_clusters,
                      const std::vector<int>& matches, const VehiclePose& pose,
                      const RunParamsProtoV2& run_params,
                      std::vector<ClusterWithObstacles>* curr_clusters,
                      std::vector<IcpTracker::TrackerResult>* results) const;

  bool Point2PlaneICP(const IcpTracker::RegPairs reg_pairs,
                      const std::vector<ClusterWithObstacles>& prev_clusters,
                      const std::vector<ClusterWithObstacles>& curr_clusters,
                      std::vector<IcpTracker::TrackerResult>* results) const;

  bool ValidType(const Cluster& curr_cluster,
                 const Cluster& prev_cluster) const;

  bool ValidType(const Cluster& cluster) const;

  bool ValidZone(const VehiclePose& ego_pose, const Cluster& cluster) const;

  bool ValidPointNum(const Cluster& cluster) const;

  bool WhetherRegistration(const VehiclePose& ego_pose,
                           const Cluster& cluster) const;

  bool WhetherRegistration(const VehiclePose& ego_pose,
                           const Cluster& curr_cluster,
                           const Cluster& prev_cluster) const;

  std::vector<ClusterWithObstacles> prev_clusters_;

  // Not owned.
  ThreadPool* const thread_pool_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_ICP_TRACKER_H_
