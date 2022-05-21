#ifndef ONBOARD_PERCEPTION_CLUSTER_FILTER_H_
#define ONBOARD_PERCEPTION_CLUSTER_FILTER_H_

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "offboard/labeling/proto/filtering.pb.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/lite_module.h"
#include "onboard/nets/mist_net.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/obstacle_util.h"

namespace qcraft {

class ClusterFilter {
 public:
  ClusterFilter(const RunParamsProtoV2& run_params,
                const NetParam& mist_net_v1_param, ThreadPool* thread_pool,
                LiteModule* module)
      : module_(module),
        mist_net_v1_(mist_net_v1_param),
        thread_pool_(thread_pool) {
    for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
      lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
    }
    av_height_ = obstacle_util::ComputeAvHeight(run_params);
  }
  ~ClusterFilter();
  void Filter(const VehiclePose& pose,
              const labeling::LabelFrameProto* latest_label_frame,
              ObstacleManager* obstacle_manager,
              SegmentedClusters* segmented_clusters);
  void AddClusterData(const VehiclePose& pose,
                      const SegmentedClusters& clusters,
                      filtering::ClustersProto* clusters_proto,
                      const labeling::LabelFrameProto* label_frame,
                      const ObstacleManager& obstacle_manager) const;

 private:
  bool IsRainSegmentedCluster(const VehiclePose& pose,
                              const SegmentedCluster& segmented_cluster,
                              ClusterFilterDebugProto* debug_proto) const;

  bool IsMirrorReflectionSegmentedCluster(
      const SegmentedCluster& segmented_cluster,
      ClusterFilterDebugProto* debug_proto) const;

  bool IsBloomingSegmentedCluster(const VehiclePose& pose,
                                  const SegmentedCluster& segmented_cluster,
                                  const ObstacleManager& obstacle_manager,
                                  ClusterFilterDebugProto* debug_proto) const;

  struct FilterOutput {
    ClusterFilterDebugProto debug_proto;
    std::vector<int> filtered_cluster_indices;
  };

  FilterOutput FilterMistClusters(
      const VehiclePose& pose,
      const SegmentedClusters& segmented_clusters) const;

  FilterOutput FilterBloomingClusters(
      const VehiclePose& pose, const SegmentedClusters& segmented_clusters,
      const ObstacleManager& obstacle_manager) const;

  FilterOutput FilterReflectionClusters(
      const VehiclePose& pose,
      const SegmentedClusters& segmented_clusters) const;

  FilterOutput FilterSmallClusters(
      const SegmentedClusters& segmented_clusters) const;

  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;
  float av_height_ = -1.0f;
  filtering::ClustersProto clusters_proto_;
  LiteModule* const module_;
  MistNet mist_net_v1_;
  // The model apply to the point which does not own 'has return behind'.
  ThreadPool* thread_pool_ = nullptr;
  mutable absl::Mutex mist_mutex_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_CLUSTER_FILTER_H_
