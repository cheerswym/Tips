#ifndef ONBOARD_PERCEPTION_CLUSTER_OBSERVER_H_
#define ONBOARD_PERCEPTION_CLUSTER_OBSERVER_H_

#include "absl/container/flat_hash_map.h"
#include "onboard/async/thread_pool.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

class ClusterObserver {
 public:
  explicit ClusterObserver(ThreadPool* thread_pool)
      : thread_pool_(thread_pool) {}

  // Compute and update cluster observation state.
  void Observe(const absl::flat_hash_map<LidarId, RangeImage>& range_images,
               SegmentedClusters* clusters);

 private:
  ThreadPool* const thread_pool_ = nullptr;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_CLUSTER_OBSERVER_H_
