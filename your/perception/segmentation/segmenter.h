#ifndef ONBOARD_PERCEPTION_SEGMENTATION_SEGMENTER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_SEGMENTER_H_

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lite/logging.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/proposer.h"
#include "onboard/perception/segmentation/proposer_tree.h"
#include "onboard/perception/segmentation/types.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

namespace segmentation {

class Segmenter {
  friend class SegmenterTest;

 public:
  explicit Segmenter(ThreadPool* thread_pool);
  Segmenter(const Segmenter&) = delete;
  Segmenter& operator=(const Segmenter&) = delete;

  SegmentedClusters Segment(const VehiclePose&, const ObstacleManager&,
                            const SemanticMapManager&, const LocalImagery&,
                            const CoordinateConverter&, const RunParamsProtoV2&,
                            const Context&, ClusterVector);

  SegmentationObjectsProto& segmentation_objects_proto() {
    return *segmentation_objects_proto_;
  }

  const std::map<ProposerType, SegmentedClusters>& noise_clusters() const {
    return noise_clusters_;
  }

 private:
  // Traverse proposer tree and get all proposals.
  SceneMap Propose(const ProposerEnvInfo&, ProposedClusters);
  // Select final segmented clusters among proposals.
  SegmentedClusters Select(const SceneMap&);

  ThreadPool* thread_pool_;
  ProposerTree proposer_tree_;

  uint32_t latest_cluster_id_ = 0;

  std::unique_ptr<SegmentationObjectsProto> segmentation_objects_proto_;
  std::map<ProposerType, SegmentedClusters> noise_clusters_;
};

}  // namespace segmentation
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_SEGMENTER_H_
