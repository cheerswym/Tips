#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_H_

#include <memory>

#include "absl/synchronization/mutex.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/perception/segmentation/types.h"

namespace qcraft::segmentation {

class Proposer {
 public:
  Proposer(const ProposerType type, ThreadPool* thread_pool)
      : type_(type), thread_pool_(thread_pool) {
    QCHECK_NE(type_, PT_NONE);
  }
  virtual ~Proposer() = default;

  ProposedClusters Propose(const ProposerEnvInfo& env_info,
                           const ProposedClusters& clusters) {
    SCOPED_QTRACE_ARG2("Proposer::Propose", "type", ProposerType_Name(type_),
                       "num_clusters", clusters.size());
    auto proposed_cluster = ProposeInternal(env_info, clusters);
    // Check if there are some duplicated or trimed obstacles after proposer.
    const int num_input_obstacles = CalculateTotalNumObstacles(clusters);
    const int num_proposed_obstacles =
        CalculateTotalNumObstacles(proposed_cluster);
    if (num_input_obstacles != num_proposed_obstacles) {
      QEVENT("dongchen", "obstacles_are_duplicated_or_trimed_after_proposer",
             [=](QEvent* qevent) {
               qevent->AddField("Type", ProposerType_Name(type_))
                   .AddField("num_input_obstacles", num_input_obstacles)
                   .AddField("num_proposed_obstacles", num_proposed_obstacles);
             });
    }

    return proposed_cluster;
  }

  ProposerType type() const { return type_; }

 protected:
  virtual ProposedClusters ProposeInternal(
      const ProposerEnvInfo& env_info, const ProposedClusters& clusters) = 0;

  ProposerType type_ = PT_NONE;
  ThreadPool* thread_pool_ = nullptr;
};

using ProposerRef = std::unique_ptr<Proposer>;

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_H_
