#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_BUILDER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_BUILDER_H_

#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/test_util/cluster_builder.h"

namespace qcraft::segmentation {

class ProposedClusterBuilder : public ClusterBuilder {
 public:
  ProposedClusterBuilder();

  ProposedClusterBuilder& set_is_proposed(bool flag);

  ProposedClusterBuilder& AppendProposerHistory(ProposerType proposer_type);

  ProposedClusterBuilder& AddProposerScore(double score);

  ProposedCluster Build();

 private:
  ProposedCluster* cluster_ptr();
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_BUILDER_H_
