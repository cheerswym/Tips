#include "onboard/perception/segmentation/proposed_cluster_builder.h"

#include <memory>

#include "onboard/lite/logging.h"

namespace qcraft::segmentation {

ProposedClusterBuilder::ProposedClusterBuilder() {
  cluster_.reset(new ProposedCluster());
}

ProposedClusterBuilder& ProposedClusterBuilder::set_is_proposed(
    const bool flag) {
  cluster_ptr()->set_is_proposed(flag);
  return *this;
}

ProposedClusterBuilder& ProposedClusterBuilder::AppendProposerHistory(
    const ProposerType proposer_type) {
  QCHECK_NE(proposer_type, PT_NONE);
  cluster_ptr()->AppendProposerHistory(proposer_type);
  return *this;
}

ProposedClusterBuilder& ProposedClusterBuilder::AddProposerScore(
    const double score) {
  QCHECK_NE(score, 0.0);
  cluster_ptr()->AddProposerScore(score);
  return *this;
}

ProposedCluster ProposedClusterBuilder::Build() { return *cluster_ptr(); }

ProposedCluster* ProposedClusterBuilder::cluster_ptr() {
  return QCHECK_NOTNULL(static_cast<ProposedCluster*>(cluster_.get()));
}

}  // namespace qcraft::segmentation
