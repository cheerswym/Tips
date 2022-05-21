#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_H_

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "onboard/perception/cluster.h"
#include "onboard/perception/segmentation/types.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::segmentation {

class ProposedCluster;
using ProposedClusters = std::vector<ProposedCluster>;
using ProposedClusterRef = std::shared_ptr<ProposedCluster>;
using ProposedClusterRefs = std::vector<std::shared_ptr<ProposedCluster>>;
using ProposedClusterPtr = const ProposedCluster*;
using ProposedClusterPtrs = std::vector<ProposedClusterPtr>;

class ProposedCluster final : public Cluster {
  friend class ProposerTreeNode;
  friend class ProposedClusterBuilder;
  friend ProposedClusters InitProposedClusters(ClusterVector);

 public:
  // NOTE(dong): Each proposed cluster needs to inherit from one or multi
  // parents for initializing its default state.
  static ProposedCluster InheritFrom(const ProposedCluster& cluster) {
    ProposedCluster new_cluster;
    new_cluster.property_state_ = cluster.property_state_;
    new_cluster.proposer_history_ = cluster.proposer_history_;
    new_cluster.proposer_score_ = cluster.proposer_score_;
    return new_cluster;
  }
  // TODO(dong): Need a more appropriate method to merge histories of multiple
  // proposed clusters.
  static ProposedCluster InheritFrom(const ProposedClusters& clusters) {
    QCHECK(!clusters.empty());
    ProposedCluster new_cluster;
    for (const auto& cluster : clusters) {
      QCHECK(!cluster.HasProperty(PP_NOISE));
      if (cluster.proposer_score_ > new_cluster.proposer_score_) {
        new_cluster.property_state_ = cluster.property_state_;
        new_cluster.proposer_history_ = cluster.proposer_history_;
        new_cluster.proposer_score_ = cluster.proposer_score_;
      }
    }
    return new_cluster;
  }
  // NOTE(dong): Construct base cluster from obstacles. This should be called
  // after inherit.
  ProposedCluster& ConstructBase(ObstaclePtrs obstacles) & {
    MutableBase() = Cluster(std::move(obstacles));
    return *this;
  }
  ProposedCluster&& ConstructBase(ObstaclePtrs obstacles) && {
    MutableBase() = Cluster(std::move(obstacles));
    return std::move(*this);
  }
  // NOTE(dong): Construct base cluster. This should be called after inherit.
  ProposedCluster& ConstructBase(Cluster cluster) & {
    MutableBase() = std::move(cluster);
    return *this;
  }
  ProposedCluster&& ConstructBase(Cluster cluster) && {
    MutableBase() = std::move(cluster);
    return std::move(*this);
  }

  const Cluster& Base() const { return *this; }
  Cluster& MutableBase() { return *this; }

  ProposedCluster Clone() const { return *this; }

  void set_is_proposed(const bool flag) { is_proposed_ = flag; }
  bool is_proposed() const { return is_proposed_; }
  // NOTE(dong): We regard a cluster proposed by PT_NONE, only if it's not
  // proposed by any other proposer. If it has been proposed by one or more
  // proposers, IsProposedBy(PT_NONE) will return false.
  bool IsProposedBy(const ProposerType type) const {
    return proposer_history_.empty() ? type == PT_NONE
                                     : ContainsProposerType(type);
  }

  ProposerType last_proposer() const {
    return proposer_history_.empty() ? PT_NONE : proposer_history_.back();
  }
  const std::deque<ProposerType>& proposer_history() const {
    return proposer_history_;
  }
  std::deque<ProposerType>* mutable_proposer_history() {
    return &proposer_history_;
  }

  void set_property(const ProposedProperty property) {
    property_state_ |= 1 << static_cast<int>(property);
  }
  void reset_property(const ProposedProperty property) {
    property_state_ &= ~(1 << static_cast<int>(property));
  }

  uint64_t property_state() const { return property_state_; }
  void clear_property_state() { property_state_ = 0; }
  void set_property_state(const uint64_t property_state) {
    property_state_ = property_state;
  }

  // NOTE(dong): Proposed Cluster can hold multiple properties.
  bool HasProperty(const ProposedProperty property) const {
    return (property_state_ >> static_cast<int>(property)) & 1;
  }

  std::vector<ProposedProperty> GetAllProperties() const {
    if (property_state_ == 0) return {};

    std::vector<ProposedProperty> properties;
    for (int i = 0; i < std::numeric_limits<uint64_t>::digits; ++i) {
      if (property_state_ >> i & 1) {
        properties.emplace_back(static_cast<ProposedProperty>(i));
      }
    }
    return properties;
  }

  double proposer_score() const { return proposer_score_; }
  void set_proposer_score(const double score) { proposer_score_ = score; }

 private:
  // NOTE(dong): Because proposed clusters hold historical information, such as
  // property_state_, proposer_history_ and proposer_score_, we do not allow
  // user to default construct a proposed cluster.
  ProposedCluster() = default;
  // NOTE(dong): This constructor is only called by friend function
  // InitProposedClusters for initializing root proposed clusters.
  explicit ProposedCluster(Cluster raw_cluster)
      : Cluster(std::move(raw_cluster)) {}
  //
  void AppendProposerHistory(const ProposerType type) {
    QCHECK(type != PT_NONE && !ContainsProposerType(type));
    proposer_history_.emplace_back(type);
  }
  bool ContainsProposerType(const ProposerType type) const {
    return std::find(proposer_history_.begin(), proposer_history_.end(),
                     type) != proposer_history_.end();
  }

  void AddProposerScore(const double score) { proposer_score_ += score; }

 private:
  // Whether the cluster is proposed by current proposer. Need to explicitly
  // marked in each proposer.
  bool is_proposed_ = false;
  std::deque<ProposerType> proposer_history_;
  uint64_t property_state_ = 0;
  double proposer_score_ = 0.0f;
};

ProposedClusters InitProposedClusters(ClusterVector);

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_H_
