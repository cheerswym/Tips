#include "onboard/perception/segmentation/proposed_cluster_dag.h"

#include <algorithm>
#include <deque>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "gflags/gflags.h"

namespace qcraft::segmentation {

namespace {
template <typename Container>
bool IsValueInContainer(const Container& vec,
                        const typename Container::value_type& val) {
  return (std::find(vec.begin(), vec.end(), val) != vec.end());
}
int CalculateTotalNumObstacles(const ProposedClusters& clusters) {
  return std::accumulate(clusters.begin(), clusters.end(), 0,
                         [](const int init, const ProposedCluster& cluster) {
                           return init + cluster.NumObstacles();
                         });
}
// NOTE(dong): just use maximal proposed cluster score as pack score.
void CalculateWeightedScoreForPack(ProposedClusterPack* pack) {
  for (const auto* child_cluster : pack->child_clusters) {
    pack->weighted_score =
        std::max(pack->weighted_score, child_cluster->proposer_score());
  }
}
}  // namespace

int CalculateTotalNumParentClusters(const ProposedClusterPacks& packs) {
  return std::accumulate(packs.begin(), packs.end(), 0,
                         [](const int init, const ProposedClusterPack& pack) {
                           return init + pack.parent_clusters.size();
                         });
}

int CalculateTotalNumChildClusters(const ProposedClusterPacks& packs) {
  return std::accumulate(packs.begin(), packs.end(), 0,
                         [](const int init, const ProposedClusterPack& pack) {
                           return init + pack.child_clusters.size();
                         });
}

ProposedClusterPtrs SelectClustersFromPacks(ProposedClusterPacks packs) {
  absl::flat_hash_set<ProposedClusterPtr> parent_cluster_set;
  for (const auto& pack : packs) {
    for (const auto* parent_cluster : pack.parent_clusters) {
      parent_cluster_set.emplace(parent_cluster);
    }
  }

  std::stable_sort(packs.begin(), packs.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.weighted_score > rhs.weighted_score;
                   });

  ProposedClusterPtrs selected_child_clusters;
  for (const auto& pack : packs) {
    bool parent_cluster_has_been_selected = false;
    for (const auto* parent_cluster : pack.parent_clusters) {
      if (!ContainsKey(parent_cluster_set, parent_cluster)) {
        parent_cluster_has_been_selected = true;
        break;
      }
    }

    if (parent_cluster_has_been_selected) continue;

    for (const auto* child_cluster : pack.child_clusters) {
      selected_child_clusters.emplace_back(child_cluster);
    }
    for (const auto* parent_cluster : pack.parent_clusters) {
      QCHECK_EQ(parent_cluster_set.erase(parent_cluster), 1);
    }
    if (parent_cluster_set.empty()) break;
  }

  QCHECK(parent_cluster_set.empty());

  return selected_child_clusters;
}

ProposedClusterPacks GroupPacks(const ProposedClustersBigraph& bigraph) {
  ProposedClusterPacks packs;
  absl::flat_hash_set<ProposedClusterPtr> selected_child_clusters;
  selected_child_clusters.reserve(bigraph.child_to_parents_map.size());
  for (const auto& [child_cluster, _] : bigraph.child_to_parents_map) {
    if (ContainsKey(selected_child_clusters, child_cluster)) {
      continue;
    }
    ProposedClusterPack pack;
    std::deque<ProposedClusterPtr> deque;
    deque.push_back(child_cluster);
    while (!deque.empty()) {
      const auto* child_cluster = deque.front();
      deque.pop_front();

      QCHECK(!IsValueInContainer(pack.child_clusters, child_cluster));
      pack.child_clusters.emplace_back(child_cluster);

      const auto& parent_clusters =
          FindOrDie(bigraph.child_to_parents_map, child_cluster);

      std::vector<ProposedClusterPtr> outlier_parent_clusters;
      for (const auto* parent_cluster : parent_clusters) {
        if (!IsValueInContainer(pack.parent_clusters, parent_cluster)) {
          pack.parent_clusters.emplace_back(parent_cluster);
          outlier_parent_clusters.emplace_back(parent_cluster);
        }
      }
      for (const auto* outlier_parent_cluster : outlier_parent_clusters) {
        const auto& child_clusters =
            FindOrDie(bigraph.parent_to_childs_map, outlier_parent_cluster);
        for (const auto* child_cluster : child_clusters) {
          if (!IsValueInContainer(pack.child_clusters, child_cluster) &&
              !IsValueInContainer(deque, child_cluster)) {
            deque.push_back(child_cluster);
          }
        }
      }
    }
    for (const auto* child_cluster : pack.child_clusters) {
      selected_child_clusters.emplace(child_cluster);
    }
    CalculateWeightedScoreForPack(&pack);
    packs.push_back(std::move(pack));
  }

  QCHECK_EQ(selected_child_clusters.size(),
            bigraph.child_to_parents_map.size());
  // check num obstacles
  int num_total_parent_obstacles = 0;
  int num_total_child_obstacles = 0;
  for (const auto& pack : packs) {
    for (const auto* parent_cluster : pack.parent_clusters) {
      num_total_parent_obstacles +=
          QCHECK_NOTNULL(parent_cluster)->NumObstacles();
    }
    for (const auto* child_cluster : pack.child_clusters) {
      num_total_child_obstacles +=
          QCHECK_NOTNULL(child_cluster)->NumObstacles();
    }
  }
  QCHECK_EQ(num_total_parent_obstacles, num_total_child_obstacles);

  return packs;
}

void ProposedClusterDag::InitObstacleToClusterMap() {
  for (const auto& [proposer_type, scene] : scene_map_) {
    auto& obstacle_to_cluster_map = obstacle_to_cluster_maps_[proposer_type];
    const auto& clusters = scene.GetResult();
    const size_t num_total_obstacles = CalculateTotalNumObstacles(clusters);
    obstacle_to_cluster_map.reserve(num_total_obstacles);
    for (const auto& cluster : clusters) {
      for (const auto* obstacle : cluster.obstacles()) {
        InsertOrDie(&obstacle_to_cluster_map, obstacle, &cluster);
      }
    }
  }
}

ProposedClustersBigraph ProposedClusterDag::ComputeBigraph(
    const ProposerType parent_type, const ProposerType child_type,
    const ProposedClusterPtrs& child_clusters) const {
  ProposedClustersBigraph bigraph{.parent_type = parent_type,
                                  .child_type = child_type};
  const auto& obstacle_to_parent_cluster_map =
      FindOrDie(obstacle_to_cluster_maps_, parent_type);
  auto& parent_to_childs_map = bigraph.parent_to_childs_map;
  parent_to_childs_map.reserve(scene_map_.GetResult(parent_type).size());
  auto& child_to_parents_map = bigraph.child_to_parents_map;
  child_to_parents_map.reserve(child_clusters.size());
  for (const auto* child_cluster : child_clusters) {
    for (const auto* obstacle : child_cluster->obstacles()) {
      const auto* parent_cluster =
          FindOrDie(obstacle_to_parent_cluster_map, obstacle);
      auto& parent_clusters = child_to_parents_map[child_cluster];
      if (!IsValueInContainer(parent_clusters, parent_cluster)) {
        parent_clusters.emplace_back(parent_cluster);
      }
      auto& child_clusters = parent_to_childs_map[parent_cluster];
      if (!IsValueInContainer(child_clusters, child_cluster)) {
        child_clusters.emplace_back(child_cluster);
      }
    }
  }
  QCHECK_EQ(parent_to_childs_map.size(),
            scene_map_.GetResult(parent_type).size());
  QCHECK_EQ(child_to_parents_map.size(), child_clusters.size());

  return bigraph;
}

ProposedClusterHistory ProposedClusterDag::GetProposedClusterHistory(
    const ProposedCluster& cluster) const {
  if (cluster.proposer_history().empty()) {
    return {};
  }
  ProposedClusterHistory history;
  std::deque<ProposerType> proposer_history_inv(
      cluster.proposer_history().crbegin(), cluster.proposer_history().crend());
  proposer_history_inv.emplace_back(PT_NONE);
  proposer_history_inv.pop_front();

  ProposedClusterPtrs pre_clusters{&cluster};
  for (const auto proposer_type : proposer_history_inv) {
    const auto& obstacle_to_cluster_map =
        FindOrDie(obstacle_to_cluster_maps_, proposer_type);
    ProposedClusterPtrs cur_clusters;
    for (const auto* pre_cluster : pre_clusters) {
      for (const auto* obstacle : pre_cluster->obstacles()) {
        const auto* cur_cluster = FindOrDie(obstacle_to_cluster_map, obstacle);
        if (!IsValueInContainer(cur_clusters, cur_cluster)) {
          cur_clusters.emplace_back(cur_cluster);
        }
      }
    }
    history.push_back({proposer_type, cur_clusters});
    pre_clusters = std::move(cur_clusters);
  }
  return history;
}

}  // namespace qcraft::segmentation
