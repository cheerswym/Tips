#include "onboard/planner/initializer/motion_graph_cache.h"

namespace qcraft::planner {

MotionGraphCache::MotionGraphCache(const GeometryGraph *geom_graph)
    : geom_graph_(geom_graph) {}

void MotionGraphCache::BatchGetOrFail(
    const std::vector<MotionEdgeKey> &keys,
    std::vector<DpMotionInfo> *ptr_dp_motion_infos,
    std::vector<int> *ptr_failed_idx) const {
  auto &dp_motion_infos = *ptr_dp_motion_infos;
  auto &failed_idx = *ptr_failed_idx;
  for (int i = 0; i < keys.size(); i++) {
    const auto &key = keys[i];
    const auto ptr_edge_info = FindOrNull(cache_, key);
    if (ptr_edge_info != nullptr) {
      dp_motion_infos[i].costs.resize(ptr_edge_info->costs.size());
      dp_motion_infos[i].costs = ptr_edge_info->costs;
      dp_motion_infos[i].motion_form = ptr_edge_info->ptr_motion_form.get();
    } else {
      failed_idx.push_back(i);
    }
  }
}

void MotionGraphCache::Insert(const MotionEdgeKey &key,
                              const std::vector<double> &costs,
                              std::unique_ptr<MotionForm> ptr_motion_form) {
  if (!cache_.contains(key)) {
    MotionEdgeCache edge_cache = {.ptr_motion_form = std::move(ptr_motion_form),
                                  .costs = std::move(costs)};
    auto [it, success] =
        cache_.emplace(std::make_pair(std::move(key), std::move(edge_cache)));
    QCHECK(success);
  }
}

void MotionGraphCache::BatchInsert(std::vector<NewCacheInfo> new_motion_forms) {
  for (auto it = new_motion_forms.begin(); it != new_motion_forms.end(); ++it) {
    if (!cache_.contains(it->key)) {
      auto [iter, success] = cache_.emplace(
          std::make_pair(std::move(it->key), std::move(it->cache)));
      QCHECK(success);
    }
  }
}

absl::StatusOr<MotionForm *> MotionGraphCache::GetMotionForm(
    const MotionEdgeKey &key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->ptr_motion_form.get();
  } else {
    return absl::NotFoundError("Queried motion form not in cache.");
  }
}

absl::StatusOr<std::vector<double>> MotionGraphCache::GetCosts(
    const MotionEdgeKey &key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->costs;
  } else {
    return absl::NotFoundError("Queried motion costs not in cache.");
  }
}

}  // namespace qcraft::planner
