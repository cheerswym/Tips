#ifndef ONBOARD_PERCEPTION_SEGMENTATION_SCENE_MAP_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_SCENE_MAP_H_

#include <map>
#include <memory>
#include <utility>
#include <variant>

#include "onboard/lite/logging.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/types.h"
#include "onboard/utils/map_util.h"

namespace qcraft::segmentation {

// NOTO(dong): A scene is defined as all proposed results for a certain
// proposer. After traversing the entire proposer tree, we get a scene map
// collected from each proposer.
class SceneMap {
  class Scene {
    friend class SceneMap;

   public:
    const ProposedClusters& GetResult() const { return result_; }

   private:
    explicit Scene(ProposedClusters proposed_clusters) {
      result_ = std::move(proposed_clusters);
    }

   private:
    ProposedClusters result_;
  };

 public:
  void emplace(const ProposerType type, ProposedClusters proposed_clusters) {
    QCHECK(std::get<bool>(
        scene_map_.try_emplace(type, Scene(std::move(proposed_clusters)))));
  }

  bool contains(const ProposerType type) {
    return ContainsKey(scene_map_, type);
  }

  auto begin() const { return scene_map_.begin(); }
  auto end() const { return scene_map_.end(); }

  const ProposedClusters& GetResult(const ProposerType type) const {
    return FindOrDie(scene_map_, type).GetResult();
  }

  const ProposedClusters& GetRootClusters() const { return GetResult(PT_NONE); }

 private:
  std::map<ProposerType, Scene> scene_map_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_SCENE_MAP_H_
