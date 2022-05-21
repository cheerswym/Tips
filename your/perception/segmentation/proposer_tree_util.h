#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_UTIL_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_UTIL_H_

#include <string>

#include "onboard/async/thread_pool.h"
#include "onboard/perception/segmentation/proposer_tree.h"

constexpr char kDefaultProposerTreeConfPath[] =
    "onboard/perception/segmentation/conf/proposer_tree.pb.txt";
namespace qcraft::segmentation {

ProposerTree BuildProposerTree(
    ThreadPool* thread_pool,
    const std::string& conf_path = kDefaultProposerTreeConfPath);

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_UTIL_H_
