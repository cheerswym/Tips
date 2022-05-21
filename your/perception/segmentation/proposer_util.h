#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_UTIL_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_UTIL_H_

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/segmentation/proposed_cluster.h"

DECLARE_bool(segmentation_profile);

namespace qcraft::segmentation {

#define CONCAT_TOKEN_(foo, bar) CONCAT_TOKEN_IMPL_(foo, bar)
#define CONCAT_TOKEN_IMPL_(foo, bar) foo##bar

#define FUNC_TIME_PRINTER(name)                                      \
  static_assert(name "", "Must pass a string literal");              \
  qcraft::segmentation::FuncTimePrinter CONCAT_TOKEN_(time_printer_, \
                                                      __LINE__)(name)

#define FUNC_TIME_PRINTER_ARG1(name, key, val)                          \
  static_assert(name "", "Must pass a string literal");                 \
  static_assert(key "", "Must pass a string literal for argument key"); \
  qcraft::segmentation::FuncTimePrinter CONCAT_TOKEN_(                  \
      time_printer_, __LINE__)(name, key, std::to_string(val))

class FuncTimePrinter {
 public:
  explicit FuncTimePrinter(std::string name, std::string key = "",
                           std::string val = "")
      : init_time_(absl::Now()),
        name_(std::move(name)),
        key_val_(std::make_pair(std::move(key), std::move(val))) {}

  ~FuncTimePrinter() {
    if (UNLIKELY(FLAGS_segmentation_profile)) {
      const double time_diff =
          absl::ToInt64Microseconds(absl::Now() - init_time_) * 1e-3;
      if (key_val_.first.empty()) {
        QLOG(INFO) << absl::StrFormat("[%s] takes %.3fms.", name_, time_diff);
      } else {
        QLOG(INFO) << absl::StrFormat("[%s] takes %.3fms. %s: %s", name_,
                                      time_diff, key_val_.first,
                                      key_val_.second);
      }
    }
  }

 private:
  // Only use absl::Now() for latency calculation and logging, use
  // Clock::Now() for other purposes.
  absl::Time init_time_;
  std::string name_;
  std::pair<std::string, std::string> key_val_;
};
// Accumulate num obstacles from clusters.
int CalculateTotalNumObstacles(const ProposedClusters& clusters);
// Accumulate num obstacles from cluster ptrs.
int CalculateTotalNumObstacles(const ProposedClusterPtrs& clusters);
// Returns two clusters by cuts through along the main axis.
ProposedClusters DivideClusterIntoTwoHalves(const ProposedCluster& cluster);
// Segment cluster until all its children satisfy the condition.
ProposedClusters SegmentAndProposeClusterIf(
    const ProposedCluster& cluster,
    const std::function<bool(const ProposedCluster&)>& condition);
ProposedClusters SegmentAndProposeClusterWithConnectedComponents(
    const ProposedCluster& cluster,
    const obstacle_util::NeighborRange& neighbor_range,
    const obstacle_util::ConnectedCondition& condition = {});
// NOTE(dong): Noise obstacle indices should be sorted in ascending order.
// If no cluster remained, std::nullopt will be returned.
std::optional<ProposedCluster> TrimNoiseObstaclesAndGetRemainingCluster(
    const ProposedCluster& cluster,
    const std::vector<size_t>& noise_obstacle_indices,
    ProposedClusters* proposed_clusters);
// NOTE(dong): Trim noise obstacles as noise cluster and return remaining
// cluster depending on noise obstacle set. If no cluster remained, std::nullopt
// will be returned.
std::optional<ProposedCluster> TrimNoiseObstaclesAndGetRemainingCluster(
    const ProposedCluster& cluster,
    const absl::flat_hash_set<ObstaclePtr>& noise_obstacle_set,
    ProposedClusters* proposed_clusters);
// Compute mean heading for multiple cluster bounding boxes.
double ComputeClusterBoundingBoxMeanHeading(const std::vector<double>& angles);

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_UTIL_H_
