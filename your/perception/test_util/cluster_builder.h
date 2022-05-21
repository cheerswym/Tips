#ifndef ONBOARD_PERCEPTION_TEST_UTIL_CLUSTER_BUILDER_H_
#define ONBOARD_PERCEPTION_TEST_UTIL_CLUSTER_BUILDER_H_

#include <memory>

#include "onboard/perception/cluster.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft {

// Creates a cluster.
class ClusterBuilder {
 public:
  ClusterBuilder();

  ClusterBuilder& set_type(MeasurementType type);

  ClusterBuilder& set_type_source(MeasurementTypeSource type_source);

  ClusterBuilder& set_obstacles(ObstaclePtrs&& obstacle_ptrs);

  // TODO(yu): Add more setters here if needed.

  Cluster Build();

 protected:
  std::unique_ptr<Cluster> cluster_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TEST_UTIL_CLUSTER_BUILDER_H_
