#include "onboard/perception/test_util/cluster_builder.h"

#include <algorithm>
#include <random>
#include <utility>

#include "onboard/perception/laser_point.h"
#include "onboard/proto/lidar.pb.h"

namespace qcraft {

ClusterBuilder::ClusterBuilder() : cluster_(new Cluster()) {}

ClusterBuilder& ClusterBuilder::set_type(MeasurementType type) {
  cluster_->set_type(type);
  return *this;
}

ClusterBuilder& ClusterBuilder::set_type_source(
    MeasurementTypeSource type_source) {
  cluster_->set_type_source(type_source);
  return *this;
}

ClusterBuilder& ClusterBuilder::set_obstacles(ObstaclePtrs&& obstacle_ptrs) {
  *cluster_->mutable_obstacles() = obstacle_ptrs;
  return *this;
}

Cluster ClusterBuilder::Build() { return *cluster_; }

}  // namespace qcraft
