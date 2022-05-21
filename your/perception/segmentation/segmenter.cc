#include "onboard/perception/segmentation/segmenter.h"

#include <algorithm>
#include <functional>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/segmentation/proposed_cluster_dag.h"
#include "onboard/perception/segmentation/proposer_tree_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/perception/segmentation/pyramid.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(enable_segmentation_debug, false,
            "Check propose & select results.");

namespace qcraft::segmentation {

namespace {

void RenderTwoObstacleSet(absl::flat_hash_set<ObstaclePtr> lhs,
                          absl::flat_hash_set<ObstaclePtr> rhs) {
  vis::Canvas& canvas = vantage_client_man::GetCanvas("perception/segmenter");
  std::vector<ObstaclePtr> shared_obstacles;
  shared_obstacles.reserve(lhs.size());
  for (auto it = lhs.begin(); it != lhs.end();) {
    if (ContainsKey(rhs, *it)) {
      shared_obstacles.emplace_back(*it);
      rhs.erase(*it);
      lhs.erase(it++);
    } else {
      ++it;
    }
  }
  for (const auto* obstacle : lhs) {
    canvas.DrawBox(Vec3d(obstacle->coord(), obstacle->ground_z), 0.0,
                   {Obstacle::kDiameter, Obstacle::kDiameter},
                   vis::Color::kBlack.WithAlpha(0.0),
                   vis::Color::kRed.WithAlpha(0.8));  // Red
  }
  for (const auto* obstacle : lhs) {
    canvas.DrawBox(Vec3d(obstacle->coord(), obstacle->ground_z), 0.0,
                   {Obstacle::kDiameter, Obstacle::kDiameter},
                   vis::Color::kBlack.WithAlpha(0.0),
                   vis::Color::kGreen.WithAlpha(0.8));  // Green
  }
  for (const auto* obstacle : shared_obstacles) {
    canvas.DrawBox(Vec3d(obstacle->coord(), obstacle->ground_z), 0.0,
                   {Obstacle::kDiameter, Obstacle::kDiameter},
                   vis::Color::kBlack.WithAlpha(0.0),
                   vis::Color::kBlue.WithAlpha(0.8));  // Blue
  }
  vantage_client_man::FlushAll();
}

bool CompareAndRenderObstacles(const ProposedClusters& lhs,
                               const ProposedClusters& rhs,
                               const std::string& info = "") {
  absl::flat_hash_set<ObstaclePtr> lhs_set, rhs_set;
  for (const auto& cluster : lhs) {
    for (const auto* obstacle : cluster.obstacles()) {
      QCHECK(lhs_set.insert(obstacle).second) << info;
    }
  }
  for (const auto& cluster : rhs) {
    for (const auto* obstacle : cluster.obstacles()) {
      QCHECK(rhs_set.insert(obstacle).second) << info;
    }
  }
  if (lhs_set.size() != rhs_set.size()) {
    RenderTwoObstacleSet(lhs_set, rhs_set);
    QCHECK_EQ(lhs_set.size(), rhs_set.size()) << info;
  }

  return true;
}

bool CompareAndRenderObstacles(const ProposedClusterPtrs& lhs,
                               const ProposedClusterPtrs& rhs,
                               const std::string& info = "") {
  absl::flat_hash_set<ObstaclePtr> lhs_set, rhs_set;
  for (const auto* cluster : lhs) {
    for (const auto* obstacle : cluster->obstacles()) {
      QCHECK(lhs_set.insert(obstacle).second) << info;
    }
  }
  for (const auto* cluster : rhs) {
    for (const auto* obstacle : cluster->obstacles()) {
      QCHECK(rhs_set.insert(obstacle).second) << info;
    }
  }
  if (lhs_set.size() != rhs_set.size()) {
    RenderTwoObstacleSet(lhs_set, rhs_set);
    QCHECK_EQ(lhs_set.size(), rhs_set.size()) << info;
  }

  return true;
}

bool CheckIfTwoResultsAreTheSame(ProposedClusters lhs, ProposedClusters rhs) {
  QCHECK_EQ(lhs.size(), rhs.size());

  CompareAndRenderObstacles(lhs, rhs);

  auto sort_by_pos = [](const auto& lhs, const auto& rhs) {
    return lhs.obstacles()[0]->points[0].x != rhs.obstacles()[0]->points[0].x
               ? lhs.obstacles()[0]->points[0].x <
                     rhs.obstacles()[0]->points[0].x
               : lhs.obstacles()[0]->points[0].y <
                     rhs.obstacles()[0]->points[0].y;
  };

  std::sort(lhs.begin(), lhs.end(), sort_by_pos);
  std::sort(rhs.begin(), rhs.end(), sort_by_pos);

  for (size_t i = 0; i < lhs.size(); ++i) {
    QCHECK_EQ(lhs[i].NumPoints(), rhs[i].NumPoints());
    QCHECK_EQ(lhs[i].NumObstacles(), rhs[i].NumObstacles());
    QCHECK_EQ(lhs[i].score(), rhs[i].score());
    QCHECK_EQ(lhs[i].type(), rhs[i].type());
    QCHECK_EQ(lhs[i].property_state(), rhs[i].property_state());
    QCHECK_EQ(static_cast<bool>(lhs[i].bounding_box()),
              static_cast<bool>(rhs[i].bounding_box()));
    QCHECK_EQ(lhs[i].type_source(), rhs[i].type_source());
  }

  return true;
}

absl::Status CheckPyramidAndReportQEvent(const Pyramid& pyramid,
                                         const SceneMap& scene_map) {
  bool success = true;
  const int total_num_obstacles =
      CalculateTotalNumObstacles(scene_map.GetRootClusters());
  for (const auto& [depth, layer] : pyramid.cluster_pyramid()) {
    for (const auto& [type, clusters] : layer) {
      const auto type_name = ProposerType_Name(type);
      QCHECK(clusters) << absl::StrFormat(": depth %d, %s", depth, type_name);
      const int num_selected_obstacles = CalculateTotalNumObstacles(*clusters);
      if (total_num_obstacles != num_selected_obstacles) {
        QEVENT("dongchen", "wrong_num_selected_obstacles_in_the_pyramid",
               [=](QEvent* qevent) {
                 qevent->AddField("Type", type_name)
                     .AddField("total_num_obstacles", total_num_obstacles)
                     .AddField("num_selected_obstacles",
                               num_selected_obstacles);
               });
        success = false;
      }
    }
  }
  return success ? absl::OkStatus()
                 : absl::InternalError("Check pyramid failed.");
}

void CheckSceneMapResultsForDebug(const SceneMap& scene_map) {
  const auto& root_clusters = scene_map.GetRootClusters();
  for (const auto& [proposer_type, scene] : scene_map) {
    CompareAndRenderObstacles(root_clusters, scene.GetResult(),
                              ProposerType_Name(proposer_type));
  }
}

void CheckPyramidResultsForDebug(const Pyramid& pyramid,
                                 const ProposedClusters& root_clusters) {
  ProposedClusterPtrs root_cluster_ptrs;
  root_cluster_ptrs.reserve(root_clusters.size());
  for (const auto& root_cluster : root_clusters) {
    root_cluster_ptrs.emplace_back(&root_cluster);
  }
  for (const auto& [depth, layer] : pyramid.cluster_pyramid()) {
    for (const auto& [type, cluster_ptrs] : layer) {
      QCHECK(cluster_ptrs);
      CompareAndRenderObstacles(
          root_cluster_ptrs, *cluster_ptrs,
          absl::StrFormat(": depth %d, %s", depth, ProposerType_Name(type)));
    }
  }
}

ProposerEnvInfo InitProposerEnvInfo(
    const VehiclePose& pose, const ObstacleManager& obstacle_manager,
    const SemanticMapManager& semantic_map_manager,
    const LocalImagery& local_imagery,
    const CoordinateConverter& coordinate_converter,
    const RunParamsProtoV2& run_params, const Context& context) {
  return ProposerEnvInfo(pose, obstacle_manager, semantic_map_manager,
                         local_imagery, coordinate_converter, run_params,
                         context);
}

Box2dProto ToBox2dProto(const Box2d& box2d) {
  Box2dProto proto;
  proto.set_x(box2d.center_x());
  proto.set_y(box2d.center_y());
  proto.set_heading(box2d.heading());
  proto.set_length(box2d.length());
  proto.set_width(box2d.width());
  return proto;
}

SegmentationStageObjectProto ToSegmentationStageObjectProto(
    const ProposedCluster& cluster) {
  SegmentationStageObjectProto stage_object_proto;
  stage_object_proto.set_type(cluster.type());
  stage_object_proto.set_type_source(cluster.type_source());
  const auto& contour = cluster_util::ComputeContour(cluster);
  for (const auto& p : contour.points()) {
    auto* contour_point = stage_object_proto.add_contour();
    contour_point->set_x(p.x());
    contour_point->set_y(p.y());
  }
  const auto& centroid = contour.centroid();
  stage_object_proto.mutable_centroid()->set_x(centroid.x());
  stage_object_proto.mutable_centroid()->set_y(centroid.y());
  if (cluster.bounding_box()) {
    *stage_object_proto.mutable_bounding_box() =
        ToBox2dProto(*cluster.bounding_box());
  }
  stage_object_proto.set_last_proposer(cluster.last_proposer());

  return stage_object_proto;
}

SegmentationStageProto ToSegmentationStageProto(
    const ProposedClusterStage& stage) {
  SegmentationStageProto stage_proto;
  stage_proto.set_type(stage.type);
  for (const auto* cluster : stage.clusters) {
    *stage_proto.add_objects() = ToSegmentationStageObjectProto(*cluster);
  }

  return stage_proto;
}

SegmentationObjectProto ToSegmentationObjectProto(
    const ProposedCluster& cluster, const ProposedClusterDag& dag,
    const uint32_t id) {
  SegmentationObjectProto object_proto;
  object_proto.set_id(id);
  object_proto.set_timestamp(cluster.timestamp());
  object_proto.set_type(cluster.type());
  object_proto.set_type_source(cluster.type_source());
  const auto& contour = cluster_util::ComputeContour(cluster);
  for (const auto& p : contour.points()) {
    auto* contour_point = object_proto.add_contour();
    contour_point->set_x(p.x());
    contour_point->set_y(p.y());
  }
  const auto& centroid = contour.centroid();
  object_proto.mutable_centroid()->set_x(centroid.x());
  object_proto.mutable_centroid()->set_y(centroid.y());
  if (cluster.bounding_box()) {
    *object_proto.mutable_bounding_box() =
        ToBox2dProto(*cluster.bounding_box());
  }
  const auto& properties = cluster.GetAllProperties();
  for (const auto property : properties) {
    object_proto.add_proposed_properties(property);
  }
  object_proto.set_proposer_score(cluster.proposer_score());
  object_proto.set_last_proposer(cluster.last_proposer());

  const auto& proposed_cluster_history = dag.GetProposedClusterHistory(cluster);
  for (const auto& stage : proposed_cluster_history) {
    *object_proto.add_history() = ToSegmentationStageProto(stage);
  }

  return object_proto;
}

void UpdateTreeNodesProto(const ProposerTree& proposer_tree,
                          SegmentationObjectsProto* objects_proto) {
  auto func = [&](ProposerTreeNodePtr node) {
    auto* node_proto = objects_proto->add_tree_nodes();
    node_proto->set_type(node->type());
    node_proto->set_score(node->score());
    node_proto->set_depth(node->depth());
    node_proto->set_parent_type(node->parent() ? node->parent()->type()
                                               : PT_NONE);
  };
  proposer_tree.Traverse(std::move(func));
}

}  // namespace

Segmenter::Segmenter(ThreadPool* thread_pool)
    : thread_pool_(thread_pool),
      proposer_tree_(BuildProposerTree(thread_pool_)),
      segmentation_objects_proto_(
          std::make_unique<SegmentationObjectsProto>()) {}

SegmentedClusters Segmenter::Segment(
    const VehiclePose& pose, const ObstacleManager& obstacle_manager,
    const SemanticMapManager& semantic_map_manager,
    const LocalImagery& local_imagery,
    const CoordinateConverter& coordinate_converter,
    const RunParamsProtoV2& run_params, const Context& context,
    ClusterVector raw_clusters) {
  SCOPED_QTRACE_ARG1("Segmenter::Segment", "num_clusters", raw_clusters.size());
  FUNC_TIME_PRINTER_ARG1("Segmenter::Segment", "num_clusters",
                         raw_clusters.size());

  ScheduleFuture(ThreadPool::DisposalPool(),
                 [x = std::move(segmentation_objects_proto_),
                  y = std::move(noise_clusters_)] {});
  segmentation_objects_proto_ = std::make_unique<SegmentationObjectsProto>();
  noise_clusters_.clear();

  const auto env_info = InitProposerEnvInfo(
      pose, obstacle_manager, semantic_map_manager, local_imagery,
      coordinate_converter, run_params, context);
  auto scene_map =
      Propose(env_info, InitProposedClusters(std::move(raw_clusters)));
  auto selected_clusters = Select(scene_map);

  // scene_map is huge, clear it asynchronously to prevent possible blocking.
  ScheduleFuture(ThreadPool::DisposalPool(),
                 [scene_map = std::move(scene_map)] {});

  return selected_clusters;
}

SceneMap Segmenter::Propose(const ProposerEnvInfo& env_info,
                            ProposedClusters clusters) {
  SCOPED_QTRACE("Segmenter::Propose");
  FUNC_TIME_PRINTER("Segmenter::Propose");
  return proposer_tree_.TraverseAndPropose(env_info, std::move(clusters));
}

SegmentedClusters Segmenter::Select(const SceneMap& scene_map) {
  SCOPED_QTRACE("Segmenter::Select");
  FUNC_TIME_PRINTER("Segmenter::Select");
  // Init dag & Build cluster pyramid
  const ProposedClusterDag dag(scene_map);
  Pyramid pyramid(proposer_tree_);
  pyramid.BuildClusterPyramid(scene_map, dag);
  QLOG_IF_NOT_OK(WARNING, CheckPyramidAndReportQEvent(pyramid, scene_map));

  // Selected clusters at the top of the cluster pyramid should be the final
  // selection result.
  const auto& selected_proposed_clusters = pyramid.GetClusters(0, PT_NONE);
  SegmentedClusters segmented_clusters;
  segmented_clusters.reserve(selected_proposed_clusters.size());

  const int num_clusters = selected_proposed_clusters.size();
  std::vector<uint32_t> ids(num_clusters);
  auto& segmentation_objects =
      *segmentation_objects_proto_->mutable_segmentation_objects();
  segmentation_objects.Reserve(num_clusters);
  for (int i = 0; i < num_clusters; ++i) {
    segmentation_objects.Add();
  }
  ParallelFor(0, num_clusters, thread_pool_, [&](int i) {
    ids[i] = latest_cluster_id_ + i;
    segmentation_objects[i] =
        ToSegmentationObjectProto(*selected_proposed_clusters[i], dag, ids[i]);
  });
  latest_cluster_id_ += num_clusters;

  for (int i = 0; i < num_clusters; ++i) {
    const auto* proposed_cluster = selected_proposed_clusters[i];
    const uint32_t id = ids[i];
    if (!proposed_cluster->HasProperty(PP_NOISE)) {
      segmented_clusters.emplace_back(id, *proposed_cluster);
    } else {
      noise_clusters_[proposed_cluster->last_proposer()].emplace_back(
          id, *proposed_cluster);
    }
  }

  // Add tree nodes to proto
  UpdateTreeNodesProto(proposer_tree_, segmentation_objects_proto_.get());

  // Debug check
  if (UNLIKELY(FLAGS_enable_segmentation_debug)) {
    CheckSceneMapResultsForDebug(scene_map);
    CheckPyramidResultsForDebug(pyramid, scene_map.GetRootClusters());
  }

  return segmented_clusters;
}

}  // namespace qcraft::segmentation
