#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_ST_MAP_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_ST_MAP_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {

class ObjectStMap {
  // Define a s-t region for a specified predicted trajectory of an
  // object.
 public:
  explicit ObjectStMap(const ObjectConflictManager& conflict_mgr,
                       const ObjectProto& object_proto,
                       const planner::DiscretizedPath& path,
                       const planner::SpeedVector& ref_speed,
                       const PredictedTrajectory& predicted_trajectory,
                       const ConflictResolverParams* params,
                       ThreadPool* thread_pool);

  absl::Status MapStationaryObjects(absl::Span<const std::string> traj_ids);
  absl::Status MapMovingObjects(absl::Span<const std::string> traj_ids);
  void MapTLRedLanes(
      const absl::flat_hash_set<mapping::ElementId>& tl_red_lanes,
      absl::Span<const mapping::ElementId> possible_lane_path);

  // TODO(changqing): Change to StBoundaryWithDecision later.
  std::vector<double> QueryStoplines() const;
  std::vector<double> QueryStationaryObjects() const;
  std::vector<const planner::StBoundary*> QueryMovingObjects() const;
  inline const planner::SpeedVector& RefSpeedVector() const {
    return *ref_speed_;
  }
  inline const planner::DiscretizedPath& DiscretizedPath() const {
    return *path_;
  }
  inline double max_length() const { return max_length_; }
  inline ObjectType object_type() const { return av_proto_->type(); }
  inline std::string id() const {
    return absl::StrFormat("%s-idx%d", av_proto_->id(),
                           predicted_traj_->index());
  }

  bool HasConflict() const {
    return (!stationary_objects_.empty()) || (!stop_lines_.empty());
  }

  std::string Annotation() const;

  std::string DebugString() const;

 private:
  ThreadPool* thread_pool_ = nullptr;
  const ObjectConflictManager* conflict_mgr_;
  const PredictedTrajectory* predicted_traj_;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config_;
  const ConflictResolutionConfigProto::ObjectConflictResolverConfig*
      object_config_;

  // From object prediction results.
  const planner::DiscretizedPath* path_;
  const planner::SpeedVector* ref_speed_;
  const ObjectProto* av_proto_;

  std::vector<Box2d> av_boxes_on_path_;

  std::unique_ptr<SegmentMatcherKdtree> path_kd_tree_;
  double av_radius_;
  double max_plan_time_ = 10.0;
  double max_length_;

  // Boundaries mapped onto ObjectStMap.
  // TODO(changqing) : Change to StBoundaryWithDecisions when start mapping
  // moving objects. (maybe?)
  std::vector<planner::StBoundaryRef> stationary_objects_;
  std::vector<planner::StBoundaryRef> stop_lines_;
  std::vector<planner::StBoundaryRef> moving_objects_;
};

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_ST_MAP_H_
