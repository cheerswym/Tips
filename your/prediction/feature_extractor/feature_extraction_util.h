#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"
namespace qcraft {
namespace prediction {
std::vector<PredictedTrajectoryPointProto> AlignPredictedTrajectoryPoints(
    const std::vector<PredictedTrajectoryPointProto> &prev_pts,
    double prev_time, double cur_time, double new_horizon, double dt);

std::vector<PredictedTrajectoryPointProto>
ConvertAVTrajectoryPointsToPredictedTrajectoryPoints(
    absl::Span<const ApolloTrajectoryPointProto> av_pts);

std::vector<PredictedTrajectoryPointProto> SmoothPredictedTrajectory(
    absl::Span<const PredictedTrajectoryPointProto> raw_traj);

std::vector<const ObjectHistory *> GetObjectsInBox2d(
    absl::Span<const ObjectHistory *const>, const Box2d &region_box);

// Get objects in box2d without ego object.
ResampledObjectsHistory GetResampledObjectsInBox2d(
    const ResampledObjectsHistory &objects_history, const Box2d &region_box,
    const std::string &ego_id, int max_object_num);

std::vector<const mapping::LaneInfo *> GetLanesInBox2d(
    const SemanticMapManager &semantic_map_manager, const Box2d &region_box,
    const int max_lanes_num);
Box2d GetRegionBox(const Vec2d &pos, const double heading,
                   double detection_region_front,
                   double detection_region_behind, double detection_half_width);
void SetHistoryStatesAndCurPoses(int coords, int history_num, double time_step,
                                 const Vec2d &ref_pos, double rot_rad,
                                 const ResampledObjectsHistory &objects_history,
                                 std::vector<float> *traj_ptrs,
                                 std::vector<float> *speed_ptrs,
                                 std::vector<float> *heading_ptrs,
                                 std::vector<float> *cur_pos_ptrs);

inline std::vector<float> GetEgoPos(const Vec2d &pos, int coord_num) {
  std::vector<float> ego_pos(coord_num);
  ego_pos[0] = pos.x();
  ego_pos[1] = pos.y();
  return ego_pos;
}

inline std::vector<float> GetRotMatrix(double rot_rad, int coord_num) {
  std::vector<float> rot_mat(coord_num * coord_num);
  const double sin_theta = std::sin(rot_rad);
  const double cos_theta = std::cos(rot_rad);
  rot_mat[0] = cos_theta;
  rot_mat[1] = -sin_theta;
  rot_mat[2] = sin_theta;
  rot_mat[3] = cos_theta;
  return rot_mat;
}

template <typename T>
std::vector<T> VotingTypes(const ResampledObjectsHistory &objects_history) {
  const int objects_num = objects_history.size();
  std::vector<T> types(objects_num);
  for (int object_idx = 0; object_idx < objects_num; ++object_idx) {
    const auto &object_history = objects_history[object_idx];
    std::map<ObjectType, int> type_count;
    for (const auto &object : object_history) {
      const auto type = object.type();
      // When there is no key in map, [] will implement a default construct.
      type_count[type] += 1;
    }
    types[object_idx] = static_cast<T>(
        std::max_element(type_count.begin(), type_count.end(),
                         [](const std::pair<ObjectType, int> &p1,
                            const std::pair<ObjectType, int> &p2) {
                           return p1.second < p2.second;
                         })
            ->first +
        1);
  }
  return types;
}

inline void SetLaneControls(int lane_control_num, int index,
                            const mapping::LaneInfo *lane_ptr,
                            std::vector<float> *lane_control_ptrs) {
  (*lane_control_ptrs)[index * lane_control_num] =
      !lane_ptr->traffic_lights.empty();
  (*lane_control_ptrs)[index * lane_control_num + 1] =
      lane_ptr->speed_limit * 3.6;  // kph
}

void SetLaneCentersAndSegments(int max_lane_point_num, int coord_num,
                               Vec2d ref_pos, double rot_rad, int index,
                               const mapping::LaneInfo *lane_ptr,
                               std::vector<float> *lane_center_ptrs,
                               std::vector<float> *lane_segment_ptrs);

std::vector<Vec2d> GetExitsOfIntersection(
    const SemanticMapManager &semantic_map_manager,
    const qcraft::mapping::IntersectionInfo &intersection,
    double reduction_radius);

std::vector<Vec2d> PickPointsByGreedy(absl::Span<const Vec2d> points,
                                      double radius);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
