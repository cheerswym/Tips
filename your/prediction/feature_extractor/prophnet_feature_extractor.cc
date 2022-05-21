#include "onboard/prediction/feature_extractor/prophnet_feature_extractor.h"

#include <algorithm>
#include <memory>
#include <ostream>
#include <utility>

#include "ext/alloc_traits.h"
#include "glog/logging.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/prediction_util.h"
namespace qcraft {
namespace prediction {
namespace {

std::vector<std::vector<int>> GetInputDims(const int objects_num,
                                           const int lanes_num) {
  std::vector<std::vector<int>> input_dims;
  input_dims = prophnet::kInitInputDims;
  input_dims[2][1] = objects_num;
  input_dims[3][1] = objects_num;
  input_dims[4][1] = objects_num;
  input_dims[5][1] = objects_num;
  input_dims[6][1] = objects_num;
  input_dims[7][1] = lanes_num;
  return input_dims;
}

void SetObjectAttributes(const ResampledObjectsHistory &objects_history,
                         std::vector<float> *shapes, std::vector<float> *max_ks,
                         std::vector<float> *max_lat_accs,
                         std::vector<float> *dist_to_racs,
                         std::vector<float> *is_statics) {
  shapes->clear();
  shapes->reserve(2 * objects_history.size());
  max_ks->clear();
  max_ks->reserve(objects_history.size());
  max_lat_accs->clear();
  max_lat_accs->reserve(objects_history.size());
  is_statics->clear();
  is_statics->reserve(objects_history.size());
  for (int obj_idx = 0; obj_idx < objects_history.size(); ++obj_idx) {
    const auto &obj_proto = objects_history[obj_idx].back();
    const auto &bbox = obj_proto.bounding_box();
    shapes->push_back(bbox.length());
    shapes->push_back(bbox.width());
    max_ks->push_back(kLengthToCurvaturePlf(bbox.length()));
    max_lat_accs->push_back(kLengthToMaxLatAccPlf(bbox.length()));
    dist_to_racs->push_back(
        0.5 * std::min(kLengthToWheelbasePlf(bbox.length()), bbox.length()));
    is_statics->push_back(obj_proto.moving_state() == ObjectProto::MS_STATIC);
  }
}

prophnet::ActorsFeature ExtractActorsFeature(
    Vec2d ref_pos, double rot_rad,
    const ResampledObjectsHistory &objects_history) {
  const int objects_num = objects_history.size();
  std::vector<float> trajs(objects_num * prophnet::kHistoryNum *
                           prophnet::kCoords);
  std::vector<float> speeds(objects_num * prophnet::kHistoryNum);
  std::vector<float> headings(objects_num * prophnet::kHistoryNum *
                              prophnet::kCoords);
  std::vector<float> cur_poses(objects_num * prophnet::kCoords);
  SetHistoryStatesAndCurPoses(
      prophnet::kCoords, prophnet::kHistoryNum, prophnet::kTimeStep, ref_pos,
      rot_rad, objects_history, &trajs, &speeds, &headings, &cur_poses);
  std::vector<float> shapes, max_ks, max_lat_accs, dist_to_racs, is_statics;
  SetObjectAttributes(objects_history, &shapes, &max_ks, &max_lat_accs,
                      &dist_to_racs, &is_statics);
  return prophnet::ActorsFeature{
      .ego_pos = GetEgoPos(ref_pos, prophnet::kCoords),
      .rot_mat = GetRotMatrix(rot_rad, prophnet::kCoords),
      .trajs = std::move(trajs),
      .speeds = std::move(speeds),
      .headings = std::move(headings),
      .types = VotingTypes<float>(objects_history),
      .cur_poses = std::move(cur_poses),
      .shapes = std::move(shapes),
      .max_ks = std::move(max_ks),
      .max_lat_accs = std::move(max_lat_accs),
      .dist_to_racs = std::move(dist_to_racs),
      .is_statics = std::move(is_statics),
  };
}

void SetLaneLights(absl::Span<const mapping::LaneInfo *const> lane_ptrs,
                   const TrafficLightManager::TLStateHashMap &tl_states,
                   std::vector<float> *lane_lights) {
  lane_lights->clear();
  lane_lights->reserve(lane_ptrs.size() * prophnet::kLaneLightsFeatNum);
  for (const auto *lane_ptr : lane_ptrs) {
    int num_green = 0;
    int num_yellow = 0;
    int num_red = 0;
    int num_unknown = 0;
    for (const auto *light : lane_ptr->traffic_lights) {
      const auto *light_stat = FindOrNull(tl_states, light->id());
      if (light_stat == nullptr) {
        ++num_unknown;
        continue;
      }
      switch (light_stat->color()) {
        case TL_GREEN:
          ++num_green;
          break;
        case TL_YELLOW:
          ++num_yellow;
          break;
        case TL_RED:
          ++num_red;
          break;
        case TL_UNKNOWN:
          ++num_unknown;
          break;
      }
    }
    lane_lights->push_back(num_green);
    lane_lights->push_back(num_yellow);
    lane_lights->push_back(num_red);
    lane_lights->push_back(num_unknown);
  }
}

prophnet::LanesFeature ExtractLanesFeature(
    Vec2d ref_pos, double rot_rad,
    const TrafficLightManager::TLStateHashMap &tl_states,
    const std::vector<const mapping::LaneInfo *> lane_ptrs) {
  const int lane_size = lane_ptrs.size();
  std::vector<float> lane_centers(lane_size * prophnet::kLanePointsNum *
                                  prophnet::kCoords);
  std::vector<float> lane_segments(lane_size * prophnet::kLanePointsNum *
                                   prophnet::kCoords);
  std::vector<float> lane_controls(lane_size * prophnet::kLaneControls);
  for (int i = 0; i < lane_size; ++i) {
    SetLaneCentersAndSegments(prophnet::kLanePointsNum, prophnet::kCoords,
                              ref_pos, rot_rad, i, lane_ptrs[i], &lane_centers,
                              &lane_segments);
    SetLaneControls(prophnet::kLaneControls, i, lane_ptrs[i], &lane_controls);
  }
  std::vector<float> lane_lights;
  SetLaneLights(lane_ptrs, tl_states, &lane_lights);
  return prophnet::LanesFeature{.lane_centers = std::move(lane_centers),
                                .lane_segments = std::move(lane_segments),
                                .lane_controls = std::move(lane_controls),
                                .lane_lights = std::move(lane_lights)};
}

}  // namespace

prophnet::ProphnetFeature ExtractProphnetFeature(
    const ResampledObjectsHistory &objects_history,
    const TrafficLightManager::TLStateHashMap &tl_states,
    const SemanticMapManager *semantic_map_manager) {
  const int objects_num = objects_history.size();

  prophnet::ProphnetFeature input_features;
  const auto &current_ego = objects_history.back().back();
  Vec2d ego_pos = Vec2dFromProto(current_ego.pos());
  double rot_rad = -current_ego.yaw();
  input_features.actors_feature =
      ExtractActorsFeature(ego_pos, rot_rad, objects_history);
  const Box2d map_region_box =
      GetRegionBox(ego_pos, current_ego.yaw(), kProphnetMapRegionFront,
                   kProphnetMapRegionBehind, kProphnetMapHalfWidth);
  std::vector<const mapping::LaneInfo *> lane_ptrs = GetLanesInBox2d(
      *semantic_map_manager, map_region_box, prophnet::kMaxLanesNum);
  VLOG(2) << "Number of objects: " << objects_history.size()
          << ", Number of lanes: " << lane_ptrs.size()
          << ", total sum: " << objects_history.size() + lane_ptrs.size();
  input_features.lanes_feature =
      ExtractLanesFeature(ego_pos, rot_rad, tl_states, lane_ptrs);

  const int lanes_num = lane_ptrs.size();

  const int total_dim = prophnet::kMaxObjectsNum + prophnet::kMaxLanesNum;
  input_features.masks.resize(total_dim);
  for (int i = 0; i < total_dim; ++i) {
    if (i < objects_num) {
      // 0.0 means there is an object/lane at this position, need to attend (in
      // float form just for model inference).
      input_features.masks[i] = 0.0;
    } else if (i >= prophnet::kMaxObjectsNum &&
               i < prophnet::kMaxObjectsNum + lanes_num) {
      input_features.masks[i] = 0.0;
    } else {
      // 1.0 means there is no object at this position, no need to attend.
      input_features.masks[i] = 1.0;
    }
  }

  input_features.input_dims = GetInputDims(objects_num, lanes_num);

  return input_features;
}

}  // namespace prediction
}  // namespace qcraft
