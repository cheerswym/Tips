#ifndef ONBOARD_PREDICTION_PREDICTOR_VEHICLE_TNT_FEATURE_H_
#define ONBOARD_PREDICTION_PREDICTOR_VEHICLE_TNT_FEATURE_H_

#include <string>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
static constexpr struct VehicleTNTConfig {
  double history_time_step = 0.1;  // s.
  int history_num = 10;
  double future_time_step = 0.1;  // s.
  int future_num = 50;
  int coord_num = 2;
  int max_other_objects_num = 16;
  int max_lane_num = 128;
  int max_lane_point_num = 20;
  int lane_control_num = 2;
  int lane_light_num = 4;
  double lane_search_radius = 100.0;
  double scan_box_front = 70.0;
  double scan_box_back = 30.0;
  double scan_box_half_width = 30.0;
  int max_targets_num = 32;
  int target_control_num = 2;
} kVehicleTNTConfig;
/*
Input dimension:
ego traj: {history_num, coord_num}
ego headings: {history_num, coord_num}
ego speeds: {history_num}
cur_poses: {kMaxObjectsNum, kCoords}
trajs: {max_other_objects_num, history_num, coord_num}
headings: {max_other_objects_num, history_num, coord_num}
speeds: {max_other_objects_num, history_num}
types: {max_other_objects_num}
lane_centers:    {max_lane_num, max_lane_point_num, coord_num}
lane_segments:    {max_lane_num, max_lane_point_num, coord_num}
lane_controls:    {max_lane_num, lane_control_num}
targets: {max_targets_num, kCoords}
controls: {max_targets_num,target_control_num}
*/
struct VehicleTNTActorsFeature {
  std::vector<float> ego_traj;
  std::vector<float> ego_speeds;
  std::vector<float> ego_headings;
  std::vector<float> trajs;
  std::vector<float> speeds;
  std::vector<float> headings;
  std::vector<float> cur_poses;
  std::vector<PredictionObjectType> types;
};
struct VehicleTNTLanesFeature {
  // TODO(shenlong): Add ingoing ougoing and neighbour lanes info.
  // TODO(shenlong): Add traffic light info.
  std::vector<float> lane_centers;
  std::vector<float> lane_segments;
  std::vector<float> lane_controls;
  std::vector<float> lane_lights;
};
struct VehicleTNTTargets {
  std::vector<float> targets;
  std::vector<float> controls;
};
struct VehicleTNTFeature {
  std::string ego_id;
  Vec2d ref_pose;
  float rot_rad;
  VehicleTNTActorsFeature actors_feature;
  VehicleTNTLanesFeature lanes_feature;
  VehicleTNTTargets targets;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_VEHICLE_TNT_FEATURE_H_
