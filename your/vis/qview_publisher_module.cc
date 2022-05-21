#include "onboard/vis/qview_publisher_module.h"

#include <algorithm>
#include <limits>
#include <regex>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "boost/algorithm/string/case_conv.hpp"
#include "glog/logging.h"
#include "onboard/async/async_util.h"
#include "onboard/camera/utils/image_util.h"
#include "onboard/crypto/base64.h"
#include "onboard/global/trace.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/proto/port.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/thread_util.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/qview_data_serializer.h"
#include "opencv2/opencv.hpp"

DEFINE_bool(enable_qview, false, "Enable QView.");
DEFINE_double(qview_refresh_rate, 0.025, "QView refresh rate in seconds.");
DEFINE_bool(qview_discard_obsolete_pose, true,
            "Whether to discard obsolete pose.");
DEFINE_bool(publish_images_to_qview, false,
            "Whether to publish base64 encoded images to qivew.");
DEFINE_bool(publish_offroad_objects, true,
            "Enable publishing pedestrians and cyclists offroad.");
DEFINE_string(qview_request_ip, "0.0.0.0", "QView request ip.");
DEFINE_bool(enable_qview_grpc, true, "Enable qview grpc instead of websocket");
DEFINE_string(qview_front_camera, "CAM_L_FRONT", "front camera for qview .");
DEFINE_string(qview_rear_camera, "CAM_INSIDE_DRIVER", "rear camera qivew.");
DEFINE_string(goal, "DEFAULT_GOAL", "route goal.");
DEFINE_double(qview_camera_rate, 0.1, "QView refresh camera interval second.");
DEFINE_double(qview_image_down_scale, 4, "QView image down scale factor.");
DEFINE_bool(qview_use_downsized_image, false,
            "Whether to take and publish downsized encoded image directly.");
DEFINE_bool(publish_points_to_qview, true,
            "Whether to publish pedestrian, cyclist and offroad points.");

namespace qcraft {
namespace {
constexpr double kLineWidth = 0.15;                 // m
constexpr double kLineHeight = 0.01;                // m
constexpr double kDoubleLineDist = 0.25;            // m
constexpr double kBrokenLineLength = 2.0;           // m
constexpr double kMapElementMaxDistFront = 170.0;   // m
constexpr double kMapElementMaxDistRear = 20.0;     // m
constexpr double kMapElementMaxDistLateral = 40.0;  // m
constexpr double kCrosswalkLineGapWidth = 1.0;      // m
constexpr double kCrosswalkLineWidth = 0.48;        // m
constexpr double kDistanceSet = 50;         // m length of perpendicular line
constexpr double kTrajectoryHeight = 0.02;  // m

constexpr double kPerceptionRoiFrontValue = 50.0;
constexpr double kPerceptionRoiFrontMaxValue = 100.0;
constexpr double kPerceptionRoiRearValue = 0;
constexpr double kPerceptionRoiSideValue = 48.0;
constexpr int kMaxObjsKeepingCounts = 30;
constexpr double kDynamicOrStaticVelocity = 0.03;  // m/s
constexpr int kObjectBufferSize = 10;
constexpr int kLeftTurningLight = 0;
constexpr int kStraightLight = 1;
constexpr int kRightTurningLight = 2;
constexpr int kUTurnLight = 3;

constexpr int kRedLightViolationWarning = 1;
constexpr int kBusFirst = 2;
constexpr int kSpeedAdvisory = 3;
constexpr int kObstacleCollision = 4;
constexpr int kVulnerableRoadUserCollisionWarning = 5;
constexpr int kIllegalVehicleWarning = 6;
constexpr int kSpeedLimitWarning = 7;
constexpr int kSpeedLimitReminder = 8;
constexpr int kInVehicleSignage = 9;
constexpr int kConstruction = 10;
constexpr int kWrongWay = 11;
constexpr int kForwardCollisionWarning = 12;
constexpr int kIntersectionCollisionWarning = 13;
constexpr int kLeftTurnAssist = 14;
constexpr int kBlindSpotWarning = 15;
constexpr int kLaneChangeWarning = 16;
constexpr int kEmergencyBrakeWarning = 17;
constexpr int kAbnormalVehicleWarning = 18;
constexpr int kOutOfControlWarning = 19;
constexpr int kEmergencyVehicleWarning = 20;
constexpr int kTrafficJamWarning = 21;
constexpr int kSpeedingWarning = 24;
constexpr int kPedestrianOnMotorWay = 25;
constexpr int kNonMotorOnMotorWay = 26;

constexpr int kNoLane = 1;
constexpr int kHaveLanes = 2;

constexpr int kHaveDistance = 1;
constexpr int kNoDistance = 2;
// advisory speed type
constexpr int kAdvisorySpeed = 0;
constexpr int kAdvisorySpeedRange = 1;

constexpr int kMapPolygonSerializeRound = 6;
constexpr double kMinVehicleArea = 2;

enum class WanjiPtcpType {
  UNKNOWN = 0,
  MOTOR = 1,
  NON_MOTOR = 2,
  PEDESTRIAN = 3,
  RSU = 4,
  ROADWORK = 5,
  ACCIDENT = 6,
  CAR = 7,
  TRUCK = 8,
  BUS = 9,
  BICYCLE = 11,
  MOTORBIKE = 12,
};

std::map<int, QViewObjectProto::Type> TypeFromPtcpToQView = {
    {static_cast<int>(WanjiPtcpType::MOTOR), QViewObjectProto::VEHICLE},
    {static_cast<int>(WanjiPtcpType::NON_MOTOR), QViewObjectProto::CYCLIST},
    {static_cast<int>(WanjiPtcpType::PEDESTRIAN), QViewObjectProto::PEDESTRIAN},
    {static_cast<int>(WanjiPtcpType::CAR), QViewObjectProto::VEHICLE},
    {static_cast<int>(WanjiPtcpType::TRUCK), QViewObjectProto::VEHICLE},
    {static_cast<int>(WanjiPtcpType::BUS), QViewObjectProto::VEHICLE},
    {static_cast<int>(WanjiPtcpType::BICYCLE), QViewObjectProto::CYCLIST},
    {static_cast<int>(WanjiPtcpType::MOTORBIKE), QViewObjectProto::CYCLIST}};

// enum value is consistent with the old version.
enum class VehicleModelType {
  UNKNOWN = 0,
  SUV = 1,
  ROBOTAXI_CAR = 2,
  ROBOBUS_ONE = 3,
  ROBOBUS_SPACE = 4,
  ROBOBUS_LONG = 5,
};

const std::map<VehicleModel, VehicleModelType> VehicleModelToQView = {
    {VehicleModel::VEHICLE_UNKNOWN, VehicleModelType::UNKNOWN},
    {VehicleModel::VEHICLE_DONGFENG, VehicleModelType::UNKNOWN},
    {VehicleModel::VEHICLE_TEST_BENCH, VehicleModelType::UNKNOWN},

    {VehicleModel::VEHICLE_AION_LX, VehicleModelType::SUV},
    {VehicleModel::VEHICLE_MARVELX, VehicleModelType::SUV},
    {VehicleModel::VEHICLE_MARVELR, VehicleModelType::SUV},

    {VehicleModel::VEHICLE_LINCOLN_MKZ, VehicleModelType::ROBOTAXI_CAR},
    {VehicleModel::VEHICLE_LINCOLN_MKZ_AS_PACMOD,
     VehicleModelType::ROBOTAXI_CAR},
    {VehicleModel::VEHICLE_BYD, VehicleModelType::ROBOTAXI_CAR},

    {VehicleModel::VEHICLE_HIGER, VehicleModelType::ROBOBUS_ONE},
    {VehicleModel::VEHICLE_JINLV_MINIBUS, VehicleModelType::ROBOBUS_ONE},
    {VehicleModel::VEHICLE_ZHONGTONG6, VehicleModelType::ROBOBUS_ONE},

    {VehicleModel::VEHICLE_SHUNFENG, VehicleModelType::ROBOBUS_SPACE},
    {VehicleModel::VEHICLE_PIXLOOP, VehicleModelType::ROBOBUS_SPACE},
    {VehicleModel::VEHICLE_SKYWELL, VehicleModelType::ROBOBUS_SPACE},
    {VehicleModel::VEHICLE_ZHONGXING, VehicleModelType::ROBOBUS_SPACE},
    {VehicleModel::VEHICLE_ZHONGTONG55, VehicleModelType::ROBOBUS_SPACE},

    {VehicleModel::VEHICLE_ZHONGTONG, VehicleModelType::ROBOBUS_LONG},
    {VehicleModel::VEHICLE_POLERSTAR, VehicleModelType::ROBOBUS_LONG},
    {VehicleModel::VEHICLE_POLERSTAR_2, VehicleModelType::ROBOBUS_LONG}};

int GetVehicleModel(const ParamManager &param_manager) {
  RunParamsProtoV2 run_params;
  param_manager.GetRunParams(&run_params);
  const VehicleModel vehicle_model =
      run_params.vehicle_params().vehicle_params().model();
  if (const auto *vehicle_type =
          FindOrNull(VehicleModelToQView, vehicle_model)) {
    return static_cast<int>(*vehicle_type);
  }
  return static_cast<int>(VehicleModelType::UNKNOWN);
}

std::optional<std::string> GetImageFromCamera(const CameraImage camera_image) {
  const auto image_mat = camera_image.ToMat();
  if (image_mat.empty()) return std::nullopt;
  std::vector<unsigned char> encoded_image =
      image_util::ScaleDownImageAndEncode(image_mat,
                                          FLAGS_qview_image_down_scale,
                                          /*jpg_quality=*/40);
  const std::string image = {std::make_move_iterator(encoded_image.begin()),
                             std::make_move_iterator(encoded_image.end())};
  return image;
}

bool CheckCameraInWhiteList(CameraId camera_id) {
  static const std::array<CameraId, 3> kCameraWhitelist{CAM_FRONT, CAM_L_FRONT,
                                                        CAM_INSIDE_DRIVER};
  return (std::find(kCameraWhitelist.begin(), kCameraWhitelist.end(),
                    camera_id) != kCameraWhitelist.end());
}

bool CheckLanePathLanesExistInMap(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePathProto &lane_path_proto) {
  for (const mapping::ElementId lane_id : lane_path_proto.lane_ids()) {
    if (semantic_map_manager.FindLaneByIdOrNull(lane_id) == nullptr) {
      return false;
    }
  }
  return true;
}

void RenderPlannerTrafficLightInfo(
    const std::shared_ptr<const PlannerDebugProto> &planner_debug,
    QViewElements *elements) {
  auto *traffic_lights = elements->add_traffic_lights();

  // NOTE(Sun Zhenye): It seems that traffic light state will not be refreshed
  // if it returns directly without setting COLOR_UNKNOWN.
  if (!planner_debug->has_selected_traffic_light_info()) {
    traffic_lights->set_color(QViewTrafficLightProto::COLOR_UNKNOWN);
    return;
  }
  const auto &tl_info = planner_debug->selected_traffic_light_info();

  if (!tl_info.has_isolated_tl_info() &&
      (!tl_info.has_left_tl_info() || !tl_info.has_straight_tl_info())) {
    traffic_lights->set_color(QViewTrafficLightProto::COLOR_UNKNOWN);
    return;
  }

  const auto &render_tl_info = tl_info.has_isolated_tl_info()
                                   ? tl_info.isolated_tl_info()
                                   : tl_info.left_tl_info();

  switch (render_tl_info.color()) {
    case planner::TL_COLOR_RED:
      traffic_lights->set_color(QViewTrafficLightProto::RED);
      break;
    case planner::TL_COLOR_YELLOW:
      traffic_lights->set_color(QViewTrafficLightProto::YELLOW);
      break;
    case planner::TL_COLOR_GREEN:
      traffic_lights->set_color(QViewTrafficLightProto::GREEN);
      break;
    case planner::TL_COLOR_UNKNOWN:
      traffic_lights->set_color(QViewTrafficLightProto::COLOR_UNKNOWN);
      break;
  }
  traffic_lights->set_shape(QViewTrafficLightProto::ROUND);
  traffic_lights->set_flashing(render_tl_info.flashing());
}

void RenderSpeedInfo(const SemanticMapManager &semantic_map_manager,
                     const std::shared_ptr<const PoseProto> &pose,
                     const std::shared_ptr<const TrajectoryProto> &trajectory,
                     QViewElements *elements) {
  if (!trajectory->has_target_lane_path_from_current()) {
    return;
  }
  if (!CheckLanePathLanesExistInMap(
          semantic_map_manager, trajectory->target_lane_path_from_current())) {
    // Map mismatch.
    LOG_EVERY_N(WARNING, 3600)
        << "Map mismatch between route and Vantage's currently loaded map.";
    return;
  }
  const mapping::LanePath lane_path(
      &semantic_map_manager, trajectory->target_lane_path_from_current());
  if (lane_path.IsEmpty()) return;
  const mapping::ElementId current_lane_id = lane_path.front().lane_id();
  const mapping::LaneProto *lane_proto =
      semantic_map_manager.FindLaneByIdOrNull(current_lane_id);
  if (lane_proto == nullptr) return;
  if (!lane_proto->has_speed_limit_mph()) return;

  const int speed_limit = lane_proto->has_speed_limit_kph()
                              ? lane_proto->speed_limit_kph()
                              : lane_proto->speed_limit_mph();

  auto *speed_info = elements->mutable_speed_info();

  speed_info->set_speed(pose->vel_body().x());
  speed_info->set_accel(pose->accel_body().x());
  speed_info->set_speed_limit(speed_limit);
}

void RenderControlInfo(
    const std::shared_ptr<const ControlCommand> &control_command,
    const std::shared_ptr<const TrajectoryProto> &trajectory,
    const PlannerDebugProto *planner_debug, QViewElements *elements) {
  // 1.0 for max steering left; -1.0 for right.
  const double steering = control_command->steering_target() / 100.0;
  // 1.0 for max braking.
  const double braking = control_command->brake() / 100.0;
  // 1.0 for max throttle.
  const double throttle = control_command->throttle() / 100.0;
  const bool left_blinker = trajectory->turn_signal() == TURN_SIGNAL_LEFT ||
                            trajectory->turn_signal() == TURN_SIGNAL_EMERGENCY;
  const bool right_blinker = trajectory->turn_signal() == TURN_SIGNAL_RIGHT ||
                             trajectory->turn_signal() == TURN_SIGNAL_EMERGENCY;

  auto *control_info = elements->mutable_control_info();

  control_info->set_steering(steering);
  control_info->set_braking(braking);
  control_info->set_throttle(throttle);
  control_info->set_left_blinker(left_blinker);
  control_info->set_right_blinker(right_blinker);
  control_info->set_blinker_reason(
      planner_debug == nullptr ? 0 : planner_debug->turn_signal_reason_enum());
}

// longitude/latitude in degree, timestamp in second
VehiclePose GetNearestPose(
    const boost::circular_buffer<std::pair<double, VehiclePose>>
        &pose_histories,
    const double timestamp) {
  auto it = std::lower_bound(
      pose_histories.begin(), pose_histories.end(), timestamp,
      [](const auto &pose, const double time) { return pose.first < time; });
  auto pose = (it == pose_histories.end() ? pose_histories.back() : *it).second;
  return pose;
}

bool IsOutOfVehicleRange(const double x, const double y) {
  constexpr double kMaxVehicleRange = 500.;
  return x * x + y * y > Sqr(kMaxVehicleRange);
}

// length aligned to x axis, width aligned to y axis.
Box2d GetObjectAxisAlignedBoundingBox(const ObjectProto &object) {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto &point : object.contour()) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  return Box2d{Vec2d{(min_x + max_x) * 0.5, (min_y + max_y) * 0.5}, 0.0,
               max_x - min_x, max_y - min_y};
}

void RenderGlosa(const std::shared_ptr<const V2xProto> &v2x_proto,
                 QViewElements *elements) {
  if (!v2x_proto ||
      v2x_proto->green_light_optimal_speed_advisory().glosa_info_size() == 0) {
    return;
  }

  if (IsOnboardMode()) {
    absl::Time latest = absl::FromUnixMicros(v2x_proto->header().timestamp());
    if (absl::Now() - latest > absl::Seconds(1)) {
      LOG(ERROR) << "glosa msg too old, deprecated.";
      return;
    }
  }
  static auto convert_color_state =
      [](const auto glosa_state) -> QViewTrafficLightProto::Color {
    switch (glosa_state) {
      case GreenLightOptimalSpeedAdvisoryItemProto::RED:
        return QViewTrafficLightProto::RED;
      case GreenLightOptimalSpeedAdvisoryItemProto::YELLOW:
        return QViewTrafficLightProto::YELLOW;
      case GreenLightOptimalSpeedAdvisoryItemProto::GREEN:
        return QViewTrafficLightProto::GREEN;
      case GreenLightOptimalSpeedAdvisoryItemProto::YELLOW_FLASH:
        return QViewTrafficLightProto::YELLOW_FLASH;
      case GreenLightOptimalSpeedAdvisoryItemProto::UNKNOWN:
        return QViewTrafficLightProto::COLOR_UNKNOWN;
    }
  };
  const auto &glosa_info =
      v2x_proto->green_light_optimal_speed_advisory().glosa_info();
  for (int i = 0;
       i < v2x_proto->green_light_optimal_speed_advisory().glosa_info_size();
       i++) {
    auto *glosa_light = elements->add_glosa_lights();
    glosa_light->set_color(convert_color_state(glosa_info[i].state()));
    glosa_light->set_flashing(false);
    glosa_light->set_timing(glosa_info[i].timing());
    if (i == kLeftTurningLight) {
      glosa_light->set_shape(QViewTrafficLightProto::LEFT_ARROW);
    } else if (i == kStraightLight) {
      glosa_light->set_shape(QViewTrafficLightProto::UP_ARROW);
    } else if (i == kRightTurningLight) {
      glosa_light->set_shape(QViewTrafficLightProto::RIGHT_ARROW);
    } else if (i == kUTurnLight) {
      glosa_light->set_shape(QViewTrafficLightProto::U_TURN_ARROW);
    }
  }
}

void FindAndUpdateElements(const Vec2d &rv_object, const double yaw,
                           QViewElements *elements) {
  // do not rander rv 500m away
  if (IsOutOfVehicleRange(rv_object.x(), rv_object.y())) {
    return;
  }
  //
  const auto get_distance = [](const auto &object, const auto &rv_object) {
    return Hypot(object.bounding_box().x() - rv_object.x(),
                 object.bounding_box().y() - rv_object.y());
  };
  QViewObjectProto *related_object = nullptr;
  double min_distance = std::numeric_limits<double>::max();
  for (auto &object : *elements->mutable_objects()) {
    if (object.type() == QViewObjectProto::VEHICLE) {
      double dis = get_distance(object, rv_object);
      if (dis < min_distance) {
        min_distance = dis;
        related_object = &object;
      }
    }
  }
  constexpr double kIsSameDistance = 3.5;
  if (min_distance < kIsSameDistance) {
    related_object->set_type(QViewObjectProto::ABNORMAL_VEHICLE);
  } else {
    auto new_object = elements->add_objects();
    new_object->set_type(QViewObjectProto::ABNORMAL_VEHICLE);
    new_object->set_id(std::to_string(1));
    new_object->set_data_src(QViewObjectProto::V2X);
    auto rect = new_object->mutable_bounding_box();
    rect->set_x(rv_object.x());
    rect->set_y(rv_object.y());
    rect->set_length(4.5);
    rect->set_width(1.8);
    rect->set_heading(NormalizeAngle(-yaw));
  }
}

bool InMapRegion(const Box2d &map_region, const std::vector<Vec2d> &points) {
  for (const auto &p : points) {
    if (map_region.IsPointIn(p)) {
      return true;
    }
  }
  return false;
}

}  // namespace

QViewPublisherModule::QViewPublisherModule(LiteClientBase *lite_client)
    : LiteModule(lite_client),
      lidar_frame_pre_update_time_(0),
      lidar_frame_update_time_(0),
      map_polygon_serialize_count_(0) {}

void QViewPublisherModule::CollectMapFeatures() {
  CollectCurbs();
  CollectLaneBoundaries();

  const auto &semantic_map = semantic_map_manager_.semantic_map();
  std::map<int, std::vector<mapping::LaneProto>> lanes;
  for (const auto &lane : semantic_map.lanes()) {
    for (const auto level_id : lane.belonging_levels())
      lanes[level_id].emplace_back(lane);
  }

  std::map<int, std::vector<CrosswalkPolygon>> crosswalk_polygons;
  for (const auto &crosswalk : semantic_map.crosswalks()) {
    std::vector<Vec2d> crosswalk_points;
    for (const auto &point : crosswalk.polygon().points()) {
      crosswalk_points.push_back(
          GlobalToLocal({point.longitude(), point.latitude()}));
    }
    for (const auto level_id : crosswalk.belonging_levels())
      crosswalk_polygons[level_id].push_back(
          {crosswalk.id(), Polygon2d(crosswalk_points)});
  }
  std::set<int> all_levels;
  for (const auto &[level_id, _] : lanes) all_levels.insert(level_id);
  for (const auto &[level_id, _] : crosswalk_polygons)
    all_levels.insert(level_id);
  for (const auto level_id : all_levels) {
    CollectCrosswalks(level_id, lanes[level_id], crosswalk_polygons[level_id]);
  }

  CollectBuildings();
  CollectBusStations();
  CollectPickUpPoints();
}

void QViewPublisherModule::CollectBuildings() {
  const auto &semantic_map = semantic_map_manager_.semantic_map();
  for (const auto &building : semantic_map.buildings()) {
    std::vector<Vec2d> points;
    for (const auto &point : building.polygon().points()) {
      points.emplace_back(point.longitude(), point.latitude());
    }
    for (const auto level_id : building.belonging_levels()) {
      buildings_[level_id].push_back(
          {building.id(), building.height(), points});
    }
  }
}

void QViewPublisherModule::CollectBusStations() {
  const auto &semantic_map = semantic_map_manager_.semantic_map();
  for (const auto &station : semantic_map.bus_stations()) {
    for (const auto level_id : station.belonging_levels()) {
      bus_stations_[level_id].push_back(
          {station.id(),
           Vec2d(station.point().longitude(), station.point().latitude()),
           station.heading(), station.height()});
    }
  }
}

void QViewPublisherModule::CollectPickUpPoints() {
  const auto &semantic_map = semantic_map_manager_.semantic_map();
  absl::flat_hash_set<std::string> pick_up_points_list;
  if (hmi_contents_.size() > 0) {
    const auto &hmi_content = *hmi_contents_.back();
    const auto &stations_list = hmi_content.route_content().stations_list();
    for (const auto &station_name : stations_list) {
      pick_up_points_list.emplace(station_name);
    }
  }
  if (pick_up_points_list.empty()) {
    VLOG(2) << "pick up points list is empty.";
  }
  for (const auto &named_spot : semantic_map.named_spots()) {
    if (!pick_up_points_list.empty() &&
        pick_up_points_list.count(named_spot.name()) == 0)
      continue;
    if (named_spot.type() != mapping::NamedSpotProto::PICK_UP_POINT) continue;
    for (const auto level_id : named_spot.belonging_levels()) {
      double heading = 0.0;
      const auto smooth_point =
          semantic_map_manager_.coordinate_converter().GlobalToSmooth(
              {named_spot.point().longitude(), named_spot.point().latitude()});
      const auto *lane_info = semantic_map_manager_.GetNearestLaneInfoAtLevel(
          level_id, smooth_point);
      double min_dist = std::numeric_limits<double>::max();
      if (lane_info) {
        for (int i = 0; i + 1 < lane_info->points_smooth.size(); ++i) {
          Segment2d segment(lane_info->points_smooth[i],
                            lane_info->points_smooth[i + 1]);
          const double curr_dist = segment.DistanceTo(smooth_point);
          if (min_dist > curr_dist) {
            min_dist = curr_dist;
            heading = segment.heading();
          }
        }
      } else {
        VLOG(2) << absl::StrFormat(
            "Can't find a nearby lane, so I can't infer the heading of the "
            "named spot, id = %d",
            named_spot.id());
      }
      pick_up_points_[level_id].push_back(
          {named_spot.id(),
           Vec2d(named_spot.point().longitude(), named_spot.point().latitude()),
           heading});
    }
  }
}

void QViewPublisherModule::CollectCurbs() {
  const auto &semantic_map = semantic_map_manager_.semantic_map();
  // 0 ---- the curb of the current level has not been consumed.
  // 1 ---- the curb of the current level has been consumed.
  absl::flat_hash_map<int32_t, absl::flat_hash_map<mapping::ElementId, int>>
      all_level_curb_consumed_flags;
  absl::flat_hash_map<int32_t, std::vector<mapping::ElementId>>
      all_level_curb_ids;
  absl::flat_hash_map<int32_t, std::vector<std::vector<mapping::ElementId>>>
      all_level_rings;
  absl::flat_hash_map<mapping::ElementId, const mapping::LaneBoundaryProto *>
      lane_boundaries_map;
  for (const auto &lane_boundary : semantic_map.lane_boundaries()) {
    if (lane_boundary.type() == mapping::LaneBoundaryProto::CURB) {
      lane_boundaries_map[lane_boundary.id()] = &lane_boundary;
      for (const auto level_id : lane_boundary.belonging_levels()) {
        all_level_curb_consumed_flags[level_id].emplace(lane_boundary.id(), 0);
        all_level_curb_ids[level_id].emplace_back(lane_boundary.id());
        all_level_rings[level_id];
      }
    }
  }

  const auto get_next_curb_id =
      [&lane_boundaries_map](mapping::ElementId *curb_id, const int level_id,
                             const absl::flat_hash_map<mapping::ElementId, int>
                                 &curb_consumed_flags) {
        if (!curb_id) return false;
        bool have_next_curb_id = false;
        for (const auto curr_curb_id :
             lane_boundaries_map[*curb_id]->end_connection_ids()) {
          if (curb_consumed_flags.count(curr_curb_id) == 1) {
            have_next_curb_id = true;
            *curb_id = curr_curb_id;
            break;
          }
        }
        if (!have_next_curb_id) {
          VLOG(2) << absl::StrFormat(
              "curb id = %d, does no has end connection ids at %d levle.",
              *curb_id, level_id);
        }
        return have_next_curb_id;
      };

  for (const auto &[level_id, curb_ids] : all_level_curb_ids) {
    // Connect consecutive curbs.
    auto &curb_consumed_flags = all_level_curb_consumed_flags[level_id];
    auto &rings = all_level_rings[level_id];
    for (const auto curb_id : curb_ids) {
      if (curb_consumed_flags[curb_id]) continue;
      rings.emplace_back();
      auto &ring = rings.back();
      mapping::ElementId next_curb_id = curb_id;
      while (curb_consumed_flags.count(next_curb_id) != 0 &&
             !curb_consumed_flags[next_curb_id]) {
        ring.emplace_back(next_curb_id);
        curb_consumed_flags[next_curb_id] = 1;
        if (!get_next_curb_id(&next_curb_id, level_id, curb_consumed_flags)) {
          break;
        }
        if (next_curb_id == curb_id) break;
      }
    }
    // Generate a polygon without curb
    if (rings.empty()) continue;
    auto &curbs = curbs_[level_id];
    for (const auto &ring : rings) {
      curbs.emplace_back();
      auto &curb = curbs.back();
      curb.id = ring.front();
      for (const auto curb_id : ring) {
        for (const auto &point :
             lane_boundaries_map[curb_id]->polyline().points()) {
          curb.points.emplace_back(
              GlobalToLocal({point.longitude(), point.latitude()}));
        }
      }
    }
  }
}

void QViewPublisherModule::OnInit() {
  if (!FLAGS_enable_qview) return;

  // Start websocket server to handle frontend page request for qview 1.0
  websocket_server_v1_.EnableHttpService();
  websocket_server_v1_.AsyncRun(8080);

  // Start websocket server to handle frontend page request for qview 2.0 and
  // later
  websocket_server_v2_.AsyncRun(8081);

  semantic_map_manager_.LoadWholeMap().Build();
  ScheduleFuture(&thread_pool_, [this] { SendDataAsync(); });
}

void QViewPublisherModule::OnSubscribeChannels() {
  if (!FLAGS_enable_qview) return;
  if (FLAGS_publish_points_to_qview) {
    SubscribeLidarFrame([this](const LidarFrame &lidar_frame) {
      const SpinMetadata &spin_meta = lidar_frame.lidar_frame_metadata();
      // Only process full lidar frames for now.
      if (spin_meta.is_partial()) {
        return;
      }
      auto &lidar_frame_buffer = lidar_frame_buffer_[spin_meta.id()];
      if (lidar_frame_buffer.capacity() == 0) {
        lidar_frame_buffer.set_capacity(5);
      }
      lidar_frame_buffer.push_back(std::move(lidar_frame));
      lidar_frame_update_time_ = absl::ToUnixMicros(qcraft::Clock::Now());
    });
  }
  Subscribe(&QViewPublisherModule::OnObstacles, this);
  Subscribe(&QViewPublisherModule::OnObjects, this);
  Subscribe(&QViewPublisherModule::UpdatePose, this);
  Subscribe(&QViewPublisherModule::UpdateLocalizationTransform, this);
  Subscribe(&QViewPublisherModule::UpdateTrajectory, this);
  Subscribe(&QViewPublisherModule::UpdatePlannerDebug, this);
  Subscribe(&QViewPublisherModule::UpdateControlCommand, this);
  Subscribe(&QViewPublisherModule::UpdateHmiContentProto, this);
  if (FLAGS_publish_images_to_qview) {
    if (FLAGS_qview_use_downsized_image) {
      SubscribeEncodedImage(
          [=](std::shared_ptr<ShmMessage> shm_message) {
            OnDownsizedEncodedImage(std::move(shm_message));
          },
          "downsized_encoded_image");
      LOG(INFO) << "QViewPublisher subscribed downsized_encoded_image";
    } else {
      SubscribeDecodedImage(
          [=](const CameraImage &camera_image) { OnImage(camera_image); });
      LOG(INFO) << "QViewPublisher subscribed decoded_image";
    }
  }
  Subscribe(&QViewPublisherModule::UpdateV2x, this, "hmi_v2x_proto");
  Subscribe(&QViewPublisherModule::UpdateSemanticMap, this);
  Subscribe(&QViewPublisherModule::UpdateRunParams, this);
  Subscribe(&QViewPublisherModule::UpdateQRunEvents, this);
}

void QViewPublisherModule::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  localization_valid_ = true;
  coordinate_converter_.UpdateLocalizationTransform(
      *localization_transform_proto);
  current_level_ = coordinate_converter_.GetLevel();
}

void QViewPublisherModule::CollectLaneBoundary(
    const mapping::LaneBoundaryProto &lane_boundary_proto) {
  LaneBoundary lane_boundary;
  std::vector<Vec2d> points;
  points.reserve(lane_boundary_proto.polyline().points_size());
  for (const auto &point : lane_boundary_proto.polyline().points()) {
    points.push_back(GlobalToLocal({point.longitude(), point.latitude()}));
  }
  lane_boundary.id = lane_boundary_proto.id();
  lane_boundary.points = points;

  const auto collect_line = [&](Vec2d start, Vec2d end) {
    const Box2d box(Segment2d(start, end), kLineWidth);
    lane_boundary.polygons.push_back(box.GetCornersCounterClockwise());
  };

  const auto collect_double_line = [&](Vec2d start, Vec2d end) {
    const Box2d box(Segment2d(start, end), kDoubleLineDist);
    const auto corners = box.GetCornersCounterClockwise();
    collect_line(corners[0], corners[1]);
    collect_line(corners[3], corners[2]);
  };

  switch (lane_boundary_proto.type()) {
    case mapping::LaneBoundaryProto::SOLID_YELLOW:
    case mapping::LaneBoundaryProto::SOLID_WHITE:
      for (int i = 0; i < points.size() - 1; ++i) {
        collect_line(points[i], points[i + 1]);
      }
      break;
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
      for (int i = 0; i < points.size() - 1; ++i) {
        collect_double_line(points[i], points[i + 1]);
      }
      break;
    case mapping::LaneBoundaryProto::BROKEN_YELLOW:
    case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_WHITE: {
      // Calculate the total length of this broken line.
      double length = 0.0;
      for (int i = 0; i < points.size() - 1; ++i) {
        length += (points[i] - points[i + 1]).norm();
      }
      // Calculate the proper gap between broken lines.
      const int num_segments = RoundToInt(length / (kBrokenLineLength * 3));
      const double gap = length / num_segments - kBrokenLineLength;
      for (int i = 0; i < num_segments; ++i) {
        // Collect the solid part.
        double curr_segment_len = kBrokenLineLength;
        while (curr_segment_len > 0.0) {
          const auto start = points.back();
          points.pop_back();
          const auto end = points.back();
          const auto line_len = (start - end).norm();
          if (line_len <= curr_segment_len) {
            collect_line(start, end);
            curr_segment_len -= line_len;
          } else {
            const auto new_end =
                start + (end - start) * (curr_segment_len / line_len);
            collect_line(start, new_end);
            points.push_back(new_end);
            break;
          }
        }

        // Skip gap.
        double curr_gap_len = gap;
        while (curr_gap_len > 0.0) {
          const auto start = points.back();
          points.pop_back();
          const auto end = points.back();
          const auto line_len = (start - end).norm();
          if (line_len <= curr_gap_len) {
            curr_gap_len -= line_len;
          } else {
            points.push_back(start + (end - start) * (curr_gap_len / line_len));
            break;
          }
        }
      }
      break;
    }
    default:
      break;
  }

  for (const auto level_id : lane_boundary_proto.belonging_levels()) {
    lane_boundaries_[level_id].push_back(lane_boundary);
  }
}

void QViewPublisherModule::CollectLaneBoundaries() {
  const auto &semantic_map = semantic_map_manager_.semantic_map();
  for (const auto &lane_boundary : semantic_map.lane_boundaries()) {
    if (lane_boundary.type() == mapping::LaneBoundaryProto::CURB) {
      continue;
    }
    CollectLaneBoundary(lane_boundary);
  }
}

void QViewPublisherModule::RenderLaneBoundaries(const Box2d &map_region,
                                                QViewElements *elements) const {
  SCOPED_QTRACE("QViewPublisherModule::RenderLaneBoundaries");

  auto *lane_boundaries = FindOrNull(lane_boundaries_, current_level_);
  if (!lane_boundaries) return;
  for (const auto &lane_boundary : *lane_boundaries) {
    if (!InMapRegion(map_region, lane_boundary.points)) {
      continue;
    }

    int number = 0;
    for (const auto &poly : lane_boundary.polygons) {
      auto *polygon = elements->add_polygons();
      polygon->set_color(QViewPolygonProto::WHITE);
      polygon->set_height_cm(RoundToInt(kLineHeight) * 1e2);
      polygon->set_is_map_element(true);
      polygon->set_polygon_id(std::to_string(lane_boundary.id) + "_" +
                              std::to_string(number));
      polygon->mutable_points()->Reserve(poly.size());
      for (const auto &p : poly) {
        auto *point = polygon->add_points();
        point->set_x_cm(RoundToInt(p.x() * 1e2));
        point->set_y_cm(RoundToInt(p.y() * 1e2));
      }
      number++;
    }
  }
}

void QViewPublisherModule::CollectCrosswalks(
    int level_id, const std::vector<mapping::LaneProto> &lanes,
    std::vector<CrosswalkPolygon> &crosswalk_polygons) {  // NOLINT
  // Try to merge crosswalks that belong to the same physical one.
  bool done = false;
  while (!done) {
    done = true;
    for (int i = 0; i < crosswalk_polygons.size(); ++i) {
      auto &p1 = crosswalk_polygons[i].polygon;
      for (int j = i + 1; j < crosswalk_polygons.size(); ++j) {
        const auto &p2 = crosswalk_polygons[j].polygon;
        if ((p1.points()[0] - p2.points()[0]).squaredNorm() > Sqr(300.0)) {
          continue;
        }
        double dist = std::numeric_limits<double>::infinity();
        for (const auto &point_1 : p1.points()) {
          for (const auto &point_2 : p2.points())
            dist = std::min(dist, (point_1 - point_2).squaredNorm());
        }
        dist = std::sqrt(dist);
        if (dist < 0.5) {
          // Only pick four corners to form the new crosswalk.
          std::vector<Vec2d> all_points = p1.points();
          all_points.insert(all_points.end(), p2.points().begin(),
                            p2.points().end());
          std::vector<Vec2d> new_points;
          for (int k = 0; k < all_points.size(); ++k) {
            bool has_neighbor_point = false;
            for (int l = 0; l < all_points.size(); ++l) {
              if (k != l &&
                  (all_points[k] - all_points[l]).squaredNorm() < Sqr(0.5)) {
                has_neighbor_point = true;
                break;
              }
            }
            if (!has_neighbor_point) {
              new_points.push_back(all_points[k]);
            }
          }

          // Merge those two polygons.
          Polygon2d new_polygon;
          Polygon2d::ComputeConvexHull(new_points, &new_polygon);

          // Check if the area of the merged polygon is similar to the sum of
          // the area of the two polygons.
          const double area_ratio =
              new_polygon.area() / (p1.area() + p2.area());
          if (std::abs(area_ratio - 1.0) < 0.2) {
            // Merge those two polygons.
            p1 = new_polygon;
            crosswalk_polygons.erase(crosswalk_polygons.begin() + j);
            j--;
            done = false;
          }
        }
      }
    }
  }
  // get overlap crosswalk
  std::vector<std::pair<int, int>> crosswalk_cross_relationship;
  for (int i = 0, n = crosswalk_polygons.size(); i < n - 1; ++i) {
    const auto &first_polygon = crosswalk_polygons[i].polygon;
    for (int j = i + 1; j < n; ++j) {
      const auto &second_polygon = crosswalk_polygons[j].polygon;
      if (first_polygon.HasOverlap(second_polygon))
        crosswalk_cross_relationship.emplace_back(i, j);
    }
  }
  for (const auto &crosswalk : crosswalk_polygons) {
    CollectCrosswalk(level_id, lanes, crosswalk.id, crosswalk.polygon);
  }
  CHECK_EQ(crosswalk_polygons.size(), crosswalks_[level_id].size());
  // merge two cross crosswalk's cross rendered line segment
  const auto merge_if_required = [&](int first_crosswalk_index,
                                     int second_crosswalk_index) {
    // first polygon ABCD
    const auto &first_polygon =
        crosswalk_polygons[first_crosswalk_index].polygon;
    // second polygon EFGH
    const auto &second_polygon =
        crosswalk_polygons[second_crosswalk_index].polygon;
    int64_t first_crosswalk_id = crosswalk_polygons[first_crosswalk_index].id;
    int64_t second_crosswalk_id = crosswalk_polygons[second_crosswalk_index].id;
    const std::string merge_rendered_line_segment_debug_string =
        absl::StrFormat("Crosswalk %d and %d:", first_crosswalk_id,
                        second_crosswalk_id);
    // overlap polygon PBQH
    Polygon2d overlap_polygon;
    first_polygon.ComputeOverlap(second_polygon, &overlap_polygon);
    if (overlap_polygon.points().empty()) {
      VLOG(1) << merge_rendered_line_segment_debug_string
              << "overlap polygon point is empty";
      return false;
    }
    auto &first_rendered_line_segments =
        crosswalks_[level_id][first_crosswalk_index].polygons;
    auto &second_rendered_line_segments =
        crosswalks_[level_id][second_crosswalk_index].polygons;
    if (first_rendered_line_segments.empty() ||
        second_rendered_line_segments.empty()) {
      LOG(ERROR) << merge_rendered_line_segment_debug_string
                 << "crosswalk does not have line segments to render";
      return false;
    }
    // Modify the storage order between
    // the rendered line segments of the crosswalk
    if (first_rendered_line_segments.front().empty() ||
        second_rendered_line_segments.front().empty()) {
      LOG(ERROR) << merge_rendered_line_segment_debug_string
                 << "crosswalk first rendered line segment not have vertex";
      return false;
    }
    if (!overlap_polygon.HasOverlap(
            Polygon2d(first_rendered_line_segments.front()))) {
      std::reverse(first_rendered_line_segments.begin(),
                   first_rendered_line_segments.end());
    }
    if (!overlap_polygon.HasOverlap(
            Polygon2d(second_rendered_line_segments.front()))) {
      std::reverse(second_rendered_line_segments.begin(),
                   second_rendered_line_segments.end());
    }
    // Update the point data of each rendered line segment.
    // Take quadrilateral as an example.
    // D_____________________C
    // |                     |
    // |      first     H____|Q__G
    // |________________|____|   |
    // A               P|    B   |
    //                  |        |
    //                  | second |
    //                  |        |
    //                  |________|
    //                  E        F
    const auto get_point_in_polygon_indices =
        [](const Polygon2d &first_polygon, const Polygon2d &second_polygon) {
          const auto &first_polygon_points = first_polygon.points();
          std::vector<int> point_in_polygon_indices;
          for (int m = 0; m < first_polygon_points.size(); ++m) {
            const auto &first_polygon_point = first_polygon_points[m];
            if (second_polygon.IsPointIn(first_polygon_point))
              point_in_polygon_indices.push_back(m);
          }
          return point_in_polygon_indices;
        };

    // point_in_polygon_indices.size() = 0 or 1
    const auto get_projection_points =
        [](const Polygon2d &polygon,
           const std::vector<int> &point_in_polygon_indices,
           const Vec2d &point) {
          const auto &polygon_points = polygon.points();
          if (point_in_polygon_indices.empty()) {
            Vec2d first_point, second_point;
            if ((polygon_points[0] - polygon_points[1]).squaredNorm() <
                (polygon_points[1] - polygon_points[2]).squaredNorm()) {
              first_point = polygon_points[0];
              second_point = polygon_points[1];
            } else {
              first_point = polygon_points[1];
              second_point = polygon_points[2];
            }
            const double dot_product =
                (second_point - first_point).dot(point - first_point);
            constexpr double kProjectionThreshold = 0.5;
            if (dot_product / (second_point - first_point).squaredNorm() >
                kProjectionThreshold)
              return std::make_pair(first_point, second_point);
            return std::make_pair(second_point, first_point);
          }
          int point_index = point_in_polygon_indices.front();
          int left_index =
              (point_index - 1 + polygon_points.size()) % polygon_points.size();
          int right_index = (point_index + 1) % polygon_points.size();
          const Vec2d left_vector =
              polygon_points[left_index] - polygon_points[point_index];
          const Vec2d right_vector =
              polygon_points[right_index] - polygon_points[point_index];
          int other_point_index =
              left_vector.squaredNorm() > right_vector.squaredNorm()
                  ? right_index
                  : left_index;
          return std::make_pair(polygon_points[other_point_index],
                                polygon_points[point_index]);
        };

    const std::vector<int> first_point_in_polygon_indices =
        get_point_in_polygon_indices(first_polygon, second_polygon);
    const std::vector<int> second_point_in_polygon_indices =
        get_point_in_polygon_indices(second_polygon, first_polygon);
    Vec2d B, C, G, H;
    std::pair<Vec2d, Vec2d> first_point_pair, second_point_pair;
    if (0 == first_point_in_polygon_indices.size() &&
        0 == second_point_in_polygon_indices.size()) {
      auto it = first_rendered_line_segments.begin();
      for (; it != first_rendered_line_segments.end();) {
        if (it->size() < 3) continue;
        if (overlap_polygon.HasOverlap(Polygon2d(*it))) {
          it = first_rendered_line_segments.erase(it);
        } else {
          ++it;
        }
      }
      return true;
    } else if (1 == first_point_in_polygon_indices.size() &&
               1 == second_point_in_polygon_indices.size()) {
      Vec2d point_none;
      first_point_pair = get_projection_points(
          first_polygon, first_point_in_polygon_indices, point_none);
      second_point_pair = get_projection_points(
          second_polygon, second_point_in_polygon_indices, point_none);
    } else if (1 == (first_point_in_polygon_indices.size() +
                     second_point_in_polygon_indices.size())) {
      const auto &in_polygon_point =
          0 == first_point_in_polygon_indices.size()
              ? second_polygon.points()[second_point_in_polygon_indices[0]]
              : first_polygon.points()[first_point_in_polygon_indices[0]];
      const Vec2d point =
          (overlap_polygon.points()[0] - in_polygon_point).squaredNorm() < 1E-6
              ? overlap_polygon.points()[1]
              : overlap_polygon.points()[0];
      first_point_pair = get_projection_points(
          first_polygon, first_point_in_polygon_indices, point);
      second_point_pair = get_projection_points(
          second_polygon, second_point_in_polygon_indices, point);
    } else {
      LOG(ERROR) << merge_rendered_line_segment_debug_string
                 << "This requires looking at the street view map";
      return false;
    }
    C = first_point_pair.first;
    B = first_point_pair.second;
    G = second_point_pair.first;
    H = second_point_pair.second;
    for (int i = 0; i < first_rendered_line_segments.size(); ++i) {
      for (int j = 0; j < second_rendered_line_segments.size(); ++j) {
        if (first_rendered_line_segments[i].size() < 3 ||
            second_rendered_line_segments[j].size() < 3) {
          LOG(WARNING) << merge_rendered_line_segment_debug_string
                       << "long polygon points less than three";
          continue;
        }
        Polygon2d first_rendered_line_segment(first_rendered_line_segments[i]);
        Polygon2d second_rendered_line_segment(
            second_rendered_line_segments[j]);
        Polygon2d rendered_line_segment_overlap_polygon;
        bool is_overlap = first_rendered_line_segment.ComputeOverlap(
            second_rendered_line_segment,
            &rendered_line_segment_overlap_polygon);
        if (is_overlap) {
          Polygon2d first_offset_polygon;
          Polygon2d second_offset_polygon;

          const auto get_offset_polygon = [](const Polygon2d &polygon,
                                             Polygon2d *offset_polygon) {
            if (offset_polygon && polygon.points().size() > 3) {
              std::vector<Vec2d> polygon_points;
              polygon_points.reserve(polygon.points().size());
              if ((polygon.points()[0] - polygon.points()[1]).squaredNorm() <
                  (polygon.points()[1] - polygon.points()[2]).squaredNorm()) {
                polygon_points.insert(polygon_points.begin(),
                                      polygon.points().begin() + 1,
                                      polygon.points().end());
                polygon_points.push_back(polygon.points().front());
              } else {
                polygon_points = polygon.points();
              }
              // offset polygon
              polygon_points[0] = 2 * polygon_points[0] - polygon_points[1];
              polygon_points[1] = 2 * polygon_points[1] - polygon_points[0];
              polygon_points[3] = 2 * polygon_points[3] - polygon_points[2];
              polygon_points[2] = 2 * polygon_points[2] - polygon_points[3];
              (*offset_polygon) = Polygon2d(polygon_points);
            }
          };

          get_offset_polygon(first_rendered_line_segment,
                             &first_offset_polygon);
          get_offset_polygon(second_rendered_line_segment,
                             &second_offset_polygon);
          if (!first_offset_polygon.is_convex() ||
              !second_offset_polygon.is_convex()) {
            LOG(ERROR) << merge_rendered_line_segment_debug_string
                       << "offset_polygon is not convex";
            continue;
          }
          first_offset_polygon.ComputeOverlap(
              second_offset_polygon, &rendered_line_segment_overlap_polygon);

          auto get_two_smaller_projection_point =
              [](const Vec2d &first_point, const Vec2d &second_point,
                 const std::vector<Vec2d> &polygon,
                 std::vector<Vec2d> *update_polygon) {
                if (!update_polygon) return;
                const Vec2d first_second_vector = second_point - first_point;
                std::multimap<double, int> points_projection_values;
                for (int k = 0; k < polygon.size(); ++k) {
                  const Vec2d first_current_vector = polygon[k] - first_point;
                  double projection_value =
                      std::fabs(first_second_vector.Dot(first_current_vector));
                  points_projection_values.emplace(projection_value, k);
                }
                int first_point_index =
                    points_projection_values.begin()->second;
                int second_point_index =
                    (++points_projection_values.begin())->second;
                update_polygon->push_back(polygon[first_point_index]);
                update_polygon->push_back(polygon[second_point_index]);
              };

          std::vector<Vec2d> first_update_polygon;
          std::vector<Vec2d> second_update_polygon;
          get_two_smaller_projection_point(
              C, B, first_rendered_line_segments[i], &first_update_polygon);
          get_two_smaller_projection_point(
              B, C, rendered_line_segment_overlap_polygon.points(),
              &first_update_polygon);
          const auto first_update =
              Polygon2d::ComputeConvexHullPoints(first_update_polygon);
          const bool first_update_success = first_update.has_value();
          if (first_update_success) {
            first_rendered_line_segments[i] = std::move(*first_update);
          }
          get_two_smaller_projection_point(
              G, H, second_rendered_line_segments[j], &second_update_polygon);
          get_two_smaller_projection_point(
              H, G, rendered_line_segment_overlap_polygon.points(),
              &second_update_polygon);
          const auto second_update =
              Polygon2d::ComputeConvexHullPoints(second_update_polygon);
          const bool second_update_success = second_update.has_value();
          if (second_update_success) {
            second_rendered_line_segments[j] = std::move(*second_update);
          }
          if (!first_update_success || !second_update_success) {
            LOG(ERROR) << merge_rendered_line_segment_debug_string
                       << "merge rendered line segment fail";
            return false;
          }
        }
      }
    }
    return true;
  };
  for (const auto &[first_index, second_index] : crosswalk_cross_relationship) {
    merge_if_required(first_index, second_index);
  }
}

void QViewPublisherModule::CollectCrosswalk(
    int level_id, const std::vector<mapping::LaneProto> &lanes, int64_t id,
    const Polygon2d &crosswalk_polygon) {
  CHECK_GT(crosswalk_polygon.num_points(), 2);
  const std::vector<Vec2d> &crosswalk_points = crosswalk_polygon.points();

  Crosswalk crosswalk;
  crosswalk.points = crosswalk_points;
  crosswalk.id = id;

  std::vector<Segment2d> crosswalk_polyline;
  for (int i = 0; i < crosswalk_points.size() - 1; ++i) {
    crosswalk_polyline.push_back(
        Segment2d({crosswalk_points[i].x(), crosswalk_points[i].y()},
                  {crosswalk_points[i + 1].x(), crosswalk_points[i + 1].y()}));
  }
  crosswalk_polyline.push_back(
      Segment2d({crosswalk_points[crosswalk_points.size() - 1].x(),
                 crosswalk_points[crosswalk_points.size() - 1].y()},
                {crosswalk_points[0].x(), crosswalk_points[0].y()}));

  // The number of points where every crosswalk polyline and lanes intersect.
  std::vector<int> num_intersect(crosswalk_polyline.size(), 0);

  for (const auto &lane : lanes) {
    std::vector<Vec2d> lane_points;
    for (const auto &point : lane.polyline().points()) {
      lane_points.push_back(
          GlobalToLocal({point.longitude(), point.latitude()}));
    }
    for (int i = 0; i < lane_points.size() - 1; ++i) {
      for (int j = 0; j < crosswalk_polyline.size(); ++j) {
        bool is_intersect = crosswalk_polyline[j].HasIntersect(
            Segment2d(lane_points[i], lane_points[i + 1]));
        if (is_intersect) {
          num_intersect[j]++;
        }
      }
    }
  }

  // Maximum number of num_intersect and line in crosswalk polyline which has
  // maximum num_intersect. If two polylines have the same number of
  // num_intersect, choose the longer one.
  int max_intersect_polyline_index = 0;
  int max_intersect_polyline = num_intersect[0];
  for (int i = 1; i < num_intersect.size(); ++i) {
    if (num_intersect[i] > max_intersect_polyline) {
      max_intersect_polyline_index = i;
      max_intersect_polyline = num_intersect[i];
    } else if (num_intersect[i] == max_intersect_polyline) {
      if (crosswalk_polyline[i].length() >
          crosswalk_polyline[max_intersect_polyline_index].length()) {
        max_intersect_polyline_index = i;
        max_intersect_polyline = num_intersect[i];
      }
    }
  }

  // Calculate the slope of the crosswalk line.
  const double left_index =
      (max_intersect_polyline_index - 1 + crosswalk_polyline.size()) %
      crosswalk_polyline.size();
  const double right_index =
      (max_intersect_polyline_index + 1) % crosswalk_polyline.size();
  const auto vector_to_slope = [](const Vec2d vector_value) {
    if (std::fabs(vector_value.x()) < std::numeric_limits<double>::epsilon()) {
      if (vector_value.y() > 0) {
        return std::numeric_limits<double>::infinity();
      } else if (vector_value.y() < 0) {
        return -std::numeric_limits<double>::infinity();
      } else {
        return 0.0;
      }
    } else {
      return vector_value.y() / vector_value.x();
    }
  };
  const auto average_vector = crosswalk_polyline[left_index].unit_direction() -
                              crosswalk_polyline[right_index].unit_direction();
  const double k_crosswalklines_found = vector_to_slope(average_vector);
  CollectCrosswalkLines(crosswalk_polyline, max_intersect_polyline_index,
                        /*is_k_found=*/true, k_crosswalklines_found,
                        &crosswalk);
  crosswalks_[level_id].push_back(crosswalk);
}

void QViewPublisherModule::RenderCrosswalks(
    const mapping::SemanticMapProto &semantic_map, const Box2d &map_region,
    QViewElements *elements) const {
  auto *crosswalks = FindOrNull(crosswalks_, current_level_);
  if (!crosswalks) return;
  for (const auto &crosswalk : *crosswalks) {
    if (!InMapRegion(map_region, crosswalk.points)) {
      continue;
    }

    int number = 0;
    for (const auto &poly : crosswalk.polygons) {
      auto *polygon = elements->add_polygons();
      polygon->set_color(QViewPolygonProto::WHITE);
      polygon->set_height_cm(RoundToInt(kLineHeight) * 1e2);
      polygon->set_is_map_element(true);
      polygon->set_polygon_id(std::to_string(crosswalk.id) + "_" +
                              std::to_string(number));
      polygon->mutable_points()->Reserve(poly.size());
      for (const auto &p : poly) {
        auto *point = polygon->add_points();
        point->set_x_cm(RoundToInt(p.x() * 1e2));
        point->set_y_cm(RoundToInt(p.y() * 1e2));
      }
      number++;
    }
  }
}

void QViewPublisherModule::RenderOffroadZone(
    int32 id, const absl::Span<const Vec2d> local_curb,
    QViewElements *elements) const {
  auto *polygon = elements->add_polygons();
  polygon->set_color(QViewPolygonProto::DIMGRAY);
  polygon->set_height_cm(10);
  polygon->set_is_map_element(true);
  // TODO(liying): Modify the strategy of drawing curb
  // to ensure that a curb has only one polygon paramete.
  polygon->set_polygon_id(absl::StrCat(id, "_", current_level_));
  polygon->mutable_points()->Reserve(local_curb.size());
  for (int i = local_curb.size() - 1; i >= 0; --i) {
    auto *point = polygon->add_points();
    point->set_x_cm(RoundToInt(local_curb[i].x() * 1e2));
    point->set_y_cm(RoundToInt(local_curb[i].y() * 1e2));
  }
}

void QViewPublisherModule::RenderBuilding(
    int32 id, const std::vector<Vec2d> &building_polygon, double height,
    QViewElements *elements) const {
  auto *polygon = elements->add_polygons();
  polygon->set_color(QViewPolygonProto::DIMGRAY);
  polygon->set_height_cm(RoundToInt(height) * 1e2);
  polygon->set_is_map_element(true);
  polygon->set_polygon_id(std::to_string(id));
  polygon->mutable_points()->Reserve(building_polygon.size());
  for (int i = 0; i < building_polygon.size(); ++i) {
    auto *point = polygon->add_points();
    const auto local_point =
        GlobalToLocal({building_polygon[i].x(), building_polygon[i].y()});
    point->set_x_cm(RoundToInt(local_point.x() * 1e2));
    point->set_y_cm(RoundToInt(local_point.y() * 1e2));
  }
}

void QViewPublisherModule::RenderBusStation(Vec2d pos_global, double heading,
                                            uint32_t height,
                                            QViewElements *elements) const {
  auto *bus_station = elements->add_bus_stations();
  const auto local_point = GlobalToLocal({pos_global.x(), pos_global.y()});
  bus_station->set_x_cm(RoundToInt(local_point.x() * 1e2));
  bus_station->set_y_cm(RoundToInt(local_point.y() * 1e2));
  bus_station->set_z_cm(height);
  bus_station->set_heading(GlobalYawToLocal(heading));
}

void QViewPublisherModule::RenderPickUpPoint(Vec2d pos_global, double heading,
                                             mapping::ElementId id,
                                             QViewElements *elements) const {
  auto *pick_up_point = elements->add_pick_up_points();
  const auto local_point = GlobalToLocal({pos_global.x(), pos_global.y()});
  pick_up_point->set_id(std::to_string(id));
  pick_up_point->set_heading(GlobalYawToLocal(heading));
  pick_up_point->set_x_cm(RoundToInt(local_point.x() * 1e2));
  pick_up_point->set_y_cm(RoundToInt(local_point.y() * 1e2));
}

void QViewPublisherModule::CollectCrosswalkLines(
    const std::vector<Segment2d> &crosswalk_polyline,
    int max_intersect_polyline_index, bool is_k_found,
    double k_crosswalklines_found, Crosswalk *crosswalk) {
  if (max_intersect_polyline_index > crosswalk_polyline.size() - 1) {
    return;
  }

  const double slope_sign = k_crosswalklines_found > 0 ? 1 : -1;
  const auto collect_line = [&](Vec2d start, Vec2d end) {
    const Box2d box(Segment2d(start, end), kCrosswalkLineWidth);
    crosswalk->polygons.push_back(box.GetCornersCounterClockwise());
  };

  Vec2d point_start = crosswalk_polyline[max_intersect_polyline_index].start();
  Vec2d point_end = crosswalk_polyline[max_intersect_polyline_index].end();
  double polyline_length =
      crosswalk_polyline[max_intersect_polyline_index].length();
  double k_crosswalklines;
  if (is_k_found) {
    k_crosswalklines = k_crosswalklines_found;
  } else {
    // k_perpendicularline is the slope of line perpendicular to
    // crosswalk_polyline[max_intersect_polyline_index]
    double k_perpendicularline =
        (point_start.x() - point_end.x()) / (point_end.y() - point_start.y());
    k_crosswalklines = k_perpendicularline;
  }

  double pow_k = Sqr(k_crosswalklines);
  double step_width = 0;

  // Render lines perpendicular to crosswalk polyline from crosswalk polyline
  // start point to end point.
  while (step_width < polyline_length + kCrosswalkLineGapWidth) {
    // Find a line which is perpendicular to crosswalk polyline. Length of
    // perpendicular line is kDistanceSet.
    // First point of perpendicular line.
    Vec2d point_base =
        (polyline_length - step_width) / polyline_length * point_start +
        step_width / polyline_length * point_end;

    // Second point of perpendicular line.
    Vec2d point_in_perpendicularline(
        point_base.x() + kDistanceSet * std::sqrt(1 / (1 + pow_k)),
        point_base.y() +
            kDistanceSet * slope_sign * std::sqrt(1 / (1 + 1 / pow_k)));

    bool has_intersect = false;
    for (int i = 0; i < crosswalk_polyline.size(); ++i) {
      if (i == max_intersect_polyline_index) {
        continue;
      }
      Vec2d start_i, end_i;
      // Extend crosswalk polyline to fill the gap between crosswalk lines and
      // curb.
      if (!crosswalk_polyline[i].HasIntersect(
              crosswalk_polyline[max_intersect_polyline_index])) {
        constexpr double kExtendLength = 0.1;  // m
        const Vec2d start_to_end_delta =
            kExtendLength * crosswalk_polyline[i].unit_direction();
        start_i = crosswalk_polyline[i].start() - start_to_end_delta;
        end_i = crosswalk_polyline[i].end() + start_to_end_delta;
      } else {
        start_i = crosswalk_polyline[i].start();
        end_i = crosswalk_polyline[i].end();
      }

      Vec2d point_end;
      has_intersect =
          Segment2d(start_i, end_i)
              .GetIntersect(Segment2d(point_base, point_in_perpendicularline),
                            &point_end);
      if (has_intersect) {
        // When step_width is 0, one intersect point is a crosswalk point
        // which is not the intersect we want.
        if (step_width == 0 &&
            Segment2d(point_base, point_end).length() < 0.001) {
          has_intersect = false;
          continue;
        }
        // draw line
        collect_line(point_base, point_end);
        step_width += kCrosswalkLineGapWidth;
        break;
      }
    }
    if (has_intersect) {
      continue;
    }

    // Find another line whose heading is opposite to the first perpendicular
    // line.
    // Second point of perpendicular line.
    Vec2d point_in_perpendicularline_2(
        point_base.x() - kDistanceSet * std::sqrt(1 / (1 + pow_k)),
        point_base.y() -
            kDistanceSet * slope_sign * std::sqrt(1 / (1 + 1 / pow_k)));

    for (int i = 0; i < crosswalk_polyline.size(); ++i) {
      if (i == max_intersect_polyline_index) {
        continue;
      }
      Vec2d point_end;
      has_intersect = crosswalk_polyline[i].GetIntersect(
          Segment2d(point_base, point_in_perpendicularline_2), &point_end);
      if (has_intersect) {
        // When step_width is 0, one intersect point is a crosswalk point
        // which is not the intersect we want.
        if (step_width == 0 &&
            Segment2d(point_base, point_end).length() < 0.001) {
          has_intersect = false;
          continue;
        }
        // draw line
        collect_line(point_base, point_end);
        step_width += kCrosswalkLineGapWidth;
        break;
      }
    }
    if (!has_intersect) {
      step_width += kCrosswalkLineGapWidth;
    }
  }
}

void QViewPublisherModule::RenderOffroadZones(const Box2d &map_region,
                                              QViewElements *elements) const {
  SCOPED_QTRACE("QViewPublisherModule::RenderOffroadZones");

  const auto *curbs = FindOrNull(curbs_, current_level_);
  if (!curbs) return;
  for (const auto &curb : *curbs) {
    if (!InMapRegion(map_region, curb.points)) {
      continue;
    }
    int num_points = curb.points.size();
    if (curb.points.front() == curb.points.back()) {
      num_points--;
    }
    RenderOffroadZone(curb.id, absl::Span(curb.points.data(), num_points),
                      elements);
  }
}

void QViewPublisherModule::RenderBuildings(const Box2d &map_region,
                                           QViewElements *elements) const {
  auto *buildings = FindOrNull(buildings_, current_level_);
  if (!buildings) return;
  for (auto &building : *buildings) {
    bool to_display = false;
    for (const auto &point_global : building.polygon_global) {
      if (map_region.IsPointIn(GlobalToLocal(point_global))) {
        to_display = true;
        break;
      }
    }
    if (!to_display) continue;

    RenderBuilding(building.id, building.polygon_global, building.height,
                   elements);
  }
}

void QViewPublisherModule::RenderBusStations(const Box2d &map_region,
                                             QViewElements *elements) const {
  auto *bus_stations = FindOrNull(bus_stations_, current_level_);
  if (!bus_stations) return;
  for (const auto &station : *bus_stations) {
    if (map_region.IsPointIn(GlobalToLocal(station.pos_global))) {
      RenderBusStation(station.pos_global, station.heading, station.height,
                       elements);
    }
  }
}

void QViewPublisherModule::RenderPickUpPoints(const Box2d &map_region,
                                              QViewElements *elements) const {
  SCOPED_QTRACE("QViewPublisherModule::RenderPickUpPoints");

  auto *pick_up_points = FindOrNull(pick_up_points_, current_level_);
  if (!pick_up_points) return;
  for (const auto &pick_up_point : *pick_up_points) {
    if (map_region.IsPointIn(GlobalToLocal(pick_up_point.pos_global))) {
      RenderPickUpPoint(pick_up_point.pos_global, pick_up_point.heading,
                        pick_up_point.id, elements);
    }
  }
}

void QViewPublisherModule::GetPedCycAndOffroadPoints(
    const double max_obj_time,
    absl::flat_hash_map<std::string, std::pair<Vec2d, std::vector<Vec3f>>>
        *points_with_ids) const {
  SCOPED_QTRACE("QViewPublisherModule::GetPedCycAndOffroadPoints");

  if (points_with_ids == nullptr) return;

  // Find all row/cols that belong to ped/cyclists.
  constexpr double kCoordToRCScale = 1 / 0.2;
  absl::flat_hash_map<std::pair<int, int>, CalibratedReturnPtrs> points_grid;
  absl::flat_hash_map<std::pair<int, int>, const Obstacle *> obstacles_grid;
  for (const auto &[_, objects] : all_objects_) {
    if (objects.empty()) continue;
    const auto &object = objects.back();
    if (!object.has_bounding_box()) continue;
    if (object.type() == OT_VEHICLE) continue;
    // Reduce search scope.
    const Box2d aabb = GetObjectAxisAlignedBoundingBox(object);
    for (int row = (aabb.center().x() - aabb.half_length()) * kCoordToRCScale;
         row <= (aabb.center().x() + aabb.half_length()) * kCoordToRCScale;
         ++row) {
      for (int col = (aabb.center().y() - aabb.half_width()) * kCoordToRCScale;
           col <= (aabb.center().y() + aabb.half_width()) * kCoordToRCScale;
           ++col) {
        points_grid[{row, col}];
        obstacles_grid[{row, col}] = nullptr;
      }
    }
  }

  points_with_ids->reserve(all_objects_.size());
  constexpr float kMaxPedCycHeight = 2.0f;         // m
  constexpr float kGroundZErrorBuffer = 0.1f;      // m
  constexpr float kGroundZNearCurbBuffer = 0.25f;  // m
  constexpr float kFrontEndModelHeight = 0.25f;    // m
  // Get lidar frames.
  std::vector<LidarFrame> lidar_use_frames;
  lidar_use_frames.reserve(6);
  double min_lidar_start_timestamp = std::numeric_limits<double>::max();
  for (const auto &[lidar_id, lidar_frames] : lidar_frame_buffer_) {
    for (const auto &lidar_frame : lidar_frames) {
      // NOTE(zhenye): we assume that objects message is published 0.1
      // second later than lidar message.
      if (max_obj_time - lidar_frame.StartTimestamp() > 0.0 &&
          max_obj_time - lidar_frame.StartTimestamp() < 0.1) {
        lidar_use_frames.emplace_back(lidar_frame);
        min_lidar_start_timestamp =
            std::min(min_lidar_start_timestamp, lidar_frame.StartTimestamp());
      }
    }
  }
  constexpr float kMaxUsingPointRange = 60.f;
  for (const auto &lidar_frame : lidar_use_frames) {
    if (!lidar_frame.is_spin()) continue;
    const auto *spin = CHECK_NOTNULL(lidar_frame.spin());
    for (const auto &scan : *spin) {
      for (const auto &shot : scan) {
        if (shot.num_returns == 0) continue;
        const auto &calib_ret = shot.calibrated_returns[0];
        if (calib_ret.range > kMaxUsingPointRange) continue;
        const int row = FloorToInt(calib_ret.x * kCoordToRCScale);
        const int col = FloorToInt(calib_ret.y * kCoordToRCScale);
        if (auto *points = FindOrNull(points_grid, {row, col})) {
          if (points->empty()) points->reserve(32);
          points->emplace_back(&calib_ret);
        }
      }
    }
  }
  // Remapping obstacles to rc.
  if (const auto *all_obstacles = obstacles_buffer_.GetValueWithTimeAtLeast(
          min_lidar_start_timestamp)) {
    for (const auto &obstacle : *all_obstacles) {
      const int row = FloorToInt(obstacle.x * kCoordToRCScale);
      const int col = FloorToInt(obstacle.y * kCoordToRCScale);
      if (auto *obs = FindOrNull(obstacles_grid, {row, col})) {
        *obs = &obstacle;
      }
    }
  }

  for (const auto &[_, objects] : all_objects_) {
    if (objects.empty()) continue;
    const auto &object = objects.back();
    if (!object.has_bounding_box()) continue;
    if (object.type() == OT_VEHICLE) continue;
    // Reduce search scope.
    const Box2d aabb = GetObjectAxisAlignedBoundingBox(object);
    const int max_use_size = (aabb.length() * kCoordToRCScale + 1) *
                             (aabb.width() * kCoordToRCScale + 1);
    std::vector<CalibratedReturnPtrs> points_grid_use;
    points_grid_use.reserve(max_use_size);
    std::vector<const Obstacle *> obstacles_grid_use;
    obstacles_grid_use.reserve(max_use_size);
    for (int row = (aabb.center().x() - aabb.half_length()) * kCoordToRCScale;
         row <= (aabb.center().x() + aabb.half_length()) * kCoordToRCScale;
         ++row) {
      for (int col = (aabb.center().y() - aabb.half_width()) * kCoordToRCScale;
           col <= (aabb.center().y() + aabb.half_width()) * kCoordToRCScale;
           ++col) {
        if (const auto *obstacles_use =
                FindOrNull(obstacles_grid, {row, col})) {
          if (*obstacles_use != nullptr) {
            obstacles_grid_use.emplace_back(*obstacles_use);
          }
        }
        if (const auto *points_use = FindOrNull(points_grid, {row, col})) {
          points_grid_use.emplace_back(*points_use);
        }
      }
    }

    float min_ground_z = std::numeric_limits<float>::max();
    float max_dist_to_curb = std::numeric_limits<float>::lowest();
    Box2d obj_box(object.bounding_box());
    for (const auto *obs : obstacles_grid_use) {
      if (!obj_box.IsPointIn(Vec2d{obs->x, obs->y})) {
        continue;
      }
      min_ground_z = std::min(min_ground_z, obs->ground_z);
      max_dist_to_curb = std::max(max_dist_to_curb, obs->dist_to_curb);
    }
    const float z_buffer = std::abs(max_dist_to_curb) < 1.0f
                               ? kGroundZNearCurbBuffer
                               : kGroundZErrorBuffer;
    auto &info = (*points_with_ids)[object.id()];
    info.first = object.has_pos() ? Vec2d(object.pos().x(), object.pos().y())
                                  : Vec2d(object.bounding_box().x(),
                                          object.bounding_box().y());
    for (const auto &points : points_grid_use) {
      if (points.empty()) continue;
      Vec2d pt(points.front()->x, points.front()->y);
      if (!obj_box.IsPointIn(pt)) {
        continue;
      }
      for (const auto *p : points) {
        if (p->z < min_ground_z + z_buffer ||
            p->z - min_ground_z > kMaxPedCycHeight) {
          continue;
        }
        info.second.emplace_back(
            Vec3f{p->x, p->y, p->z - min_ground_z + kFrontEndModelHeight});
      }
    }
  }
}

void QViewPublisherModule::RenderObjects(double timestamp,
                                         QViewElements *elements) {
  SCOPED_QTRACE_ARG1("QViewPublisherModule::RenderObjects", "num_objects",
                     all_objects_.size());
  static absl::flat_hash_map<std::string, std::pair<Vec2d, std::vector<Vec3f>>>
      points_with_ids;
  if (FLAGS_publish_points_to_qview) {
    static double max_object_time = 0.0;
    double object_time = 0.0;
    for (const auto &[_, objects] : all_objects_) {
      if (objects.empty()) continue;
      const auto &object = objects.back();
      if (object.has_laser_timestamp()) {
        object_time = std::max(object_time, object.laser_timestamp());
      }
    }
    if (object_time != max_object_time) {
      max_object_time = object_time;
      points_with_ids.clear();
      GetPedCycAndOffroadPoints(object_time, &points_with_ids);
    }
  }

  const auto is_filtered_object = [](const ObjectProto &object) {
    if (!object.has_bounding_box()) return true;
    const auto obj_type = object.type();
    if (obj_type != OT_VEHICLE && obj_type != OT_MOTORCYCLIST &&
        obj_type != OT_CYCLIST && obj_type != OT_PEDESTRIAN) {
      return true;
    }
    // When FEN miss detection, tracker will provide min area box, but its
    // heading is always inaccurate.
    if (obj_type == OT_VEHICLE) {
      const Box2d aabb = GetObjectAxisAlignedBoundingBox(object);
      const double area = aabb.length() * aabb.width();
      const bool f1 = area < kMinVehicleArea;
      const bool f2 = object.bounding_box_source() == ObjectProto::MIN_AREA_BOX;
      return f1 || f2;
    }
    return false;
  };

  for (const auto &[object_id, objects] : all_objects_) {
    const auto &object = objects.back();
    if (is_filtered_object(object)) continue;
    if (!FLAGS_publish_offroad_objects && object.offroad()) continue;

    // TODO(cong, yu): use linear regression to smooth BBs.
    const auto get_pos = [](const auto &object) {
      return object.has_pos()
                 ? Vec2d(object.pos().x(), object.pos().y())
                 : Vec2d(object.bounding_box().x(), object.bounding_box().y());
    };
    // Get predicted pos.
    const auto get_predicted_pos = [&](const auto &object, double timestamp) {
      const auto pos = get_pos(object);
      const double time_diff = timestamp - object.timestamp();
      const Vec2d predicted_pos =
          pos + Vec2d(object.vel().x(), object.vel().y()) * time_diff;
      return predicted_pos;
    };

    // Compute the intepolated and smoothed BB of each object.
    std::unordered_map<ObjectType, int> type_histogram;
    std::vector<double> headings;
    headings.reserve(objects.size());
    double length_sum = 0.0;
    double width_sum = 0.0;
    constexpr double kSmoothWindow = 0.3;  // s
    double avg_speed = 0.0;
    constexpr double kYawFlipThresh = d2r(90.0);
    constexpr double kWinHeadingJumpThresh = d2r(30.0);
    constexpr double kBboxReduceScale = 0.78;
    for (int i = 0; i < objects.size(); ++i) {
      if (is_filtered_object(objects[i])) continue;
      if (std::abs(objects[i].timestamp() - timestamp) < kSmoothWindow) {
        // Flip heading when the heading is opposite to history heading.
        double heading = objects[i].bounding_box().heading();
        all_objects_heading_.emplace(object_id, heading);
        all_objects_pos_.emplace(object_id, get_pos(objects[i]));
        if (std::abs(NormalizeAngle(
                heading - all_objects_heading_[object_id])) > kYawFlipThresh) {
          heading = NormalizeAngle(heading + M_PI);
        }
        double speed = Vec2d(objects[i].vel().x(), objects[i].vel().y()).norm();
        avg_speed += speed;
        headings.push_back(heading);
        width_sum += objects[i].bounding_box().width() * kBboxReduceScale;
        length_sum += objects[i].bounding_box().length() * kBboxReduceScale;
      }
      type_histogram[objects[i].type()]++;
    }
    if (type_histogram.empty()) continue;
    const double width = headings.empty() ? object.bounding_box().width()
                                          : width_sum / headings.size();
    const double length = headings.empty() ? object.bounding_box().length()
                                           : length_sum / headings.size();
    constexpr double kVehicleTypeLengthdivide = 6.5;  // m
    const auto max_ele_iter =
        std::max_element(type_histogram.begin(), type_histogram.end(),
                         [](const auto &lhs, const auto &rhs) {
                           return lhs.second < rhs.second;
                         });

    QViewObjectProto::Type type;
    switch (max_ele_iter->first) {
      case OT_VEHICLE:
        type = length < kVehicleTypeLengthdivide
                   ? QViewObjectProto::VEHICLE
                   : QViewObjectProto::LARGE_VEHICLE;
        break;
      case OT_MOTORCYCLIST:
        type = QViewObjectProto::MOTORCYCLIST;
        break;
      case OT_CYCLIST:
        type = QViewObjectProto::CYCLIST;
        break;
      case OT_PEDESTRIAN:
        type = QViewObjectProto::PEDESTRIAN;
        break;
      default:
        continue;
    }

    // Compute max heading and min heading affter normalize, use the diff
    // to judge if the heading is stable in the window.
    const auto [min_iter, max_iter] =
        std::minmax_element(headings.begin(), headings.end());
    const double heading_diff =
        headings.empty() ? 0.0 : std::fabs(*max_iter - *min_iter);

    int i = std::distance(
        objects.begin(),
        std::lower_bound(objects.begin(), objects.end(), timestamp,
                         [](const ObjectProto &object, double t) {
                           return object.timestamp() < t;
                         }));
    if (i == 0) continue;

    const bool all_objects_stale = (i == objects.size());
    if (all_objects_stale) i--;

    Vec2d center;
    const double obj_speed_sqr =
        Vec2d(objects[i].vel().x(), objects[i].vel().y()).squaredNorm();

    if (all_objects_stale) {
      constexpr double kMaxObjectsTimeDiff = 0.5;  // s
      if (objects[i].timestamp() > timestamp - kMaxObjectsTimeDiff) {
        // Note(zheng): Some small velocity maybe shooters, in that case we
        // use the latest object pos instead of predicted pos.
        if (objects[i].type() == OT_VEHICLE && obj_speed_sqr > Sqr(2.0)) {
          center = get_predicted_pos(objects[i], timestamp);
        } else {
          center = get_pos(objects[i]);
        }
      } else {
        continue;
      }
    } else {
      const auto &center1 = get_pos(objects[i - 1]);
      const auto &center2 = get_pos(objects[i]);
      const double ratio =
          (timestamp - objects[i - 1].timestamp()) /
          (objects[i].timestamp() - objects[i - 1].timestamp());
      center = center2 * ratio + center1 * (1.0 - ratio);
    }

    const double current_obj_heading = object.bounding_box().heading();
    double heading =
        headings.empty() ? current_obj_heading : MeanAngle(headings);

    // BBs overlap judgement.
    // Some frame-crossed objects with different ids are associated if BBs
    // overlapping.
    std::string obj_id = object.id();
    bool is_overlap_obj = false;
    constexpr double kSuitableDistance = 10.0;
    for (const auto &[object_id, objs] : all_objects_) {
      const auto &obj = objs.back();
      if (objects[i].id() == object_id || !obj.has_bounding_box()) {
        continue;
      }
      const auto obj_pos = get_pos(obj);
      if ((center - obj_pos).squaredNorm() > Sqr(kSuitableDistance)) continue;

      // Calculate BBs overlap.
      const Box2d obj_src = GetObjectAxisAlignedBoundingBox(object);
      const Box2d obj_dst(
          {obj_pos.x(), obj_pos.y()}, obj.bounding_box().heading(),
          obj.bounding_box().length(), obj.bounding_box().width());

      if (!obj_src.HasOverlap(obj_dst)) continue;

      const Polygon2d src_region(obj_src);
      const Polygon2d dst_region(obj_dst);
      // Calculate IoU
      Polygon2d intersection;
      if (!src_region.ComputeOverlap(dst_region, &intersection)) {
        continue;
      }
      const double src_area = src_region.area();
      const double dst_area = dst_region.area();
      const double intersection_area = intersection.area();
      if (intersection_area > src_area * 0.5 && src_area < dst_area) {
        is_overlap_obj = true;
      }
      if (src_area < dst_area && intersection_area > src_area * 0.2 &&
          type == QViewObjectProto::VEHICLE && obj_src.length() < 2.5 &&
          obj_src.width() < 2.5) {
        is_overlap_obj = true;
      }
    }
    if (is_overlap_obj) continue;

    // Heavy pos smoothing to avoid moving objects shaking.
    if (const auto kv = all_objects_pos_.emplace(object_id, center);
        !kv.second) {
      auto &prev_pos = kv.first->second;
      if (obj_speed_sqr > Sqr(kDynamicOrStaticVelocity)) {
        center = center * 0.2 + prev_pos * 0.8;
      } else {
        center = center * 0.01 + prev_pos * 0.99;
      }
      prev_pos = center;
    }

    // Note(zhenye): filter perception objects roi
    const auto pose = GetNearestPose(pose_history_, objects[i].timestamp());
    const auto pose_vehicle = pose.ToTransform().Inverse();
    const auto box_vehicle_center =
        pose_vehicle.TransformPoint(Vec3d{center.x(), center.y(), 0});
    if (v2x_objs_.size() > 0) {
      if (box_vehicle_center.x() > kPerceptionRoiFrontValue ||
          std::abs(box_vehicle_center.y()) > kPerceptionRoiSideValue) {
        continue;
      }
    }
    // Fill lidar points.
    if (FLAGS_publish_points_to_qview && type != QViewObjectProto::VEHICLE) {
      std::vector<LaserPoint> object_points;
      // Calculate center coordinate offset.

      if (const auto *obj_points_info =
              FindOrNull(points_with_ids, object_id)) {
        const Vec2d offset = center - obj_points_info->first;
        for (const auto &g_point : obj_points_info->second) {
          const float trans_x = g_point.x() + offset.x();
          const float trans_y = g_point.y() + offset.y();
          const float trans_z = g_point.z();
          const Vec2d local_trans_point =
              SmoothToLocal(Vec2d{trans_x, trans_y});
          auto *new_point = (type == QViewObjectProto::PEDESTRIAN)
                                ? elements->add_ped_points()
                                : elements->add_cyc_points();
          new_point->set_x(-local_trans_point.y());
          new_point->set_y(trans_z);
          new_point->set_z(-local_trans_point.x());
        }
      }
    }

    auto *new_object = elements->add_objects();
    new_object->set_id(obj_id);
    new_object->set_type(type);
    new_object->set_data_src(QViewObjectProto::PERCEPTION);
    auto *bb = new_object->mutable_bounding_box();

    // Smoothing heading by using history heading.
    if (const auto kv = all_objects_heading_.emplace(object_id, heading);
        !kv.second) {
      double &prev_heading = kv.first->second;
      // Add smoothing logic in case of the heading is wrong at the first.
      // If the track is static, and the heading is not stable in the
      // window, there maybe a lot of noise, we don't apply smooth logic in
      // this case.
      if (avg_speed < 1.0 && heading_diff > kWinHeadingJumpThresh) {
        heading = prev_heading;
      }
      if (std::abs(NormalizeAngle(current_obj_heading - prev_heading)) <
          kYawFlipThresh) {
        const double sin_sum =
            0.9 * fast_math::Sin(prev_heading) + 0.1 * fast_math::Sin(heading);
        const double cos_sum =
            0.9 * fast_math::Cos(prev_heading) + 0.1 * fast_math::Cos(heading);
        heading = fast_math::Atan2(sin_sum, cos_sum);
      } else {
        heading = NormalizeAngle(heading + M_PI);
      }
      // Correct heading by velocity heading.
      constexpr double kMinSpeedToCorrectHeadingByMotion = 2.0;
      const double vel_heading =
          fast_math::Atan2(objects[i].vel().y(), objects[i].vel().x());
      const double speed_sqr =
          Vec2d(objects[i].vel().y(), objects[i].vel().x()).squaredNorm();
      if (speed_sqr > Sqr(kMinSpeedToCorrectHeadingByMotion) &&
          std::abs(NormalizeAngle(heading - vel_heading)) > kYawFlipThresh) {
        heading = NormalizeAngle(heading + M_PI);
      }
      prev_heading = heading;
    }

    const Vec2d bb_center_in_vehicle_frame = SmoothToLocal(center);
    bb->set_x(bb_center_in_vehicle_frame.x());
    bb->set_y(bb_center_in_vehicle_frame.y());
    bb->set_heading(SmoothYawToLocal(heading));
    bb->set_width(width);
    bb->set_length(length);
  }
}

void QViewPublisherModule::RenderCones(QViewElements *elements) {
  for (const auto &[obj_id, objects] : all_objects_) {
    const auto &object = objects.back();
    if (object.type() != OT_CONE) continue;
    all_cones_[obj_id] = object;
  }
  for (const auto &[id, obj] : all_cones_) {
    auto *new_object = elements->add_objects();
    new_object->set_id(id);
    new_object->set_type(QViewObjectProto::CONE);
    new_object->set_data_src(QViewObjectProto::PERCEPTION);
    auto *bb = new_object->mutable_bounding_box();
    const Vec2d center_in_vehicle =
        SmoothToLocal(Vec2d(obj.pos().x(), obj.pos().y()));
    bb->set_x(center_in_vehicle.x());
    bb->set_y(center_in_vehicle.y());
    bb->set_heading((0.0));
    bb->set_width(0.2);
    bb->set_length(0.2);
  }
}

void QViewPublisherModule::RenderV2xObjects(QViewElements *elements) {
  if (pose_history_.empty()) {
    return;
  }
  // v2x data keep about 300ms for v2x obj
  for (auto it = v2x_objs_.begin(); it != v2x_objs_.end();) {
    it->second.first > 0 ? it->second.first-- : (it->second.first = 0);
    if (it->second.second.x < kPerceptionRoiFrontValue + 1.0 &&
        it->second.second.y < kPerceptionRoiSideValue + 1.0 &&
        it->second.second.y > -kPerceptionRoiSideValue - 1.0) {
      v2x_objs_.erase(it++);
      continue;
    }
    auto new_object = elements->add_objects();
    new_object->set_id(std::to_string(it->second.second.id));
    new_object->set_type(TypeFromPtcpToQView.at(it->second.second.type));
    new_object->set_data_src(QViewObjectProto::V2X);
    auto rect = new_object->mutable_bounding_box();
    it->second.second.x += it->second.second.x_interval;
    rect->set_x(it->second.second.x);
    it->second.second.y += it->second.second.y_interval;
    rect->set_y(it->second.second.y);
    rect->set_length(it->second.second.length);
    rect->set_width(it->second.second.width);
    rect->set_heading(it->second.second.heading);
    if (it->second.first == 0) {
      v2x_objs_.erase(it++);
    } else {
      ++it;
    }
  }
  // Render V2X data
  if (!v2x_proto_.empty()) {
    // Render glosas
    if (v2x_proto_.back()->has_green_light_optimal_speed_advisory()) {
      RenderGlosa(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->abnormal_vehicle_warning_proto_array_size() > 0) {
      ProcessAvw(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->remote_vehicle_proto_size() > 0) {
      RenderRvs(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->traffic_participants_proto_size() > 0) {
      RenderPtcp(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->construction_proto_size() > 0) {
      ProcessV2xConstruction(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_traffic_jam_warning_proto()) {
      ProcessTrafficJamWarning(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_speed_limit_proto()) {
      ProcessSpeedLimit(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->wrong_way_proto_size() > 0) {
      ProcessWrongWay(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->speeding_proto_size() > 0) {
      ProcessSpeeding(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->pedestrian_on_motorway_proto_size() > 0) {
      ProcessPedestrianOnMotorWay(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->non_motor_on_motorway_proto_size() > 0) {
      ProcessNonMotorOnMotorWay(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_red_light_violation_warning_proto()) {
      ProcessRedLightViolationWarning(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_bus_first_proto()) {
      ProcessBusFirst(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_speed_advisory()) {
      ProcessSpeedAdvisory(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->obstacle_collision_proto_size() > 0) {
      ProcessObstacleCollision(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_vulnerable_road_user_collision_warning_proto()) {
      ProcessNonMotorCollision(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->illegal_vehicle_proto_size() > 0) {
      ProcessIllegalVehicle(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_speed_limit_warning_proto()) {
      ProcessSpeedLimitWarning(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->in_vehicle_signage_proto_size() > 0) {
      ProcessInVehicleSignage(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_forward_collision_warning_proto()) {
      ProcessForwardCollision(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_intersection_collision_warning_proto()) {
      ProcessIntersectionCollision(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_left_turn_assistant_proto()) {
      ProcessLeftTurnAssist(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_blind_spot_warning_proto()) {
      ProcessBlindSpot(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_lane_change_warning_proto()) {
      ProcessLaneChange(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->has_emergency_brake_warning_proto()) {
      ProcessEmergencyBrake(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->out_of_control_vehicle_size() > 0) {
      ProcessOutOfControlVehicle(v2x_proto_.back(), elements);
    }
    if (v2x_proto_.back()->emergency_vehicles_proto_size() > 0) {
      ProcessEmergencyVehicle(v2x_proto_.back(), elements);
    }

    v2x_proto_.pop_back();
  }
}

void QViewPublisherModule::RenderTrajectory(
    const std::shared_ptr<const TrajectoryProto> &trajectory,
    const VehiclePose &pose,
    std::vector<QViewPolygonProto> *trajectory_polygons) const {
  const int num_traj_points = trajectory->trajectory_point_size();
  QCHECK_GT(num_traj_points, 0);

  auto start_point =
      SmoothToLocal(Vec2d(trajectory->trajectory_point(0).path_point().x(),
                          trajectory->trajectory_point(0).path_point().y()));

  // Shift trajectory start point to vehicle_pose + kFrontOffset m. And entire
  // trajectory shifts as well. kFrontOffset 1.6m is an experimental value to
  // keep trajectory on lane center when making a turn
  constexpr double kFrontOffset = 1.6;
  const Vec2d offset = pose.coord2d() +
                       kFrontOffset * Vec2d::UnitFromAngle(pose.yaw) -
                       start_point;

  const double line_width = 0.8;
  const int range = std::min(num_traj_points, 20);
  const auto color = QViewPolygonProto::GREEN;
  std::vector<Vec2d> prev_corners;
  for (int i = 0, counter = range; i < num_traj_points - 1; ++i) {
    auto alpha = std::max(0, counter--) * 1.0 / range;

    const Vec2d start =
        SmoothToLocal(Vec2d(trajectory->trajectory_point(i).path_point().x(),
                            trajectory->trajectory_point(i).path_point().y())) +
        alpha * offset;
    const Vec2d end =
        SmoothToLocal(
            Vec2d(trajectory->trajectory_point(i + 1).path_point().x(),
                  trajectory->trajectory_point(i + 1).path_point().y())) +
        alpha * offset;
    if ((start - end).squaredNorm() < Sqr(0.1)) continue;
    const Box2d box(Segment2d(start, end), line_width);

    trajectory_polygons->push_back(QViewPolygonProto());
    auto &polygon = trajectory_polygons->back();
    polygon.set_color(color);
    polygon.set_height_cm(kTrajectoryHeight * 1e2);
    polygon.mutable_points()->Reserve(4);
    const auto add_point = [&](const auto &corner) {
      auto *point = polygon.add_points();
      point->set_x_cm(RoundToInt(corner.x() * 1e2));
      point->set_y_cm(RoundToInt(corner.y() * 1e2));
    };

    const auto curr_corners = box.GetCornersCounterClockwise();
    if (!prev_corners.empty()) {
      add_point(prev_corners[0]);
      add_point(prev_corners[3]);
    } else {
      add_point(curr_corners[1]);
      add_point(curr_corners[2]);
    }
    add_point(curr_corners[3]);
    add_point(curr_corners[0]);
    prev_corners = curr_corners;
  }
}

void QViewPublisherModule::ProcessAvw(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &avw = v2x_proto->abnormal_vehicle_warning_proto_array(0);
  auto notification = elements->add_v2x_notifications();
  notification->set_type(kAbnormalVehicleWarning);
  notification->set_distance_m(GetDistanceByPose(
      v2x_proto->header().timestamp(), avw.longitude(), avw.latitude()));
}

void QViewPublisherModule::RenderRvs(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  auto pose =
      GetNearestPose(pose_history_, v2x_proto->header().timestamp() * 1e-6);
  for (const auto &rv : v2x_proto->remote_vehicle_proto()) {
    if (rv.status() == 2 || rv.status() == 3) {
      auto pose_vehicle = GlobalToLocal(
          {rv.longitude() * M_PI / 180, rv.latitude() * M_PI / 180});
      FindAndUpdateElements(pose_vehicle, pose.yaw, elements);
    }
  }
}

void QViewPublisherModule::RenderPtcp(
    const std::shared_ptr<const V2xProto> &v2x_proto, QViewElements *elements) {
  if (pose_history_.empty()) {
    return;
  }
  auto pose =
      GetNearestPose(pose_history_, v2x_proto->header().timestamp() * 1e-6);
  const auto pose_vehicle = pose.ToTransform().Inverse();
  for (const auto &ptcp : v2x_proto->traffic_participants_proto()) {
    auto pose_smooth = coordinate_converter_.GlobalToSmooth(
        {ptcp.longitude() * M_PI / 180, ptcp.latitude() * M_PI / 180});
    auto center_vehicle =
        pose_vehicle.TransformPoint(Vec3d{pose_smooth.x(), pose_smooth.y(), 0});
    if (center_vehicle.x() < kPerceptionRoiFrontValue &&
        center_vehicle.x() > kPerceptionRoiRearValue &&
        std::abs(center_vehicle.y()) < kPerceptionRoiSideValue) {
      continue;
    }
    if (center_vehicle.x() > kPerceptionRoiFrontMaxValue ||
        center_vehicle.x() < kPerceptionRoiRearValue) {
      continue;
    }
    auto pose_ptcp = GlobalToLocal(
        {ptcp.longitude() * M_PI / 180, ptcp.latitude() * M_PI / 180});
    if (TypeFromPtcpToQView.count(ptcp.type()) == 0) continue;
    V2xObjs v2x_obj;
    v2x_obj.heading =
        GlobalYawToLocal(NormalizeAngle(d2r(360.0 - ptcp.heading() + 270.0)));
    v2x_obj.speed = ptcp.speed();
    v2x_obj.id = ptcp.ptcpid();
    v2x_obj.x = pose_ptcp.x();
    v2x_obj.y = pose_ptcp.y();
    v2x_obj.type = ptcp.type();
    v2x_obj.length = ptcp.length() * 1e-3;
    v2x_obj.width = ptcp.width() * 1e-3;
    if (v2x_objs_.count(ptcp.ptcpid())) {
      v2x_obj.x_interval = (v2x_obj.x - v2x_objs_[ptcp.ptcpid()].second.x) /
                           kMaxObjsKeepingCounts;
      v2x_obj.y_interval = (v2x_obj.y - v2x_objs_[ptcp.ptcpid()].second.y) /
                           kMaxObjsKeepingCounts;
    } else {
      v2x_obj.x_interval = 0;
      v2x_obj.y_interval = 0;
    }
    v2x_objs_[ptcp.ptcpid()] =
        std::make_pair(kMaxObjsKeepingCounts - 1, v2x_obj);
    auto new_object = elements->add_objects();
    new_object->set_id(std::to_string(v2x_obj.id));
    new_object->set_type(TypeFromPtcpToQView.at(v2x_obj.type));
    new_object->set_data_src(QViewObjectProto::V2X);
    auto rect = new_object->mutable_bounding_box();
    rect->set_x(v2x_objs_[ptcp.ptcpid()].second.x + v2x_obj.x_interval);
    rect->set_y(v2x_objs_[ptcp.ptcpid()].second.y + v2x_obj.y_interval);
    rect->set_length(v2x_obj.length);
    rect->set_width(v2x_obj.width);
    rect->set_heading(v2x_obj.heading);
  }
}

int32 QViewPublisherModule::GetDistanceByPose(const double timestamp,
                                              const double longitude,
                                              const double latitude) const {
  auto pose = GetNearestPose(pose_history_, timestamp * 1e-6);
  const auto pose_vehicle = pose.ToTransform().Inverse();
  auto pose_smooth = coordinate_converter_.GlobalToSmooth(
      {longitude * M_PI / 180, latitude * M_PI / 180});
  auto construction_pose =
      pose_vehicle.TransformPoint(Vec3d{pose_smooth.x(), pose_smooth.y(), 0});

  return Hypot(construction_pose.x(), construction_pose.y());
}

void QViewPublisherModule::ProcessV2xConstruction(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  for (const auto &construction : v2x_proto->construction_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kConstruction);
    notification->set_distance_m(
        GetDistanceByPose(v2x_proto->header().timestamp(),
                          construction.longitude(), construction.latitude()));
    if (construction.lane_id_size() > 0) {
      for (const auto &lane_id : construction.lane_id()) {
        notification->add_data(lane_id);
      }
      notification->set_sub_type(kHaveLanes);
    } else {
      notification->set_sub_type(kNoLane);
    }
  }
}

void QViewPublisherModule::ProcessSpeeding(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  for (const auto &speeding : v2x_proto->speeding_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kSpeedingWarning);
    notification->add_data(speeding.longitude());
    notification->add_data(speeding.latitude());
  }
}

void QViewPublisherModule::ProcessPedestrianOnMotorWay(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  for (const auto &pedestrian : v2x_proto->pedestrian_on_motorway_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kPedestrianOnMotorWay);
    notification->add_data(pedestrian.longitude());
    notification->add_data(pedestrian.latitude());
  }
}

void QViewPublisherModule::ProcessNonMotorOnMotorWay(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  for (const auto &non_motor : v2x_proto->non_motor_on_motorway_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kNonMotorOnMotorWay);
    notification->add_data(non_motor.longitude());
    notification->add_data(non_motor.latitude());
  }
}

// Process Traffic jam warning, caculate the distance between traffic jam side
void QViewPublisherModule::ProcessTrafficJamWarning(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  const auto &tjw = v2x_proto->traffic_jam_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kTrafficJamWarning);
  int distance = GetDistanceByPose(v2x_proto->header().timestamp(),
                                   tjw.longitude(), tjw.latitude());
  if (distance == 0) {
    notification->set_sub_type(kNoDistance);
  } else {
    notification->set_sub_type(kHaveDistance);
  }
  notification->set_distance_m(distance);
  if (tjw.lane_id_size() > 0) {
    for (const auto &lane_id : tjw.lane_id()) {
      notification->add_data(lane_id);
    }
  }
}

void QViewPublisherModule::ProcessSpeedLimit(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  const auto &speed_limit = v2x_proto->speed_limit_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kSpeedLimitReminder);
  notification->add_data(speed_limit.min_speed_limit());
  notification->add_data(speed_limit.max_speed_limit());
}

void QViewPublisherModule::ProcessWrongWay(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  if (pose_history_.empty()) {
    return;
  }
  for (const auto &wrong_way : v2x_proto->wrong_way_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kWrongWay);
    notification->add_data(wrong_way.longitude());
    notification->add_data(wrong_way.latitude());
  }
}

void QViewPublisherModule::ProcessRedLightViolationWarning(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &red_light_violation =
      v2x_proto->red_light_violation_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kRedLightViolationWarning);
  notification->set_direction(red_light_violation.direction());
  notification->set_distance_m(red_light_violation.distance());
}

void QViewPublisherModule::ProcessBusFirst(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &bus_first = v2x_proto->bus_first_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kBusFirst);
  notification->set_sub_type(bus_first.type());
}

void QViewPublisherModule::ProcessSpeedAdvisory(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &speed_advisory = v2x_proto->speed_advisory();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kSpeedAdvisory);
  notification->set_sub_type(speed_advisory.type());
  if (speed_advisory.type() == kAdvisorySpeed) {
    notification->add_data(speed_advisory.advisory_speed());
  } else if (speed_advisory.type() == kAdvisorySpeedRange) {
    notification->add_data(speed_advisory.min_speed());
    notification->add_data(speed_advisory.max_speed());
  }
}

void QViewPublisherModule::ProcessObstacleCollision(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  for (const auto &obstacle_collision : v2x_proto->obstacle_collision_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kObstacleCollision);
    notification->set_alarm_level(obstacle_collision.level());
    notification->set_direction(obstacle_collision.direction());
    notification->set_distance_m(obstacle_collision.distance());
  }
}

void QViewPublisherModule::ProcessNonMotorCollision(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &non_motor =
      v2x_proto->vulnerable_road_user_collision_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kVulnerableRoadUserCollisionWarning);
  notification->set_sub_type(non_motor.type());
  notification->set_alarm_level(non_motor.level());
  notification->set_direction(non_motor.direction());
  notification->set_distance_m(non_motor.distance());
}

void QViewPublisherModule::ProcessIllegalVehicle(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  for (const auto &illegal_vehicle : v2x_proto->illegal_vehicle_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kIllegalVehicleWarning);
    notification->set_alarm_level(illegal_vehicle.level());
    notification->set_direction(illegal_vehicle.direction());
    notification->set_distance_m(illegal_vehicle.distance());
  }
}

void QViewPublisherModule::ProcessSpeedLimitWarning(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &speed_limit_warning = v2x_proto->speed_limit_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kSpeedLimitWarning);
  notification->add_data(speed_limit_warning.min_speed());
  notification->add_data(speed_limit_warning.max_speed());
}

void QViewPublisherModule::ProcessInVehicleSignage(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  for (const auto &vehicle_signage : v2x_proto->in_vehicle_signage_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kInVehicleSignage);
    notification->set_sub_type(vehicle_signage.sign_type());
    notification->set_distance_m(vehicle_signage.distance());
    notification->add_data(vehicle_signage.data());
  }
}

void QViewPublisherModule::ProcessForwardCollision(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &forward_collision = v2x_proto->forward_collision_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kForwardCollisionWarning);
  notification->set_alarm_level(forward_collision.level());
  notification->set_direction(forward_collision.direction());
  notification->set_distance_m(forward_collision.distance());
}

void QViewPublisherModule::ProcessIntersectionCollision(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &intersection_collision =
      v2x_proto->intersection_collision_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kIntersectionCollisionWarning);
  notification->set_alarm_level(intersection_collision.level());
  notification->set_direction(intersection_collision.direction());
  notification->set_distance_m(intersection_collision.distance());
}

void QViewPublisherModule::ProcessLeftTurnAssist(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &left_turn_assist = v2x_proto->left_turn_assistant_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kLeftTurnAssist);
  notification->set_alarm_level(left_turn_assist.level());
  notification->set_direction(left_turn_assist.direction());
  notification->set_distance_m(left_turn_assist.distance());
}

void QViewPublisherModule::ProcessBlindSpot(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &blind_spot = v2x_proto->blind_spot_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kBlindSpotWarning);
  notification->set_alarm_level(blind_spot.alarm_level());
  notification->set_direction(blind_spot.direction());
  notification->set_distance_m(
      GetDistanceByPose(v2x_proto->header().timestamp(), blind_spot.longitude(),
                        blind_spot.latitude()));
}

void QViewPublisherModule::ProcessLaneChange(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &lane_change = v2x_proto->lane_change_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kLaneChangeWarning);
  notification->set_alarm_level(lane_change.alarm_level());
  notification->set_direction(lane_change.direction());
  notification->set_distance_m(
      GetDistanceByPose(v2x_proto->header().timestamp(),
                        lane_change.longitude(), lane_change.latitude()));
}

void QViewPublisherModule::ProcessEmergencyBrake(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  const auto &emergency_brake = v2x_proto->emergency_brake_warning_proto();
  auto *notification = elements->add_v2x_notifications();
  notification->set_type(kEmergencyBrakeWarning);
  notification->set_direction(emergency_brake.direction());
  notification->set_distance_m(GetDistanceByPose(
      v2x_proto->header().timestamp(), emergency_brake.longitude(),
      emergency_brake.latitude()));
}

void QViewPublisherModule::ProcessOutOfControlVehicle(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  for (const auto &out_of_control : v2x_proto->out_of_control_vehicle()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kOutOfControlWarning);
    notification->set_alarm_level(out_of_control.level());
    notification->set_direction(out_of_control.direction());
    notification->set_distance_m(out_of_control.distance());
  }
}

void QViewPublisherModule::ProcessEmergencyVehicle(
    const std::shared_ptr<const V2xProto> &v2x_proto,
    QViewElements *elements) const {
  for (const auto &emergency_vehicle : v2x_proto->emergency_vehicles_proto()) {
    auto *notification = elements->add_v2x_notifications();
    notification->set_type(kEmergencyVehicleWarning);
    notification->set_alarm_level(emergency_vehicle.level());
    notification->set_direction(emergency_vehicle.direction());
    notification->set_distance_m(emergency_vehicle.distance());
  }
}

void QViewPublisherModule::RenderQviewImages(
    double timestamp, std::pair<std::string, std::string> *image_info) {
  // serialize publish images data
  if (FLAGS_publish_images_to_qview) {
    SCOPED_QTRACE("QViewPublisherModule::UpdatePose_PublishImage");

    const auto now = FromUnixDoubleSeconds(timestamp);
    if (now >=
        (prev_image_publish_time_ + absl::Seconds(FLAGS_qview_camera_rate))) {
      prev_image_publish_time_ = now;
      const std::vector<std::pair<std::string, std::string>>
          camera_id_with_json_name = {
              {FLAGS_qview_front_camera, "front_image"},
              {FLAGS_qview_rear_camera, "rear_image"},
          };
      for (const auto &[camera_id_str, json_name] : camera_id_with_json_name) {
        CameraId camera_id;
        CameraId_Parse(camera_id_str, &camera_id);
        if (FLAGS_qview_use_downsized_image) {
          absl::ReaderMutexLock m(&downsized_encoded_images_mutex_);
          if (const ShmMessage *encoded_image_shm = FindLinkedPtrOrNull(
                  latest_downsized_encoded_images_, camera_id)) {
            std::string_view encoded_image(
                static_cast<const char *>(encoded_image_shm->buffer()),
                encoded_image_shm->buffer_size());
            image_info->first = json_name;
            image_info->second = ::qcraft::crypto::Base64Encode(encoded_image);
          }
        } else {
          absl::ReaderMutexLock m(&decoded_images_mutex_);
          if (const CameraImage *camera_image =
                  FindOrNull(latest_decoded_images_, camera_id)) {
            if (const auto image = GetImageFromCamera(*camera_image)) {
              image_info->first = json_name;
              image_info->second = ::qcraft::crypto::Base64Encode(*image);
            } else {
              VLOG(3) << json_name << " data empty.";
            }
          }
        }
      }
    }
  }
}

void QViewPublisherModule::RenderRouteInfo() {
  // serialize route info data
  if (!q_run_event_proto_.empty()) {
    if (route_info_.update_timestamp_ != q_run_event_proto_.back().first) {
      auto ret =
          route_info_.Parse(q_run_event_proto_.back().second.string_value());
      if (!ret) {
        LOG_EVERY_N(INFO, 10)
            << "route_info_.Parse error "
            << q_run_event_proto_.back().second.string_value();
        return;
      }
      for (auto &station : route_info_.stations_) {
        Vec2d smooth_coord_2d(station.longtitude_, station.latitude_);
        Vec3d smooth_coord_3d(station.longtitude_, station.latitude_,
                              station.altitude_);
        auto level_id =
            semantic_map_manager_.InferLevelIdFromNearbyLanes(smooth_coord_3d);
        station.heading_ = semantic_map_manager_.GetPointHeadingWithLaneAtLevel(
            level_id, smooth_coord_2d);
      }
      route_info_.update_timestamp_ = q_run_event_proto_.back().first;
    }
  }
}

void QViewPublisherModule::UpdateJsonObject(
    double timestamp, const VehiclePose &vehicle_pose,
    const QViewElements &elements,
    const std::vector<QViewPolygonProto> &trajectory_polygons,
    const std::pair<std::string, std::string> &image_info, int qview_version) {
  json_string_.clear();

  // new data serializer with the main necessary elements data
  std::shared_ptr<QviewDataSerializer> data_serializer;
  if (qview_version == 1) {
    data_serializer.reset(new QviewDataSerializerV1(elements));
  } else {
    data_serializer.reset(new QviewDataSerializerV2(elements));
  }

  data_serializer->SerializeObjects(objects_update_time_, v2x_update_time_);

  if (map_polygon_serialize_count_ % kMapPolygonSerializeRound == 0) {
    data_serializer->SerializeMapPolygons();
    map_polygon_serialize_count_ = 0;
  }

  data_serializer->SerializeTrajectoryPolygons(trajectory_polygons);
  data_serializer->SerializeTrafficLights();
  data_serializer->SerializeSpeedInfo();
  data_serializer->SerializeControlInfo();

  // prepare and serialize pose data
  data_serializer->SerializePose(vehicle_pose);

  // serialize station data
  if (hmi_contents_.size() > 0) {
    data_serializer->SerializeStationInfo(hmi_contents_.back());
  }

  // serialize publish images data
  if (image_info.first.size() != 0 && image_info.second.size() != 0) {
    data_serializer->SerializeString(image_info.first.c_str(),
                                     image_info.second.c_str());
  }

  data_serializer->SerializeGlosaLights();
  data_serializer->SerializeConstructions();
  data_serializer->SerializeString("goal", FLAGS_goal.c_str());
  data_serializer->SerializeInt("vehicle_type",
                                GetVehicleModel(param_manager()));
  data_serializer->SerializeBusStations();

  // serialize route info data
  if (route_info_.has_route_info_) {
    data_serializer->SerializeRouteInfo(route_info_);
  }

  data_serializer->SerializePickUpPoint();
  data_serializer->SerializePublishDuration(publish_duration_);

  if (qview_version >= 2 &&
      lidar_frame_pre_update_time_ != lidar_frame_update_time_) {
    data_serializer->SerializePedPoints();
    data_serializer->SerializeCycPoints();
    lidar_frame_pre_update_time_ = lidar_frame_update_time_;
  }

  data_serializer->SerializeString(
      "timestamp",
      absl::StrFormat("%.0f", ToUnixDoubleSeconds(Clock::Now()) * 1000)
          .c_str());
  data_serializer->SerializeInt64("last_update_time_tra",
                                  trajectory_update_time_ / 1000);
  data_serializer->SerializeV2xNotifications();

  // dump serialized json data to json_string_
  json_string_ = data_serializer->Dump();
}

void QViewPublisherModule::UpdatePose(std::shared_ptr<const PoseProto> pose) {
  SCOPED_QTRACE("QViewPublisherModule::UpdatePose");

  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  // Skip if the data is too old.
  constexpr double kPoseMaxDelay = 0.02;  // 20 ms
  const double pose_delayed_sec = pose->timestamp() - timestamp_now;
  if (std::abs(pose_delayed_sec) > kPoseMaxDelay && IsOnboardMode() &&
      FLAGS_qview_discard_obsolete_pose) {
    LOG_EVERY_N(WARNING, 20)
        << "Discard obsolete pose, delayed sec: " << pose_delayed_sec;
    return;
  }

  if (pose_history_.size() > 50 && !local_coordinate_converter_initialized_) {
    local_coordinate_converter_initialized_ = true;
    local_coordinate_converter_.SetSmoothCoordinateOrigin(
        coordinate_converter_.SmoothToGlobal(
            {pose->pos_smooth().x(), pose->pos_smooth().y()}));

    // Start collecting map features after the coordinate converter has set
    // its origin. In order not to block the main thread, implement it
    // asynchronously.
    if (map_thread_.joinable()) {
      map_thread_.join();
    }
    map_thread_ = std::thread([this]() {
      QSetThreadName("map_thread_");
      CollectMapFeatures();
      map_features_collected_ = true;
    });
  }

  if (local_coordinate_converter_initialized_ && !map_features_collected_) {
    VLOG(1) << "map not initialized. Skip updating pose";
    return;
  }

  if (!localization_valid_) return;

  // In playback mode, clear the history if we get an older pose (this
  // usually happens when we seek to an older timstamp in the same run).
  if (!pose_history_.empty() &&
      pose->timestamp() < pose_history_.back().first - 0.1) {
    LOG(INFO) << absl::StrFormat(
        "Clear history on out-of-order pose. Current pose %.3f; last pose "
        "%.3f.; diff %.3f ",
        pose->timestamp(), pose_history_.back().first,
        pose_history_.back().first - pose->timestamp());
    pose_history_.clear();
    all_objects_.clear();
    all_cones_.clear();
    all_objects_heading_.clear();
    all_objects_pos_.clear();
    prev_publish_ts_ = 0.0;
  }

  pose_history_.push_back(
      std::make_pair(pose->timestamp(), VehiclePose(*pose)));

  // Erase old objects.
  for (auto iter = all_objects_.begin(); iter != all_objects_.end();) {
    const auto &[id, objects] = *iter;
    const auto next_iter = std::next(iter);
    if (objects.back().timestamp() < pose->timestamp() - 1.0) {
      all_objects_heading_.erase(id);
      all_objects_pos_.erase(id);
      all_objects_.erase(iter);
    }
    iter = next_iter;
  }

  // Erase old cones.
  for (auto iter = all_cones_.begin(); iter != all_cones_.end();) {
    const auto &[id, object] = *iter;
    const auto next_iter = std::next(iter);
    if (object.timestamp() < pose->timestamp() - 5.0) {
      all_cones_.erase(id);
    }
    iter = next_iter;
  }

  // Render the scene for kPoseDelayNum poses ago.
  constexpr int kPoseDelayNum = 30;
  if (pose_history_.size() <= kPoseDelayNum) {
    LOG_EVERY_N(INFO, 10) << "Pose history not long enough. Current size "
                          << pose_history_.size();
    return;
  }

  const auto [timestamp, vehicle_pose] =
      *(pose_history_.rbegin() + kPoseDelayNum);
  if (timestamp < prev_publish_ts_ + FLAGS_qview_refresh_rate) return;
  prev_publish_ts_ = timestamp;

  QViewElements elements;
  elements.set_timestamp(timestamp);

  //////////////////////////////////////////////////////////////////////////////
  // Fill map elements.

  const auto &semantic_map = semantic_map_manager_.semantic_map();

  VehiclePose local_pose;
  const Vec2d local_pose_2d = SmoothToLocal({vehicle_pose.x, vehicle_pose.y});
  local_pose.x = local_pose_2d.x();
  local_pose.y = local_pose_2d.y();
  local_pose.yaw = SmoothYawToLocal(vehicle_pose.yaw);

  // The map region: only render map elements in this region.
  const double local_yaw = SmoothYawToLocal(vehicle_pose.yaw);
  Box2d map_region(local_pose.coord2d(), local_pose.yaw,
                   kMapElementMaxDistFront + kMapElementMaxDistRear,
                   kMapElementMaxDistLateral * 2.0);
  map_region.Shift(
      Vec2d((kMapElementMaxDistFront - kMapElementMaxDistRear) * 0.5, 0.0)
          .FastRotate(local_yaw));

  {
    SCOPED_QTRACE("QViewPublisherModule::UpdatePose_RenderMap");

    RenderLaneBoundaries(map_region, &elements);
    RenderCrosswalks(semantic_map, map_region, &elements);
    RenderOffroadZones(map_region, &elements);
    RenderBuildings(map_region, &elements);
    RenderBusStations(map_region, &elements);
    RenderPickUpPoints(map_region, &elements);
  }

  SCOPED_QTRACE("QViewPublisherModule::UpdatePose_RenderMapFinished");

  std::vector<QViewPolygonProto> trajectory_polygons;
  if (!trajectories_.empty()) {
    RenderTrajectory(trajectories_.back(), local_pose, &trajectory_polygons);
    RenderSpeedInfo(semantic_map_manager_, pose, trajectories_.back(),
                    &elements);

    if (!control_commands_.empty()) {
      RenderControlInfo(
          control_commands_.back(), trajectories_.back(),
          (planner_debugs_.empty() ? nullptr : planner_debugs_.back().get()),
          &elements);
    }
  }

  if (!planner_debugs_.empty()) {
    RenderPlannerTrafficLightInfo(planner_debugs_.back(), &elements);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Fill perception objects.
  RenderV2xObjects(&elements);
  RenderObjects(timestamp, &elements);

  RenderCones(&elements);

  std::pair<std::string, std::string> image_info;
  RenderQviewImages(timestamp, &image_info);

  RenderRouteInfo();

  ++map_polygon_serialize_count_;

  UpdateJsonObject(timestamp, vehicle_pose, elements, trajectory_polygons,
                   image_info, 1);

  {
    SCOPED_QTRACE_ARG1("QViewPublisherModule::UpdatePose_DumpJsonData",
                       "num_elem", json_string_.size());

    absl::MutexLock lock(&mutex_);
    data_to_send_ = std::move(json_string_);
    qview_version_ = 1;
    cond_var_.Signal();
  }

  UpdateJsonObject(timestamp, local_pose, elements, trajectory_polygons,
                   image_info, 2);

  {
    SCOPED_QTRACE_ARG1("QViewPublisherModule::UpdatePose_DumpJsonData",
                       "num_elem", json_string_.size());

    absl::MutexLock lock(&mutex_);
    data_to_send_ = std::move(json_string_);
    qview_version_ = 2;
    cond_var_.Signal();
  }
}

void QViewPublisherModule::SendDataAsync() {
  std::string data;
  int version = 1;
  while (!stop_notification_.HasBeenNotified()) {
    {
      absl::MutexLock lock(&mutex_);
      while (!stop_notification_.HasBeenNotified() && data_to_send_.empty()) {
        cond_var_.Wait(&mutex_);
      }
      data = std::move(data_to_send_);
      data_to_send_.clear();
      version = qview_version_;
    }
    SendData(data, version);
  }
}

void QViewPublisherModule::SendData(const std::string &data, int version) {
  SCOPED_QTRACE("QViewPublisherModule::SendData");

  int64 start_timestamp = absl::ToUnixMicros(qcraft::Clock::Now());
  if (count_ == 0) {
    global_start_timestamp_ = start_timestamp;
  }
  {
    SCOPED_QTRACE("QViewPublisherModule::BroadcastMessage websocket");

    // websocket_server_ will distribute data to all connected clients
    if (version == 1) {
      websocket_server_v1_.BroadcastMessage(data);
    } else {
      websocket_server_v2_.BroadcastMessage(data);
    }

    /////////////////////// for performance profiling ////////////////////////
    //////////////////////////////////////////////////////////////////////////
    publish_duration_ =
        absl::ToUnixMicros(qcraft::Clock::Now()) - start_timestamp;
    VLOG(2) << (++count_) << ":" << publish_duration_;
    if (!caught_ && absl::ToUnixMicros(qcraft::Clock::Now()) >
                        (global_start_timestamp_ + 300 * 1000000)) {
      message_count_per_minutes_ = count_;
      caught_ = true;
    }
    VLOG(2) << "message_count_per_minutes_:" << message_count_per_minutes_;
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
  }
}

void QViewPublisherModule::OnObstacles(
    std::shared_ptr<const ObstaclesProto> obstacles_proto) {
  SCOPED_QTRACE("QViewPublisherModule::OnObstacles");
  obstacles_buffer_.push_back(obstacles_proto->timestamp(),
                              Obstacle::ObstaclesFromProto(*obstacles_proto));
  obstacles_buffer_.ClearOlderThan(1.);
}

void QViewPublisherModule::OnObjects(
    std::shared_ptr<const ObjectsProto> objects) {
  objects_update_time_ = objects->header().timestamp();
  for (const auto &object : objects->objects()) {
    auto &object_buffer = all_objects_[object.id()];
    object_buffer.set_capacity(kObjectBufferSize);
    // Make sure objects of the same object ID is sorted in timestamp in
    // the buffer.
    if (object_buffer.empty() ||
        object.timestamp() > object_buffer.back().timestamp()) {
      object_buffer.push_back(object);
    }
  }
}

void QViewPublisherModule::UpdateTrajectory(
    std::shared_ptr<const TrajectoryProto> trajectory) {
  trajectory_update_time_ = trajectory->header().timestamp();
  if (trajectory->trajectory_point_size() == 0) {
    return;
  }
  trajectories_.push_back(trajectory);
}

void QViewPublisherModule::UpdatePlannerDebug(
    std::shared_ptr<const PlannerDebugProto> planner_debug) {
  planner_debugs_.push_back(planner_debug);
}

void QViewPublisherModule::UpdateControlCommand(
    std::shared_ptr<const ControlCommand> control_command) {
  control_commands_.push_back(control_command);
}

void QViewPublisherModule::UpdateHmiContentProto(
    std::shared_ptr<const HmiContentProto> hmi_content) {
  if (!hmi_content->has_route_content()) {
    return;
  }

  hmi_contents_.push_back(hmi_content);
}

void QViewPublisherModule::OnImage(const CameraImage &camera_image) {
  const auto camera_id = camera_image.camera_id();
  if (CheckCameraInWhiteList(camera_id) && camera_image.width() > 0 &&
      camera_image.height() > 0) {
    absl::WriterMutexLock m(&decoded_images_mutex_);
    latest_decoded_images_[camera_id] = camera_image;
  }
}

void QViewPublisherModule::OnDownsizedEncodedImage(
    std::shared_ptr<ShmMessage> shm_message) {
  const auto encoded_image_meta = shm_message->shm_msg_metadata().GetExtension(
      EncodedImageMetadata::encoded_image_meta);
  const auto camera_id = encoded_image_meta.camera_id();
  if (CheckCameraInWhiteList(camera_id)) {
    absl::WriterMutexLock m(&downsized_encoded_images_mutex_);
    latest_downsized_encoded_images_[camera_id] = std::move(shm_message);
  }
}

void QViewPublisherModule::UpdateV2x(
    std::shared_ptr<const V2xProto> v2x_proto) {
  v2x_update_time_ = v2x_proto->header().timestamp();
  v2x_proto_.push_back(v2x_proto);
}

void QViewPublisherModule::UpdateSemanticMap(
    std::shared_ptr<const UpdateSemanticMapProto> update_semantic_map_proto) {
  SCOPED_QTRACE("QViewPublisherModule::UpdateSemanticMap");
  semantic_map_manager_
      .LoadWholeMap(update_semantic_map_proto->map_dir(),
                    update_semantic_map_proto->map(),
                    update_semantic_map_proto->map_meta())
      .Build();

  ClearHistory();
}

void QViewPublisherModule::UpdateRunParams(
    std::shared_ptr<const UpdateRunParamsProto> run_params_proto) {
  LiteModule::UpdateRunParams(*run_params_proto);
}

void QViewPublisherModule::UpdateQRunEvents(
    std::shared_ptr<const QRunEventsProto> q_run_events_proto) {
  for (const auto &run_event : q_run_events_proto->run_events()) {
    if (run_event.key() == QRunEvent::KEY_QVIEW_ROUTE_INFO) {
      QCHECK_EQ(run_event.value_type(), QRunEvent::TYPE_STRING);
      q_run_event_proto_.push_back(
          std::make_pair(q_run_events_proto->header().timestamp(), run_event));
      break;
    }
  }
}

void QViewPublisherModule::ClearHistory() {
  if (map_thread_.joinable()) {
    map_thread_.join();
  }
  pose_history_.clear();
  lidar_frame_buffer_.clear();
  obstacles_buffer_.clear();
  all_cones_.clear();
  all_objects_.clear();
  all_objects_heading_.clear();
  all_objects_pos_.clear();
  buildings_.clear();
  curbs_.clear();
  lane_boundaries_.clear();
  crosswalks_.clear();
  bus_stations_.clear();

  trajectories_.clear();
  planner_debugs_.clear();
  control_commands_.clear();
  hmi_contents_.clear();
  {
    absl::WriterMutexLock m(&decoded_images_mutex_);
    latest_decoded_images_.clear();
  }
  {
    absl::WriterMutexLock m(&downsized_encoded_images_mutex_);
    latest_downsized_encoded_images_.clear();
  }
  v2x_proto_.clear();
  q_run_event_proto_.clear();

  prev_publish_ts_ = 0.0;

  local_coordinate_converter_initialized_ = false;

  qview_error_count_ = 0;
  prev_image_publish_time_ = absl::Time();
  count_ = 0;
  global_start_timestamp_ = 0;
  message_count_per_minutes_ = 0;
  caught_ = false;
  publish_duration_ = 0;

  map_features_collected_ = false;

  localization_valid_ = false;
}

}  // namespace qcraft
