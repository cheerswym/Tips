#ifndef ONBOARD_VIS_QVIEW_PUBLISHER_MODULE_H_
#define ONBOARD_VIS_QVIEW_PUBLISHER_MODULE_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/perception/obstacle.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/qview.pb.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/vis/qview_route_info.h"
#include "onboard/vis/websockets_server.h"

namespace qcraft {

// A module that publishes vis elements for onboard visualizer.
class QViewPublisherModule : public LiteModule {
 public:
  explicit QViewPublisherModule(LiteClientBase *lite_client);

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override {}

  ~QViewPublisherModule() {
    stop_notification_.Notify();
    cond_var_.Signal();
    if (map_thread_.joinable()) {
      map_thread_.join();
    }

    websocket_server_v1_.Stop();
    websocket_server_v2_.Stop();
  }

 private:
  struct Building {
    int64_t id;
    uint32_t height;
    std::vector<Vec2d> polygon_global;
  };

  struct Curb {
    int64_t id;
    std::vector<Vec2d> points;
  };

  struct LaneBoundary {
    int64_t id;
    std::vector<Vec2d> points;
    std::vector<std::vector<Vec2d>> polygons;
  };

  struct Crosswalk {
    int64_t id;
    std::vector<Vec2d> points;
    std::vector<std::vector<Vec2d>> polygons;
  };

  struct CrosswalkPolygon {
    int64_t id;
    Polygon2d polygon;
  };

  struct BusStation {
    int64_t id;
    Vec2d pos_global;
    double heading;
    uint32_t height;
  };

  struct PickUpPoint {
    int64_t id;
    Vec2d pos_global;
    double heading;
  };

  struct V2xObjs {
    int32_t id;
    double heading;
    double speed;
    double x;
    double y;
    int32_t type;
    int32_t length;
    int32_t width;
    double x_interval;
    double y_interval;
  };

  Vec2d GlobalToLocal(Vec2d point) const {
    return local_coordinate_converter_.GlobalToSmooth(point);
  }
  Vec2d SmoothToLocal(Vec2d point) const {
    return local_coordinate_converter_.GlobalToSmooth(
        coordinate_converter_.SmoothToGlobal(point));
  }
  double SmoothYawToLocal(double yaw) const {
    return local_coordinate_converter_.GlobalYawToSmooth(
        coordinate_converter_.SmoothYawToGlobal(yaw));
  }
  double GlobalYawToLocal(double yaw) const {
    return local_coordinate_converter_.GlobalYawToSmooth(yaw);
  }
  void SendDataAsync();
  void SendData(const std::string &data, int version);
  void OnObstacles(std::shared_ptr<const ObstaclesProto> obstacles);
  void OnObjects(std::shared_ptr<const ObjectsProto> objects);
  void OnImage(const CameraImage &camera_image);
  void OnDownsizedEncodedImage(std::shared_ptr<ShmMessage> shm_message);
  void UpdatePose(std::shared_ptr<const PoseProto> pose);
  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);
  void UpdateTrajectory(std::shared_ptr<const TrajectoryProto> trajectory);
  void UpdatePlannerDebug(
      std::shared_ptr<const PlannerDebugProto> planner_debug);
  void UpdateControlCommand(
      std::shared_ptr<const ControlCommand> control_command);
  void UpdateHmiContentProto(
      std::shared_ptr<const HmiContentProto> hmi_content);
  void UpdateV2x(std::shared_ptr<const V2xProto> v2x_proto);
  void UpdateSemanticMap(
      std::shared_ptr<const UpdateSemanticMapProto> update_semantic_map_proto);
  void UpdateRunParams(
      std::shared_ptr<const UpdateRunParamsProto> run_params_proto);
  void UpdateQRunEvents(
      std::shared_ptr<const QRunEventsProto> q_run_events_proto);

  void CollectMapFeatures();
  void CollectCurbs();
  void CollectLaneBoundaries();
  void CollectLaneBoundary(
      const mapping::LaneBoundaryProto &lane_boundary_proto);
  void CollectCrosswalks(
      int level_id, const std::vector<mapping::LaneProto> &lanes,
      std::vector<CrosswalkPolygon> &crosswalk_polygons);  // NOLINT
  void CollectCrosswalk(int level_id,
                        const std::vector<mapping::LaneProto> &lanes,
                        int64_t id, const Polygon2d &crosswalk_polygon);
  void CollectCrosswalkLines(const std::vector<Segment2d> &crosswalk_polyline,
                             int max_intersect_polyline_index, bool find_k,
                             double k_crosswalklines_find,
                             Crosswalk *crosswalk);
  void CollectBuildings();
  void CollectBusStations();
  void CollectPickUpPoints();

  void RenderLaneBoundaries(const Box2d &map_region,
                            QViewElements *elements) const;
  void RenderCrosswalks(const mapping::SemanticMapProto &semantic_map,
                        const Box2d &map_region, QViewElements *elements) const;
  void RenderOffroadZones(const Box2d &map_region,
                          QViewElements *elements) const;
  void RenderOffroadZone(int32 id, absl::Span<const Vec2d> local_curb,
                         QViewElements *elements) const;
  void RenderBuilding(int32 id, const std::vector<Vec2d> &building_polygon,
                      double height, QViewElements *elements) const;
  void RenderBusStation(Vec2d pos_global, double heading, uint32_t height,
                        QViewElements *elements) const;
  void RenderPickUpPoint(Vec2d pos_global, double heading,
                         mapping::ElementId id, QViewElements *elements) const;
  void RenderBuildings(const Box2d &map_region, QViewElements *elements) const;
  void RenderBusStations(const Box2d &map_region,
                         QViewElements *elementes) const;
  void RenderPickUpPoints(const Box2d &map_region,
                          QViewElements *elements) const;
  void RenderObjects(double timestamp, QViewElements *elements);
  void RenderCones(QViewElements *elements);
  void RenderV2xObjects(QViewElements *elements);
  void RenderTrajectory(
      const std::shared_ptr<const TrajectoryProto> &trajectory,
      const VehiclePose &pose,
      std::vector<QViewPolygonProto> *trajectory_polygons) const;
  void ProcessAvw(const std::shared_ptr<const V2xProto> &v2x_proto,
                  QViewElements *elements) const;
  void RenderRvs(const std::shared_ptr<const V2xProto> &v2x_proto,
                 QViewElements *elements) const;
  void RenderPtcp(const std::shared_ptr<const V2xProto> &v2x_proto,
                  QViewElements *elements);
  void ProcessTrafficJamWarning(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;
  void ProcessV2xConstruction(const std::shared_ptr<const V2xProto> &v2x_proto,
                              QViewElements *elements) const;
  void ProcessSpeedLimit(const std::shared_ptr<const V2xProto> &v2x_proto,
                         QViewElements *elements) const;
  void ProcessWrongWay(const std::shared_ptr<const V2xProto> &v2x_proto,
                       QViewElements *elements) const;
  void ProcessPedestrianOnMotorWay(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;
  void ProcessNonMotorOnMotorWay(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;
  void ProcessSpeeding(const std::shared_ptr<const V2xProto> &v2x_proto,
                       QViewElements *elements) const;
  void ProcessRedLightViolationWarning(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessBusFirst(const std::shared_ptr<const V2xProto> &v2x_proto,
                       QViewElements *elements) const;

  void ProcessSpeedAdvisory(const std::shared_ptr<const V2xProto> &v2x_proto,
                            QViewElements *elements) const;

  void ProcessObstacleCollision(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessNonMotorCollision(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessIllegalVehicle(const std::shared_ptr<const V2xProto> &v2x_proto,
                             QViewElements *elements) const;

  void ProcessSpeedLimitWarning(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessInVehicleSignage(const std::shared_ptr<const V2xProto> &v2x_proto,
                               QViewElements *elements) const;

  void ProcessForwardCollision(const std::shared_ptr<const V2xProto> &v2x_proto,
                               QViewElements *elements) const;

  void ProcessIntersectionCollision(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessLeftTurnAssist(const std::shared_ptr<const V2xProto> &v2x_proto,
                             QViewElements *elements) const;

  void ProcessBlindSpot(const std::shared_ptr<const V2xProto> &v2x_proto,
                        QViewElements *elements) const;

  void ProcessLaneChange(const std::shared_ptr<const V2xProto> &v2x_proto,
                         QViewElements *elements) const;

  void ProcessEmergencyBrake(const std::shared_ptr<const V2xProto> &v2x_proto,
                             QViewElements *elements) const;

  void ProcessOutOfControlVehicle(
      const std::shared_ptr<const V2xProto> &v2x_proto,
      QViewElements *elements) const;

  void ProcessEmergencyVehicle(const std::shared_ptr<const V2xProto> &v2x_proto,
                               QViewElements *elements) const;
  void GetPedCycAndOffroadPoints(
      const double max_obj_time,
      absl::flat_hash_map<std::string, std::pair<Vec2d, std::vector<Vec3f>>>
          *points_with_ids) const;
  void RenderQviewImages(double timestamp,
                         std::pair<std::string, std::string> *image_info);
  void RenderRouteInfo();
  void UpdateJsonObject(
      double timestamp, const VehiclePose &vehicle_pose,
      const QViewElements &elements,
      const std::vector<QViewPolygonProto> &trajectory_polygons,
      const std::pair<std::string, std::string> &image_info, int qview_version);
  void ClearHistory();
  int32 GetDistanceByPose(const double timestamp, const double longitude,
                          const double latitude) const;

  boost::circular_buffer<std::pair<double, VehiclePose>> pose_history_{100};
  absl::flat_hash_map<LidarId, boost::circular_buffer<LidarFrame>>
      lidar_frame_buffer_;
  HistoryBuffer<std::vector<Obstacle>> obstacles_buffer_;
  absl::flat_hash_map<std::string, ObjectProto> all_cones_;
  absl::flat_hash_map<std::string, boost::circular_buffer<ObjectProto>>
      all_objects_;
  absl::flat_hash_map<std::string, double> all_objects_heading_;
  absl::flat_hash_map<std::string, Vec2d> all_objects_pos_;

  int current_level_ = 0;
  absl::flat_hash_map<int, std::vector<Building>> buildings_;
  absl::flat_hash_map<int, std::vector<Curb>> curbs_;
  absl::flat_hash_map<int, std::vector<LaneBoundary>> lane_boundaries_;
  absl::flat_hash_map<int, std::vector<Crosswalk>> crosswalks_;
  absl::flat_hash_map<int, std::vector<BusStation>> bus_stations_;
  absl::flat_hash_map<int, std::vector<PickUpPoint>> pick_up_points_;

  boost::circular_buffer<std::shared_ptr<const TrajectoryProto>> trajectories_{
      10};
  boost::circular_buffer<std::shared_ptr<const PlannerDebugProto>>
      planner_debugs_{10};
  boost::circular_buffer<std::shared_ptr<const ControlCommand>>
      control_commands_{10};
  boost::circular_buffer<std::shared_ptr<const HmiContentProto>> hmi_contents_{
      10};
  absl::Mutex decoded_images_mutex_;
  std::map<CameraId, CameraImage> latest_decoded_images_
      GUARDED_BY(&decoded_images_mutex_);
  absl::Mutex downsized_encoded_images_mutex_;
  std::map<CameraId, std::shared_ptr<ShmMessage>>
      latest_downsized_encoded_images_
          GUARDED_BY(&downsized_encoded_images_mutex_);
  boost::circular_buffer<std::shared_ptr<const V2xProto>> v2x_proto_{10};
  boost::circular_buffer<std::pair<int64, QRunEvent>> q_run_event_proto_{10};

  // record the latest update time for subscribe messages of OnObjects,
  // UpdateV2x and UpdateTrajectory Used for frontend page to filter calculatint
  // operation when updating information this will optimaze showing performance
  // in frontend
  int64 objects_update_time_;
  int64 v2x_update_time_;
  int64 trajectory_update_time_;
  int64 lidar_frame_pre_update_time_;
  int64 lidar_frame_update_time_;

  double prev_publish_ts_ = 0.0;

  // Used to convert between smooth and global coordinate.
  CoordinateConverter coordinate_converter_;

  // Used to convert between local and global coordinate.
  bool local_coordinate_converter_initialized_ = false;
  CoordinateConverter local_coordinate_converter_;

  int qview_error_count_ = 0;
  absl::Time prev_image_publish_time_;
  int count_ = 0;
  int64 global_start_timestamp_ = 0;
  int64 message_count_per_minutes_ = 0;
  bool caught_ = false;
  int64 publish_duration_ = 0;
  QviewRouteInfo route_info_;

  absl::Mutex mutex_;
  std::string data_to_send_ GUARDED_BY(mutex_);

  std::atomic<bool> map_features_collected_ = false;
  std::thread map_thread_;

  bool localization_valid_ = false;

  // When stop is requested, we will wait for the callback thread to finish.
  absl::Notification stop_notification_;
  absl::CondVar cond_var_;
  ThreadPool thread_pool_{1};

  SemanticMapManager semantic_map_manager_;
  std::map<int, std::pair<int, V2xObjs>> v2x_objs_;

  // json message to be send to qview frontend
  std::string json_string_;
  int qview_version_ = 1;

  int map_polygon_serialize_count_;

  // braodcast qview 1.0 massages
  websocket::WebsocketServer websocket_server_v1_;

  // braodcast qview 2.0 massages
  websocket::WebsocketServer websocket_server_v2_;
};

REGISTER_LITE_MODULE(QViewPublisherModule);

}  // namespace qcraft

#endif  // ONBOARD_VIS_QVIEW_PUBLISHER_MODULE_H_
