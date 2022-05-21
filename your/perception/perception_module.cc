#include "onboard/perception/perception_module.h"

#include <snappy.h>

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "offboard/vis/ark/ark_server/ark_client_man.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/camera/utils/image_util.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/lidar_util.h"
#include "onboard/lidar/spin_reader_test_util.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/spin_util.h"
#include "onboard/lidar/vehicle_pose_util.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_io.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/nets/fiery_eye_net_classifier_constants.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/measurement_util.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/perception_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/perception/range_image/range_image_util.h"
#include "onboard/perception/registration/icp.h"
#include "onboard/perception/segmentation/fiery_eye_net_proposer.h"
#include "onboard/perception/utils/point_filtering_utils.h"
#include "onboard/proto/image_overlay.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/colormap.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

DEFINE_bool(
    enable_fen, true,
    "Enable Fiery Eye Net to detection vehicles/peds/etc for segmentation");
DEFINE_bool(enable_pfen, false,
            "Enable pfen by subscribing to semantic segmentation result.");
DEFINE_bool(enable_pcn, true,
            "Enable image patch classifier net to classify pedestrians from "
            "image patches");
DEFINE_bool(enable_pcn_filter_cyc, true,
            "Enable image patch classifier net to classify cyclist from "
            "image patches");
DEFINE_bool(dump_trace, false,
            "Log trace periodically for offboard performance testing");
DEFINE_bool(enable_range_image, true,
            "Whether build and publish range images.");
DEFINE_bool(enable_sensor_fov, true,
            "Whether compute and publish sensor fov result.");
DEFINE_bool(render_label_frame_cvs, false,
            "Enable render label frame using canvas.");
DEFINE_bool(fen_collect_point_async, true,
            "Whether to collect points asynchronously in FEN classifier.");
DEFINE_bool(enable_ace_association, false,
            "Enable ACE association net to extract association feature "
            "embedding for laser measurements.");

namespace qcraft {
namespace {

// How many latest images are stored in the image buffer.
constexpr int kImageBufferSize = 3;
constexpr char kFieryEyeNetParamKey[] = "fiery_eye_net_param";
constexpr char kImagePatchNetParamKey[] = "image_patch_net_param";
constexpr char kMistNewNetParamKey[] = "mist_net_v1_param";
constexpr char kAceAssociationNetParamKey[] = "ace_association_net_param";

// A cluster is at road side if all obstacles are within the following
// distance to curb.
constexpr float kRoadSideMinDist = -1.0f;

// Maximum number of lidar frames stored in buffer per lidar.
constexpr int kMaxNumLidarFramesInBuffer = 5;

// Maximum number of tasks stored in queue per topic.
static const std::map<std::string, int> kMaxNumTasksInQueue{
    {"label_frame_proto", 20},
    {"lidar_host_time_diff_proto", 20},
    {"localization_transform_proto", 20},
    {"pose_proto", 100},
    {"decoded_image", 10},
    {"measurements_proto", 30},
    {"radar_measurements_proto", 30},
    {"semantic_segmentation_results_proto", 20}};

void RenderDetectionBox(const Box2d& box, double score, vis::Color color,
                        const VehiclePose& pose) {
  std::vector<Vec3d> contour_points;
  for (const auto& point : box.GetCornersCounterClockwise()) {
    contour_points.emplace_back(point.x(), point.y(), pose.z);
  }
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/fiery_eye_net_result");
  canvas.DrawPolygon(contour_points, color, /*size=*/2);
  canvas.DrawLine(
      {box.center() + Vec2d(box.length() * 0.5, 0).FastRotate(box.heading()),
       pose.z},
      {box.center() + Vec2d(box.length() * 1.0, 0).FastRotate(box.heading()),
       pose.z},
      color, /*size=*/2);
  if (score > 0.0) {
    canvas.DrawText(absl::StrFormat("%.2f", score),
                    {box.center().x(), box.center().y(), 0.01}, 0, 1.0,
                    vis::Color::kWhite);
  }
}

void RenderLabelFrameCvs(const labeling::LabelFrameProto& label_frame) {
  using namespace labeling;  // NOLINT
  const auto& pose = label_frame.pose();
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/label_frame_cvs");
  for (const auto& label : label_frame.labels()) {
    canvas.DrawBox({label.x(), label.y(), label.z()}, label.heading(),
                   {label.length(), label.width()}, vis::Color::kRed, 2);
    canvas.DrawText(absl::StrFormat("[Category] %s",
                                    Label::Category_Name(label.category())),
                    {label.x(), label.y(), label.z()}, 0.0, 0.2,
                    vis::Color::kRed);
  }
  for (const auto& zone : label_frame.zones()) {
    if (zone.xs().size() < 3) continue;
    std::vector<Vec3d> vertices(zone.xs().size());
    QCHECK_EQ(zone.xs().size(), zone.ys().size());
    QCHECK_EQ(zone.xs().size(), zone.zs().size());
    for (int i = 0; i < zone.xs().size(); ++i) {
      vertices[i] = {zone.xs()[i], zone.ys()[i], zone.zs()[i]};
    }
    canvas.DrawPolygon(vertices, GetZoneTypeColor(zone.type()), 2);
    std::vector<Vec2d> vertices_2d(vertices.size());
    for (int i = 0; i < vertices.size(); ++i) {
      vertices_2d[i] = {vertices[i].x(), vertices[i].y()};
    }
    const auto& centroid = Polygon2d(vertices_2d).centroid();
    if (zone.type() != Zone::FULL_SEMANTIC) {
      canvas.DrawText(
          absl::StrFormat("[Type] %s", Zone::ZoneType_Name(zone.type())),
          {centroid.x(), centroid.y(), pose.z() + 0.1}, 0.0, 0.2,
          vis::Color::kRed);
    }
  }
  canvas.DrawText(
      absl::StrFormat("[LabelFrame] Pose: x %.2f y %.2f z %.2f "
                      "roll %.2f pitch %.2f yaw %.2f, timestamp: %.3f",
                      pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(),
                      pose.yaw(), label_frame.timestamp()),
      {pose.x(), pose.y(), pose.z() + 0.1}, 0.0, 0.2, vis::Color::kRed);
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

void SetBox2dProto(const Box2d& box2d, Box2dProto* box2d_proto) {
  QCHECK_NOTNULL(box2d_proto);
  box2d_proto->set_x(box2d.center_x());
  box2d_proto->set_y(box2d.center_y());
  box2d_proto->set_width(box2d.width());
  box2d_proto->set_length(box2d.length());
  box2d_proto->set_heading(box2d.heading());
  return;
}

// Return if the cluster is likely on road side and tends to be a static
// object.
bool ClusterIsOnRoadSide(const Cluster& cluster) {
  for (const auto* obstacle : cluster.obstacles()) {
    if (obstacle->dist_to_curb < kRoadSideMinDist) {
      return false;
    }
  }
  return true;
}

void PublishPoseCorrectionProto(
    const double mid_scan_timestamp,
    const std::unique_ptr<ObstacleDetector>& obstacle_detector,
    LiteModule* lite_module) {
  if (obstacle_detector->pose_difference()) {
    PoseCorrectionProto pose_correction_proto;
    const auto& pose_difference = *obstacle_detector->pose_difference();
    pose_correction_proto.set_mid_scan_timestamp(mid_scan_timestamp);
    pose_correction_proto.set_pitch(pose_difference.pitch);
    pose_correction_proto.set_roll(pose_difference.roll);
    pose_correction_proto.set_z(pose_difference.z);
    QLOG_IF_NOT_OK(WARNING, lite_module->Publish(pose_correction_proto));
  }
  PoseCorrectionDebugProto pose_correction_debug_proto =
      obstacle_detector->pose_correction_debug_proto();
  QLOG_IF_NOT_OK(WARNING, lite_module->Publish(pose_correction_debug_proto));
}

std::optional<Box2d> ComputeRefinedBbox(
    const SegmentedCluster& segmented_cluster, const Polygon2d& cluster_contour,
    const VehiclePose& pose) {
  if (!segmented_cluster.bounding_box()) {
    return std::nullopt;
  }
  const auto& bbox = *(segmented_cluster.bounding_box());
  const Polygon2d bbox_polygon(bbox);
  if (bbox_polygon.Contains(cluster_contour)) {
    return segmented_cluster.bounding_box();
  }

  const double rot_angle =
      NormalizeAngle(segmented_cluster.bounding_box()->heading());
  // Transform polygon points to object coord.
  const Eigen::Vector2d smooth2object_trans =
      segmented_cluster.bounding_box()->center().Rotate(-rot_angle);
  const auto cluster_bbox = cluster_contour.BoundingBoxWithHeading(
      segmented_cluster.bounding_box()->heading());

  // Transform cluster bbox to object coord.
  const auto transformed_cluster_bbox =
      cluster_bbox.AffineTransform(-smooth2object_trans, -rot_angle);
  const auto transformed_cluster_aabbox = transformed_cluster_bbox.GetAABox();

  const auto& det_bbox = *(segmented_cluster.bounding_box());
  double min_x =
      std::min(-det_bbox.half_length(), transformed_cluster_aabbox.min_x());
  double max_x =
      std::max(det_bbox.half_length(), transformed_cluster_aabbox.max_x());
  double min_y =
      std::min(-det_bbox.half_width(), transformed_cluster_aabbox.min_y());
  double max_y =
      std::max(det_bbox.half_width(), transformed_cluster_aabbox.max_y());

  const double center_offset_x = 0.5 * (min_x + max_x);
  const double center_offset_y = 0.5 * (min_y + max_y);
  Vec2d refined_center = Vec2d(center_offset_x, center_offset_y);

  const double refined_length = max_x - min_x;
  const double refined_width = max_y - min_y;
  // Transform to smooth coord.
  const Eigen::Vector2d object2smooth_trans =
      segmented_cluster.bounding_box()->center();
  Box2d object_coord_refined_bbox(refined_center, 0.0, refined_length,
                                  refined_width);
  const auto refined_bbox =
      object_coord_refined_bbox.AffineTransform(object2smooth_trans, rot_angle);

  return refined_bbox;
}

}  // namespace

std::string PerceptionModule::LidarInfo::StatusString() const {
  switch (status) {
    case LidarInfo::kInactive:
      return "inactive";
    case LidarInfo::kHealthy:
      return "healthy";
    case LidarInfo::kUnhealthy:
      return "unhealthy";
    default:
      QLOG(FATAL) << "Should not reach here.";
  }
}

PerceptionModule::~PerceptionModule() {
  stop_notification_.Notify();

  // Send signal to quit the waiting loop, otherwise it'll wait for new lidar
  // frames.
  lidar_frame_cond_var_.Signal();
  // Wait for futures to avoid undefined behaviors.
  serial_executor_future_.Wait();
}

std::map<LidarId, LidarParametersProto>
PerceptionModule::GetCurrentLidarParams() const {
  const auto run_params = GetRunParams();
  const auto sensor_scenario_config = GetSensorScenarioConfig();
  std::map<LidarId, LidarParametersProto> lidar_params;
  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    const auto lidar_id = lidar_param.installation().lidar_id();
    if (ContainsKey(sensor_scenario_config.lidars, lidar_id)) {
      lidar_params.emplace(lidar_id, lidar_param);
    }
  }
  if (lidar_params.empty()) {
    for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
      lidar_params.emplace(lidar_param.installation().lidar_id(), lidar_param);
    }
  }
  QCHECK(!lidar_params.empty()) << "No lidar parameters found.";
  return lidar_params;
}

void PerceptionModule::RunWorkLoop(bool try_once) {
  while (!stop_notification_.HasBeenNotified()) {
    // Finish all queued tasks.
    RunAllTasksInQueue();
    // Avoid falling behind of input lidar frames, we drop all stale ones and
    // only process the latest lidar frame group here.
    bool lidar_frames_ready = false;
    std::vector<LidarFrame> latest_lidar_frames;
    {
      absl::MutexLock lock(&lidar_frame_mutex_);

      // Find the latest updated lidar.
      double latest_start_timestamp = 0.0;
      LidarId latest_lidar_id = LDR_UNKNOWN;
      for (auto& [lidar_id, lidar_frames] : lidar_frame_buffer_) {
        if (lidar_frames.empty()) continue;
        if (latest_start_timestamp < lidar_frames.back().StartTimestamp()) {
          latest_start_timestamp = lidar_frames.back().StartTimestamp();
          latest_lidar_id = lidar_id;
        }
      }

      // Delete timeout lidar (250ms older than latest updated lidar) from
      // lidar_frame_buffer_.
      constexpr double kLidarTimeoutMaxTimeDiff = 0.25;  // 250ms
      for (auto it = lidar_frame_buffer_.begin();
           it != lidar_frame_buffer_.end();) {
        if (latest_start_timestamp != 0.0 &&
            latest_start_timestamp -
                    lidar_info_map_[it->first].latest_start_timestamp >
                kLidarTimeoutMaxTimeDiff) {
          lidar_info_map_[it->first].status = LidarInfo::kUnhealthy;
          std::string args_message = absl::StrFormat(
              "[%s] timeout level 1 -> Mark unhealthy and key deleted from "
              "lidar_frame_buffer_. [%s] latest start timestamp is %.3f, "
              "current lidar latest start timestamp is %.3f.",
              LidarId_Name(it->first), LidarId_Name(latest_lidar_id),
              latest_start_timestamp,
              lidar_info_map_[it->first].latest_start_timestamp);
          QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_DEVICE,
                            QIssueSubType::QIST_LIDAR_STATE_UNHEALTHY,
                            "Check lidar state: UNHEALTHY", args_message);
          it = lidar_frame_buffer_.erase(it);
        } else {
          ++it;
        }
      }

      // Construct the lidar frame group the latest updated lidar is in. Clear
      // the whole buffer if the corresponding lidar is outdated.
      for (auto& [lidar_id, lidar_frames] : lidar_frame_buffer_) {
        if (lidar_frames.empty()) continue;

        // TODO(dong, yu): Change this back to a smaller threshold once we solve
        // ouster's time sync issue (ouster can not sync correctly on .10, .20,
        // .30).
        constexpr double kSameLidarFrameGroupMaxTimeDiff = 0.080;  // 80ms.
        // If the latest frame from the current lidar is within the same lidar
        // frame group, keep it. Else, drop all frames in the queue.
        if (std::fabs(latest_start_timestamp -
                      lidar_frames.back().StartTimestamp()) <=
            kSameLidarFrameGroupMaxTimeDiff) {
          latest_lidar_frames.push_back(lidar_frames.back());
        } else {
          QLOG(INFO) << absl::StrFormat(
              "[%s] timeout level 2 -> Buffer cleared. It's latest frame older "
              "than latest lidar [%s] timestamp. current lidar frame start "
              "timestamp is %.3f, latest_start_timestamp is %.3f.",
              LidarId_Name(lidar_id), LidarId_Name(latest_lidar_id),
              lidar_frames.back().StartTimestamp(), latest_start_timestamp);
          lidar_frames.clear();
        }
      }

      lidar_frames_ready =
          !latest_lidar_frames.empty() &&
          latest_lidar_frames.size() == lidar_frame_buffer_.size();

      // Drop previous frames in buffer if the current lidar frame group is
      // ready.
      if (lidar_frames_ready) {
        for (auto& [lidar_id, lidar_frames] : lidar_frame_buffer_) {
          const int num_dropped_lidar_frames = lidar_frames.size() - 1;
          // Inform if there's any needs to be dropped.
          for (int i = 0; i < num_dropped_lidar_frames; ++i) {
            QLOG(INFO) << absl::StrFormat(
                "[%s] at %.3f was dropped with latest frame of %s at %.3f.",
                LidarId_Name(lidar_id), lidar_frames[i].StartTimestamp(),
                LidarId_Name(latest_lidar_id), latest_start_timestamp);
          }

          // qissue would be reported if we dropped more than one spins.
          if (num_dropped_lidar_frames > 1) {
            // Emit an execution issue here. We can at most drop one lidar frame
            // each time.
            std::string args_message = absl::StrFormat(
                "[%s] Dropping more than 1 lidar frame consecutively.\nCurrent "
                "time: %.3f\nRef lidar %s at %.3f.\nLidar frames in buffer:\n",
                LidarId_Name(lidar_id), ToUnixDoubleSeconds(Clock::Now()),
                LidarId_Name(latest_lidar_id), latest_start_timestamp);
            for (const auto& lidar_frame : lidar_frames) {
              args_message +=
                  absl::StrFormat("%.3f\n", lidar_frame.StartTimestamp());
            }
            QISSUEX_WITH_ARGS(
                QIssueSeverity::QIS_WARNING, QIssueType::QIT_PERFORMANCE,
                QIssueSubType::QIST_PERCEPTION_LIDAR_FRAME_DROP,
                "Check perception process speed: SLOW(drop lidar frame)",
                args_message);
          }
          lidar_frames.clear();
        }
      }

      // Check if any lidar frame from a lidar is too stale, and we'd better
      // wait for the next one.
      if (lidar_frames_ready) {
        constexpr double kStaleSpinMinDelay = 0.15;  // 150ms
        if (latest_start_timestamp - ToUnixDoubleSeconds(Clock::Now()) <
            -kStaleSpinMinDelay) {
          QLOG(WARNING) << absl::StrFormat(
              "Constructed lidar frame group is too stale from "
              "now. Skip this lidar frame group and wait for the "
              "next one. [%s] start timestamp %.3f and now is %.3f",
              LidarId_Name(latest_lidar_id), latest_start_timestamp,
              ToUnixDoubleSeconds(Clock::Now()));
          lidar_frames_ready = false;
        }
      }

      if (!lidar_frames_ready && try_once) return;
      // Waiting for lidar frames to come if not ready.
      if (!lidar_frames_ready) {
        // Stop waiting for lidar frame, if notified as stopped.
        if (!stop_notification_.HasBeenNotified()) {
          lidar_frame_cond_var_is_waiting_ = true;  // is waiting
          lidar_frame_cond_var_.Wait(&lidar_frame_mutex_);
          lidar_frame_cond_var_is_waiting_ = false;  // finish waiting
        }
        continue;
      }
    }
    // Only use absl::Now() for latency calculation and logging, use
    // Clock::Now() for other purposes.
    const auto now = absl::Now();

    // Preload imagery.
    PreloadImagery();

    // Drop lidar frames with invalid poses. Those lidar frames may exist in the
    // beginning of a run when the positioning module was not ready to produce
    // valid poses.
    bool lidar_frames_have_valid_poses = true;
    for (const auto& lidar_frame : latest_lidar_frames) {
      if (lidar_frame.StartPose() == VehiclePose()) {
        lidar_frames_have_valid_poses = false;
        break;
      }
    }

    double max_timestamp = std::numeric_limits<double>::lowest();
    double min_timestamp = std::numeric_limits<double>::max();
    for (const auto& lidar_frame : latest_lidar_frames) {
      max_timestamp = std::max(max_timestamp, lidar_frame.StartTimestamp());
      min_timestamp = std::min(min_timestamp, lidar_frame.StartTimestamp());
    }
    constexpr double kMaxLidarFrameTimeDiff = 0.01;  // 10ms
    if (max_timestamp - min_timestamp > kMaxLidarFrameTimeDiff) {
      QISSUEX(QIssueSeverity::QIS_WARNING, QIssueType::QIT_CLOCK_SYNC,
              QIssueSubType::QIST_LIDAR_STATE_FRAME_TIMESTAMP_EXCEPTION,
              "Collected lidar frames time difference out of range.");
    }

    // Only process spins if pose and localization transform is received.
    if (lidar_frames_have_valid_poses && coordinate_converter_.is_valid()) {
      ProcessSpins(latest_lidar_frames);
    } else {
      QLOG(WARNING) << absl::StrFormat(
          "Lidar frames have valid poses: %s. Localization transform is valid: "
          "%s.",
          lidar_frames_have_valid_poses ? "YES" : "NO",
          coordinate_converter_.is_valid() ? "YES" : "NO");
    }

    // Only use absl::Now() for latency calculation and logging, use
    // Clock::Now() for other purposes.
    const auto elapsed = absl::Now() - now;
    QCHECK(!latest_lidar_frames.empty());
    std::string lidar_frame_info;
    for (const auto& frame : latest_lidar_frames) {
      lidar_frame_info.append(absl::StrFormat(" %s(%.3f) ",
                                              LidarId_Name(frame.lidar_id()),
                                              frame.StartTimestamp()));
    }
    QLOG(INFO) << absl::StrFormat("Processing lidar frame takes %dms for %s",
                                  absl::ToInt64Milliseconds(elapsed),
                                  lidar_frame_info);

    if (FLAGS_dump_trace) {
      QISSUEX(QIssueSeverity::QIS_DEBUG, QIssueType::QIT_PERFORMANCE,
              QIssueSubType::QIST_PERCEPTION, "perception_trace");
    }

    // Publish canvas.
    PublishCanvas();

    if (try_once) return;
  }
}

PerceptionModule::PerceptionModule(LiteClientBase* lite_client)
    : LiteModule(lite_client),
      icp_tracker_(std::make_unique<IcpTracker>(&thread_pool_)),
      segmenter_(std::make_unique<segmentation::Segmenter>(&thread_pool_)),
      human_pipeline_manager_(std::make_unique<HumanPipelineManager>(
          MutableLiteClient(), &thread_pool_)),
      serial_executor_thread_pool_(1),
      thread_pool_(2),
      nn_thread_pool_(1),
      imagery_preloading_thread_pool_(1) {}

void PerceptionModule::OnInit() {
  vantage_client_man::CreateVantageClientMan(param_manager());
  ark_client_man::CreateArkClientMan(param_manager());

  // Using LoadWholeMap() is strongly discouraged, use UpdateSmoothPos()
  // instead. The later only loads portion of the map as needed
  // semantic_map_manager_.LoadWholeMap().Build();
  imagery_manager_.SetDisabled(ImageryLayer::kZOccupancy |
                               ImageryLayer::kIntensity | ImageryLayer::kZSpan);

  // Get run params and update lidar/camera params.
  run_params_ = GetRunParams();
  camera_params_ = ComputeAllCameraParams(run_params_.vehicle_params());
  const auto& lidar_params = run_params_.vehicle_params().lidar_params();
  for (const auto& lidar_param : lidar_params) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }

  if (FLAGS_enable_fen) {
    QLOG(INFO) << "Initializing FEN...";

    // Initialize fiery eye net.
    NetParam fiery_eye_net_param;
    CHECK_OK(param_manager().GetProtoParam(kFieryEyeNetParamKey,
                                           &fiery_eye_net_param));
    fen_classifier_ = std::make_unique<FieryEyeNetClassifier>(
        run_params_, fiery_eye_net_param);

    QLOG(INFO) << "Initializing FEN... Done!";
  }

  if (FLAGS_enable_pcn) {
    QLOG(INFO) << "Initializing image patch net...";

    NetParam image_patch_net_param;
    CHECK_OK(param_manager().GetProtoParam(kImagePatchNetParamKey,
                                           &image_patch_net_param));

    QLOG(INFO) << "Initializing image patch net... Done!";
    human_pipeline_manager_->InitImagePatchClassifier(
        MutableLiteClient(), run_params_, image_patch_net_param);
  }

  QLOG(INFO) << "Initializing mist net v1...";
  NetParam mist_net_v1_param;
  CHECK_OK(
      param_manager().GetProtoParam(kMistNewNetParamKey, &mist_net_v1_param));
  QLOG(INFO) << "Initializing mist net v1... Done!";
  cluster_filter_ = std::make_unique<ClusterFilter>(
      run_params_, mist_net_v1_param, &thread_pool_, this);

  sensor_fov_builder_ = std::make_unique<sensor_fov::SensorFovBuilder>(
      run_params_, &thread_pool_);

  cluster_observer_ = std::make_unique<ClusterObserver>(&thread_pool_);

  retroreflector_detector_ =
      std::make_unique<RetroreflectorDetector>(&thread_pool_);

  // Initialize mist obstacle filter net.
  if (FLAGS_enable_mof_net_filter) {
    const auto run_context = ModuleRunContext();
    MofNet::kPointsDim =
        !IsDBQConext(run_context) || IsDBQv4(run_context) ? 5 : 6;
    NetParam mof_net_param;
    constexpr char kMofNetParamKey[] = "mof_net_param";
    CHECK_OK(param_manager().GetProtoParam(kMofNetParamKey, &mof_net_param));
    mof_net_ = std::make_unique<MistObstacleNet>(run_params_, mof_net_param);
  }

  // Initialize camera image buffers.
  for (int i = CameraId_MIN; i <= CameraId_MAX; ++i) {
    const CameraId cam_id = static_cast<CameraId>(i);
    if (!IsFunctionCamera(cam_id) || cam_id == CAM_UNKNOWN) {
      continue;
    }
    camera_images_.emplace(std::piecewise_construct, std::make_tuple(cam_id),
                           std::make_tuple(kImageBufferSize));
  }

  obstacle_detector_ = std::make_unique<ObstacleDetector>(
      run_params_, &imagery_manager_, &thread_pool_);
  obstacle_clusterer_ = std::make_unique<ObstacleClusterer>(
      obstacle_detector_->obstacle_grid_width(),
      obstacle_detector_->obstacle_grid_height(), &thread_pool_);

  obstacle_semantic_manager_ = std::make_unique<ObstacleSemanticManager>(
      obstacle_detector_->obstacle_grid_width(),
      obstacle_detector_->obstacle_grid_height());

  laser_embedding_manager_ = std::make_unique<tracker::LaserEmbeddingManager>(
      run_params_, param_manager());

  // Initialize ACE Associator
  if (FLAGS_enable_ace_association) {
    NetParam ace_association_net_param;
    CHECK_OK(param_manager().GetProtoParam(kAceAssociationNetParamKey,
                                           &ace_association_net_param));
    ace_associator_ =
        std::make_unique<AceAssociator>(run_params_, ace_association_net_param);
  }

  if (!IsDSimMode()) {
    serial_executor_ = &serial_executor_thread_pool_;
    serial_executor_future_ = ScheduleFuture(
        serial_executor_,
        std::bind(&PerceptionModule::RunWorkLoop, this, /*try_once=*/false));
  }
}

void PerceptionModule::EmitLidarFrameBacklogIssue(
    const LidarFrame& lidar_frame,
    const boost::circular_buffer<LidarFrame>& lidar_frame_buffer) {
  const double earlist_lidar_frame_ts =
      lidar_frame_buffer.front().StartTimestamp();
  const double current_lidar_frame_ts = lidar_frame.StartTimestamp();
  std::string args_message = absl::StrFormat(
      "[Perception][%s] Too many lidar frames waiting for processing. "
      "earlist lidar frame: %.4f, current lidar_frame: %.4f\n",
      LidarId_Name(lidar_frame.lidar_id()), earlist_lidar_frame_ts,
      current_lidar_frame_ts);
  args_message += "  [In lidar info map]:  ";
  for (const auto& [lidar_id, lidar_info] : lidar_info_map_) {
    args_message +=
        absl::StrFormat("[%s] info: {status [%s] latest frame start ts %.4f}\n",
                        LidarId_Name(lidar_id), lidar_info.StatusString(),
                        lidar_info.latest_start_timestamp);
  }
  args_message += "  [In lidar frame buffer]:  ";
  for (const auto& [lidar_id, lidar_frames] : lidar_frame_buffer_) {
    args_message += absl::StrFormat(
        "[%s] frame size: %d", LidarId_Name(lidar_id), lidar_frames.size());
    if (!lidar_frames.empty()) {
      args_message += " with frames start timestamp: ";
      for (const auto& lidar_frame : lidar_frames) {
        args_message += absl::StrFormat(" %.4f ", lidar_frame.StartTimestamp());
      }
    } else {
      args_message += "\n";
    }
  }
  QISSUEX_WITH_ARGS(
      QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
      QIssueSubType::QIST_PERCEPTION_LIDAR_FRAME_BACKLOG,
      "Check perception process speed: SLOW(too many lidar frames)",
      args_message);
}

void PerceptionModule::OnSubscribeChannels() {
  if (IsOnboardMode()) {
    Subscribe(&PerceptionModule::UpdatePose, this, "pose_proto");
  } else {
    Subscribe(&PerceptionModule::UpdatePose, this, "sensor_pose");
  }
  Subscribe(&PerceptionModule::UpdateLocalizationTransform, this);
  Subscribe(&PerceptionModule::UpdateLidarHostTimeDiff, this,
            "lidar_host_time_diff_proto");
  Subscribe(&PerceptionModule::UpdateLabels, this);
  Subscribe(&PerceptionModule::UpdateRadarMeasurementsDeprecated, this,
            "radar_measurements_proto");
  Subscribe(&PerceptionModule::UpdateRadarMeasurements, this,
            "radar_measurements");
  Subscribe(&PerceptionModule::UpdateSemanticSegmentationResults, this,
            "semantic_segmentation_results_proto");
  // Subscribe lidar frames.
  SubscribeLidarFrame([this](const LidarFrame& lidar_frame) {
    // NOTE(dong): Skip LDR_REAR as it's only used for temporary test.
    if (lidar_frame.lidar_id() == LDR_REAR) return;
    const SpinMetadata& spin_meta = lidar_frame.lidar_frame_metadata();
    // Only process full lidar frames for now.
    if (spin_meta.is_partial()) {
      if (FLAGS_fen_collect_point_async && lidar_frame.is_spin()) {
        // Use disposal pool to serialize partial spin points collection from
        // all lidars.
        fen_classifier_->CollectSpinAsync(
            lidar_frame, spin_meta.num_scans(),
            FLAGS_enable_pfen
                ? CollectSyncedSemanticSegmentationResults({lidar_frame},
                                                           /* 150ms */ 0.15)
                : SemanticSegmentationResults(),
            camera_params_,
            IsDSimMode() ? nullptr : ThreadPool::DisposalPool());
      }
      return;
    }

    SCOPED_QTRACE("PerceptionModule::OnNewLidarFrame");

    absl::MutexLock lock(&lidar_frame_mutex_);

    // TODO(dong): check lidar info healthy status later.
    lidar_info_map_[spin_meta.id()] = {LidarInfo::kHealthy,
                                       lidar_frame.StartTimestamp()};

    auto& lidar_frame_buffer = lidar_frame_buffer_[spin_meta.id()];
    if (lidar_frame_buffer.size() == kMaxNumLidarFramesInBuffer) {
      EmitLidarFrameBacklogIssue(lidar_frame, lidar_frame_buffer);
    }

    if (lidar_frame_buffer.capacity() == 0) {
      lidar_frame_buffer.set_capacity(kMaxNumLidarFramesInBuffer);
    }
    lidar_frame_buffer.push_back(lidar_frame);
    lidar_frame_cond_var_.Signal();
  });

  // Subscribe images.
  SubscribeDecodedImage([this](const CameraImage& camera_image) {
    const auto camera_id = camera_image.camera_id();
    // Skip the camera if it's from auxiliary camera.
    if (!IsFunctionCamera(camera_id) || camera_id == CAM_UNKNOWN) {
      return;
    }

    SCOPED_QTRACE("PerceptionModule::OnNewImage");

    AppendTaskToQueue(
        [this, camera_image, camera_id] {
          const double image_center_timestamp_lidar_time =
              camera_image.center_timestamp() + lidar_host_time_diff_;

          // Find the closest pose to image_center_timestamp_lidar_time
          const auto image_pose = ComputeEstimatedVehiclePose(
              pose_history_, image_center_timestamp_lidar_time);
          if (!image_pose.has_value()) {
            QLOG(ERROR) << "Cannot compute the pose of the image from "
                        << CameraId_Name(camera_id);
            return;
          }

          const auto smooth_to_camera_transform =
              ComputeVehicleToCameraTransform(camera_id) *
              image_pose->ToTransform().Inverse();
          auto camera_image_with_pose = camera_image;
          camera_image_with_pose.set_pose(*image_pose);
          camera_images_.at(camera_id).push_back(std::make_pair(
              std::move(camera_image_with_pose), smooth_to_camera_transform));
        },
        "decoded_image", CameraId_Name(camera_id));
  });
}

void PerceptionModule::OnSetUpTimers() {
  if (IsDSimMode()) {
    AddTimerOrDie(
        "process_spins_timer", [=] { RunWorkLoop(/*try_once=*/true); },
        absl::Milliseconds(20), false /*=one_shot*/);
  }
}

void PerceptionModule::AppendTaskToQueue(std::function<void()>&& task,
                                         const std::string& task_field,
                                         const std::string& task_channel) {
  SCOPED_QTRACE_ARG1("PerceptionModule::AppendTaskToQueue", "task_field",
                     task_field);
  absl::MutexLock lock(&queued_tasks_mutex_);
  const std::string& task_name =
      task_channel.empty() ? task_field : task_field + "_" + task_channel;
  auto& queued_tasks = queued_tasks_of_msgs_[task_name];
  queued_tasks.emplace(ToUnixDoubleSeconds(Clock::Now()), std::move(task));
  const int max_queue_size = FindOrDie(kMaxNumTasksInQueue, task_field);
  while (queued_tasks.size() > max_queue_size) {
    std::string all_queues_size_info;
    for (const auto& [name, queue] : queued_tasks_of_msgs_) {
      all_queues_size_info +=
          absl::StrFormat("[%s] current queue size (%d). ", name, queue.size());
    }
    std::string args_message = absl::StrFormat(
        "[%s] size (%d) exceeds the maximum queue size (%d). "
        "Old tasts will be dropped. All queues size: %s",
        task_name, queued_tasks.size(), max_queue_size, all_queues_size_info);
    if (lidar_frame_cond_var_is_waiting_) {
      args_message = absl::StrFormat(
          "Is waiting for lidar frames in RunWorkLoop. Need to check why lidar "
          "frames is not ready. %s",
          args_message);
    }
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_PIPELINE,
                      QIssueSubType::QIST_PERCEPTION_LITE_MSG_BACKLOG,
                      "Check perception process speed: SLOW(too many messages)",
                      args_message);
    queued_tasks.pop();
  }
}

void PerceptionModule::RunAllTasksInQueue() {
  SCOPED_QTRACE("PerceptionModule::RunAllTasksInQueue");
  absl::MutexLock lock(&queued_tasks_mutex_);
  std::vector<std::pair<double, std::function<void()>>> all_queued_tasks;
  all_queued_tasks.reserve(240);
  for (auto& [task_name, queued_tasks] : queued_tasks_of_msgs_) {
    while (!queued_tasks.empty()) {
      all_queued_tasks.emplace_back(std::move(queued_tasks.front()));
      queued_tasks.pop();
    }
  }
  std::sort(
      all_queued_tasks.begin(), all_queued_tasks.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  for (auto& task : all_queued_tasks) {
    task.second();
  }
}

void PerceptionModule::UpdatePose(std::shared_ptr<const PoseProto> pose) {
  SCOPED_QTRACE("PerceptionModule::UpdatePose");
  AppendTaskToQueue(
      [this, pose] {
        latest_pose_ = *pose;
        semantic_map_manager_.UpdateSmoothPos(
            Vec2d{pose->pos_smooth().x(), pose->pos_smooth().y()});
        semantic_map_manager_.ApplyUpdate();
        pose_history_.push_back(
            std::make_pair(pose->timestamp(), VehiclePose(*pose)));
      },
      ProtoMessageToName<PoseProto>());
}

void PerceptionModule::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("PerceptionModule::UpdateLocalizationTransform");
  AppendTaskToQueue(
      [=] {
        coordinate_converter_.UpdateLocalizationTransform(
            *localization_transform_proto);
        semantic_map_manager_.UpdateLocalizationTransform(
            *localization_transform_proto, &thread_pool_);
      },
      ProtoMessageToName<LocalizationTransformProto>());
}
void PerceptionModule::UpdateRadarMeasurementsDeprecated(
    std::shared_ptr<const RadarMeasurementsProto>
        raw_radar_measurements_proto) {
  SCOPED_QTRACE("PerceptionModule::UpdateRadarMeasurements");
  // NOTE(zheng): The radar measurement type in old data is not
  // accurate(always be unknown static), it may cause some trouble
  // in prt, so we modify it to unknown type
  // for offline sim.
  auto radar_measurements_proto = *raw_radar_measurements_proto;
  for (int i = 0; i < radar_measurements_proto.radar_measurements_size(); ++i) {
    radar_measurements_proto.mutable_radar_measurements(i)->set_type(
        RadarObjectType::ROT_UNKNOWN_MOVABLE);
  }
  AppendTaskToQueue(
      [=] {
        const auto radar_id = radar_measurements_proto.radar_id();
        std::vector<RadarMeasurementProto> radar_measurements;
        for (const auto& radar_m :
             radar_measurements_proto.radar_measurements()) {
          radar_measurements.emplace_back(radar_m);
        }
        radar_measurements_buffer_[radar_id].PushBackAndClearStale(
            radar_measurements_proto.timestamp(), radar_measurements,
            kRadarMeasurementsBufferLength);
      },
      ProtoMessageToName<RadarMeasurementsProto>());

  MeasurementsProto measurements;
  double rmin_timestamp = DBL_MAX;
  double rmax_timestamp = -DBL_MAX;
  AddMeasurementsFromRadar(radar_measurements_proto, &rmin_timestamp,
                           &rmax_timestamp, &measurements);
  measurements.set_min_timestamp(rmin_timestamp);
  measurements.set_max_timestamp(rmax_timestamp);
  measurements.set_group_type(MeasurementsProto::RADAR);

  QLOG_IF_NOT_OK(WARNING, Publish(measurements));
}

void PerceptionModule::UpdateRadarMeasurements(
    std::shared_ptr<const MeasurementsProto> measurements_proto) {
  SCOPED_QTRACE("PerceptionModule::UpdateRadarMeasurements");
  AppendTaskToQueue(
      [=] {
        std::vector<RadarMeasurementProto> radar_measurements;
        std::optional<RadarId> radar_id;
        for (const auto& radar_m : measurements_proto->measurements()) {
          if (radar_m.has_radar_measurement()) {
            if (!radar_id.has_value()) {
              radar_id = radar_m.radar_measurement().radar_id();
            }
            radar_measurements.push_back(radar_m.radar_measurement());
          }
        }
        if (radar_id.has_value()) {
          radar_measurements_buffer_[radar_id.value()].PushBackAndClearStale(
              measurements_proto->min_timestamp(), radar_measurements,
              kRadarMeasurementsBufferLength);
        }
      },
      ProtoMessageToName<MeasurementsProto>());
}

std::unordered_map<RadarId, std::vector<RadarMeasurementProto>>
PerceptionModule::CollectRadarMeasurements(const double timestamp) {
  std::unordered_map<RadarId, std::vector<RadarMeasurementProto>>
      radar_measurements;
  for (const auto& [radar_id, radar_data] : radar_measurements_buffer_) {
    const auto lower_iter = std::lower_bound(
        radar_data.begin(), radar_data.end(), timestamp,
        [](const auto& data, double t) { return data.first < t; });
    if (lower_iter != radar_data.end()) {
      const int index = std::distance(radar_data.begin(), lower_iter);
      radar_measurements[radar_id] = radar_data[index].second;
    } else {
      if (!radar_data.empty()) {
        radar_measurements[radar_id] = radar_data.back().second;
      }
    }
  }
  return radar_measurements;
}

AffineTransformation PerceptionModule::ComputeVehicleToCameraTransform(
    CameraId camera_id) const {
  if (!ContainsKey(camera_params_, camera_id)) {
    return AffineTransformation();
  }
  const auto& camera_param = FindOrDie(camera_params_, camera_id);
  return camera_param.camera_to_vehicle_extrinsics().ToTransform().Inverse();
}

SemanticSegmentationResults
PerceptionModule::CollectSyncedSemanticSegmentationResults(
    const std::vector<LidarFrame>& lidar_frames,
    const double obsolete_threshold) const {
  SemanticSegmentationResults semseg_results;
  semseg_results.reserve(semantic_segmentation_results_buffer_.size());
  for (const auto& [camera_id, semseg_result_buffer] :
       semantic_segmentation_results_buffer_) {
    if (semseg_result_buffer.empty()) continue;

    const auto& camera_param = FindOrDie(camera_params_, camera_id);
    const auto lidar_mid_timestamp =
        projection_util::FindRefLidarMidTimestamp(camera_param, lidar_frames);

    // Find closest semseg result to lidar timestamp.
    int i = semseg_result_buffer.GetIndexWithTimeAtLeast(lidar_mid_timestamp);
    if (i == semseg_result_buffer.size() ||
        (i != 0 &&
         std::abs(semseg_result_buffer[i - 1].first - lidar_mid_timestamp) <
             std::abs(semseg_result_buffer[i].first - lidar_mid_timestamp))) {
      --i;
    }
    const auto& semseg_result = semseg_result_buffer.value(i);

    if (std::abs(lidar_mid_timestamp -
                 semseg_result->image_center_timestamp()) >
        obsolete_threshold) {
      QLOG(INFO) << absl::StrFormat(
          "[SemSegResult] Can't find synced lidar frame for [%s %.3f], ref "
          "lidar info [%s %.3f].",
          CameraId_Name(camera_id), semseg_result->image_center_timestamp(),
          LidarId_Name(camera_param.ref_lidar()), lidar_mid_timestamp);
      continue;
    }

    semseg_results.emplace_back(semseg_result);
  }

  return semseg_results;
}

void PerceptionModule::UpdateLabels(
    std::shared_ptr<const labeling::LabelFrameProto> label_frame) {
  SCOPED_QTRACE("PerceptionModule::UpdateLabels");
  AppendTaskToQueue(
      [=] {
        const auto& label_pose_ref_lidars = labeling::GetLabelPoseRefLidars();
        std::optional<VehiclePose> lidar_frame_pose;
        {
          absl::MutexLock lock(&lidar_frame_mutex_);
          for (const auto& [lidar_id, lidar_frames] : lidar_frame_buffer_) {
            if (ContainsKey(label_pose_ref_lidars, lidar_id) &&
                !lidar_frames.empty()) {
              const double time_diff =
                  lidar_frames.back().MidTimestamp() - label_frame->timestamp();
              constexpr double kLabelLidarMaxTimeDiff = 0.005;  // s
              if (time_diff > 0 ||
                  std::abs(time_diff) < kLabelLidarMaxTimeDiff) {
                lidar_frame_pose = lidar_frames.back().MidPose();
                break;
              }
            }
          }
        }
        if (!lidar_frame_pose) {
          QLOG_EVERY_N_SEC(WARNING, 1.0)
              << "Can't find ref lidar pose for current label.";
          return;
        }
        latest_label_frame_ = std::make_shared<const labeling::LabelFrameProto>(
            labeling::GenerateLabelsAndZonesFromCurrentPose(*label_frame,
                                                            *lidar_frame_pose));
      },
      ProtoMessageToName<labeling::LabelFrameProto>());
}

void PerceptionModule::UpdateLidarHostTimeDiff(
    std::shared_ptr<const LidarHostTimeDiffProto> lidar_host_time_diff_proto) {
  SCOPED_QTRACE("PerceptionModule::UpdateLidarHostTimeDiff");
  AppendTaskToQueue(
      [this, lidar_host_time_diff_proto] {
        lidar_host_time_diff_ = lidar_host_time_diff_proto->time_diff();
      },
      ProtoMessageToName<LidarHostTimeDiffProto>());
}

void PerceptionModule::UpdateSemanticSegmentationResults(
    std::shared_ptr<const SemanticSegmentationResultsProto> ss_results) {
  SCOPED_QTRACE("PerceptionModule::UpdateSemanticSegmentationResults");
  AppendTaskToQueue(
      [this, ss_results, lidar_host_time_diff = lidar_host_time_diff_] {
        for (const auto& ss_result : ss_results->results()) {
          const auto& camera_id = ss_result.camera_id();
          const double camera_center_timestamp_lidar_time =
              ss_result.image_center_timestamp() + lidar_host_time_diff;
          semantic_segmentation_results_buffer_[camera_id]
              .PushBackAndClearStale(
                  camera_center_timestamp_lidar_time,
                  std::make_shared<SemanticSegmentationResult>(
                      ss_result, FindOrDie(camera_params_, camera_id)),
                  kSemanticSegmentationResultBufferLength);
        }
      },
      ProtoMessageToName<SemanticSegmentationResultsProto>());
}

void PerceptionModule::PublishDetectionResult(
    const FieryEyeNetClassifier::DetectionResult& fen_result,
    const HumanPipelineManager::PedFilteringResult& ped_filtering_result,
    const double lidar_timestamp, const VehiclePose& pose) {
  SCOPED_QTRACE("PerceptionModule::PublishDetectionResult");

  FenDetectionsProto fen_detections_proto;
  fen_detections_proto.set_timestamp(lidar_timestamp);
  const auto add_detection_boxes =
      [&](const auto& det_boxes, const auto type,
          const std::vector<std::string>& skip_infos,
          const std::vector<std::string>& track_infos) {
        for (int i = 0; i < det_boxes.size(); i++) {
          const auto& det_box = det_boxes[i];
          auto* detection = fen_detections_proto.add_detections();
          SetBox2dProto(det_box.box, detection->mutable_bounding_box());
          detection->set_type(type);
          detection->set_confidence_score(det_box.score);
          detection->set_velocity_x(det_box.velocity.x());
          detection->set_velocity_y(det_box.velocity.y());
          detection->set_front_edge_confidence(det_box.edge_confidence(0));
          detection->set_back_edge_confidence(det_box.edge_confidence(1));
          detection->set_left_edge_confidence(det_box.edge_confidence(2));
          detection->set_right_edge_confidence(det_box.edge_confidence(3));
          detection->set_motion_state(det_box.motion_state);
          detection->set_heading_confidence(det_box.heading_confidence);
          detection->set_velocity_x_confidence(det_box.velocity_confidence(0));
          detection->set_velocity_y_confidence(det_box.velocity_confidence(1));
          detection->set_box_front(det_box.box_front);
          detection->set_box_back(det_box.box_back);
          detection->set_box_left(det_box.box_left);
          detection->set_box_right(det_box.box_right);
          // NOTE(yanlun) : Add box z height
          detection->set_box_z(det_box.box_z);
          detection->set_box_height(det_box.box_height);
          if (!track_infos.empty()) {
            detection->set_pcn_track_info(track_infos[i]);
          }
          if (!skip_infos.empty()) {
            detection->set_pcn_skip_info(skip_infos[i]);
          }
        }
      };
  add_detection_boxes(fen_result.car_boxes, FenDetectionProto::CAR, {}, {});
  add_detection_boxes(fen_result.ped_boxes, FenDetectionProto::PED,
                      ped_filtering_result.ped_skip_infos,
                      ped_filtering_result.ped_track_infos);
  add_detection_boxes(fen_result.cyc_boxes, FenDetectionProto::CYC,
                      ped_filtering_result.cyc_skip_infos,
                      ped_filtering_result.cyc_track_infos);
  add_detection_boxes(fen_result.pole_boxes, FenDetectionProto::POLE, {}, {});

  const auto add_low_score_detection_boxes = [&](const auto& det_boxes,
                                                 const auto type) {
    for (int i = 0; i < det_boxes.size(); i++) {
      const auto& det_box = det_boxes[i];
      auto* detection = fen_detections_proto.add_low_score_detections();
      SetBox2dProto(det_box.box, detection->mutable_bounding_box());
      detection->set_type(type);
      detection->set_confidence_score(det_box.score);
      detection->set_velocity_x(det_box.velocity.x());
      detection->set_velocity_y(det_box.velocity.y());
      detection->set_front_edge_confidence(det_box.edge_confidence(0));
      detection->set_back_edge_confidence(det_box.edge_confidence(1));
      detection->set_left_edge_confidence(det_box.edge_confidence(2));
      detection->set_right_edge_confidence(det_box.edge_confidence(3));
      detection->set_motion_state(det_box.motion_state);
      detection->set_heading_confidence(det_box.heading_confidence);
      detection->set_velocity_x_confidence(det_box.velocity_confidence(0));
      detection->set_velocity_y_confidence(det_box.velocity_confidence(1));
      detection->set_box_front(det_box.box_front);
      detection->set_box_back(det_box.box_back);
      detection->set_box_left(det_box.box_left);
      detection->set_box_right(det_box.box_right);

      // NOTE(yanlun) : Add box z height
      detection->set_box_z(det_box.box_z);
      detection->set_box_height(det_box.box_height);
    }
  };

  add_low_score_detection_boxes(fen_result.low_score_car_boxes,
                                FenDetectionProto::CAR);
  add_low_score_detection_boxes(fen_result.low_score_ped_boxes,
                                FenDetectionProto::PED);
  add_low_score_detection_boxes(fen_result.low_score_cyc_boxes,
                                FenDetectionProto::CYC);
  add_low_score_detection_boxes(fen_result.low_score_pole_boxes,
                                FenDetectionProto::POLE);
  const auto add_nms_suppressed_boxes = [&](const auto& boxes,
                                            const auto type) {
    for (const auto& det_box : boxes) {
      const auto& box = det_box.box;
      const auto& score = det_box.score;

      auto* detection = fen_detections_proto.add_detections();
      SetBox2dProto(box, detection->mutable_bounding_box());
      detection->set_type(type);
      detection->set_confidence_score(score);
      detection->set_filtered_by_nms(true);
      detection->set_velocity_x(det_box.velocity.x());
      detection->set_velocity_y(det_box.velocity.y());
      detection->set_front_edge_confidence(det_box.edge_confidence(0));
      detection->set_back_edge_confidence(det_box.edge_confidence(1));
      detection->set_left_edge_confidence(det_box.edge_confidence(2));
      detection->set_right_edge_confidence(det_box.edge_confidence(3));
    }
  };
  add_nms_suppressed_boxes(fen_result.suppressed_car_boxes,
                           FenDetectionProto::CAR);
  add_nms_suppressed_boxes(fen_result.suppressed_ped_boxes,
                           FenDetectionProto::PED);
  add_nms_suppressed_boxes(fen_result.suppressed_cyc_boxes,
                           FenDetectionProto::CYC);

  const auto add_deleted_detection_boxes = [&](const auto& boxes,
                                               const auto type) {
    for (const auto& box_score : boxes) {
      const auto& box =
          std::get<FieryEyeNetClassifier::DetectionBox>(box_score);
      const auto& pcn_score = std::get<float>(box_score);
      auto* detection = fen_detections_proto.add_detections();
      SetBox2dProto(box.box, detection->mutable_bounding_box());
      detection->set_type(type);
      detection->set_confidence_score(box.score);
      detection->set_velocity_x(box.velocity.x());
      detection->set_velocity_y(box.velocity.y());
      detection->set_filtered_by_pcn(true);
      detection->set_pcn_score(pcn_score);
      detection->set_front_edge_confidence(box.edge_confidence(0));
      detection->set_back_edge_confidence(box.edge_confidence(1));
      detection->set_left_edge_confidence(box.edge_confidence(2));
      detection->set_right_edge_confidence(box.edge_confidence(3));
    }
  };
  add_deleted_detection_boxes(ped_filtering_result.deleted_ped_boxes,
                              FenDetectionProto::PED);
  add_deleted_detection_boxes(ped_filtering_result.deleted_cyc_boxes,
                              FenDetectionProto::CYC);
  auto* det_range = fen_detections_proto.mutable_det_range();
  auto* ped_range = fen_detections_proto.mutable_ped_det_range();
  det_range->set_front(fen::kDetectionRegionFront);
  det_range->set_behind(fen::kDetectionRegionBehind);
  det_range->set_left(fen::kDetectionRegionLeft);
  det_range->set_right(fen::kDetectionRegionRight);

  ped_range->set_front(fen::kPedDetectionRegionFront);
  ped_range->set_behind(fen::kPedDetectionRegionBehind);
  ped_range->set_left(fen::kPedDetectionRegionLeft);
  ped_range->set_right(fen::kPedDetectionRegionRight);

  const auto add_pillar_semantic_result =
      [&](const cv::Mat& pillar_semantic_mat, const VehiclePose& pose) {
        std::string compressed_pillar_semantic_mat;
        snappy::Compress(reinterpret_cast<char*>(pillar_semantic_mat.data),
                         pillar_semantic_mat.cols * pillar_semantic_mat.rows,
                         &compressed_pillar_semantic_mat);
        auto* pillar_semantic_result =
            fen_detections_proto.mutable_pillar_semantic_result();
        pillar_semantic_result->set_rows(pillar_semantic_mat.rows);
        pillar_semantic_result->set_cols(pillar_semantic_mat.cols);
        pillar_semantic_result->set_element_type(pillar_semantic_mat.type());
        pillar_semantic_result->set_element_data(
            compressed_pillar_semantic_mat);

        const auto add_current_pose = [&](const VehiclePose& pose) {
          auto* current_pose = pillar_semantic_result->mutable_current_pose();
          current_pose->set_x(pose.x);
          current_pose->set_y(pose.y);
          current_pose->set_z(pose.z);
          current_pose->set_yaw(pose.yaw);
          current_pose->set_pitch(pose.pitch);
          current_pose->set_roll(pose.roll);
        };

        add_current_pose(pose);
      };

  add_pillar_semantic_result(fen_result.bev_seg, pose);
  QLOG_IF_NOT_OK(WARNING, Publish(fen_detections_proto));
}

void PerceptionModule::CreateMeasurementsFromLaser(
    const VehiclePose& pose, const SegmentedClusters& segmented_clusters,
    const std::vector<tracker::LaserEmbeddingManager::LaserFeatureEmbedding>&
        laser_feature_embeddings,
    const std::vector<std::vector<float>>& association_feature_embeddings,
    MeasurementsProto* measurements) const {
  SCOPED_QTRACE("PerceptionModule::CreateMeasurementsFromLaser");
  QCHECK_NOTNULL(measurements);
  QCHECK_EQ(segmented_clusters.size(), laser_feature_embeddings.size());
  QCHECK_EQ(segmented_clusters.size(), association_feature_embeddings.size());

  measurements->Clear();
  measurements->mutable_measurements()->Reserve(segmented_clusters.size());

  double min_timestamp = DBL_MAX;
  double max_timestamp = -DBL_MAX;
  for (int i = 0; i < segmented_clusters.size(); ++i) {
    const auto& cluster = segmented_clusters[i];
    min_timestamp = std::min(cluster.timestamp(), min_timestamp);
    max_timestamp = std::max(cluster.timestamp(), max_timestamp);
    measurements->mutable_measurements()->Add();
  }

  ParallelFor(0, segmented_clusters.size(), &thread_pool_, [&](int i) {
    const auto& cluster = segmented_clusters[i];
    const auto& point_feature_embeddings =
        laser_feature_embeddings[i].point_feature;
    const auto& image_feature_embeddings =
        laser_feature_embeddings[i].image_feature;
    const auto& association_feature = association_feature_embeddings[i];
    auto& measurement = *measurements->mutable_measurements(i);
    auto& laser_measurement = *measurement.mutable_laser_measurement();

    measurement.set_timestamp(cluster.timestamp());
    const auto contour =
        cluster_util::ComputeContourWithRefinement(pose, cluster);
    for (const auto& point : contour.points()) {
      auto& contour_point = *laser_measurement.add_contour();
      contour_point.set_x(point.x());
      contour_point.set_y(point.y());
    }
    // Set fen velocity.
    const auto& cluster_fen_velocity = cluster.fen_velocity();
    if (cluster_fen_velocity) {
      auto& fen_velocity_proto = *laser_measurement.mutable_fen_velocity();
      fen_velocity_proto.set_x(cluster_fen_velocity->x());
      fen_velocity_proto.set_y(cluster_fen_velocity->y());
    }
    std::optional<Box2d> bounding_box = cluster.bounding_box();
    std::optional<Box2d> refined_box;
    if (bounding_box) {
      // In the future we maybe tunning bounding box according
      // to contour, and use it in tracker module, so we compute
      // refined bbox here.
      refined_box = ComputeRefinedBbox(cluster, contour, pose);
    }

    if (bounding_box) {
      auto& bb = *laser_measurement.mutable_detection_bounding_box();
      bb.set_x(bounding_box->center().x());
      bb.set_y(bounding_box->center().y());
      bb.set_heading(bounding_box->heading());
      bb.set_length(bounding_box->length());
      bb.set_width(bounding_box->width());

      auto& min_area_bb = *laser_measurement.mutable_min_area_bounding_box();
      Box2d min_area_bounding_box =
          contour.BoundingBoxWithHeading(bounding_box->heading());
      min_area_bb.set_x(min_area_bounding_box.center().x());
      min_area_bb.set_y(min_area_bounding_box.center().y());
      min_area_bb.set_heading(min_area_bounding_box.heading());
      min_area_bb.set_length(min_area_bounding_box.length());
      min_area_bb.set_width(min_area_bounding_box.width());

      auto& refined_bbox = *laser_measurement.mutable_refined_bounding_box();
      refined_bbox.set_x(refined_box->center().x());
      refined_bbox.set_y(refined_box->center().y());
      refined_bbox.set_heading(refined_box->heading());
      refined_bbox.set_length(refined_box->length());
      refined_bbox.set_width(refined_box->width());
    }

    auto& cluster_measurement =
        *laser_measurement.mutable_cluster_measurement();
    cluster_measurement.set_height(cluster.ComputeHeight());
    cluster_measurement.set_near_curb(ClusterIsOnRoadSide(cluster));
    cluster_measurement.set_cluster_id(cluster.id());
    cluster_measurement.set_timestamp(cluster.timestamp());
    cluster_measurement.set_clearance(cluster.ComputeClearance());
    cluster_measurement.set_num_points(cluster.NumPoints());
    cluster_measurement.set_ground_z(cluster.ComputeGroundZ());

    double min_dist_to_curb = std::numeric_limits<double>::max();
    double max_dist_to_curb = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto* obstacle : cluster.obstacles()) {
      min_dist_to_curb =
          std::min<double>(min_dist_to_curb, obstacle->dist_to_curb);
      max_dist_to_curb =
          std::max<double>(max_dist_to_curb, obstacle->dist_to_curb);
      auto* obstacle_info = cluster_measurement.add_obstacle_info();
      obstacle_info->mutable_center()->set_x(obstacle->x);
      obstacle_info->mutable_center()->set_y(obstacle->y);
      obstacle_info->set_min_z(obstacle->min_z);
      obstacle_info->set_max_z(obstacle->max_z);
      obstacle_info->set_num_points(obstacle->points.size());
      min_z = std::min<double>(min_z, obstacle->min_z);
      max_z = std::max<double>(max_z, obstacle->max_z);
    }
    QCHECK_GT(cluster.NumObstacles(), 0);
    cluster_measurement.set_min_dist_to_curb(min_dist_to_curb);
    cluster_measurement.set_max_dist_to_curb(max_dist_to_curb);
    cluster_measurement.set_min_z(min_z);
    cluster_measurement.set_max_z(max_z);

    if (cluster.icp_matched()) {
      auto& icp_measurement = *laser_measurement.mutable_icp_measurement();
      icp_measurement.mutable_vel()->set_x(cluster.icp_velocity().x());
      icp_measurement.mutable_vel()->set_y(cluster.icp_velocity().y());
      auto* icp_vel_cov = icp_measurement.mutable_velocity_covariance();
      Mat2dToProto(cluster.icp_velocity_covariance(), icp_vel_cov);
      icp_measurement.set_mse(cluster.icp_mse());
      icp_measurement.set_matched_prev_cluster_id(
          cluster.matched_prev_cluster_id());
      icp_measurement.set_matched_prev_cluster_timestamp(
          cluster.matched_prev_cluster_timestamp());
      icp_measurement.set_matched_curr_cluster_timestamp(cluster.timestamp());
    }
    // TODO(zheng, yu): Set MT_UNKNOWN as MT_UNKNOWN, set MT_UNKNOWN as
    // MT_STATIC_OBJECT for now as a NFC change since we were set MT_UNKNOWN as
    // MT_STATIC_OBJECT.
    measurement.set_type(cluster.type() == MT_UNKNOWN ? MT_STATIC_OBJECT
                                                      : cluster.type());
    measurement.set_type_source(cluster.type_source());
    if (!point_feature_embeddings.empty()) {
      *laser_measurement.mutable_point_feature_embeddings() = {
          point_feature_embeddings.begin(), point_feature_embeddings.end()};
    }
    if (!image_feature_embeddings.empty()) {
      *laser_measurement.mutable_image_feature_embeddings() = {
          image_feature_embeddings.begin(), image_feature_embeddings.end()};
    }
    if (!association_feature.empty()) {
      *laser_measurement.mutable_association_embeddings() = {
          association_feature.begin(), association_feature.end()};
    }
    laser_measurement.set_observation_state(cluster.observation_state());
  });

  measurements->set_min_timestamp(min_timestamp);
  measurements->set_max_timestamp(max_timestamp);
  measurements->set_group_type(MeasurementsProto::LIDAR);
  const auto occluded_info_list = measurement_util::GetOccludedMeasurementIndex(
      pose, lidar_params_, *measurements);
  for (int i = 0; i < measurements->measurements_size(); ++i) {
    auto* m = measurements->mutable_measurements(i);
    if (measurement_util::ShouldRefineLaserDetectionBBox(
            *m, lidar_params_, pose, occluded_info_list[i])) {
      measurement_util::RefineLaserMeasurementBoundingBox(pose, lidar_params_,
                                                          m);
    }
  }
}

void PerceptionModule::AddMeasurementsFromRadar(
    const RadarMeasurementsProto& radar_measurements, double* min_timestamp,
    double* max_timestamp, MeasurementsProto* measurements) {
  SCOPED_QTRACE("PerceptionModule::AddMeasurementsFromRadar");

  for (int i = 0; i < radar_measurements.radar_measurements_size(); ++i) {
    const auto& radar_obj = radar_measurements.radar_measurements(i);
    // TODO(zheng): Filter offroad radar measurement in tracker module.
    // NOTE(zheng): The static radar objects may be a false positive,
    // so we should filter out static radar objects.
    constexpr double kVelNoiseForStaticObjectSqr = 0.04;  // 0.2 m/s
    const auto radar_vel = Vec2dFromProto(radar_obj.vel());
    if (radar_vel.squaredNorm() < kVelNoiseForStaticObjectSqr) {
      continue;
    }

    auto* measurement = measurements->add_measurements();
    // This will probably behind current tracker timestamp, which renders
    // tracker roll back necessary.
    *measurement->mutable_radar_measurement() = radar_obj;
    measurement->set_timestamp(radar_measurements.timestamp() +
                               lidar_host_time_diff_);
    *min_timestamp = std::min(measurement->timestamp(), *min_timestamp);
    *max_timestamp = std::max(measurement->timestamp(), *max_timestamp);

    switch (radar_obj.type()) {
      case ROT_CYCLIST:
        measurement->set_type(MT_CYCLIST);
        break;
      case ROT_FOD:
        measurement->set_type(MT_FOD);
        break;
      case ROT_MOTORCYCLIST:
        measurement->set_type(MT_MOTORCYCLIST);
        break;
      case ROT_PEDESTRIAN:
        measurement->set_type(MT_PEDESTRIAN);
        break;
      case ROT_UNKNOWN_MOVABLE:
        measurement->set_type(MT_UNKNOWN);
        break;
      case ROT_UNKNOWN_STATIC:
      case ROT_STATIC_IN_RADIAL:
        measurement->set_type(MT_STATIC_OBJECT);
        break;
      case ROT_VEGETATION:
        measurement->set_type(MT_VEGETATION);
        break;
      case ROT_VEHICLE:
        measurement->set_type(MT_VEHICLE);
        break;
    }
    measurement->set_type_source(MeasurementTypeSource::MTS_RADAR);
  }
}

void PerceptionModule::GetAssociationFeaturesFromLaser(
    std::vector<std::vector<float>>* association_feature_embeddings) {
  SCOPED_QTRACE("PerceptionModule::GetAssociationFeaturesFromLaser");

  int num_batch = 0;
  std::vector<cv::Mat> input_image_patches;
  std::vector<cv::Mat> input_points_patches;
  std::vector<int> image_indices;
  const auto& cluster_data_vec = laser_embedding_manager_->GetClusterDataVec();
  association_feature_embeddings->resize(cluster_data_vec.size());

  if (!FLAGS_enable_ace_association) {
    return;
  }

  for (int i = 0; i < cluster_data_vec.size(); ++i) {
    auto& cluster_image_patch = cluster_data_vec[i].second;
    const auto& image_data = cluster_image_patch.image(cluster_image_patch.roi);
    const auto& projected_points_data = cluster_image_patch.project_points;

    // Skip when there is no points in the cluster
    if (projected_points_data.size() == 0) {
      continue;
    }
    // Skip when the image size is 0
    if (image_data.size().width == 0 || image_data.size().height == 0) {
      continue;
    }

    // Create points image from projected points data
    cv::Mat points_image = cv::Mat::zeros(image_data.size(), CV_8UC1);
    auto points_image_ptr = points_image.ptr<uchar>(0);
    for (const auto& pt : projected_points_data) {
      const int pt_x = pt.second.x();
      const int pt_y = pt.second.y();
      if (pt_x >= points_image.rows || pt_x <= 0 || pt_y >= points_image.cols ||
          pt_y <= 0) {
        continue;
      }
      points_image_ptr = points_image.ptr<uchar>(pt_x);
      points_image_ptr[pt_y] = 1;
    }

    input_points_patches.push_back(points_image);
    input_image_patches.push_back(image_data);
    image_indices.push_back(i);
    num_batch++;

    // TODO(Yao): do this cleverer
    if (num_batch >= ace_associator_->GetMaxBatchSize()) {
      break;
    }
  }

  // Extract association features by ACE associator
  auto out_features = ace_associator_->ExtractAssociationFeatures(
      input_image_patches, input_points_patches);

  // Copy the output data
  const int feature_length = ace_associator_->GetFeatureLength();
  int cur_pos = 0;
  for (const int i : image_indices) {
    (*association_feature_embeddings)[i].resize(feature_length);
    memcpy((*association_feature_embeddings)[i].data(),
           out_features.data() + cur_pos, feature_length * sizeof(float));
    cur_pos += feature_length;
  }
}

void PerceptionModule::PreloadImagery() {
  SCOPED_QTRACE("PerceptionModule::PreloadImagery");
  // Preload imagery.
  const Vec2d pose_global = coordinate_converter_.SmoothToGlobal(
      {latest_pose_.pos_smooth().x(), latest_pose_.pos_smooth().y()});
  const auto preload_imagery_future = imagery_manager_.PreloadAsync(
      pose_global.x(), pose_global.y(),
      /*clear_far_patches=*/true, &imagery_preloading_thread_pool_);
  if (IsDSimMode()) {
    preload_imagery_future.Wait();
  }
  local_imagery_.Update(imagery_manager_, pose_global.x(), pose_global.y());
}

void PerceptionModule::ProcessSpins(
    const std::vector<LidarFrame>& lidar_frames) {
  SCOPED_QTRACE("PerceptionModule::ProcessSpins");

  QCHECK(!lidar_frames.empty());
  const auto run_params = GetRunParams();
  const auto lidar_params = GetCurrentLidarParams();
  const auto& select_lidar_param =
      perception_util::SelectLidarParams(lidar_params);
  const LidarId select_lidar_id = select_lidar_param.installation().lidar_id();
  int selected_index = -1;
  for (int i = 0; i < lidar_frames.size(); ++i) {
    if (lidar_frames[i].lidar_id() == select_lidar_id) {
      selected_index = i;
      break;
    }
  }
  if (selected_index == -1) {
    selected_index = 0;

    QLOG(ERROR) << absl::StrFormat(
        "can't find lidar frame for selected lidar: %s, fallback to lidar: %s",
        LidarId_Name(select_lidar_id),
        LidarId_Name(lidar_frames[selected_index].lidar_id()));
    QEVENT("haijun", "no_frame_found_for_selected_lidar", [=](QEvent* qevent) {
      qevent->AddField("selected_lidar", LidarId_Name(select_lidar_id))
          .AddField("used_lidar",
                    LidarId_Name(lidar_frames[selected_index].lidar_id()))
          .AddField("num_lidars", lidar_params.size())
          .AddField("num_frames", lidar_frames.size());
    });
  }

  const VehiclePose& pose = lidar_frames[selected_index].MidPose();
  const double timestamp = lidar_frames[selected_index].MidTimestamp();

  // NOTE(dong): Allow a larger delay for obstacle manager and segmentation as
  // they currently only deal with static objects.
  auto semantic_segmentation_results =
      CollectSyncedSemanticSegmentationResults(lidar_frames, /* 350ms */ 0.35);

  const auto range_images_future = ScheduleFuture(
      &thread_pool_, [this, &lidar_frames, &semantic_segmentation_results]() {
        SCOPED_QTRACE("PerceptionModule::RangeImageFuture");
        auto range_images =
            std::make_shared<absl::flat_hash_map<LidarId, RangeImage>>();
        if (!FLAGS_enable_range_image) {
          return range_images;
        }
        for (const auto& lidar_frame : lidar_frames) {
          const auto& lidar_param =
              FindOrDie(lidar_params_, lidar_frame.lidar_id());
          const auto selected_semseg_results = range_image_util::
              CollectSemanticSegmentationResultsForCurrentLidar(
                  lidar_frame.lidar_id(), semantic_segmentation_results);
          range_images->try_emplace(lidar_frame.lidar_id(), lidar_param,
                                    lidar_frame, selected_semseg_results);
        }
        return range_images;
      });

  const auto semantic_segmentation_results_for_fen =
      FLAGS_enable_pfen ? CollectSyncedSemanticSegmentationResults(
                              lidar_frames, /* 150ms */ 0.15)
                        : SemanticSegmentationResults();

  const auto fen_future = ScheduleFuture(
      &nn_thread_pool_, [this, &lidar_frames = std::as_const(lidar_frames),
                         &semantic_segmentation_results_for_fen, &pose]() {
        FieryEyeNetClassifier::DetectionResult detection_result;
        if (fen_classifier_ != nullptr) {
          detection_result = fen_classifier_->Classify(
              lidar_frames, semantic_segmentation_results_for_fen,
              camera_params_, pose);
        }
        return std::make_shared<FieryEyeNetClassifier::DetectionResult>(
            std::move(detection_result));
      });

  const auto& vehicle_params = run_params.vehicle_params();
  auto obstacle_manager = obstacle_detector_->DetectObstacles(
      lidar_frames, pose, coordinate_converter_, latest_label_frame_.get(),
      semantic_map_manager_, vehicle_params);
  PublishPoseCorrectionProto(timestamp, obstacle_detector_, this);
  // Wait until the fiery eye net inference to finish, and render its results.
  FieryEyeNetClassifier::DetectionResult fen_result;
  {
    SCOPED_QTRACE("PerceptionModule::GetFenResult");
    fen_result = std::move(*fen_future.Get());
  }
  // NOTE(yu): Make sure classify obstacle with detection after obstacle level
  // filtering in order to promote obstacle in detection correctly.
  if (FLAGS_enable_obstacle_noisy_filter) {
    obstacle_manager->FilterObstacleNoise();
  }
  if (FLAGS_enable_obstacle_mist_filter) {
    obstacle_manager->FilterRainObstacleV2(fen_result);
  }
  if (FLAGS_enable_mof_net_filter) {
    obstacle_manager->MofNetRainFilter(pose, fen_result, mof_net_);
  }
  if (FLAGS_enable_obstacle_semantic_map_filter) {
    obstacle_manager->FilterWithObstacleSemanticMap(pose, semantic_map_manager_,
                                                    coordinate_converter_);
  }

  const ObstaclePtrs all_obstacle_ptrs = obstacle_manager->obstacle_ptrs();

  // Filter FEN pedestrian detections by image patch classifier.
  const auto pcn_future = ScheduleFuture(
      &nn_thread_pool_,
      [&]() -> std::shared_ptr<HumanPipelineManager::PedFilteringResult> {
        auto ped_filtering_result =
            std::make_shared<HumanPipelineManager::PedFilteringResult>();
        if (!FLAGS_enable_pcn) return ped_filtering_result;

        if ((fen_result.ped_boxes.size() + fen_result.cyc_boxes.size()) > 0) {
          std::vector<FieryEyeNetClassifier::DetectionBox> unused_cyc_boxes;
          *ped_filtering_result = human_pipeline_manager_->FilterPedBoxes(
              all_obstacle_ptrs, lidar_frames, camera_images_,
              lidar_host_time_diff_, &fen_result.ped_boxes,
              FLAGS_enable_pcn_filter_cyc ? &fen_result.cyc_boxes
                                          : &unused_cyc_boxes);
        }
        return ped_filtering_result;
      });

  // TODO(yu): Use other sources to classify obstacles as well.
  obstacle_semantic_manager_->ClassifyObstacles(pose, timestamp,
                                                obstacle_manager.get());
  auto ped_filtering_result = pcn_future.Get();
  // Whitelist FEN detections with highest priority since we trust FEN detection
  // more for now.
  // TODO(dong, yu): Remove once have a better fusion strategy in obstacle
  // semantic manager.
  obstacle_manager->WhitelistObstaclesWithDetections(
      fen_result, semantic_segmentation_results, pose);

  PublishDetectionResult(fen_result, *ped_filtering_result, timestamp, pose);

  const auto rc_coord_converter = obstacle_manager->RCCoordConverter();
  auto init_clusters = obstacle_clusterer_->ClusterObstacles(
      all_obstacle_ptrs, rc_coord_converter, pose);

  // Get range images.
  auto range_images = range_images_future.Get();

  // Detect retroreflectors.
  const auto pose_correction_result =
      obstacle_detector_->pose_correction_result()
          ? *obstacle_detector_->pose_correction_result()
          : VehiclePose();
  auto retroreflectors = retroreflector_detector_->Detect(
      *range_images, local_imagery_, coordinate_converter_, pose,
      pose_correction_result);

  // Generate segmentation contexts.
  auto segmentation_context = std::make_unique<segmentation::Context>();
  segmentation_context->fiery_eye_net_result = std::move(fen_result);
  segmentation_context->ped_filtering_result = std::move(*ped_filtering_result);
  segmentation_context->ll_net_result =
      std::move(semantic_segmentation_results);
  segmentation_context->range_images = std::move(*range_images);
  segmentation_context->retroreflectors = std::move(retroreflectors);
  // Get front radar measurements for filter.
  auto radar_measurements = CollectRadarMeasurements(timestamp);
  if (radar_measurements.find(RadarId::RAD_FRONT) != radar_measurements.end()) {
    segmentation_context->front_radar_measurements =
        std::move(radar_measurements[RadarId::RAD_FRONT]);
  }

  SegmentedClusters segmented_clusters =
      segmenter_->Segment(pose, *obstacle_manager, semantic_map_manager_,
                          local_imagery_, coordinate_converter_, run_params_,
                          *segmentation_context, std::move(init_clusters));

  const auto noise_clusters = segmenter_->noise_clusters();
  obstacle_manager->BlacklistObstaclesWithinSegmentationNoiseClusters(
      noise_clusters);

  QLOG_IF_NOT_OK(WARNING, Publish(segmenter_->segmentation_objects_proto()));

  cluster_filter_->Filter(pose, latest_label_frame_.get(),
                          obstacle_manager.get(), &segmented_clusters);

  // Build and publish sensor fovs.
  sensor_fov::SensorFovRefs sensor_fovs;
  if (FLAGS_enable_sensor_fov) {
    sensor_fovs = sensor_fov_builder_->Compute(
        all_obstacle_ptrs, segmented_clusters, {timestamp, pose});
    QCHECK(!sensor_fovs.empty());
    auto sensor_fovs_proto = std::make_unique<SensorFovsProto>();
    auto& sensor_fov_protos = *(sensor_fovs_proto->mutable_sensor_fovs());
    sensor_fov_protos.Reserve(sensor_fovs.size());
    sensor_fov_protos.Add(sensor_fov::SensorFov::SensorFovToProto(
        *GetLidarViewSensorFov(sensor_fovs)));
    // Publish camera view sensor fovs in sim mode.
    if (!IsOnboardMode()) {
      const auto camera_view_sensor_fovs = GetCameraViewSensorFovs(sensor_fovs);
      for (const auto& [camera_id, sensor_fov] : camera_view_sensor_fovs) {
        sensor_fov_protos.Add(
            sensor_fov::SensorFov::SensorFovToProto(*sensor_fov));
      }
    }
    QLOG_IF_NOT_OK(WARNING, Publish(std::move(sensor_fovs_proto)));
  }

  cluster_observer_->Observe(segmentation_context->range_images,
                             &segmented_clusters);
  // Extract laser feature embeddings when segmented clusters fully processed
  // (segmented, filtered).
  auto laser_feature_embedding_future =
      ScheduleFuture(&nn_thread_pool_, [this, &segmented_clusters, &pose] {
        return std::make_shared<
            std::vector<tracker::LaserEmbeddingManager::LaserFeatureEmbedding>>(
            laser_embedding_manager_->ExtractLaserFeatureEmbeddings(
                segmented_clusters, camera_images_, pose,
                lidar_host_time_diff_));
      });

  const auto publish_obstacles_future =
      ScheduleFuture(&thread_pool_, [this, &all_obstacle_ptrs] {
        SCOPED_QTRACE_ARG1("PerceptionModule::PublishObstacles",
                           "num_obstacles", all_obstacle_ptrs.size());
        auto obstacles_proto = std::make_unique<ObstaclesProto>(
            Obstacle::ObstaclesToProto(all_obstacle_ptrs));
        QLOG_IF_NOT_OK(WARNING, Publish(std::move(obstacles_proto)));
      });

  const auto icp_results =
      icp_tracker_->FastTrack(pose, run_params, segmented_clusters);

  for (int i = 0; i < icp_results.size(); ++i) {
    const auto& icp_result = icp_results[i];
    if (!icp_result.success) continue;

    // Set cluster velocity from ICP.
    auto& cluster = segmented_clusters[i];
    cluster.set_icp_velocity_and_covariance(icp_result.velocity,
                                            icp_result.velocity_cov);
    cluster.set_icp_mse(icp_result.mse);
    cluster.set_matched_prev_cluster_id(icp_result.prev_cluster_id);
    cluster.set_matched_prev_cluster_timestamp(
        icp_result.prev_cluster_timestamp);
  }

  // Get association feature embeddings for each cluster
  WaitForFuture(laser_feature_embedding_future);
  std::vector<std::vector<float>> association_feature_embeddings;
  GetAssociationFeaturesFromLaser(&association_feature_embeddings);

  auto laser_measurements = std::make_unique<MeasurementsProto>();
  CreateMeasurementsFromLaser(
      pose, segmented_clusters, *laser_feature_embedding_future.Get(),
      association_feature_embeddings, laser_measurements.get());
  QLOG_IF_NOT_OK(WARNING,
                 Publish(std::move(laser_measurements), "laser_measurements"));

  WaitForFuture(publish_obstacles_future);

  // Destroy obstacle manager asynchronously.
  DestroyContainerAsync(std::move(obstacle_manager));
  // Destroy segmentation context asynchronously.
  DestroyContainerAsync(std::move(segmentation_context));
  // Destroy sensor fovs asynchronously.
  DestroyContainerAsync(std::move(sensor_fovs));

  if (FLAGS_render_label_frame_cvs && latest_label_frame_) {
    RenderLabelFrameCvs(*latest_label_frame_);
  }
}

void PerceptionModule::PublishCanvas() {
  SCOPED_QTRACE("PerceptionModule::PublishCanvas");
  // Publish canvas.
  auto canvas_buffers = vantage_client_man::CollectBuffersAndClear();
  if (canvas_buffers.buffers_size() > 0) {
    QLOG_IF_NOT_OK(WARNING, Publish(canvas_buffers));
  }
}

}  // namespace qcraft
