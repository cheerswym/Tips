#include "onboard/perception/tracker_module.h"

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <unordered_map>
#include <utility>

#include "onboard/camera/utils/camera_util.h"
#include "onboard/lidar/vehicle_pose_util.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/type_util.h"

DEFINE_bool(enable_radar_measurement, true,
            "Whether to enable radar measurement.");
DEFINE_bool(enable_camera_measurement, false,
            "Whether to enable camera measurement in DBQ mode, camera "
            "measurement will always be enabled in PBQ mode.");

namespace qcraft {

constexpr double kTimeIntervalForUpdateNonLidarMeasurements = 0.066;  // s
constexpr int kTrackerThreadPoolSize = 0;
constexpr int kImageBufferSize = 3;

void TrackerModule::OnSubscribeChannels() {
  if (IsOnboardMode()) {
    Subscribe(&TrackerModule::UpdatePose, this, "pose_proto");
  } else {
    Subscribe(&TrackerModule::UpdatePose, this, "sensor_pose");
  }
  Subscribe(&TrackerModule::UpdateLabels, this);

  Subscribe(&TrackerModule::UpdateLocalizationTransform, this);

  SubscribeDecodedImage([this](const CameraImage& camera_image) {
    TrackerModule::UpdateImage(camera_image);
  });

  Subscribe(&TrackerModule::UpdateSensorFov, this, "sensor_fovs_proto");
  Subscribe(&TrackerModule::UpdateMeasurementsProto, this,
            "laser_measurements");
  Subscribe(&TrackerModule::UpdateMeasurementsProto, this,
            "radar_measurements");
  Subscribe(&TrackerModule::UpdateMeasurementsProto, this,
            "camera_measurements");
  // For backward comptible, only used for offline simulation.
  Subscribe(&TrackerModule::UpdateMeasurementsProto, this,
            "measurements_proto");
}

TrackerModule::TrackerModule(LiteClientBase* lite_client)
    : LiteModule(lite_client),
      tracker_thread_pool_(
          kTrackerThreadPoolSize > 0
              ? std::make_unique<ThreadPool>(kTrackerThreadPoolSize)
              : nullptr) {
  // Using LoadWholeMap() is strongly discouraged, use UpdateSmoothPos()
  // instead. The later only loads portion of the map as needed
  // semantic_map_manager_.LoadWholeMap().Build();
  tracker_ = std::make_unique<tracker::Tracker>(this, &semantic_map_manager_,
                                                tracker_thread_pool_.get(),
                                                param_manager());
  // Initialize camera image buffers.
  for (int i = CameraId_MIN; i <= CameraId_MAX; ++i) {
    const CameraId cam_id = static_cast<CameraId>(i);
    if (!IsFunctionCamera(cam_id) || cam_id == CAM_UNKNOWN) {
      continue;
    }
    camera_images_.emplace(std::piecewise_construct, std::make_tuple(cam_id),
                           std::make_tuple(kImageBufferSize));
  }
}

std::map<CameraId, CameraImageWithTransform>
TrackerModule::GetBestMatchImageForCluster(
    std::shared_ptr<const MeasurementsProto> measurements_proto) {
  std::map<CameraId, CameraImageWithTransform> results;
  // Get middle timespame for measurements
  const double mid_timestamp = (measurements_proto->min_timestamp() +
                                measurements_proto->max_timestamp()) /
                               2.0;
  for (const auto& [camera_id, images] : camera_images_) {
    double min_diff = std::numeric_limits<double>::max();
    auto matched = images.rbegin();
    for (auto it = images.rbegin(); it != images.rend(); ++it) {
      const auto& image = std::get<CameraImage>(*it);
      const double diff = std::abs(image.center_timestamp() - mid_timestamp);
      if (diff < min_diff) {
        matched = it;
        min_diff = diff;
      }
      if (image.center_timestamp() - mid_timestamp < 0) {
        break;
      }
    }
    if (min_diff < std::numeric_limits<double>::max()) {
      results[camera_id] = *matched;
    }
  }
  return results;
}

void TrackerModule::UpdateMeasurementsProto(
    std::shared_ptr<const MeasurementsProto> raw_measurements_proto) {
  // NOTE(yu): Update measurements_proto's type to convert old type to new type
  // by copying and reassign. This is only for backward compatibility, so
  // shouldn't affect onboard performance.
  std::shared_ptr<const MeasurementsProto> measurements_proto =
      raw_measurements_proto;
  if (!IsOnboardMode() && measurements_proto->measurements_size() > 0 &&
      !measurements_proto->measurements(0).has_type()) {
    auto measurements = *raw_measurements_proto;
    for (auto& m : *(measurements.mutable_measurements())) {
      // TODO(zheng, yu): Set MT_UNKNOWN as MT_UNKNOWN, set MT_UNKNOWN as
      // MT_STATIC_OBJECT for now as a NFC change since we were set MT_UNKNOWN
      // as MT_STATIC_OBJECT.
      const auto measurement_type =
          type_util::ToMeasurementType(m.type_deprecated());
      m.set_type((measurement_type == MT_UNKNOWN) ? MT_STATIC_OBJECT
                                                  : measurement_type);
    }
    measurements_proto =
        std::make_shared<const MeasurementsProto>(std::move(measurements));
  }

  // TODO(zheng): Accept camera measurements util we have a adequate
  // dev test.
  if (pose_history_.empty() || !coordinate_converter_.is_valid() ||
      (IsDBQConext() && !IsDBQv4() && !FLAGS_enable_camera_measurement &&
       measurements_proto->group_type() == MeasurementsProto::CAMERA) ||
      (measurements_proto->group_type() == MeasurementsProto::RADAR &&
       !FLAGS_enable_radar_measurement)) {
    return;
  }
  // Add some trace info for debugging.
  const double header_timestamp =
      1e-6 * measurements_proto->header().timestamp();
  const std::string m_type =
      MeasurementsProto::GroupType_Name(measurements_proto->group_type());

  const int measurement_num = measurements_proto->measurements_size();

  // Add some info for trace debugging, the max allowed string
  // length in trace is 65, so we should save the debug info in
  // two values in case of it beyond the max length.
  const std::string m_info =
      absl::StrFormat("header_time: %.3f,  type: %s, num: %d", header_timestamp,
                      m_type, measurement_num);
  std::string min_max_timestamp = absl::StrFormat(
      "min: %.3f, max: %.3f", measurements_proto->min_timestamp(),
      measurements_proto->max_timestamp());
  // NOTE(zheng): If the measurements is empty, the min max timestamp is
  // -DBL_MAX and DBL_MAX, his string length is beyond the max allowed length
  // in trace, so we should give it a default value.
  if (measurement_num == 0) {
    min_max_timestamp =
        absl::StrFormat("m_min_time: %.3f, m_max_time: %.3f", -1.0, -1.0);
  }

  const double time_now = ToUnixDoubleSeconds(Clock::Now());
  const double process_delay = time_now - header_timestamp;
  constexpr double kMaxAllowedProcessDelay = 0.3;  // 300 ms
  if (process_delay > kMaxAllowedProcessDelay) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_WARNING, QIssueType::QIT_PERFORMANCE,
        QIssueSubType::QIST_PERCEPTION_TRACKER_PROCESS_SLOW,
        "Check tracker process speed: SLOW(too many messages)",
        absl::StrFormat("Measurement is delayed for more than 300ms, %s "
                        "current timestamp: %.2f %s",
                        m_info, time_now, min_max_timestamp));
    return;
  }

  SCOPED_QTRACE_ARG2("TrackerModule::UpdateMeasurementsProto", "m_info", m_info,
                     "min_max_time", min_max_timestamp);
  const double min_measurement_timestamp = measurements_proto->min_timestamp();
  const double max_measurement_timestamp = measurements_proto->max_timestamp();
  // Put valid tracker input in a deque.
  const auto pose = pose_history_.back().second;
  // If no sensor_fovs_proto is provided, use an empty one.
  std::shared_ptr<const SensorFovsProto> sensor_fovs_proto =
      sensor_fovs_proto_history_.empty() ? nullptr
                                         : sensor_fovs_proto_history_.back();

  // TODO(zheng): Use std::numeric_limits to replace DBL macro in perception
  // and tracker module.
  TrackerInputVariables tracker_input = {
      .pose = pose,
      .coordinate_converter = coordinate_converter_,
      .measurements = measurements_proto,
      .label_frame = latest_label_frame_,
      .min_timestamp = min_measurement_timestamp,
      .max_timestamp = max_measurement_timestamp,
      .sensor_fovs_proto = sensor_fovs_proto,
      .camera_images = GetBestMatchImageForCluster(measurements_proto)};
  measurement_group_queue_.emplace_back(tracker_input);

  // NOTE(zheng): the lidar measurement timestamp may behind radar measurement
  // timestamp 0.1s because of processing time, so we should use header time
  // here. When lidar measurements group comes, or it has waiting 66ms, we
  // trigger tracker pipeline.
  // The max frequency is 25 hz when we add 66ms trigger mode.
  const double waiting_time = header_timestamp - latest_trigger_time_;
  if (measurements_proto->group_type() == MeasurementsProto::LIDAR ||
      waiting_time > kTimeIntervalForUpdateNonLidarMeasurements) {
    latest_trigger_time_ = header_timestamp;
    std::deque<TrackerInputVariables> tracker_input_queue;
    tracker_input_queue.swap(measurement_group_queue_);
    ProcessAllMeasurementGroupsInQueue(std::move(tracker_input_queue),
                                       latest_trigger_time_, m_type);
  }
}

void TrackerModule::UpdateSensorFov(
    std::shared_ptr<const SensorFovsProto> sensor_fovs_proto) {
  SCOPED_QTRACE("TrackerModule::UpdateSensorFov");
  sensor_fovs_proto_history_.push_back(sensor_fovs_proto);
}

void TrackerModule::UpdatePose(std::shared_ptr<const PoseProto> pose) {
  SCOPED_QTRACE("TrackerModule::UpdatePose");
  semantic_map_manager_.UpdateSmoothPos(
      Vec2d{pose->pos_smooth().x(), pose->pos_smooth().y()});
  semantic_map_manager_.ApplyUpdate();
  pose_history_.push_back(
      std::make_pair(pose->timestamp(), VehiclePose(*pose)));
}

void TrackerModule::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  SCOPED_QTRACE("TrackerModule::UpdateLocalizationTransform");
  coordinate_converter_.UpdateLocalizationTransform(
      *localization_transform_proto);
  semantic_map_manager_.UpdateLocalizationTransform(
      *localization_transform_proto, tracker_thread_pool_.get());
}

void TrackerModule::UpdateLabels(
    std::shared_ptr<const labeling::LabelFrameProto> label_frame) {
  SCOPED_QTRACE("TrackerModule::UpdateLabels");
  if (pose_history_.empty()) {
    return;
  }
  std::optional<VehiclePose> lidar_frame_pose;
  for (const auto& [timestamp, pose] : pose_history_) {
    const double time_diff = timestamp - label_frame->timestamp();
    constexpr double kLabelLidarMaxTimeDiff = 0.005;  // s
    if (time_diff > 0 || std::abs(time_diff) < kLabelLidarMaxTimeDiff) {
      lidar_frame_pose = pose;
      break;
    }
  }
  if (!lidar_frame_pose) {
    QLOG_EVERY_N_SEC(WARNING, 1.0)
        << "Can't find latest pose for current label.";
    return;
  }
  latest_label_frame_ = std::make_shared<const labeling::LabelFrameProto>(
      labeling::GenerateLabelsAndZonesFromCurrentPose(*label_frame,
                                                      *lidar_frame_pose));
}

void TrackerModule::UpdateImage(const CameraImage& camera_image) {
  UpdateCameraLidarParams();

  const auto camera_id = camera_image.camera_id();
  // Skip the camera if it's from auxiliary camera.
  if (!IsFunctionCamera(camera_id) || camera_id == CAM_UNKNOWN) {
    return;
  }

  SCOPED_QTRACE("TrackerModule::UpdateImage");
  const double image_center_timestamp_lidar_time =
      camera_image.center_timestamp();

  const auto image_pose = ComputeEstimatedVehiclePose(
      pose_history_, image_center_timestamp_lidar_time);
  if (!image_pose.has_value()) {
    QLOG(ERROR) << "Cannot compute the pose of the image from "
                << CameraId_Name(camera_id);
    return;
  }

  const auto& camera_param = camera_image.params();
  LidarParametersProto lidar_param;
  if (camera_param.has_ref_lidar() &&
      ContainsKey(lidar_params_, camera_param.ref_lidar())) {
    lidar_param = FindOrDie(lidar_params_, camera_param.ref_lidar());
  } else if (const auto* param = FindOrNull(lidar_params_, LDR_CENTER)) {
    lidar_param = *param;
  } else if (const auto* param = FindOrNull(lidar_params_, LDR_FRONT)) {
    lidar_param = *param;
  } else {
    lidar_param = lidar_params_.begin()->second;
  }
  const auto smooth_to_camera_transform =
      projection_util::GetSmoothToCameraCoordTransform(
          camera_param, lidar_param, *image_pose);
  auto camera_image_with_pose = camera_image;
  camera_image_with_pose.set_pose(*image_pose);
  camera_images_.at(camera_id).push_back(std::make_pair(
      std::move(camera_image_with_pose), smooth_to_camera_transform));
}

void TrackerModule::UpdateCameraLidarParams() {
  const RunParamsProtoV2 run_params = GetRunParams();
  const auto lidar_params = run_params.vehicle_params().lidar_params();
  for (const auto& lidar_param : lidar_params) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }
}

void TrackerModule::ProcessAllMeasurementGroupsInQueue(
    std::deque<TrackerInputVariables> tracker_input_queue, double trigger_time,
    std::string trigger_type) {
  SCOPED_QTRACE_ARG2("TrackerModule::ProcessAllMeasurementGroupsInQueue",
                     "trigger_time", std::to_string(trigger_time),
                     "trigger_type", trigger_type);
  // The measurements timestamp isn't inorder, we sort them to reduce
  // track roll back frequency.
  std::sort(tracker_input_queue.begin(), tracker_input_queue.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::make_pair(lhs.min_timestamp, lhs.max_timestamp) <
                     std::make_pair(rhs.min_timestamp, rhs.max_timestamp);
            });

  // Check if there're too many measurement groups in the queue.
  constexpr int kMaxAllowedMeasurementSize = 10;
  while (tracker_input_queue.size() > kMaxAllowedMeasurementSize) {
    const auto args_message = absl::StrFormat(
        "Tracker measurement size exceeds the maximum deque size (%d). "
        "Old measurement will be dropped. All queues size: %d",
        kMaxAllowedMeasurementSize, tracker_input_queue.size());
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_WARNING, QIssueType::QIT_PERFORMANCE,
                      QIssueSubType::QIST_PERCEPTION_TRACKER_LITE_MSG_BACKLOG,
                      "Check tracker process speed: SLOW(too many messages)",
                      args_message);
    tracker_input_queue.pop_front();
  }
  tracker_->ResetDebugProto();
  while (!tracker_input_queue.empty()) {
    auto& item = tracker_input_queue.front();
    tracker_->TrackObjects(item.pose, item.coordinate_converter,
                           item.camera_images, item.max_timestamp,
                           std::move(*(item.measurements)), item.label_frame,
                           item.sensor_fovs_proto);
    tracker_input_queue.pop_front();
  }
  // Publish Obejcts.
  tracker_->PublishObjects();
}

}  // namespace qcraft
