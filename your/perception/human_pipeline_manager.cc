#include "onboard/perception/human_pipeline_manager.h"

#include <cstdio>
#include <random>

#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/human_pipeline_tracker.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/utils/sensor_view_utils.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

DEFINE_bool(pcn_save_training_data, false,
            "Whether to save training data for pcn network");
DEFINE_string(
    pcn_save_patch_debug_image_dir,
    "/hosthome/car_data/patch_classification/debug_image",
    "The directory to save all cropped images for patch classification");
DEFINE_string(pcn_training_data_db_prefix,
              "/hosthome/car_data/patch_classification/pcn",
              "Path to the training data directory");
DEFINE_string(
    pcn_key_prefix, "",
    "Key prefix of the pcn training data db, usually run name would suffice");

namespace qcraft {

namespace {
// Skip if point is more than 2.0m above ground or under ground.
constexpr float kPointHeightForPcn = 2.0;  // m
constexpr int kMinPointsInBox = 10;
constexpr int kMinPixelNum = 48;
// Check if the camera/lidar is from the same frame or only one frame
// away, skip if not.
constexpr double kMaxTimeDiff = 0.155;  // s
// Need sufficiently complete point cloud for PCN
constexpr float kValidImagePosRatioThres = 0.4f;
constexpr int kWidthPadding = 16;
constexpr float kHeightPaddingRatio = 0.125f;
constexpr int kRandomSeed = 1234;
constexpr int kMaxBatchSize = 64;
constexpr float kPatchSwitchThres = 1.1f;
}  // namespace

HumanPipelineManager::HumanPipelineManager(LiteClientBase* lite_client,
                                           ThreadPool* thread_pool)
    : lite_client_(lite_client), thread_pool_(thread_pool) {
  human_tracker_ = std::make_unique<human_tracking::HumanTracker>();
}

void HumanPipelineManager::InitImagePatchClassifier(
    LiteClientBase* lite_client, const RunParamsProtoV2& run_params,
    const NetParam& image_patch_net_param) {
  image_patch_classifier_ = std::make_unique<ImagePatchClassifier>(
      lite_client, run_params, image_patch_net_param);
  lidar_params_.clear();
  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }
  if (ABSL_PREDICT_FALSE(FLAGS_pcn_save_training_data)) {
    leveldb::DB* dbptr;
    leveldb::Options options;
    options.create_if_missing = true;
    // options.error_if_exists = true;
    const auto& output_dir =
        boost::filesystem::path(FLAGS_pcn_training_data_db_prefix)
            .parent_path();
    if (!boost::filesystem::is_directory(output_dir)) {
      boost::filesystem::create_directories(output_dir);
    }
    const std::string training_data_db =
        FLAGS_pcn_training_data_db_prefix + "_pcn_db";
    leveldb::Status open_db_status =
        leveldb::DB::Open(options, training_data_db, &dbptr);
    if (!open_db_status.ok()) {
      QLOG(FATAL) << "Failed to open the leveldb at " << training_data_db
                  << ", err=" << open_db_status.ToString();
    }
    data_db_.reset(dbptr);
  }
}

void HumanPipelineManager::SampleBoxPoints(
    const Box2d& box, const ObstaclePtrs& obstacles, const int index,
    const std::unordered_map<LidarId, const LidarFrame*>& lidar_frame_map,
    float* real_ground_z, float* max_ped_distance,
    PcnPoints* sampled_points_in_box,
    std::vector<std::string>* pcn_skip_infos) const {
  std::vector<float> ground_z_vec;
  ground_z_vec.reserve(obstacles.size());
  std::vector<LaserPoint> points_in_box_all;
  for (const auto* obstacle : obstacles) {
    if (box.IsPointIn(obstacle->coord())) {
      ground_z_vec.push_back(obstacle->ground_z);
      if (obstacle->points.size() > 0) {
        points_in_box_all.insert(points_in_box_all.end(),
                                 obstacle->points.begin(),
                                 obstacle->points.end());
      }
    }
  }
  if (points_in_box_all.empty()) {
    (*pcn_skip_infos)[index] = "/nopoint";
    return;
  }

  std::sort(ground_z_vec.begin(), ground_z_vec.end());
  // Use 1/4th ground z to remove outlier.
  *real_ground_z = ground_z_vec[static_cast<int>(ground_z_vec.size() / 4)];

  std::vector<LaserPoint> points_in_box;
  if (points_in_box_all.size() <= PcnNet::kNumPoints) {
    points_in_box = std::move(points_in_box_all);
  } else {
    std::sample(points_in_box_all.begin(), points_in_box_all.end(),
                std::back_inserter(points_in_box), PcnNet::kNumPoints,
                std::mt19937(kRandomSeed));
  }
  for (const LaserPoint& point : points_in_box) {
    const float point_height = point.coord().z() - *real_ground_z;
    if (point_height < 0 || point_height > kPointHeightForPcn) {
      continue;
    }

    // Spin point_storage_coord is (scan_index, beam_index,
    // return_index) while PointCloud is (point_index, 0, 0).
    const auto pose =
        FindOrDie(lidar_frame_map, point.lidar_id)->ComputePoseAtPoint(point);

    sampled_points_in_box->emplace_back(point, pose, point_height);
    *max_ped_distance =
        std::max(*max_ped_distance, sampled_points_in_box->back().range_xy);
  }

  if (sampled_points_in_box->size() < kMinPointsInBox) {
    (*pcn_skip_infos)[index] =
        absl::StrFormat("too_few_pt_%d", sampled_points_in_box->size());
    return;
  }

  LaserPoint ground_point = points_in_box[0];
  ground_point.z = *real_ground_z;
  const auto pose = FindOrDie(lidar_frame_map, ground_point.lidar_id)
                        ->ComputePoseAtPoint(ground_point);
  if (sampled_points_in_box->size() == PcnNet::kNumPoints) {
    sampled_points_in_box->back() = PcnPoint(ground_point, pose, 0.f);
  } else {
    sampled_points_in_box->emplace_back(ground_point, pose, 0.f);
  }
}

void HumanPipelineManager::GetBestImagePatch(
    const std::vector<LidarFrame>& lidar_frames,
    const std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>&
        camera_images,
    const double lidar_host_time_diff, const Box2d& box, const int index,
    const PcnPoints& sampled_points_in_box, const float real_ground_z,
    const float max_ped_distance,
    std::tuple<CameraId, double, cv::Mat, cv::Rect>* expand_patch_for_labeling,
    std::vector<std::string>* pcn_skip_infos,
    std::vector<std::tuple<cv::Mat, PcnPoints, PcnRegionProto>>*
        ped_cyc_patches) const {
  const auto corners = box.GetCornersCounterClockwise();
  std::map<CameraId, std::string> camera_debug_infos;
  int camera_index = 0;
  float max_image_ratio = 0.f;
  float selected_image_area = 0.f;
  CameraId selected_id;
  double selected_timestamp;
  std::vector<CameraId> dim_cameras = GetDimCameraIds();
  for (const auto& [camera_id, images_with_transform] : camera_images) {
    if (images_with_transform.empty()) continue;
    if (std::find(dim_cameras.begin(), dim_cameras.end(), camera_id) !=
        dim_cameras.end()) {
      continue;
    }
    const auto& image_with_transform = images_with_transform.back();
    const auto& image = std::get<CameraImage>(image_with_transform);
    const double image_center_ts =
        image.center_timestamp() + lidar_host_time_diff;
    const auto& camera_param = image.params();

    const auto lidar_mid_timestamp =
        projection_util::FindRefLidarMidTimestamp(camera_param, lidar_frames);

    const double sensor_time_diff =
        std::abs(lidar_mid_timestamp - image_center_ts);
    if (sensor_time_diff > kMaxTimeDiff) {
      VLOG(2) << absl::StrFormat(
          "[HumanPipelineManager] Can't find corresponding camera image for "
          "lidar at mid timestamp: %.3f, The one found is with center "
          "timestamp lidar time: %.3f and trigger timestamp: %.3f ",
          lidar_mid_timestamp, image_center_ts, image.trigger_timestamp());
      camera_debug_infos[camera_id] +=
          absl::StrFormat("stale_%.3f", sensor_time_diff);
      continue;
    }
    const auto& smooth_to_camera_transform =
        std::get<AffineTransformation>(image_with_transform);

    std::vector<Vec2i> points_image_coord;
    points_image_coord.reserve(sampled_points_in_box.size() * sizeof(Vec2i));
    PcnPoints reserved_sampled_points_in_box;
    reserved_sampled_points_in_box.reserve(sampled_points_in_box.size() *
                                           sizeof(PcnPoint));
    // Ignore the points behind
    const double mid_z = real_ground_z + kPointHeightForPcn / 2.0;
    const auto transformed_pt = smooth_to_camera_transform.TransformPoint(
        Vec3d(box.center_x(), box.center_y(), mid_z));
    if (transformed_pt.x() < 0) {
      continue;
    }
    // If all points in corner are not in camera, skip it
    bool is_valid_camera = false;
    for (auto& point : corners) {
      const auto image_pos = projection_util::SmoothPointToImagePos(
          Vec3d(point, mid_z), smooth_to_camera_transform, camera_param);
      if (image_pos) {
        is_valid_camera = true;
        break;
      }
    }
    if (!is_valid_camera) {
      continue;
    }

    int start_x = image.width() * 2;
    int end_x = -image.width();
    int start_y = image.height() * 2;
    int end_y = -image.height();
    int valid_image_pos_number = 0;
    for (auto& point : sampled_points_in_box) {
      const auto image_pos = projection_util::SmoothPointToImagePos(
          point.pos_coord_smooth, smooth_to_camera_transform, camera_param,
          true);
      start_x = std::min(start_x, image_pos->x());
      start_y = std::min(start_y, image_pos->y());
      end_x = std::max(end_x, image_pos->x());
      end_y = std::max(end_y, image_pos->y());
      if (image_pos->x() >= 0 && image_pos->x() < image.width() &&
          image_pos->y() >= 0 && image_pos->y() < image.height()) {
        ++valid_image_pos_number;
        points_image_coord.push_back(*image_pos);
        reserved_sampled_points_in_box.push_back(point);
      }
    }

    // Min pixel num limit
    if ((end_x - start_x) < kMinPixelNum && (end_y - start_y) < kMinPixelNum) {
      camera_debug_infos[camera_id] +=
          absl::StrFormat("valid_pixel_num_in_x_%d_in_y_%d", (end_x - start_x),
                          (end_y - start_y));
      continue;
    }

    const float valid_image_pos_ratio =
        static_cast<float>(valid_image_pos_number) /
        static_cast<float>(sampled_points_in_box.size());

    if (valid_image_pos_ratio < kValidImagePosRatioThres) {
      camera_debug_infos[camera_id] +=
          absl::StrFormat("valid_pt_ratio_%.2f", valid_image_pos_ratio);
      continue;
    }

    // Ensure minimum width and height of cropped images
    // The width padding uses a fixed value to avoid projection deviation
    // caused by pedestrian movement;
    // The height padding uses a ratio to
    // ensure that the face is at a fixed height in the croped image
    start_x -= kWidthPadding;
    end_x += kWidthPadding;
    const int height = end_y - start_y;
    start_y -= static_cast<int>(height * kHeightPaddingRatio);
    end_y += static_cast<int>(height * kHeightPaddingRatio);

    // Need sufficiently valid area for PCN
    int start_x_on_image = std::max(0, start_x);
    int end_x_on_image = std::min(image.width() - 1, end_x);
    int start_y_on_image = std::max(0, start_y);
    int end_y_on_image = std::min(image.height() - 1, end_y);
    int area_on_image = (end_y_on_image - start_y_on_image) *
                        (end_x_on_image - start_x_on_image);
    int project_area = (end_y - start_y) * (end_x - start_x);
    const float valid_image_area_ratio =
        project_area > 0 ? static_cast<float>(area_on_image) /
                               static_cast<float>(project_area)
                         : 0.f;

    if (valid_image_area_ratio < kValidImagePosRatioThres) {
      camera_debug_infos[camera_id] +=
          absl::StrFormat("valid_area_%.2f", valid_image_area_ratio);
      continue;
    }

    const float image_area = (end_x - start_x) * (end_y - start_y);
    if (valid_image_pos_ratio < max_image_ratio ||
        (valid_image_pos_ratio == max_image_ratio &&
         image_area < selected_image_area * kPatchSwitchThres)) {
      camera_debug_infos[camera_id] +=
          absl::StrFormat("valid_pt_%.2f_not_max", valid_image_pos_ratio);
      continue;
    }

    max_image_ratio = valid_image_pos_ratio;
    selected_id = camera_id;
    selected_timestamp = lidar_mid_timestamp;
    selected_image_area = image_area;

    const cv::Rect rect_on_image(cv::Point(start_x_on_image, start_y_on_image),
                                 cv::Point(end_x_on_image, end_y_on_image));
    const cv::Mat camera_image = image.ToMat();
    cv::Mat padded_image_patch = camera_image(rect_on_image);
    if (area_on_image < project_area) {
      const int top = start_y_on_image - start_y;
      const int bottom = end_y - end_y_on_image;
      const int left = start_x_on_image - start_x;
      const int right = end_x - end_x_on_image;
      cv::copyMakeBorder(padded_image_patch, padded_image_patch, top, bottom,
                         left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }
    const cv::Rect rect(cv::Point(start_x, start_y), cv::Point(end_x, end_y));

    PcnRegionProto pcn_region_proto;
    pcn_region_proto.set_camera_id(camera_id);
    pcn_region_proto.set_x(rect.x);
    pcn_region_proto.set_y(rect.y);
    pcn_region_proto.set_width(rect.width);
    pcn_region_proto.set_height(rect.height);
    pcn_region_proto.set_camera_trigger_timestamp(image.trigger_timestamp());
    pcn_region_proto.set_max_ped_distance(max_ped_distance);
    for (const auto& point : points_image_coord) {
      auto* points_image_patch_coord =
          pcn_region_proto.add_points_image_patch_coord();
      points_image_patch_coord->set_x(point.x() - start_x);
      points_image_patch_coord->set_y(point.y() - start_y);
    }

    (*ped_cyc_patches)[index] = std::make_tuple(
        padded_image_patch, reserved_sampled_points_in_box, pcn_region_proto);

    if (FLAGS_pcn_save_training_data) {
      // save pcn training data for labeling
      // Padding 10%
      const int width_padding = (end_x - start_x) * 0.1;
      const int height_padding = (end_y - start_y) * 0.1;
      start_x = std::max(1, start_x - width_padding / 2);
      end_x = std::min(image.width() - 1, end_x + width_padding / 2);
      start_y = std::max(1, start_y - height_padding / 2);
      end_y = std::min(image.height() - 1, end_y + height_padding / 2);

      // Expand patch
      const int expand_size = std::max(end_x - start_x, end_y - start_y);
      const int expand_sx = std::max(0, start_x - expand_size);
      const int expand_ex = std::min(image.width() - 1, end_x + expand_size);
      const int expand_sy = std::max(0, start_y - expand_size);
      const int expand_ey = std::min(image.height() - 1, end_y + expand_size);

      const cv::Rect expand_rect(cv::Point(expand_sx, expand_sy),
                                 cv::Point(expand_ex, expand_ey));
      cv::Mat expand_image_patch = camera_image(expand_rect);

      cv::Rect roi;
      roi.x = start_x - expand_sx;
      roi.y = start_y - expand_sy;
      roi.width = end_x - start_x;
      roi.height = end_y - start_y;
      *expand_patch_for_labeling = std::make_tuple(
          selected_id, selected_timestamp, expand_image_patch, roi);
    }
  }
  // Only save to skip info if no patches added to ped_cyc_patches.
  if (camera_index == 0) {
    for (const auto& [cam_id, cam_info] : camera_debug_infos) {
      (*pcn_skip_infos)[index] += absl::StrFormat("/%d:%s", cam_id, cam_info);
    }
  }
}

void HumanPipelineManager::FilterOneBox(
    const ObstaclePtrs& obstacles, const std::vector<LidarFrame>& lidar_frames,
    const std::unordered_map<LidarId, const LidarFrame*>& lidar_frame_map,
    const std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>&
        camera_images,
    const double lidar_host_time_diff, const Box2d& box, const int index,
    std::vector<std::string>* pcn_skip_infos,
    std::vector<std::tuple<cv::Mat, PcnPoints, PcnRegionProto>>*
        ped_cyc_patches) const {
#ifndef Q_CPU_ONLY
  TimeCost filter_one_ped_box_time("filter_one_ped_box");
#endif

  float max_ped_distance = 0;
  float real_ground_z = 0;
  PcnPoints sampled_points_in_box;
  sampled_points_in_box.reserve(PcnNet::kNumPoints * sizeof(PcnPoint));
  std::tuple<CameraId, double, cv::Mat, cv::Rect> expand_patch_for_labeling;
  {
    SCOPED_QTRACE_ARG1("HumanPipelineManager::FilterOneBox-SampleBoxPoints",
                       "box_index", index);
    SampleBoxPoints(box, obstacles, index, lidar_frame_map, &real_ground_z,
                    &max_ped_distance, &sampled_points_in_box, pcn_skip_infos);
  }
  {
    SCOPED_QTRACE_ARG1("HumanPipelineManager::FilterOneBox-ProjectToImage",
                       "box_index", index);

    GetBestImagePatch(lidar_frames, camera_images, lidar_host_time_diff, box,
                      index, sampled_points_in_box, real_ground_z,
                      max_ped_distance, &expand_patch_for_labeling,
                      pcn_skip_infos, ped_cyc_patches);
  }

  if (FLAGS_pcn_save_training_data &&
      !std::get<cv::Mat>((*ped_cyc_patches)[index]).empty()) {
    const std::string image_name = absl::StrFormat(
        "%.3f_%s_%d", std::get<double>(expand_patch_for_labeling),
        CameraId_Name(std::get<CameraId>(expand_patch_for_labeling)), index);
    MaybeSaveDebugImage(image_name, (*ped_cyc_patches)[index],
                        expand_patch_for_labeling);
    MaybeSavePatchDataProto(index, (*ped_cyc_patches)[index],
                            expand_patch_for_labeling);
  }
}

void HumanPipelineManager::SetPatchResult(
    const std::vector<
        std::pair<ImagePatchClassifier::ClassificationResult, float>>& results,
    const int num_ped_boxes, const int num_boxes,
    const std::vector<int>& ped_cyc_patch_indices,
    std::vector<ImagePatchClassifier::ClassificationResult>* ped_cyc_box_types,
    std::vector<float>* ped_cyc_scores) const {
  for (int i = 0; i < num_ped_boxes; ++i) {
    (*ped_cyc_box_types)[i] = ImagePatchClassifier::kPedestrian;
  }
  for (int i = num_ped_boxes; i < num_boxes; ++i) {
    (*ped_cyc_box_types)[i] = ImagePatchClassifier::kCyclist;
  }

  for (int i = 0; i < results.size(); ++i) {
    const auto& cls_result =
        std::get<ImagePatchClassifier::ClassificationResult>(results[i]);
    const int ped_cyc_box_index = ped_cyc_patch_indices[i];
    (*ped_cyc_box_types)[ped_cyc_box_index] = cls_result;
    (*ped_cyc_scores)[ped_cyc_box_index] = std::get<float>(results[i]);
  }
}

HumanPipelineManager::PedFilteringResult HumanPipelineManager::FilterPedBoxes(
    const ObstaclePtrs& obstacles, const std::vector<LidarFrame>& lidar_frames,
    const std::map<CameraId, boost::circular_buffer<CameraImageWithTransform>>&
        camera_images,
    const double lidar_host_time_diff,
    std::vector<FieryEyeNetClassifier::DetectionBox>* ped_boxes,
    std::vector<FieryEyeNetClassifier::DetectionBox>* cyc_boxes) const {
#ifndef Q_CPU_ONLY
  TimeCost filter_ped_box_time("filter_ped_box");
#endif

  SCOPED_QTRACE_ARG2("HumanPipelineManager::FilterPedBoxes", "num_ped",
                     ped_boxes->size(), "num_cyc", cyc_boxes->size());

  const int num_ped_boxes = ped_boxes->size();
  const int num_cyc_boxes = cyc_boxes->size();
  const int num_boxes = num_ped_boxes + num_cyc_boxes;
  const int num_cameras = camera_images.size();

  QCHECK_GT(lidar_frames.size(), 0);
  QCHECK_GT(num_cameras, 0);

  std::unordered_map<LidarId, const LidarFrame*> lidar_frame_map;
  for (const auto& lidar_frame : lidar_frames) {
    lidar_frame_map[lidar_frame.lidar_id()] = &lidar_frame;
  }

  std::vector<std::tuple<cv::Mat, PcnPoints, PcnRegionProto>> ped_cyc_patches(
      num_boxes);
  std::vector<std::string> pcn_skip_infos(num_boxes);

  ParallelFor(0, num_boxes, thread_pool_, [&](int i) {
    const auto& box = (i < num_ped_boxes) ? (*ped_boxes)[i].box
                                          : (*cyc_boxes)[i - num_ped_boxes].box;
    FilterOneBox(obstacles, lidar_frames, lidar_frame_map, camera_images,
                 lidar_host_time_diff, box, i, &pcn_skip_infos,
                 &ped_cyc_patches);
  });

  std::vector<int> ped_cyc_patch_indices;
  for (int i = 0; i < ped_cyc_patches.size(); ++i) {
    if (!std::get<cv::Mat>(ped_cyc_patches[i]).empty()) {
      ped_cyc_patch_indices.push_back(i);
    }
  }
  ped_cyc_patches.erase(
      std::remove_if(
          ped_cyc_patches.begin(), ped_cyc_patches.end(),
          [](const auto& patch) { return std::get<cv::Mat>(patch).empty(); }),
      ped_cyc_patches.end());
  if (ped_cyc_patches.size() > kMaxBatchSize) {
    ped_cyc_patches.resize(kMaxBatchSize);
  }

  VLOG(2) << "Number of ped patches sent to PCN: " << ped_cyc_patches.size();
  const auto results =
      image_patch_classifier_->ClassifyImagePatches(ped_cyc_patches);

  // Aggregate the results in case there're more than one patches for one
  // box. NOTE(wanzeng): Initilize the type vector with type ped/cyc for
  // those not classified by pcn (e.g. out of range, too small) and unknown
  // by default for those that passed through pcn.
  std::vector<ImagePatchClassifier::ClassificationResult> ped_cyc_box_types(
      num_boxes);
  std::vector<float> ped_cyc_scores(num_boxes, -1.);

  SetPatchResult(results, num_ped_boxes, num_boxes, ped_cyc_patch_indices,
                 &ped_cyc_box_types, &ped_cyc_scores);

  std::vector<std::string> pcn_track_infos(num_boxes);
  std::vector<FieryEyeNetClassifier::DetectionBox> ped_cyc_boxes;
  ped_cyc_boxes.reserve(num_boxes);
  ped_cyc_boxes.insert(ped_cyc_boxes.end(), ped_boxes->begin(),
                       ped_boxes->end());
  ped_cyc_boxes.insert(ped_cyc_boxes.end(), cyc_boxes->begin(),
                       cyc_boxes->end());

  const double detection_timestamp = lidar_frames[0].MidTimestamp();

  const auto ped_cyc_box_types_tracked = human_tracker_->GetHumanTrackingResult(
      ped_cyc_boxes, detection_timestamp, ped_cyc_box_types, &pcn_track_infos);

  PedFilteringResult ped_filtering_result;
  std::vector<FieryEyeNetClassifier::DetectionBox> valid_ped_boxes;
  std::vector<FieryEyeNetClassifier::DetectionBox> valid_cyc_boxes;

  // Consider the detected result as valid as long as it's not unknown (cyc as
  // ped or vice versa is okay) and do not change box type for now.
  for (int i = 0; i < num_boxes; ++i) {
    const auto& box =
        (i < num_ped_boxes) ? (*ped_boxes)[i] : (*cyc_boxes)[i - num_ped_boxes];
    if (ped_cyc_box_types_tracked[i] != ImagePatchClassifier::kUnknown) {
      if (i < num_ped_boxes) {
        valid_ped_boxes.push_back(box);
        ped_filtering_result.ped_skip_infos.push_back(pcn_skip_infos[i]);
        ped_filtering_result.ped_track_infos.push_back(pcn_track_infos[i]);
      } else {
        valid_cyc_boxes.push_back(box);
        ped_filtering_result.cyc_skip_infos.push_back(pcn_skip_infos[i]);
        ped_filtering_result.cyc_track_infos.push_back(pcn_track_infos[i]);
      }
    } else {
      if (i < num_ped_boxes) {
        ped_filtering_result.deleted_ped_boxes.emplace_back(box,
                                                            ped_cyc_scores[i]);
      } else {
        ped_filtering_result.deleted_cyc_boxes.emplace_back(box,
                                                            ped_cyc_scores[i]);
      }
    }
  }

  *ped_boxes = std::move(valid_ped_boxes);
  *cyc_boxes = std::move(valid_cyc_boxes);
  return ped_filtering_result;
}

void HumanPipelineManager::MaybeSaveDebugImage(
    const std::string& image_name,
    const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
    const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
        expand_patch_for_labeling) const {
  QCHECK_NE(FLAGS_pcn_save_patch_debug_image_dir, "");
  if (!boost::filesystem::is_directory(FLAGS_pcn_save_patch_debug_image_dir)) {
    boost::filesystem::create_directories(FLAGS_pcn_save_patch_debug_image_dir);
  }
  const auto& image_patch = std::get<cv::Mat>(expand_patch_for_labeling);
  const auto& roi = std::get<cv::Rect>(expand_patch_for_labeling);
  cv::Mat image_to_save = image_patch.clone();
  const auto& points_in_box = std::get<PcnPoints>(patch);
  const auto& proj_pos =
      std::get<PcnRegionProto>(patch).points_image_patch_coord();
  const int num_points = points_in_box.size();
  for (int i = 0; i < num_points; i++) {
    const auto& point_image_patch_coord = proj_pos[i];
    const int x = point_image_patch_coord.x() + roi.x;
    const int y = point_image_patch_coord.y() + roi.y;
    if (x < 0 || y < 0 || x >= image_patch.cols || y >= image_patch.rows) {
      continue;
    }
    cv::circle(image_to_save, cv::Point(x, y), 1, cv::Scalar(0, 255, 255));
  }
  const std::string image_name_pt = absl::StrFormat(
      "%s/%s.jpg", FLAGS_pcn_save_patch_debug_image_dir, image_name);
  cv::imwrite(image_name_pt, image_to_save);
}

void HumanPipelineManager::MaybeSavePatchDataProto(
    const int index,
    const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
    const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
        expand_patch_for_labeling) const {
  if (ABSL_PREDICT_FALSE(FLAGS_pcn_save_training_data)) {
    const auto lidar_mid_timestamp =
        std::get<double>(expand_patch_for_labeling);
    const std::string db_key = absl::StrFormat(
        "%s/%.2f/%d", FLAGS_pcn_key_prefix, lidar_mid_timestamp, index);

    patch_classifier::PcnLabelingData frame_data =
        AddPatchData(db_key, index, patch, expand_patch_for_labeling);

    const auto satus = data_db_->Put(leveldb::WriteOptions(), db_key,
                                     frame_data.SerializeAsString());
    if (!satus.ok()) {
      QLOG(FATAL) << "HumanPipelineManager::MaybeSavePatchDataProto "
                  << satus.ToString();
    }
  }
}

patch_classifier::PcnLabelingData HumanPipelineManager::AddPatchData(
    const std::string& db_key, const int index,
    const std::tuple<cv::Mat, PcnPoints, PcnRegionProto>& patch,
    const std::tuple<CameraId, double, cv::Mat, cv::Rect>&
        expand_patch_for_labeling) const {
  const auto lidar_mid_timestamp = std::get<double>(expand_patch_for_labeling);
  const auto& expand_image = std::get<cv::Mat>(expand_patch_for_labeling);
  const auto& roi = std::get<cv::Rect>(expand_patch_for_labeling);
  const auto& camera_id = std::get<CameraId>(expand_patch_for_labeling);
  std::string frame_data_str;
  const auto key_state =
      data_db_->Get(leveldb::ReadOptions(), db_key, &frame_data_str);
  patch_classifier::PcnLabelingData frame_data;
  patch_classifier::PcnData pcn_data;
  auto* pcn_image_patch = pcn_data.mutable_image_patch();
  patch_classifier::PcnRoi* roi_proto = pcn_image_patch->mutable_roi();
  roi_proto->set_x(roi.x);
  roi_proto->set_y(roi.y);
  roi_proto->set_width(roi.width);
  roi_proto->set_height(roi.height);
  cv::Mat image_to_save = expand_image.clone();
  pcn_image_patch->mutable_pcn_region()->CopyFrom(
      std::get<PcnRegionProto>(patch));
  std::vector<uchar> encoded_image;
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  // Compression level: 0-9, default is 1
  // A higher value means a smaller size and longer compression time
  compression_params.push_back(5);
  cv::imencode(".png", image_to_save, encoded_image, compression_params);

  pcn_image_patch->set_roi_patch(
      std::string(encoded_image.begin(), encoded_image.end()));

  const auto& points_in_box = std::get<PcnPoints>(patch);
  const auto& proj_pos =
      std::get<PcnRegionProto>(patch).points_image_patch_coord();
  for (int i = 0; i < points_in_box.size(); ++i) {
    const auto& point = points_in_box[i];
    auto* pcn_laser_point = pcn_data.add_points();
    pcn_laser_point->set_x(point.pos_coord_vehicle.x());
    pcn_laser_point->set_y(point.pos_coord_vehicle.y());
    pcn_laser_point->set_z(point.pos_coord_vehicle.z());
    pcn_laser_point->set_range(point.range_xy);
    pcn_laser_point->set_intensity(point.intensity);
    pcn_laser_point->set_return_index(point.return_index);
    pcn_laser_point->set_has_returns_behind(point.has_return_behind);
    pcn_laser_point->set_proj_x(proj_pos[i].x());
    pcn_laser_point->set_proj_y(proj_pos[i].y());
    pcn_laser_point->set_lidar_id(point.lidar_id);
    pcn_laser_point->set_lidar_type(static_cast<LidarType>(point.lidar_type));
  }
  pcn_data.set_timestamp(lidar_mid_timestamp);
  pcn_data.set_obj_id(index);
  pcn_data.set_camera_id(camera_id);
  if (key_state.ok()) {
    frame_data.ParseFromString(frame_data_str);
  }
  *frame_data.add_pcn_data() = std::move(pcn_data);
  return frame_data;
}
}  // namespace qcraft
