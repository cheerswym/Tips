#include "onboard/perception/vision_module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "absl/container/flat_hash_set.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/vis/ark/ark_server/ark_client_man.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/camera/utils/image_util.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/vehicle_pose_util.h"
#include "onboard/lite/logging.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/nets/net_flags.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/tracker/track_classifier/visual_feature_extractor.h"
#include "onboard/proto/image_overlay.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/colormap.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "snappy/snappy.h"

DEFINE_bool(enable_tl_net, true, "Enable K net to classifier traffic lights");
DEFINE_bool(enable_ll_net, true, "Enable llnet for panoptic segmentation.");
DEFINE_bool(enable_mono3d_net, false,
            "Enable mono3d detection net in DBQ mode. Mono3d will always be "
            "enabled in PBQ mode.");
DEFINE_bool(enable_depth_net, true, "Enable depthnet for depth estimation.");
DEFINE_bool(enable_lane_net, false, "Enable lane detect net.");
DEFINE_bool(vision_module_dump_trace, false,
            "Log trace periodically for offboard performance testing");

namespace qcraft {
namespace {
constexpr int kPanoNetMaxInputImageSize = 2048 * 4096;
constexpr int kDepthNetMaxInputImageSize = 2048 * 4096;

// Use the tenths in camera center timestamp (x in 000.x00) as frame descriptor.
int FetchImageFrameDescriptor(const CameraImage& image) {
  const double center_timestamp = image.center_timestamp();
  const double ts_decimal = center_timestamp - FloorToInt(center_timestamp);
  return FloorToInt(10 * ts_decimal) % 10;
}
}  // namespace

VisionModule::VisionModule(LiteClientBase* lite_client)
    : LiteModule(lite_client),
      segmentation_loop_executor_thread_pool_(1),
      traffic_light_loop_executor_thread_pool_(1),
      depth_estimation_loop_executor_thread_pool_(1),
      mono3d_thread_pool_(2),
      lane_detector_loop_executor_thread_pool_(1),
      semantic_map_update_thread_pool_(1),
      multi_camera_fusion_thread_pool_(1),
      imagery_preloading_thread_pool_(1) {
#ifndef Q_CPU_ONLY
  cudaMallocHost(&pano_net_input_image_data_pinned_,
                 kPanoNetMaxInputImageSize * 3);
  cudaMallocHost(&depth_net_input_image_data_pinned_,
                 kDepthNetMaxInputImageSize * 3);
  cudaMallocHost(&depth_net_output_image_data_pinned_,
                 kDepthNetMaxInputImageSize * 3);
#else
  pano_net_input_image_data_pinned_ =
      static_cast<uint8_t*>(malloc(kPanoNetMaxInputImageSize * 3));
  depth_net_input_image_data_pinned_ =
      static_cast<uint8_t*>(malloc(kDepthNetMaxInputImageSize * 3));
  depth_net_output_image_data_pinned_ =
      static_cast<uint8_t*>(malloc(kDepthNetMaxInputImageSize * sizeof(float)));
#endif
}

VisionModule::~VisionModule() {
  for (auto& [camera_id, future] : future_pool_segmentation_) {
    future.Wait();
  }
  for (auto& [camera_id, future] : future_pool_mono3d_) {
    future.Wait();
  }
  for (auto& [camera_id, future] : future_pool_lane_) {
    future.Wait();
  }
  future_tl_.Wait();
#ifndef Q_CPU_ONLY
  cudaFreeHost(pano_net_input_image_data_pinned_);
  cudaFreeHost(depth_net_input_image_data_pinned_);
  cudaFreeHost(depth_net_output_image_data_pinned_);
#else
  free(pano_net_input_image_data_pinned_);
  free(depth_net_input_image_data_pinned_);
  free(depth_net_output_image_data_pinned_);
#endif
}

void VisionModule::CollectCamerasForMultiCameraPipelines() {
  cameras_for_mf_in_yaw_order_.clear();
  // Note(zheng): The key is camera yaw angle, we convert the yaw to degree
  // and cast it to int type, so we can get the sorted camera id in yaw
  // ascend order.
  std::map<int, std::vector<CameraId>> yaw_id_map;
  for (const auto& [id, cam_param] : camera_params_) {
    // Only use cameras in cameras for mono3d.
    if (!ContainsKey(cameras_for_mono3d_, id)) {
      continue;
    }
    const double cam_yaw_in_vehicle =
        NormalizeAngle(cam_param.camera_to_vehicle_extrinsics().yaw);
    const int key = static_cast<int>(r2d(cam_yaw_in_vehicle));
    yaw_id_map[key].push_back(id);
  }

  for (const auto& [yaw, id_list] : yaw_id_map) {
    cameras_for_mf_in_yaw_order_.insert(cameras_for_mf_in_yaw_order_.end(),
                                        id_list.begin(), id_list.end());
  }
  if (cameras_for_mf_in_yaw_order_.empty()) {
    QLOG(ERROR) << "No cameras for measurements fusion.";
  }

  cameras_for_tl_.clear();
  // Collect all camera ids facing front for traffic light pipeline.
  for (const auto& [id, cam_param] : camera_params_) {
    const double cam_yaw_in_vehicle =
        NormalizeAngle(cam_param.camera_to_vehicle_extrinsics().yaw);

    // Skip cameras that not face front.
    if (std::fabs(cam_yaw_in_vehicle) > M_PI_2) {
      continue;
    }
    cameras_for_tl_.insert(id);
  }
  if (cameras_for_tl_.empty()) {
    QLOG(ERROR) << "No cameras for traffic light classifier.";
  }
}

void VisionModule::DetectAndClassifyTrafficLights(
    const std::map<CameraId, CameraImage>& camera_images_for_tl,
    const std::map<CameraId, SemanticSegmentationResultProto>&
        semantic_segmentation_results_map) {
  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  // Get the latest trigger image as ref image.
  auto ref_camera_image = camera_images_for_tl.begin()->second;
  for (auto& [id, image] : camera_images_for_tl) {
    if (image.center_timestamp() > ref_camera_image.center_timestamp()) {
      ref_camera_image = image;
    }
  }
  // Skip the current frame if the ref camera published earlier than a
  // threshold from now. 200ms given 10Hz means there might probably be
  // a newer frame coming in the queue, so skiping the current frame to
  // be able to process result newer frame on time.
  constexpr double kTrafficLightFrameMaxDelay = 0.2;  // s
  if (std::abs(timestamp_now - ref_camera_image.center_timestamp()) >
      kTrafficLightFrameMaxDelay) {
    QLOG(ERROR) << absl::StrFormat(
        "[TL] Ref camera about to process is published "
        "more than 200ms "
        "earlier (published %.3f, now %.3f), skipping "
        "since it's stale and probably a new frame "
        "is in queue ready to be processed.",
        ref_camera_image.decoded_image_publish_timestamp(), timestamp_now);
    return;
  }
  const double ref_camera_center_timestamp =
      ref_camera_image.center_timestamp();

  // Image collection.
  std::map<CameraId, CameraImage> tl_inputs;
  double latest_center_timestamp = 0.;
  constexpr double kTrafficLightPanoResultCheckingMaxDelay = 0.15;  // s
  std::map<CameraId, const SemanticSegmentationResultProto*>
      semantic_segmentation_result_ptr_map;
  std::vector<std::string> image_obsolete_issue_msgs;
  for (const auto& tl_camera_id : cameras_for_tl_) {
    if (!ContainsKey(camera_images_for_tl, tl_camera_id)) {
      continue;
    }
    const auto& image = camera_images_for_tl.at(tl_camera_id);
    const double center_timestamp = image.center_timestamp();

    constexpr double kMaxTimeDiffForOneFrame = 0.15;  // s
    if (std::abs(center_timestamp - ref_camera_center_timestamp) >
        kMaxTimeDiffForOneFrame) {
      image_obsolete_issue_msgs.push_back(
          absl::StrFormat("  %s captured at %.3f is obsolete (0.15s) given ref "
                          "camera image %s captured at %.3f",
                          CameraId_Name(tl_camera_id), image.center_timestamp(),
                          CameraId_Name(ref_camera_image.camera_id()),
                          ref_camera_image.center_timestamp()));
      continue;
    }

    tl_inputs.emplace(tl_camera_id, image);
    latest_center_timestamp =
        std::max(latest_center_timestamp, image.center_timestamp());
    if (ContainsKey(semantic_segmentation_results_map, tl_camera_id) &&
        std::abs(image.center_timestamp() -
                 semantic_segmentation_results_map.at(tl_camera_id)
                     .image_center_timestamp()) <
            kTrafficLightPanoResultCheckingMaxDelay) {
      semantic_segmentation_result_ptr_map[tl_camera_id] =
          &semantic_segmentation_results_map.at(tl_camera_id);
    }
  }

  // Throw away the current image frame if any of the images is too
  // old, with a warning only auto-issue.
  if (!image_obsolete_issue_msgs.empty()) {
    image_obsolete_issue_msgs.push_back("with synced cameras");
    for (const auto& [camera_id, _] : tl_inputs) {
      image_obsolete_issue_msgs.push_back(CameraId_Name(camera_id));
    }
    QLOG(ERROR) << "[TL]: \n" + absl::StrJoin(image_obsolete_issue_msgs, "\n");
  }

  // Return if no valid inputs for traffic light classifier.
  if (tl_inputs.empty()) {
    QLOG(WARNING) << "No valid inputs for TL classifier.";
    return;
  }

  // Detect and classify traffic lights, and publish the
  // results.
  auto traffic_light_states_proto = tl_classifier_->ClassifyTrafficLights(
      tl_inputs, nullptr, semantic_segmentation_result_ptr_map,
      coordinate_converter_);
  tl_decider_->VoteAcrossMultipleCameras(&traffic_light_states_proto);
  QLOG_IF_NOT_OK(WARNING, Publish(traffic_light_states_proto));
}

void VisionModule::DetectTrafficLight(const CameraImage& image) {
  // Skip if current camera is not used by tl.
  if (!ContainsKey(cameras_for_tl_, image.camera_id()) ||
      tl_classifier_ == nullptr) {
    return;
  }

  const int image_frame_id = FetchImageFrameDescriptor(image);
  auto& camera_images_for_tl = camera_image_frames_for_tl_[image_frame_id];

  // Clear the buffer if the frame id contains obsolete images 0.9 second ago
  // (which shares the same frame descriptor with the current one, any time
  // greater than 0.1s is ok).
  if (!camera_images_for_tl.empty() &&
      image.center_timestamp() -
              camera_images_for_tl.begin()->second.center_timestamp() >
          0.9) {
    QLOG(WARNING) << absl::StrFormat(
        "Clearing obsolete cameras not processed from a long time ago. (%.4f, "
        "%.4f)",
        image.center_timestamp(),
        camera_images_for_tl.begin()->second.center_timestamp());
    camera_images_for_tl.clear();
  }

  camera_images_for_tl[image.camera_id()] = image;

  // Skip if current frame is not ready.
  if (camera_images_for_tl.size() != cameras_for_tl_.size()) {
    return;
  }

  // NOTE(yu): Capture read-only class variables by copying to take a
  // snapshot of the class member variables when schedule the future. Since
  // all subscribe callback functions are running in the same thread,
  // there's no need to guard the variable during copying.
  std::map<CameraId, SemanticSegmentationResultProto>
      semantic_segmentation_results_map;
  {
    absl::MutexLock lock(&panonet_results_mutex_);
    semantic_segmentation_results_map = latest_panonet_results_;
  }
  future_tl_ = ScheduleFuture(
      IsDSimMode() ? nullptr : &traffic_light_loop_executor_thread_pool_,
      [this, camera_images_for_tl = camera_images_for_tl,
       ssr_map = std::move(semantic_segmentation_results_map)] {
        DetectAndClassifyTrafficLights(camera_images_for_tl, ssr_map);
      });

  // Clear current frame buffer.
  camera_images_for_tl.clear();
}

void VisionModule::ComputePanopticSegmentation(const CameraImage& image) {
  const CameraId camera_id = image.camera_id();
  if (ContainsKey(cameras_for_segmentation_, camera_id) && pano_net_ != NULL) {
    future_pool_segmentation_[camera_id] = ScheduleFuture(
        IsDSimMode() ? nullptr : &segmentation_loop_executor_thread_pool_,
        [this, image] { RunPanoNetOnImage(image); });
  }
}

void VisionModule::RunPanoNetOnImage(const CameraImage& image) {
  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  const CameraId camera_id = image.camera_id();

  constexpr double kSegmentationMaxDelay = 0.25;  // s
  if (std::abs(image.center_timestamp() - timestamp_now) >
      kSegmentationMaxDelay) {
    return;
  }

  SCOPED_QTRACE_ARG1("VisionModule::Segmentation", "camera",
                     CameraId_Name(camera_id));

  const int origin_image_width = image.width();
  const int origin_image_height = image.height();

  // Use pinned memory to accelerate H2D memory copy.
  cv::Mat image_uint8(image.height(), image.width(), CV_8UC3,
                      pano_net_input_image_data_pinned_);
  const auto image_mat = image.ToMat();
  {
    const int image_bytes = image.height() * image.width() * 3;
    SCOPED_QTRACE_ARG1("VisionModule::Segmentation_memcpy", "size",
                       image_bytes);
    memcpy(image_uint8.data, image_mat.data, image_bytes);
  }
  const auto& pano_seg_res = pano_net_->ClassifyImagePixels(image_uint8);

  // [0] semantic mat [1] instance mat [2] semantic uncertainty mat
  QCHECK_EQ(pano_seg_res.size(), 3);

  std::array<std::string, 3> encoded_images;
  for (int i = 0; i < encoded_images.size(); ++i) {
    const int output_width =
        origin_image_width / panonet_config::kSemSegOutputScale;
    const int output_height =
        origin_image_height / panonet_config::kSemSegOutputScale;
    if (pano_seg_res[i].cols != output_width ||
        pano_seg_res[i].rows != output_height) {
      cv::resize(pano_seg_res[i], pano_seg_res[i],
                 cv::Size(output_width, output_height), cv::INTER_NEAREST);
    }

    if (i == 2) {
      // Reduce the precision of uncertainty from 8 bits to 4 bits to
      // save the space of the compressed image.
      uint8_t* uncertainty_data = pano_seg_res[2].data;
      for (int j = 0; j < output_width * output_height; ++j) {
        uncertainty_data[j] &= 0xF0;
      }
    }

    SCOPED_QTRACE_ARG3("VisionModule::Segmentation_encode", "iter", i, "width",
                       pano_seg_res[i].cols, "height", pano_seg_res[i].rows);
    snappy::Compress(reinterpret_cast<char*>(pano_seg_res[i].data),
                     pano_seg_res[i].cols * pano_seg_res[i].rows,
                     &encoded_images[i]);
  }

  // Publish results
  SemanticSegmentationResultsProto ss_results;
  SemanticSegmentationResultProto ss_result;
  ss_result.set_camera_id(camera_id);
  ss_result.set_mask_height(pano_seg_res[0].rows);
  ss_result.set_mask_width(pano_seg_res[0].cols);
  ss_result.set_image_center_timestamp(image.center_timestamp());
  ss_result.set_compression_format(SemanticSegmentationResultProto::SNAPPY);
  // semantic labels
  ss_result.set_labels(reinterpret_cast<void*>(encoded_images[0].data()),
                       encoded_images[0].size());
  ss_result.set_instance_labels(
      reinterpret_cast<void*>(encoded_images[1].data()),
      encoded_images[1].size());
  ss_result.set_semantic_uncertainty_map(
      reinterpret_cast<void*>(encoded_images[2].data()),
      encoded_images[2].size());
  *ss_result.mutable_pose() = image.pose().ToVehiclePoseProto();
  ss_result.set_sem_seg_output_scale(panonet_config::kSemSegOutputScale);
  *ss_results.add_results() = ss_result;
  QLOG_IF_NOT_OK(WARNING, Publish(ss_results));
  {
    absl::MutexLock lock(&panonet_results_mutex_);
    latest_panonet_results_[camera_id] = std::move(ss_result);
  }
}

void VisionModule::RunDepthNetOnImage(const CameraImage& image) {
  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  const CameraId camera_id = image.camera_id();
  constexpr double kDepthEstimationMaxDelay = 0.25;  // s
  if (std::abs(image.center_timestamp() - timestamp_now) >
      kDepthEstimationMaxDelay) {
    return;
  }
  SCOPED_QTRACE_ARG1("VisionModule::DepthEstimation", "camera",
                     CameraId_Name(camera_id));

  cv::Size dsize = cv::Size(1920, 1080);
  cv::Mat resized_img(dsize, CV_8UC3, depth_net_input_image_data_pinned_);
  cv::resize(image.ToMat(), resized_img, dsize, 0, 0, cv::INTER_AREA);

  std::vector<cv::Mat> images;
  images.push_back(resized_img);
  const cv::Size output_size(dsize.width / depthnet::kOutputscale,
                             dsize.height / depthnet::kOutputscale);
  cv::Mat output_img(output_size, CV_32F, depth_net_output_image_data_pinned_);
  std::vector<cv::Mat> depth_estimation_results{output_img};
  depth_net_->EstimatePixelsDistances(images, &depth_estimation_results);
  QCHECK_EQ(depth_estimation_results.size(), 1);
  const auto& depth_estimation_result = depth_estimation_results[0];

  // Compression level: 0-9, default is 1.
  // A higher value means a smaller size and longer compression time.

  std::string encoded_image;
  const int image_size =
      depth_estimation_result.rows * depth_estimation_result.cols;
  cv::Mat quantized_result;
  // Convert meters in float to centi-meters in integer.
  depth_estimation_result.convertTo(quantized_result, CV_16UC1, 100.0);
  snappy::Compress(reinterpret_cast<char*>(quantized_result.data),
                   image_size * 2, &encoded_image);

  // Publish results
  auto depth_estimation_results_proto =
      std::make_unique<DepthEstimationResultsProto>();
  DepthEstimationResultProto& depth_estimation_result_proto =
      *depth_estimation_results_proto->add_results();
  depth_estimation_result_proto.set_camera_id(camera_id);
  depth_estimation_result_proto.set_camera_trigger_timestamp(
      image.trigger_timestamp());
  depth_estimation_result_proto.set_compression_format(
      DepthEstimationResultProto::SNAPPY);
  depth_estimation_result_proto.set_camera_center_timestamp(
      image.center_timestamp());
  // depth estimation results
  depth_estimation_result_proto.set_width(quantized_result.cols);
  depth_estimation_result_proto.set_height(quantized_result.rows);
  depth_estimation_result_proto.set_scale(depthnet::kOutputscale);
  depth_estimation_result_proto.set_depth_map(
      reinterpret_cast<void*>(encoded_image.data()), encoded_image.size());

  QLOG_IF_NOT_OK(WARNING, Publish(std::move(depth_estimation_results_proto)));
}

void VisionModule::EstimateDepth(const CameraImage& image) {
  const CameraId camera_id = image.camera_id();
  if (depth_net_ == nullptr ||
      !ContainsKey(cameras_for_depth_estimation_, camera_id)) {
    return;
  }
  future_pool_depth_estimation_[camera_id] = ScheduleFuture(
      IsDSimMode() ? nullptr : &depth_estimation_loop_executor_thread_pool_,
      [this, image] { RunDepthNetOnImage(image); });
}

void VisionModule::TrackObjects(
    const CameraImage& image,
    std::optional<multi_camera_fusion::TrackerInputVariable>
        curr_mono3d_result) {
  FUNC_QTRACE();
  const CameraId camera_id = image.camera_id();

  const int image_frame_id = FetchImageFrameDescriptor(image);

  absl::MutexLock lock(&mono3d_mutex_);
  auto& mono3d_results_for_mf = mono3d_result_frames_for_mf_[image_frame_id];

  if (!mono3d_results_for_mf.empty()) {
    // Get the first valid image result as ref result.
    auto ref_mono3d_result = mono3d_results_for_mf.begin()->second;
    for (const auto& [id, mono3d_result] : mono3d_results_for_mf) {
      if (!ref_mono3d_result.has_value()) {
        ref_mono3d_result = mono3d_result;
        break;
      }
    }

    // Clear the buffer if the frame id contains obsolete images 0.9s ago
    // (which shares the same frame descriptor with the current one, any time
    // greater than 0.1s is ok).
    if (ref_mono3d_result.has_value() &&
        image.center_timestamp() - ref_mono3d_result->center_timestamp > 0.9) {
      QLOG(WARNING) << absl::StrFormat(
          "Clearing obsolete cameras not processed from a long time ago. "
          "(%.4f, "
          "%.4f)",
          image.center_timestamp(), ref_mono3d_result->center_timestamp);
      mono3d_results_for_mf.clear();
    }
  }
  // Put current image mono3d result into the buffer.
  mono3d_results_for_mf[camera_id] = std::move(curr_mono3d_result);

  // Get the latest results from the Mono3D adapter.
  const std::pair<const CameraId,
                  std::optional<multi_camera_fusion::TrackerInputVariable>>*
      latest_mono3d_result = nullptr;
  for (const auto& item : mono3d_results_for_mf) {
    if (!item.second.has_value()) {
      continue;
    }
    if (latest_mono3d_result == nullptr ||
        item.second->center_timestamp >
            latest_mono3d_result->second->center_timestamp) {
      latest_mono3d_result = &item;
    }
  }

  // Skip if current frame is not ready.
  if (mono3d_results_for_mf.size() != cameras_for_mf_in_yaw_order_.size()) {
    return;
  }
  // If the frame has be ready, but has no vaid mono3d results, it means
  // the mono3d detector has a very serious performance problem, we print
  // an error log for debugging.
  if (!latest_mono3d_result) {
    QLOG(ERROR) << absl::StrFormat(
        "No valid mono3d results for frame: %d,"
        "current image is %s %.4f",
        image_frame_id, CameraId_Name(camera_id), image.center_timestamp());
    mono3d_results_for_mf.clear();
    return;
  }
  // Update imagery.
  PreloadImagery();

  // Run camera fusion pipeline.
  std::vector<CameraId> processed_cameras_in_order;
  for (auto it = cameras_for_mf_in_yaw_order_.rbegin();
       it != cameras_for_mf_in_yaw_order_.rend(); ++it) {
    QCHECK(ContainsKey(mono3d_results_for_mf, *it));
    const auto& mono3d_result = mono3d_results_for_mf.at(*it);
    if (mono3d_result != std::nullopt) {
      processed_cameras_in_order.push_back(*it);
      multi_camera_fusion_engine_->TrackSingleCameraObjects(
          mono3d_result->camera_id, mono3d_result->pose,
          mono3d_result->center_timestamp, mono3d_result->camera_measurements);
    }
  }
  // Run multi camera fusion.
  multi_camera_fusion_engine_->TrackMultiCameraObjects(
      processed_cameras_in_order, coordinate_converter_,
      latest_mono3d_result->second->center_timestamp,
      latest_mono3d_result->second->pose);

  // Clear current frame buffer.
  mono3d_results_for_mf.clear();
}

void VisionModule::DetectAndTrackObjects(const CameraImage& image) {
  const CameraId camera_id = image.camera_id();
  if (!ContainsKey(cameras_for_mono3d_, camera_id) ||
      mono3d_adapter_ == nullptr) {
    return;
  }

  future_pool_mono3d_[camera_id] = ScheduleFuture(
      IsDSimMode() ? nullptr : &mono3d_thread_pool_, [this, image] {
        auto mono3d_result = RunMono3dNetOnImage(image);
        TrackObjects(image, std::move(mono3d_result));
      });
}

std::optional<multi_camera_fusion::TrackerInputVariable>
VisionModule::RunMono3dNetOnImage(const CameraImage& image) {
  FUNC_QTRACE();

  // Skip if the current image is too old when processing it.
  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  constexpr double kMono3DMaxDelay = 0.25;  // s
  if (std::abs(image.center_timestamp() - timestamp_now) > kMono3DMaxDelay) {
    QLOG(ERROR) << absl::StrFormat(
        "Discard stale image for mono3d detection, %s %.4f",
        CameraId_Name(image.camera_id()), image.center_timestamp());
    return std::nullopt;
  }
  // TODO(zhongying): use pinned memory to accelerate H2D memory copy.
  MultiCameraMono3dMeasurementsProto mono3d_measurements =
      mono3d_adapter_->GetMono3dMeasurements(
          FindOrDie(camera_params_, image.camera_id()), image);

  std::shared_ptr<MeasurementsProto> measurements_proto =
      std::make_shared<MeasurementsProto>();
  CreateCameraMeasurementsFromMono3d(mono3d_measurements,
                                     measurements_proto.get());

  if (visual_feature_extractor_) {
    // Prepare patches for visual feature extraction.
    const int object_num = measurements_proto->measurements_size();
    const int batch_size = visual_feature_extractor_->MaxBatchSize();
    const int batch_num = object_num % batch_size == 0
                              ? object_num / batch_size
                              : object_num / batch_size + 1;
    std::vector<cv::Mat> patches(
        batch_size,
        cv::Mat(cv::Size(visual_feature_extractor_->GetInputWidth(),
                         visual_feature_extractor_->GetInputHeight()),
                CV_8UC3, cv::Scalar(0)));
    for (int batch_id = 0; batch_id < batch_num; ++batch_id) {
      for (int patch_id = 0; patch_id < batch_size; ++patch_id) {
        const int object_id = batch_id * batch_size + patch_id;
        cv::Mat& patch = patches[patch_id];
        if (object_id >= object_num) {
          patch.setTo(cv::Scalar::all(0));
          continue;
        }
        const auto& measurement =
            measurements_proto->measurements(object_id).camera3d_measurement();
        cv::Rect rect(measurement.bbox_2d().x(), measurement.bbox_2d().y(),
                      measurement.bbox_2d().width(),
                      measurement.bbox_2d().height());
        image.ToMat()(rect).copyTo(patch);
        cv::resize(patch, patch,
                   cv::Size(visual_feature_extractor_->GetInputWidth(),
                            visual_feature_extractor_->GetInputHeight()),
                   0, 0, cv::INTER_LINEAR);
      }
      // Extract features.
      const auto features = visual_feature_extractor_->ExtractFeature(patches);
      // Save feature embedding to proto.
      for (int patch_id = 0; patch_id < batch_size; ++patch_id) {
        const int object_id = batch_id * batch_size + patch_id;
        if (object_id >= object_num) {
          continue;
        }
        auto mutable_measurement =
            measurements_proto->mutable_measurements(object_id)
                ->mutable_camera3d_measurement();
        const auto& feature = features[patch_id];
        *mutable_measurement->mutable_reid_feature_embeddings() = {
            feature.begin(), feature.end()};
      }
    }
  }

  QLOG_IF_NOT_OK(WARNING, Publish(mono3d_measurements));
  multi_camera_fusion::TrackerInputVariable tracker_input_variable = {
      .camera_id = image.camera_id(),
      .pose = image.pose(),
      .center_timestamp = image.center_timestamp(),
      .camera_measurements = measurements_proto};
  return std::make_optional<multi_camera_fusion::TrackerInputVariable>(
      std::move(tracker_input_variable));
}

void VisionModule::DetectLanes(const CameraImage& image) {
  const CameraId camera_id = image.camera_id();
  if (!ContainsKey(cameras_for_lane_, camera_id) ||
      lane_detector_adapter_ == nullptr) {
    return;
  }
  VLOG(1) << "Lane processing camera " << CameraId_Name(camera_id) << " "
          << std::to_string(image.center_timestamp());
  future_pool_lane_[camera_id] =
      ScheduleFuture(&lane_detector_loop_executor_thread_pool_,
                     [this, image] { RunLaneNetOnImage(image); });
  if (QCraftRunMode() == Q_DSIM) {
    future_pool_lane_[camera_id].Wait();
  }
  return;
}

void VisionModule::RunLaneNetOnImage(const CameraImage& image) {
  FUNC_QTRACE();

  // Skip if the current image is too old when processing it.
  const double timestamp_now = ToUnixDoubleSeconds(Clock::Now());
  constexpr double kLaneMaxDelay = 0.25;  // s
  if (std::abs(image.center_timestamp() - timestamp_now) > kLaneMaxDelay) {
    return;
  }
  // TODO(xiangxiang): use pinned memory to accelerate H2D memory copy.
  const MultiCameraLanesProto lanes = lane_detector_adapter_->GetLanes(
      FindOrDie(camera_params_, image.camera_id()), image);
  QLOG_IF_NOT_OK(WARNING, Publish(lanes));
  return;
}

bool VisionModule::SetImagePose(CameraImage* image) {
  // Find the interpolated pose for camera center, skip the current camera
  // if pose not ready.
  const double image_center_timestamp =
      image->center_timestamp() + lidar_host_time_diff_;
  // TODO(yu): Set obsolete threshold to 100ms for now to accomodate FLIR
  // camera. Change it back when it's time.
  const auto pose_or =
      ComputeEstimatedVehiclePose(pose_history_, image_center_timestamp,
                                  /*obsolete_threshold=*/0.1);

  if (!pose_or) {
    double pose_history_start_timestamp = 0.0;
    double pose_history_end_timestamp = 0.0;
    if (!pose_history_.empty()) {
      pose_history_start_timestamp = pose_history_.front().first;
      pose_history_end_timestamp = pose_history_.back().first;
    }
    QLOG(WARNING) << absl::StrFormat(
        "Could not find closest pose for camera center timestamp %.4f "
        "(exposure time: %.4f), not processing camera %s published at %.4f, "
        "with pose history from %.4f to %.4f",
        image_center_timestamp, image->exposure_time(),
        CameraId_Name(image->camera_id()),
        image->image_info().decoded_image_publish_timestamp(),
        pose_history_start_timestamp, pose_history_end_timestamp);
    return false;
  }
  const auto pose = pose_or.value();

  // Find the closest pose correction if possible to be able to localize
  // on map more accurately.
  auto corrected_pose = pose;
  if (!pose_correction_history_.empty()) {
    int index_prev =
        std::distance(
            pose_correction_history_.begin(),
            std::upper_bound(
                pose_correction_history_.begin(),
                pose_correction_history_.end(), image_center_timestamp,
                [](double ts, const auto& v) { return ts < v.first; })) -
        1;
    if (index_prev >= 0) {
      const PoseCorrectionProto& pose_correction =
          pose_correction_history_[index_prev].second;
      // NOTE(yu): Use the closest pose correction regardless of staleness since
      // pose correction is static in short time frame and it's always better to
      // have pose correction than without one.
      corrected_pose = VehiclePose::FromTransform(
          corrected_pose.ToTransform() *
          AffineTransformation::FromTranslation(0., 0., pose_correction.z())
              .ApplyYawPitchRoll(0., pose_correction.pitch(),
                                 pose_correction.roll()));
    }
  }

  image->set_pose(pose);
  image->set_corrected_pose(corrected_pose);
  return true;
}

void VisionModule::OnImage(const CameraImage& camera_image) {
  SCOPED_QTRACE("VisionModule::OnImage");

  CameraImage image = camera_image;

  const auto camera_id = image.camera_id();
  // Skip the camera if it's from auxiliary camera.
  if (!IsFunctionCamera(camera_id)) {
    return;
  }
  // Skip if the camera does not have corresponding param.
  // This would happen since, for now, it would send out all cameras
  // instead of corresponding ones for a specific run context in simulation
  // mode.
  if (!ContainsKey(camera_params_, camera_id)) {
    return;
  }

  if (!SetImagePose(&image)) return;

  // Detect traffic lights.
  DetectTrafficLight(image);

  // Estimate depth.
  EstimateDepth(image);

  // Use panonet to compute segmentation.
  ComputePanopticSegmentation(image);

  // Detect objects using mono3d net.
  DetectAndTrackObjects(image);

  // Detect lanes using lane net.
  DetectLanes(image);
}

void VisionModule::OnInit() {
  vantage_client_man::CreateVantageClientMan(param_manager());
  ark_client_man::CreateArkClientMan(param_manager());

  const RunParamsProtoV2 run_params = GetRunParams();

  auto init_tl_net = [this, &run_params] {
    if (FLAGS_enable_tl_net) {
      QLOG(INFO) << "Initializing TL net...";

      // Initialize k net.
      NetParam tl_net_param;
      constexpr char kKNetParamKey[] = "tl_net_param";
      CHECK_OK(param_manager().GetProtoParam(kKNetParamKey, &tl_net_param));
      tl_classifier_ = std::make_unique<TrafficLightClassifier>(
          &semantic_map_manager_, run_params, tl_net_param, param_manager());
      tl_decider_ = std::make_unique<TrafficLightDecider>();

      QLOG(INFO) << "Initializing TL net... Done!";
    }
  };

  auto init_pano_net = [this, &run_params] {
    if (IsDBQConext() && !IsDBQv4() && FLAGS_enable_ll_net) {
      QLOG(INFO) << "Initializing pano net...";

      // Initialize pano net.
      NetParam pano_net_param;
      if (!IsDBQConext() || IsDBQv4()) {
        constexpr char kPanoNetParamKey[] = "pbq_panonet_param";
        CHECK_OK(
            param_manager().GetProtoParam(kPanoNetParamKey, &pano_net_param));
      } else {
        constexpr char kPanoNetParamKey[] = "panonet_param";
        CHECK_OK(
            param_manager().GetProtoParam(kPanoNetParamKey, &pano_net_param));
      }

#ifdef Q_CPU_ONLY
      // Enable pano net for more cameras in cpu mode.
      cameras_for_segmentation_ = {CAM_R_FRONT,       CAM_FRONT,
                                   CAM_L_FRONT,       CAM_L_FRONT_LEFT,
                                   CAM_L_FRONT_RIGHT, CAM_R_FRONT_LEFT,
                                   CAM_R_FRONT_RIGHT};
#else
      // Use CAM_R_FRONT since all R_FRONT are horizontal.
      cameras_for_segmentation_ = {CAM_R_FRONT, CAM_FRONT};
      // Enable pano net for more cameras if there's more than 1 gpu onboard
      // or in cpu mode.
      if (!pano_net_param.use_gpu() || !FLAGS_panonet_use_gpu) {
        cameras_for_segmentation_.insert({CAM_L_FRONT, CAM_L_FRONT_LEFT,
                                          CAM_L_FRONT_RIGHT, CAM_R_FRONT_LEFT,
                                          CAM_R_FRONT_RIGHT});
      } else {
        if (!IsDBQConext()) {
          cameras_for_segmentation_.insert(
              {CAM_PBQ_REAR_LEFT, CAM_PBQ_REAR_RIGHT});
        } else if (IsDBQv4()) {
          // DBQV4 cameras.
          cameras_for_segmentation_.insert(
              {CAM_PBQ_FRONT_TELE, CAM_PBQ_REAR_LEFT, CAM_PBQ_REAR_RIGHT});
        } else {
          // DBQ cameras.
          cameras_for_segmentation_.insert(
              {CAM_L_FRONT, CAM_L_FRONT_LEFT, CAM_L_FRONT_RIGHT,
               CAM_R_FRONT_LEFT, CAM_R_FRONT_RIGHT, CAM_L_REAR_LEFT, CAM_L_LEFT,
               CAM_R_RIGHT, CAM_R_REAR_RIGHT});
        }
      }
#endif
      pano_net_ =
          std::make_unique<panonet::PanoNet>(run_params, pano_net_param);

      QLOG(INFO) << "Initializing pano net... Done!";
    }
  };

  auto init_mono3d_net = [this, &run_params] {
    if (!IsDBQConext() || IsDBQv4() || FLAGS_enable_mono3d_net) {
      QLOG(INFO) << "Initializing mono3d net...";
      // Initialize mono3d.
      NetParam mono3d_param;
      constexpr char kMono3DParamKey[] = "mono3d_param";
      CHECK_OK(param_manager().GetProtoParam(kMono3DParamKey, &mono3d_param));
      cameras_for_mono3d_ = {CAM_PBQ_REAR_LEFT,   CAM_PBQ_FRONT_LEFT,
                             CAM_PBQ_FRONT_WIDE,  CAM_PBQ_FRONT_TELE,
                             CAM_PBQ_FRONT_RIGHT, CAM_PBQ_REAR_RIGHT,
                             CAM_PBQ_REAR};
      mono3d_adapter_ = std::make_unique<Mono3DAdapter>(mono3d_param);
      QLOG(INFO) << "Initializing mono3d net... Done!";

      QLOG(INFO) << "Initializing reid net...";
      visual_feature_extractor_ =
          tracker::VisualFeatureExtractor::Create(param_manager());
      QLOG(INFO) << "Initializing reid net... Done!";
      multi_camera_fusion_engine_ =
          std::make_unique<multi_camera_fusion::MultiCameraFusionEngine>(
              this, &multi_camera_fusion_thread_pool_, &semantic_map_manager_,
              cameras_for_mono3d_, run_params);
      // Init imagery.
      imagery_manager_.SetDisabled(ImageryLayer::kZOccupancy |
                                   ImageryLayer::kIntensity |
                                   ImageryLayer::kZSpan);
    }
  };

  auto init_depth_net = [this, &run_params] {
    if (IsDBQConext() && !IsDBQv4() && FLAGS_enable_depth_net) {
      QLOG(INFO) << "Initializing depth net...";

      // Initialize depth net.
      NetParam depth_net_param;
      constexpr char kDepthNetParamKey[] = "depth_net_param";
      CHECK_OK(
          param_manager().GetProtoParam(kDepthNetParamKey, &depth_net_param));

      // Use CAM_R_FRONT/CAM_FRONT only
      if (IsOnboardMode()) {
        cameras_for_depth_estimation_ = {CAM_R_FRONT, CAM_FRONT};
      }
      depth_net_ =
          std::make_unique<depthnet::DepthNet>(run_params, depth_net_param);

      QLOG(INFO) << "Initializing depth net... Done!";
    }
  };

  auto init_lane_net = [this] {
    if (FLAGS_enable_lane_net) {
      QLOG(INFO) << "Initializing lane net...";

      // Initialize lane.
      NetParam lane_param;
      constexpr char kLaneParamKey[] = "lane_param";
      CHECK_OK(param_manager().GetProtoParam(kLaneParamKey, &lane_param));
      cameras_for_lane_ = {CAM_PBQ_FRONT_WIDE};
      lane_detector_adapter_ =
          std::make_unique<LaneDetectorAdapter>(lane_param);
      QLOG(INFO) << "Initializing lane net... Done!";
    }
  };

  std::vector<Future<void>> net_init_futures;
  ThreadPool net_init_thread_pool(0);
  net_init_futures.push_back(
      ScheduleFuture(&net_init_thread_pool, init_tl_net));
  net_init_futures.push_back(
      ScheduleFuture(&net_init_thread_pool, init_pano_net));
  net_init_futures.push_back(
      ScheduleFuture(&net_init_thread_pool, init_mono3d_net));
  net_init_futures.push_back(
      ScheduleFuture(&net_init_thread_pool, init_depth_net));
  net_init_futures.push_back(
      ScheduleFuture(&net_init_thread_pool, init_lane_net));

  for (auto& net_init_future : net_init_futures) {
    net_init_future.Wait();
  }

  // Using LoadWholeMap() is strongly discouraged, use UpdateSmoothPos()
  // instead. The later only loads portion of the map as needed
  // semantic_map_manager_.LoadWholeMap().Build();

  UpdateCameraLidarParams();
  CollectCamerasForMultiCameraPipelines();
}

void VisionModule::OnSubscribeChannels() {
  if (IsOnboardMode()) {
    Subscribe(&VisionModule::UpdatePose, this, "pose_proto");
  } else {
    Subscribe(&VisionModule::UpdatePose, this, "sensor_pose");
  }
  Subscribe(&VisionModule::UpdateLocalizationTransform, this);
  Subscribe(&VisionModule::UpdateLidarHostTimeDiff, this,
            "lidar_host_time_diff_proto");
  Subscribe(&VisionModule::UpdatePoseCorrection, this, "pose_correction_proto");

  // Subscribe images.
  SubscribeDecodedImage([this](const CameraImage& camera_image) {
    VisionModule::OnImage(camera_image);
  });
}

void VisionModule::OnSetUpTimers() {}

void VisionModule::UpdatePose(std::shared_ptr<const PoseProto> pose) {
  semantic_map_manager_.UpdateSmoothPos(
      Vec2d{pose->pos_smooth().x(), pose->pos_smooth().y()});
  semantic_map_manager_.ApplyUpdate();
  pose_history_.push_back(
      std::make_pair(pose->timestamp(), VehiclePose(*pose)));
}

void VisionModule::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  coordinate_converter_.UpdateLocalizationTransform(
      *localization_transform_proto);
  semantic_map_manager_.UpdateLocalizationTransform(
      *localization_transform_proto, /*thread_pool=*/nullptr);
}

void VisionModule::UpdateLidarHostTimeDiff(
    std::shared_ptr<const LidarHostTimeDiffProto> lidar_host_time_diff_proto) {
  lidar_host_time_diff_ = lidar_host_time_diff_proto->time_diff();
}

void VisionModule::UpdatePoseCorrection(
    std::shared_ptr<const PoseCorrectionProto> pose_correction_proto) {
  pose_correction_history_.push_back(std::make_pair(
      pose_correction_proto->mid_scan_timestamp(), *pose_correction_proto));
}

void VisionModule::UpdateCameraLidarParams() {
  const RunParamsProtoV2 run_params = GetRunParams();
  camera_params_ = ComputeAllCameraParams(
      run_params.vehicle_params(),
      /*only_collect_function_camera=*/true,
      /*only_collect_camera_for_current_run_context=*/true);

  const auto lidar_params = run_params.vehicle_params().lidar_params();
  for (const auto& lidar_param : lidar_params) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }
}

// TODO(zheng, jingwei): Put this func to a util file.
void VisionModule::CreateCameraMeasurementsFromMono3d(
    const MultiCameraMono3dMeasurementsProto& mono3d_measurements,
    MeasurementsProto* measurements_proto) {
  double min_timestamp = std::numeric_limits<double>::max();
  double max_timestamp = -std::numeric_limits<double>::max();
  constexpr double kMinAllowedImageSize = 50;         // pixels
  constexpr double kMinAllowedMono3DScore = 0.4;      // Mono3D score
  constexpr double kMaxAllowedWidth = 3.7;            // m
  constexpr double kMinAllowedVisibilityScore = 0.2;  // Visibility score
  for (const auto& single_camera_measurements :
       mono3d_measurements.single_camera_mono3d_measurements()) {
    for (const auto& m : single_camera_measurements.measurements()) {
      // If the width if more than 3.7 m we filter it out.
      if (m.width() >= kMaxAllowedWidth) {
        continue;
      }
      // Filter out object if visibility score too low.
      if (m.visibility_score() < kMinAllowedVisibilityScore) {
        continue;
      }
      // NOTE(zheng): If the bbox size is below kMinAllowedImageSize, the
      // depth error from mono3d is very big, so we use a threshold of 50
      // pixels here to filter out the measurements.
      const auto max_image_size =
          std::max(m.bbox_2d().width(), m.bbox_2d().height());
      // Filter out offroad measurements (non-ped) and small objects.
      if (max_image_size < kMinAllowedImageSize ||
          (m.type() != MT_PEDESTRIAN &&
           !IsOnRoadPoint({m.pos().x(), m.pos().y()}))) {
        continue;
      }
      // Filter out objects by score but not those in Cautious Region.
      if (!HasOverlapWithCautiousRegion(m) &&
          m.existence_confidence() < kMinAllowedMono3DScore) {
        continue;
      }

      min_timestamp = std::min(min_timestamp, m.timestamp());
      max_timestamp = std::max(max_timestamp, m.timestamp());
      auto* measurement_proto = measurements_proto->add_measurements();
      measurement_proto->set_timestamp(m.timestamp());
      measurement_proto->set_type(m.type());
      measurement_proto->set_type_source(MeasurementTypeSource::MTS_MONO_3D);
      auto* camera3d_m = measurement_proto->mutable_camera3d_measurement();
      *camera3d_m->mutable_bbox_2d() = m.bbox_2d();
      *camera3d_m->mutable_pos() = m.pos();
      camera3d_m->set_width(m.width());
      camera3d_m->set_length(m.length());
      camera3d_m->set_height(m.height());
      camera3d_m->set_camera_id(m.camera_id());
      camera3d_m->set_timestamp(m.timestamp());
      camera3d_m->set_type(m.type());
      camera3d_m->set_mono3d_depth(m.mono3d_depth());
      camera3d_m->set_existence_confidence(m.existence_confidence());
      *camera3d_m->mutable_mono3d_pos_cov() = m.pos_covariance();
      // Heading set is treated differently in critical regision;
      camera3d_m->set_heading(SetCriticalRegionHeading(m));
      camera3d_m->set_visibility_score(m.visibility_score());
    }
  }
  measurements_proto->set_min_timestamp(min_timestamp);
  measurements_proto->set_max_timestamp(max_timestamp);
  measurements_proto->set_group_type(MeasurementsProto::CAMERA);
}

double VisionModule::SetCriticalRegionHeading(const Mono3dMeasurementProto& m) {
  // If diff between target heading and ego vechile heading is less than
  // pi/2, set the target heading to ego vehicle heading, otherwise set to
  // reverse direction.
  // This section hacks the heading, put the heading of close range objects
  // to the direction of ego vehicle heading, first compute the critical
  // region.
  const auto& vehicle_pose = m.vehicle_pose();
  QCHECK(vehicle_pose.has_x() && vehicle_pose.has_y() &&
         vehicle_pose.has_yaw());
  // Length of the hack region is 10 m while the width is (3.7+1.8)*2
  constexpr double kCriticalRegionLength = 10.0;
  constexpr double kCriticalRegionWidth = (3.7 + 1.8) * 2;
  const Vec2d ego_vehicle_bev_pos = {vehicle_pose.x(), vehicle_pose.y()};
  const Box2d critical_region(ego_vehicle_bev_pos, vehicle_pose.yaw(),
                              kCriticalRegionLength, kCriticalRegionWidth);
  const Vec2d camera_bev_pos = {m.pos().x(), m.pos().y()};
  if (m.type() == MT_VEHICLE && critical_region.IsPointIn(camera_bev_pos)) {
    const double angle_diff =
        std::abs(NormalizeAngle(m.heading() - vehicle_pose.yaw()));
    if (angle_diff > -M_PI_2 && angle_diff < M_PI_2) {
      return vehicle_pose.yaw();
    } else {
      return NormalizeAngle(vehicle_pose.yaw() + M_PI);
    }
  } else {
    return m.heading();
  }
}

bool VisionModule::HasOverlapWithCautiousRegion(
    const Mono3dMeasurementProto& m) {
  // We leave a cautious region where objects will not be
  // filtered by score.
  // Length of the cautious region is 30 m and width is
  // 5 times of lane width.
  constexpr double kCautiousRegionLength = 30;      // m
  constexpr double kCautiousRegionWidth = 3.7 * 5;  // m
  const auto& vehicle_pose = m.vehicle_pose();
  QCHECK(vehicle_pose.has_x() && vehicle_pose.has_y() &&
         vehicle_pose.has_yaw());
  const Vec2d ego_vehicle_bev_pos = {vehicle_pose.x(), vehicle_pose.y()};
  const Box2d cautious_region(ego_vehicle_bev_pos, vehicle_pose.yaw(),
                              kCautiousRegionLength, kCautiousRegionWidth);
  const Vec2d camera_bev_pos = {m.pos().x(), m.pos().y()};
  const Box2d camera_bev_bbox(camera_bev_pos, m.heading(), m.length(),
                              m.width());
  return cautious_region.HasOverlap(camera_bev_bbox);
}

bool VisionModule::IsOnRoadPoint(const Vec2d& pos_smooth) {
  const auto point_global = coordinate_converter_.SmoothToGlobal(pos_smooth);
  const auto indexer = local_imagery_.GetIndexer(
      point_global.x(), point_global.y(), coordinate_converter_.GetLevel());

  if (indexer && local_imagery_.DistToCurbAt(*indexer) < 0.0) {
    return true;
  }
  return false;
}

void VisionModule::PreloadImagery() {
  SCOPED_QTRACE("VisionModule::PreloadImagery");
  if (pose_history_.empty()) {
    return;
  }
  // Preload imagery.
  const auto vehicle_smooth_pos = pose_history_.back().second.coord2d();
  const Vec2d pose_global = coordinate_converter_.SmoothToGlobal(
      {vehicle_smooth_pos.x(), vehicle_smooth_pos.y()});
  const auto preload_imagery_future = imagery_manager_.PreloadAsync(
      pose_global.x(), pose_global.y(),
      /*clear_far_patches=*/true, &imagery_preloading_thread_pool_);
  if (QCraftRunMode() == Q_DSIM) {
    preload_imagery_future.Wait();
  }
  local_imagery_.Update(imagery_manager_, pose_global.x(), pose_global.y());
}
}  // namespace qcraft
