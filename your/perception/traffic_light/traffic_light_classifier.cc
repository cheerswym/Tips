#include "onboard/perception/traffic_light/traffic_light_classifier.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/global/trace.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/semantic_segmentation_result.h"
#include "onboard/vis/canvas/canvas.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

DEFINE_bool(enable_tl_occlusion_check, true,
            "check if tl is occluded and publish as unknown if occluded");
DEFINE_bool(enable_tl_occlusion_visual_check, true,
            "check if tl is occluded and publish as unknown if occluded");
DEFINE_bool(save_traffic_light_debug_image, false,
            "Save debug image in traffic light classification");
DEFINE_bool(save_tl_occlusion_debug_image, false,
            "Save debug image for occlusion feature");
DEFINE_bool(extract_traffic_light_region_images, false,
            "Extract traffic light region images for offline training");
DEFINE_string(traffic_light_region_images_dir, "",
              "The directory to put extracted traffic light region images");
DEFINE_bool(enable_small_tl_size, true,
            "Use a smaller tl cropping size when modifying map.");
DEFINE_bool(use_hmm_to_fuse, true, "Whether to use hmm for temporal fusion.");
DEFINE_bool(merge_to_close_tl_roi, true,
            "Whether to merge tl ROI that are too close.");

namespace qcraft {
namespace {
constexpr int kTLImageWidth = 40;
constexpr int kTLImageHeight = 60;
constexpr double kTLMaxDist = 150.0;
// The total time window to keep TlResult.
// This window is used for both color voting and flashing checking. Due to the
// full pattern of flashing state would be revealed in a 1 second time window,
// so keeping one that's larger to fully capture the pattern.
constexpr double kTLResultBufferWindow = 2.5;  // Unit of s
// The time of frame to vote color during temporal fusion.
constexpr double kTLVotingWindow = 0.6;  // Unit of s
constexpr double kTLTimePerFrame = 0.1;  // Time (unit of s) for each frame
constexpr float kOcclusionUncertaintyThreshold = 0.75f;
constexpr double kTLVisualAreaOcclusionThreshold = 0.3;
constexpr double kMinVisibleRatioToEstimateTlState = 0.6;
constexpr double kMaxNearbyTLDist = 15;  // m
constexpr double kMaxTLAngleDiff = d2r(30.0);
// Set a stricter threshold for close by TL, e.g. temp TL, since it's usually
// 4-way facing and likely to be exposed to the opposite direction when angle is
// large.
constexpr double kMaxTLAngleDiffForNearbyTL = d2r(20.0);
// When the physical distance between two traffic lights is very close, it is
// judged that the traffic light is a multi-faceted traffic.
constexpr double kMaxMultiFacetedTLDistance = 0.4;  // m

bool Contains(std::string_view parent_string, std::string_view child_string) {
  return parent_string.find(child_string) != std::string::npos;
}

std::unordered_set<int64> FindMultiFacetedTLs(
    const SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter, const VehiclePose& pose) {
  SCOPED_QTRACE("FindMultiFacetedTLs");

  const auto& traffic_lights =
      semantic_map_manager.semantic_map().traffic_lights();
  // Judging whether it is a multi-faceted traffic light by distance.
  std::unordered_set<int64> multi_faceted_tl_set;
  const int num_tls = traffic_lights.size();
  std::vector<Vec3d> tl_positions;
  tl_positions.reserve(32);
  for (int i = 0; i < num_tls; i++) {
    const auto& tl = traffic_lights[i];
    const auto tl_pos = coordinate_converter.GlobalToSmooth(
        {tl.point().longitude(), tl.point().latitude(), tl.point().altitude()});
    if ((tl_pos - pose.coord()).squaredNorm() <= Sqr(kTLMaxDist)) {
      tl_positions.push_back(tl_pos);
    }
  }
  const int num_nearby_tls = tl_positions.size();
  for (int i = 0; i < num_nearby_tls; i++) {
    const auto& tl_pos = tl_positions[i];
    for (int j = i + 1; j < num_nearby_tls; j++) {
      const auto& other_tl_pos = tl_positions[j];
      const double dist2 = tl_pos.DistanceSquareTo(other_tl_pos);
      if (dist2 < Sqr(kMaxMultiFacetedTLDistance)) {
        multi_faceted_tl_set.insert(traffic_lights[i].id());
        multi_faceted_tl_set.insert(traffic_lights[j].id());
      }
    }
  }
  return multi_faceted_tl_set;
}

}  // namespace

void TrafficLightClassifier::SaveTLRegionImage(
    const TrafficLightClassifier::TlPatchInfo& patch, const cv::Mat& image) {
  QCHECK_NE(FLAGS_traffic_light_region_images_dir, "");
  const std::string filename = absl::StrFormat(
      "%s/%lld-%s-%.3f.jpg", FLAGS_traffic_light_region_images_dir,
      patch.tl_id(), CameraId_Name(patch.camera_id),
      patch.image_trigger_timestamp);
  cv::Mat img;
  cv::cvtColor(image, img, cv::COLOR_RGB2BGR);
  cv::imwrite(filename, img);
  LOG(INFO) << "Saving image " << filename;

  // Append the image info to the info file.
  const std::string info_filename =
      absl::StrFormat("%s/info.txt", FLAGS_traffic_light_region_images_dir);
  std::ofstream ofs;
  ofs.open(info_filename, std::ofstream::out | std::ofstream::app);
  ofs << absl::StrFormat("%lld %s %.3f %d %d %d %d\n", patch.tl_id(),
                         CameraId_Name(patch.camera_id),
                         patch.image_trigger_timestamp, patch.x, patch.y,
                         patch.width, patch.height);
  ofs.close();
}

TrafficLightClassifier::TrafficLightClassifier(
    const SemanticMapManager* semantic_map_manager,
    const RunParamsProtoV2& run_params, const NetParam& tl_net_param,
    const ParamManager& param_manager)
    : semantic_map_manager_(semantic_map_manager),
      param_manager_(param_manager) {
  tl_net_classifier_ =
      std::make_unique<TLNetClassifier>(run_params, tl_net_param);
  hmm_ = std::make_unique<HMM>();
  if (FLAGS_enable_small_tl_size) {
    tl_image_width_ = kTLImageWidth;
    tl_image_height_ = kTLImageHeight;
  } else {
    tl_image_width_ = TLNet::kImageWidth;
    tl_image_height_ = TLNet::kImageHeight;
  }
}

TrafficLightClassifier::TlResults
TrafficLightClassifier::ClassifyTrafficLightPatches(
    const TlPatchInfos& tl_patch_info_vec) {
  SCOPED_QTRACE("TrafficLightClassifier::ClassifyTrafficLightPatches");

  // First dimension is batch size and second dimension is sequence length.
  std::vector<std::vector<cv::Mat>> images;
  for (const auto& patch_info : tl_patch_info_vec) {
    auto& current_image_seq = tl_image_seq_[patch_info.tl_cam_id()];
    QCHECK(!current_image_seq.empty());
    const auto current_ts = current_image_seq.back_time();

    // Pop the outdated images of current traffic light
    while (current_image_seq.size() > TLNet::kSequenceLength ||
           current_ts - current_image_seq.front_time() >
               kTLTimePerFrame *
                   static_cast<double>(TLNet::kSequenceLength - 1)) {
      current_image_seq.pop_front();
    }
    QCHECK(!current_image_seq.empty());

    // If the number of recent images for current traffic light is less than
    // the sequence length required for tl_net_classifier, repeat the first
    // image
    while (current_image_seq.size() < TLNet::kSequenceLength) {
      current_image_seq.push_front(current_image_seq.front_time(),
                                   current_image_seq.front_value());
    }

    // Push recent images into img_vec for inference
    std::vector<cv::Mat> img_vec;
    for (const auto& [_, img] : current_image_seq) {
      img_vec.push_back(img);
    }
    images.push_back(std::move(img_vec));
  }

  // Get color classification results from tl_net_classifier
  const auto cls_results =
      tl_net_classifier_->ClassifyTrafficLightPatches(&images);
  /*
                  │                    │
                  ├───►   tl_patch  ◄──┼──► merged_tl_path
                  │                    │
                  ├──┬──┬──┬──┬──┬──┬──┼──┬──┐
tl_patch_info_vec │  │  │  │  │  │  │  │  │  │
                  └─┬┴─┬┴┬─┴─┬┴─┬┴─┬┴─┬┴─┬┴─┬┘
                    │  │ │   │  │  │  │  │  │
                    ▼  ▼ ▼   ▼  ▼  ▼  ▼  │  │
                  ┌──┬──┬──┬──┬──┬──┬──┐ │  │
    batch_results │  │  │  │  │  │  │  │ │  │
                  └──┴──┘▲─┴─▲└─▲└─▲└──┘ │  │
                         │   │  │  │     │  │
                         └───┼──┼──┼─────┤  │
                             │  │  │     │  │
                             └──┼──┼─────┘  │
                                │  │        │
                                └──┼────────┤
                                   │        │
                                   └────────┘
  */
  QCHECK_EQ(cls_results.color_cls_prob.size(), tl_patch_info_vec.size());
  TrafficLightClassifier::TlResults batch_results;
  // If the current state is not valid, return TL_UNKNOWN.
  std::map<TlCamId, int> tl_cam_id_to_result_index;
  for (int i = 0; i < tl_patch_info_vec.size(); i++) {
    // The merge ROI are all at the end of the list.
    if (tl_patch_info_vec[i].is_merged_tl_patch()) break;
    tl_cam_id_to_result_index[tl_patch_info_vec[i].tl_cam_id()] = i;
    batch_results.emplace_back(TL_UNKNOWN, 0.f, TL_SHAPE_UNKNOWN, 0.f, false,
                               0.f, std::vector<AABox2d>({}),
                               std::vector<AABox2d>({}),
                               TrafficLightCountDown::TL_COUNTDOWN_UNKNOWN);
  }

  auto transform_box_to_cam_image_coor =
      [](const AABox2d& patch_box, const AABox2d& box, const bool is_vertical) {
        const auto left_top_x = patch_box.center_x() - patch_box.length() / 2.;
        const auto left_top_y = patch_box.center_y() - patch_box.width() / 2.;
        double box_x, box_y, box_length, box_width;
        if (is_vertical) {
          const auto width_scale = patch_box.length() / TLNet::kImageWidth;
          const auto height_scale = patch_box.width() / TLNet::kImageHeight;
          box_x = box.center_x() * width_scale;
          box_y = box.center_y() * height_scale;
          box_length = box.length() * width_scale;
          box_width = box.width() * height_scale;
        } else {
          const auto width_scale = patch_box.length() / TLNet::kImageHeight;
          const auto height_scale = patch_box.width() / TLNet::kImageWidth;
          box_x = box.center_y() * width_scale;
          box_y = (TLNet::kImageWidth - box.center_x()) * height_scale;
          box_length = box.width() * width_scale;
          box_width = box.length() * height_scale;
        }
        return AABox2d({left_top_x + box_x, left_top_y + box_y}, box_length,
                       box_width);
      };

  for (int i = 0; i < tl_patch_info_vec.size(); i++) {
    const auto& cls_colors = cls_results.color_cls[i];
    const auto& cls_shapes = cls_results.shape_cls[i];
    const auto& cls_valids = cls_results.valid_state[i];
    const auto& cls_temporals = cls_results.temporal_state[i];
    const auto& cls_colors_prob = cls_results.color_cls_prob[i];
    const auto& cls_shapes_prob = cls_results.shape_cls_prob[i];
    const auto& cls_valids_prob = cls_results.valid_state_prob[i];
    const auto& cls_temporals_prob = cls_results.temporal_state_prob[i];
    const auto& cls_lb_boxes = cls_results.lb_boxes[i];
    const auto& cls_lt_boxes = cls_results.lt_boxes[i];
    const auto& tl_patch = tl_patch_info_vec[i];
    const auto& tl_ids = tl_patch.tl_ids;
    const auto& is_vertical = tl_patch.is_vertical;

    // Transform boxes to image cood.
    std::vector<AABox2d> image_lb_boxes, image_lt_boxes;
    AABox2d patch_box(
        {static_cast<double>(tl_patch.x), static_cast<double>(tl_patch.y)},
        static_cast<double>(tl_patch.width),
        static_cast<double>(tl_patch.height));
    for (const auto& box : cls_lb_boxes) {
      image_lb_boxes.push_back(
          transform_box_to_cam_image_coor(patch_box, box, is_vertical));
    }
    for (const auto& box : cls_lt_boxes) {
      image_lt_boxes.push_back(
          transform_box_to_cam_image_coor(patch_box, box, is_vertical));
    }

    // The number of merged tl patch detection proposals must be the same as the
    // number of traffic lights on the tl patch.
    if (tl_patch.is_merged_tl_patch() && tl_ids.size() != image_lt_boxes.size())
      continue;
    // If there is no detection proposal, output unknown.
    if (image_lt_boxes.empty()) continue;

    // Note(wanzeng): The detection results are consistent with the traffic
    // lights on the map, sorted from left to right.
    std::vector<int> sorted_box_idx(tl_ids.size());
    iota(sorted_box_idx.begin(), sorted_box_idx.end(), 0);
    std::sort(sorted_box_idx.begin(), sorted_box_idx.end(),
              [&image_lt_boxes](int i1, int i2) {
                return image_lt_boxes[i1].center() <
                       image_lt_boxes[i2].center();
              });

    for (int j = 0; j < sorted_box_idx.size(); j++) {
      const int box_index = sorted_box_idx[j];
      TlCamId tl_cam_id = {tl_ids[j], tl_patch.camera_id};
      const int result_index = tl_cam_id_to_result_index[tl_cam_id];
      std::vector<AABox2d> belong_lb_box;
      for (const auto& lb_box : image_lb_boxes) {
        if (lb_box.IsPointIn(image_lt_boxes[box_index].center())) {
          belong_lb_box.push_back(lb_box);
          break;
        }
      }
      if (cls_valids[box_index]) {
        TrafficLightShape shape = cls_shapes[box_index];
        const auto shape_score = cls_shapes_prob[box_index];
        // Switch back if the patch not vertical.
        if (!is_vertical) {
          switch (shape) {
            case TL_LEFT_ARROW:
              shape = TL_DOWN_ARROW;
              break;
            case TL_RIGHT_ARROW:
              shape = TL_UP_ARROW;
              break;
            case TL_UP_ARROW:
              shape = TL_LEFT_ARROW;
              break;
            case TL_DOWN_ARROW:
              shape = TL_RIGHT_ARROW;
              break;
            default:
              break;
          }
        }
        batch_results[result_index] =
            TlResult(cls_colors[box_index], cls_colors_prob[box_index], shape,
                     shape_score, cls_temporals[box_index],
                     cls_temporals_prob[box_index], belong_lb_box,
                     std::vector<AABox2d>({image_lt_boxes[box_index]}),
                     cls_results.countdown[i]);

      } else {
        // If it's NOT valid traffic light output.
        batch_results[result_index] =
            TlResult(TL_UNKNOWN, cls_valids_prob[box_index], TL_SHAPE_UNKNOWN,
                     cls_valids_prob[box_index], cls_temporals[box_index],
                     cls_temporals_prob[box_index], belong_lb_box,
                     std::vector<AABox2d>({image_lt_boxes[box_index]}),
                     cls_results.countdown[i]);
      }
    }
  }
  return batch_results;
}

void TrafficLightClassifier::UpdateRunParams() {
  RunParamsProtoV2 run_params;
  param_manager_.GetRunParams(&run_params);

  camera_params_ = ComputeAllCameraParams(run_params.vehicle_params());
  for (const auto& [camera_id, camera_params] : camera_params_) {
    const auto vehicle_to_camera =
        camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
    vehicle_to_camera_per_camera_.emplace(camera_id, vehicle_to_camera);
  }
}

TrafficLightClassifier::TlPatchInfos
TrafficLightClassifier::FindAndFetchTrafficLightPatches(
    const std::map<CameraId, CameraImage>& tl_images,
    const CoordinateConverter& coordinate_converter) {
  SCOPED_QTRACE("TrafficLightClassifier::FindAndFetchTrafficLightPatches");

  if (tl_images.empty()) return {};

  // Find all traffic lights and compute out the patch region.
  TlPatchInfos tl_patch_vec;
  if (!coordinate_converter.is_valid()) return tl_patch_vec;
  constexpr int kExpectedNumTrafficLightPatches = 12;
  tl_patch_vec.reserve(kExpectedNumTrafficLightPatches);

  // Judging whether it is a multi-faceted traffic light by distance.
  const std::unordered_set<int64> multi_faceted_tl_set =
      FindMultiFacetedTLs(*semantic_map_manager_, coordinate_converter,
                          tl_images.begin()->second.pose());

  for (const auto& [camera_id, camera_image] : tl_images) {
    QCHECK(camera_image.has_corrected_pose());

    // Prepare transformation matrices.
    const auto& corrected_pose = camera_image.corrected_pose();
    const auto vehicle2smooth_transform = corrected_pose.ToTransform();
    const auto smooth2vehicle_transform = vehicle2smooth_transform.Inverse();
    const auto smooth_to_camera_transform_corrected =
        FindOrDie(vehicle_to_camera_per_camera_, camera_id) *
        smooth2vehicle_transform;

    const auto& camera_params = FindOrDie(camera_params_, camera_id);
    const auto smooth_to_camera_transform =
        (corrected_pose.ToTransform() *
         camera_params.camera_to_vehicle_extrinsics().ToTransform())
            .Inverse();

    // Find nearby traffic lights.
    const auto& traffic_lights =
        semantic_map_manager_->semantic_map().traffic_lights();
    const auto& belonging_level = coordinate_converter.GetLevel();
    std::vector<mapping::TrafficLightProto> curr_image_traffic_lights;
    for (const auto& traffic_light : traffic_lights) {
      bool is_belonging_level = false;
      for (const auto& tl_belonging_level : traffic_light.belonging_levels()) {
        if (belonging_level == tl_belonging_level) {
          is_belonging_level = true;
          break;
        }
      }
      if (!is_belonging_level) continue;
      // Note: the z direction correction is ignored here in smooth coord.
      const Vec3d tl_pos = coordinate_converter.GlobalToSmooth(
          {traffic_light.point().longitude(), traffic_light.point().latitude(),
           traffic_light.point().altitude()});
      const auto tl_pos_camera_frame =
          smooth_to_camera_transform_corrected.TransformPoint(tl_pos);
      // Ignore too far TLs.
      const double dist = tl_pos_camera_frame.x();
      if (dist > kTLMaxDist) {
        continue;
      }

      // Ignore TLs that are not facing to AV and not close-by.
      const double tl_max_angle_diff =
          (dist < kMaxNearbyTLDist &&
           ContainsKey(multi_faceted_tl_set, traffic_light.id()))
              ? kMaxTLAngleDiffForNearbyTL
              : kMaxTLAngleDiff;
      const double tl_to_cam_dir =
          Vec2d(-tl_pos_camera_frame.x(), -tl_pos_camera_frame.y()).FastAngle();
      const auto traffic_light_smooth_yaw =
          coordinate_converter.GlobalYawToSmooth(traffic_light.bb_heading());
      const auto traffic_light_camera_yaw = traffic_light_smooth_yaw -
                                            corrected_pose.yaw -
                                            FindOrDie(camera_params_, camera_id)
                                                .camera_to_vehicle_extrinsics()
                                                .yaw;
      const double tl_yaw_diff =
          std::abs(NormalizeAngle(traffic_light_camera_yaw - tl_to_cam_dir));
      if (tl_yaw_diff > tl_max_angle_diff) {
        continue;
      }
      const auto image_pos = projection_util::SmoothPointToImagePos(
          tl_pos, smooth_to_camera_transform, camera_params);
      if (!image_pos) {
        continue;
      }

      // NOTE(yu, cong): Scale ratio here is to compensate for fx changes.
      // Originally, fx is around 2000. The scale of traffic light region should
      // be adjusted according to fx of new camera intrinsics.
      constexpr double kRefFx = 2000.;   // pixel
      constexpr double kRefDist = 90.0;  // m
      const double fx = camera_params.camera_matrix().fx();
      const double image_scale_factor = (kRefDist / kRefFx) * fx / dist;
      constexpr double kMinScale = 0.6;
      // Skip the image region with too low resolution.
      if (image_scale_factor < kMinScale) continue;

      int region_width = RoundToInt(tl_image_width_ * image_scale_factor);
      int region_height = RoundToInt(tl_image_height_ * image_scale_factor);
      if (!traffic_light.vertical()) {
        std::swap(region_width, region_height);
      }
      tl_patch_vec.push_back(
          {.tl_ids = {traffic_light.id()},
           .camera_id = camera_id,
           .image_trigger_timestamp = camera_image.trigger_timestamp(),
           .image_center_timestamp = camera_image.center_timestamp(),
           .is_vertical = traffic_light.vertical(),
           .x = image_pos->x(),
           .y = image_pos->y(),
           .width = region_width,
           .height = region_height,
           .original_width = region_width,
           .original_height = region_height,
           .tl_yaw_diff = tl_yaw_diff,
           .distance = dist});
    }
  }

  if (tl_patch_vec.empty()) return tl_patch_vec;
  // Sort from left to right in order to correctly assign traffic light
  // prediction results and improve the efficiency of computing intersection.
  std::sort(tl_patch_vec.begin(), tl_patch_vec.end(),
            [](const TlPatchInfo& a, const TlPatchInfo& b) {
              return a.camera_id < b.camera_id ||
                     (a.camera_id == b.camera_id && a.x < b.x) ||
                     (a.camera_id == b.camera_id && a.x == b.x && a.y < b.y);
            });

  // Resolve overlap between patches.
  // Note(zheng, wanzeng): If the two bboxs have overlap, we compute a scale and
  // then resize the bbox to make the bboxs have no overlap, at the same time,
  // we merge the overlapping boxes into one large box, and do detection
  // redundancy to resist more positioning deviations.
  std::vector<double> crop_scale(tl_patch_vec.size(), 1.0);
  std::vector<int> start_index_to_merge_vec(tl_patch_vec.size());
  iota(start_index_to_merge_vec.begin(), start_index_to_merge_vec.end(), 0);
  for (int i = 0; i < tl_patch_vec.size() - 1; ++i) {
    const int j = i + 1;
    const auto& source_patch = tl_patch_vec[i];
    const auto& target_patch = tl_patch_vec[j];
    if (source_patch.camera_id != target_patch.camera_id) continue;
    AABox2d source_bbox(Vec2d(source_patch.x, source_patch.y),
                        source_patch.width, source_patch.height);
    AABox2d target_bbox(Vec2d(target_patch.x, target_patch.y),
                        target_patch.width, target_patch.height);
    const double overlap_area = source_bbox.ComputeOverlapArea(target_bbox);
    if (overlap_area < DBL_EPSILON || source_bbox.area() < DBL_EPSILON ||
        target_bbox.area() < DBL_EPSILON) {
      continue;
    }
    // For for vertical traffic lights, we assume center_x are very close,
    // and for horizonal traffic lights, we assume center_y are very close,
    // so we can use iou to compute a scale. And A box may have overlap with
    // mutiple bbox, we select the minimum scale to resize the bbox.
    crop_scale[i] =
        std::min(crop_scale[i], 1.0 - overlap_area / source_bbox.area());
    crop_scale[j] =
        std::min(crop_scale[j], 1.0 - overlap_area / target_bbox.area());
    // To avoid the resized bbox is too small, we set a minimum scale.
    constexpr double kMinResizeScale = 0.6;
    crop_scale[i] = std::max(crop_scale[i], kMinResizeScale);
    crop_scale[j] = std::max(crop_scale[j], kMinResizeScale);
    // The merge logic is only used on vertical traffic lights at now.
    if (!source_patch.is_vertical || !target_patch.is_vertical) continue;
    start_index_to_merge_vec[j] = start_index_to_merge_vec[i];
  }

  if (FLAGS_merge_to_close_tl_roi) {
    int end_index = start_index_to_merge_vec.size() - 1;
    while (end_index >= 0) {
      int start_index = start_index_to_merge_vec[end_index];
      if (start_index != end_index) {
        TlPatchInfo merged_tl_patch = tl_patch_vec[start_index];
        for (int i = start_index + 1; i <= end_index; i++) {
          merged_tl_patch.tl_ids.insert(merged_tl_patch.tl_ids.end(),
                                        tl_patch_vec[i].tl_ids.begin(),
                                        tl_patch_vec[i].tl_ids.end());
        }
        merged_tl_patch.x =
            (tl_patch_vec[start_index].x + tl_patch_vec[end_index].x) / 2;
        merged_tl_patch.y =
            (tl_patch_vec[start_index].y + tl_patch_vec[end_index].y) / 2;
        merged_tl_patch.width +=
            (tl_patch_vec[end_index].x - tl_patch_vec[start_index].x);
        merged_tl_patch.height =
            RoundToInt(merged_tl_patch.width * TLNet::kImageHeight /
                       (1. * TLNet::kImageWidth));
        tl_patch_vec.push_back(merged_tl_patch);
        end_index = start_index;
      }
      end_index--;
    }
  }

  TlPatchInfos final_tl_patch_vec;
  final_tl_patch_vec.reserve(tl_patch_vec.size());
  for (int i = 0; i < tl_patch_vec.size(); ++i) {
    auto& patch = tl_patch_vec[i];
    if (!patch.is_merged_tl_patch()) {
      patch.width = RoundToInt(patch.original_width * crop_scale[i]);
      patch.height = RoundToInt(patch.original_height * crop_scale[i]);
    }

    const auto& camera_image = FindOrDie(tl_images, patch.camera_id);
    const auto& image = camera_image.ToMat();
    const cv::Rect rect(patch.x - patch.width / 2, patch.y - patch.height / 2,
                        patch.width, patch.height);
    const cv::Rect overlap_rect = rect & cv::Rect(0, 0, image.cols, image.rows);
    const double overlap_ratio =
        static_cast<float>(overlap_rect.width * overlap_rect.height) /
        static_cast<float>(rect.width * rect.height);
    // Skip if it's on the edge of the image.
    if (overlap_ratio < kMinVisibleRatioToEstimateTlState) {
      continue;
    }
    cv::Mat cropped_image = image(overlap_rect);
    // Resize and push the current frame to the traffic light image
    // sequence history buffer
    cv::Mat img;
    if (patch.is_vertical) {
      cv::resize(cropped_image, img,
                 cv::Size(tl_image_width_, tl_image_height_));
    } else {
      cv::resize(cropped_image, img,
                 cv::Size(tl_image_height_, tl_image_width_));
      // Rotate the image clockwise by 90-degree to turn a horizontal
      // layout to vertical.
      cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
    }

    // Save TL region images for training.
    if (FLAGS_extract_traffic_light_region_images) {
      SaveTLRegionImage(patch, img);
    }

    final_tl_patch_vec.push_back(patch);
    tl_image_seq_[patch.tl_cam_id()].push_back(patch.image_trigger_timestamp,
                                               std::move(img));
  }
  return final_tl_patch_vec;
}

TrafficLightStatesProto TrafficLightClassifier::ClassifyTrafficLights(
    const std::map<CameraId, CameraImage>& tl_images,
    const VisualNetDetectionsProto* visual_net_result,
    const std::map<CameraId, const SemanticSegmentationResultProto*>&
        panonet_results_map,
    const CoordinateConverter& coordinate_converter) {
  SCOPED_QTRACE("TrafficLightClassifier::ClassifyTrafficLights");

  // Fetch the latest run params.
  UpdateRunParams();

  TlPatchInfos tl_patch_info_vec =
      FindAndFetchTrafficLightPatches(tl_images, coordinate_converter);

  auto batch_results = ClassifyTrafficLightPatches(tl_patch_info_vec);
  QCHECK_GE(tl_patch_info_vec.size(), batch_results.size());
  // Eliminate traffic light patch used for redundant merged ROI at the end of
  // list.
  tl_patch_info_vec.resize(batch_results.size());
  // Check for occlusion.
  if (FLAGS_enable_tl_occlusion_visual_check && visual_net_result != nullptr) {
    CheckOcclusionVisually(tl_patch_info_vec, *visual_net_result,
                           &batch_results);
    QCHECK_EQ(tl_patch_info_vec.size(), batch_results.size());
  }
  if (FLAGS_enable_tl_occlusion_check) {
    CheckOcclusionVisually(tl_patch_info_vec, panonet_results_map,
                           &batch_results);
  }

  // Update history buffers given batch results.
  for (int i = 0; i < batch_results.size(); i++) {
    const auto& tl_result = batch_results[i];
    const auto& patch = tl_patch_info_vec[i];
    auto& tl_result_history = tl_result_history_[patch.tl_cam_id()];
    if (!tl_result_history.empty() &&
        tl_result_history.back_time() > patch.image_trigger_timestamp) {
      tl_result_history.clear();
    }
    tl_result_history.PushBackAndClearStale(patch.image_trigger_timestamp,
                                            tl_result, kTLResultBufferWindow);
  }

  TrafficLightClassifier::TlFusedResults batch_fused_results;
  if (FLAGS_use_hmm_to_fuse) {
    batch_fused_results = FuseTemporallyUsingHmm(tl_patch_info_vec);
  } else {
    batch_fused_results = FuseTemporally(tl_patch_info_vec);
  }
  QCHECK_EQ(tl_patch_info_vec.size(), batch_fused_results.size());

  return ConstructTLState(tl_patch_info_vec, batch_results,
                          batch_fused_results);
}

bool TrafficLightClassifier::CheckFlashing(
    const TlResultBuffer& tl_result_history, TrafficLightColor* color,
    float* score) {
  // Minimum period of time for a traffic state to be regarded as valid.
  // Life time is computed from the first to the last time the state shows up.
  constexpr double kMinValidStateLifeTime = 0.05;  // Unit of s.

  // Not flashing if it's empty or time span is less than minimum valid state
  // life time.
  if (tl_result_history.empty() ||
      tl_result_history.back_time() - tl_result_history.front_time() <=
          kMinValidStateLifeTime) {
    return false;
  }

  const auto convert_color_to_letter = [](TrafficLightColor color) -> char {
    switch (color) {
      case TL_UNKNOWN:
        return 'B';
      case TL_GREEN:
        return 'G';
      case TL_YELLOW:
        return 'Y';
      case TL_RED:
        return 'R';
      default:
        LOG(FATAL) << "Should not reach here.";
        return 'M';
    }
  };

  // Dedup the consecutive colors and smooth out wrong colors (color exists less
  // than kMinValidStateLifeTime).
  std::string dedupped_colors;
  dedupped_colors.reserve(tl_result_history.size());
  auto counting_color = tl_result_history.front_value().color;
  double counting_color_first_timestamp = tl_result_history.front_time();
  double counting_color_last_timestamp = tl_result_history.front_time();
  for (int i = 1; i < tl_result_history.size(); ++i) {
    const auto& color = tl_result_history.value(i).color;
    const double image_trigger_timestamp = tl_result_history.time(i);
    if (color != counting_color) {
      if (counting_color_last_timestamp - counting_color_first_timestamp >
          kMinValidStateLifeTime) {
        dedupped_colors.push_back(convert_color_to_letter(counting_color));
      }
      counting_color = color;
      counting_color_first_timestamp = image_trigger_timestamp;
    }
    counting_color_last_timestamp = image_trigger_timestamp;
  }
  if (counting_color_last_timestamp - counting_color_first_timestamp >
      kMinValidStateLifeTime) {
    dedupped_colors.push_back(convert_color_to_letter(counting_color));
  }

  // If the dedupped sequence contains flashing pattern and still in that
  // pattern, consider it as flashing.
  const std::string kGreenFlashingPattern1 = "GBG";
  const std::string kGreenFlashingPattern2 = "BGB";
  const bool ContainsGreenFlashing =
      Contains(dedupped_colors, kGreenFlashingPattern1) ||
      Contains(dedupped_colors, kGreenFlashingPattern2);
  const bool StillInGreenFlashingPattern =
      dedupped_colors.back() == 'G' || dedupped_colors.back() == 'B';
  if (ContainsGreenFlashing && StillInGreenFlashingPattern) {
    if (*color != TL_GREEN) {
      *color = TL_GREEN;

      // Recompute the color score.
      const double latest_tl_result_timestamp = tl_result_history.back_time();
      *score = 0;
      int num_colors = 0;
      for (const auto& [image_trigger_timestamp, tl_result] :
           tl_result_history) {
        // Count the vote if it's within voting window frame and not regarded as
        // occluded.
        if (tl_result.occlusion_ratio < kTLVisualAreaOcclusionThreshold &&
            (latest_tl_result_timestamp - image_trigger_timestamp) <=
                kTLVotingWindow &&
            tl_result.color == TL_GREEN) {
          ++num_colors;
          *score += tl_result.color_score;
        }
      }
      if (num_colors != 0) {
        *score /= num_colors;
      }
    }
    return true;
  }

  const std::string kYellowFlashingPattern1 = "YBY";
  const std::string kYellowFlashingPattern2 = "BYB";
  const bool ContainsYellowFlashing =
      Contains(dedupped_colors, kYellowFlashingPattern1) ||
      Contains(dedupped_colors, kYellowFlashingPattern2);
  const bool StillInYellowFlashingPattern =
      dedupped_colors.back() == 'Y' || dedupped_colors.back() == 'B';
  if (ContainsYellowFlashing && StillInYellowFlashingPattern) {
    if (*color != TL_YELLOW) {
      *color = TL_YELLOW;

      // Recompute the color score.
      const double latest_tl_result_timestamp = tl_result_history.back_time();
      *score = 0;
      int num_colors = 0;
      for (const auto& [image_trigger_timestamp, tl_result] :
           tl_result_history) {
        // Count the vote if it's within voting window frame and not regarded as
        // occluded.
        if (tl_result.occlusion_ratio < kTLVisualAreaOcclusionThreshold &&
            (latest_tl_result_timestamp - image_trigger_timestamp) <=
                kTLVotingWindow &&
            tl_result.color == TL_YELLOW) {
          ++num_colors;
          *score += tl_result.color_score;
        }
      }
      if (num_colors != 0) {
        *score /= num_colors;
      }
    }
    return true;
  }

  return false;
}

TrafficLightClassifier::TlFusedResults TrafficLightClassifier::FuseTemporally(
    const TlPatchInfos& tl_patch_info_vec) {
  TlFusedResults tl_fused_results;
  tl_fused_results.reserve(tl_patch_info_vec.size());

  for (int i = 0; i < tl_patch_info_vec.size(); ++i) {
    std::array<int, TrafficLightColor_ARRAYSIZE> votes = {0};
    const auto& patch = tl_patch_info_vec[i];
    const auto& id = patch.tl_cam_id();
    const auto& tl_result_history = tl_result_history_[id];
    const double latest_tl_result_timestamp = tl_result_history.back_time();

    std::map<TrafficLightColor, float> accumulated_scores;
    for (const auto& [image_trigger_timestamp, tl_result] : tl_result_history) {
      // Count the vote if it's within voting window frame and not regarded as
      // occluded.
      if (tl_result.occlusion_ratio < kTLVisualAreaOcclusionThreshold &&
          (latest_tl_result_timestamp - image_trigger_timestamp) <=
              kTLVotingWindow) {
        const auto& color = tl_result.color;
        votes[static_cast<int>(color)] += 1;
        accumulated_scores[color] += tl_result.color_score;
      }
    }

    TrafficLightColor voted_color = TL_UNKNOWN;
    float voted_score = 0.;
    const int total_votes =
        std::accumulate(std::begin(votes), std::end(votes), 0);
    if (total_votes > 0) {
      const int max_vote_index = std::distance(
          votes.begin(), std::max_element(votes.begin(), votes.end()));
      const int num_max_votes = votes[max_vote_index];  // Will not be zero;
      voted_color = static_cast<TrafficLightColor>(max_vote_index);
      voted_score = accumulated_scores[voted_color] / num_max_votes;
    }

    bool is_flashing =
        CheckFlashing(tl_result_history, &voted_color, &voted_score);

    tl_fused_results.push_back({.fused_color = voted_color,
                                .fused_color_score = voted_score,
                                .is_flashing = is_flashing});
  }

  return tl_fused_results;
}

TrafficLightClassifier::TlFusedResults
TrafficLightClassifier::FuseTemporallyUsingHmm(
    const TlPatchInfos& tl_patch_info_vec) {
  TlFusedResults tl_fused_results;
  tl_fused_results.reserve(tl_patch_info_vec.size());

  const auto convert_to_observation = [](TrafficLightColor color) {
    switch (color) {
      case TL_RED:
        return HMM::Observation::RED;
      case TL_YELLOW:
        return HMM::Observation::YELLOW;
      case TL_GREEN:
        return HMM::Observation::GREEN;
      case TL_UNKNOWN:
        return HMM::Observation::INACTIVE_OR_UNKNOWN;
    }
  };

  const auto convert_to_tl_color =
      [](HMM::State state) -> std::pair<TrafficLightColor, bool> {
    switch (state) {
      case HMM::State::RED:
        return {TL_RED, false};
      case HMM::State::YELLOW:
        return {TL_YELLOW, false};
      case HMM::State::GREEN:
        return {TL_GREEN, false};
      case HMM::State::INACTIVE_OR_UNKNOWN:
        return {TL_UNKNOWN, false};
      case HMM::State::RED_FLASHING:
        return {TL_RED, true};
      case HMM::State::YELLOW_FLASHING:
        return {TL_YELLOW, true};
      case HMM::State::GREEN_FLASHING:
        return {TL_GREEN, true};
      default:
        LOG(FATAL) << "Invalid input " << static_cast<int>(state);
        return {TL_UNKNOWN, false};
    }
  };

  for (int i = 0; i < tl_patch_info_vec.size(); ++i) {
    const auto& patch = tl_patch_info_vec[i];
    const auto& id = patch.tl_cam_id();
    const auto& tl_result_history = tl_result_history_[id];

    HMM::Observations observations;
    observations.reserve(tl_result_history.size());
    for (const auto& [_, tl_result] : tl_result_history) {
      // Skip if it's heavily occluded.
      if (tl_result.occlusion_ratio > kTLVisualAreaOcclusionThreshold) {
        continue;
      }
      observations.push_back(convert_to_observation(tl_result.color));
    }
    if (observations.size() == 0) {
      tl_fused_results.push_back({.fused_color = TrafficLightColor::TL_UNKNOWN,
                                  .fused_color_score = 0,
                                  .is_flashing = false});
      continue;
    }

    HMM::States states;
    std::vector<double> state_scores;
    std::tie(states, state_scores) = hmm_->Decode(observations);

    constexpr int kDelayFrames = 3;
    int selected_index =
        std::max(0, static_cast<int>(states.size()) - kDelayFrames);

    TrafficLightColor fused_color;
    bool is_flashing;
    std::tie(fused_color, is_flashing) =
        convert_to_tl_color(states[selected_index]);

    // BANDAID(wanzeng): To deal with the problem of fast green flashing traffic
    // lights in Shenzhen, flashing will be output after two non-consecutive
    // inactives.
    if (GetMap().find("nanshan") != std::string::npos ||
        GetMap().find("pingshan") != std::string::npos) {
      if (!is_flashing) {
        int falling_edge_num = 0;
        // Two falling edges last up to 14 frames, of which 5 frames are on and
        // two frame is off.
        constexpr int kMaxObservationFrameNumber = 14;
        for (int reverse_index = observations.size() - 2; reverse_index >= 0;
             reverse_index--) {
          if (observations.size() - reverse_index >
                  kMaxObservationFrameNumber ||
              observations[reverse_index + 1] == HMM::Observation::YELLOW ||
              observations[reverse_index + 1] == HMM::Observation::RED)
            break;
          if (observations[reverse_index] == HMM::Observation::GREEN &&
              observations[reverse_index + 1] ==
                  HMM::Observation::INACTIVE_OR_UNKNOWN) {
            falling_edge_num++;
            if (falling_edge_num >= 2) {
              QLOG(INFO) << "Flashing! two non-consecutive inactives.";
              is_flashing = true;
              break;
            }
          }
        }
        QLOG(INFO) << "observations: " << observations.size()
                   << " falling_edge_num:" << falling_edge_num;
      }
    }
    tl_fused_results.push_back(
        {.fused_color = fused_color,
         .fused_color_score = static_cast<float>(state_scores[selected_index]),
         .is_flashing = is_flashing});
  }

  return tl_fused_results;
}

TrafficLightStatesProto TrafficLightClassifier::ConstructTLState(
    const TlPatchInfos& tl_patch_info_vec, const TlResults& tl_results,
    const TlFusedResults& tl_fused_results) {
  TrafficLightStatesProto tl_state_proto;
  const int num_tl_states = tl_patch_info_vec.size();
  QCHECK_EQ(num_tl_states, tl_results.size());
  QCHECK_EQ(num_tl_states, tl_fused_results.size());

  const auto convert_to_mapping_shape =
      [](const TrafficLightShape shape) -> mapping::TrafficLightShape {
    switch (shape) {
      case TL_ROUND:
        return mapping::TrafficLightShape::TLS_ROUND;
      case TL_LEFT_ARROW:
        return mapping::TrafficLightShape::TLS_LEFT_ARROW;
      case TL_RIGHT_ARROW:
        return mapping::TrafficLightShape::TLS_RIGHT_ARROW;
      case TL_UP_ARROW:
        return mapping::TrafficLightShape::TLS_UP_ARROW;
      case TL_DOWN_ARROW:
      case TL_SHAPE_UNKNOWN:
      default:
        // Return as round when it's unknown.
        return mapping::TrafficLightShape::TLS_ROUND;
    }
  };

  for (int i = 0; i < num_tl_states; ++i) {
    const auto& patch_info = tl_patch_info_vec[i];
    const auto& tl_result = tl_results[i];
    const auto& tl_fused_result = tl_fused_results[i];

    auto* tl_state = tl_state_proto.add_states();
    tl_state->set_traffic_light_id(patch_info.tl_id());
    tl_state->set_camera_id(patch_info.camera_id);
    tl_state->set_camera_center_timestamp(patch_info.image_center_timestamp);
    tl_state->set_camera_trigger_timestamp(patch_info.image_trigger_timestamp);
    tl_state->set_tl_yaw_diff(patch_info.tl_yaw_diff);
    tl_state->set_distance(patch_info.distance);

    tl_state->set_color(tl_fused_result.fused_color);  // Temporal fused color.
    // Bandit(wanzeng): Remove until H490 is ready. Hack to resovred the common
    // frequency between the
    // camera and the traffic light. Map: suzhou_cloverleaf, Traffic light id:
    // 6597. Map: suzhou_jingjihu, Traffic light id: 3559.
    // Map: suzhou_jingjihu, Traffic light id: 6597.
    if ((6597 == patch_info.tl_id() &&
         GetMap().find("suzhou_cloverleaf") != std::string::npos) ||
        (3559 == patch_info.tl_id() &&
         GetMap().find("suzhou_jingjihu") != std::string::npos) ||
        (6597 == patch_info.tl_id() &&
         GetMap().find("suzhou_garage_new") != std::string::npos)) {
      // BANDAID without occlusion and with detection results.
      if (tl_fused_result.fused_color == TrafficLightColor::TL_UNKNOWN &&
          tl_result.occlusion_ratio < kTLVisualAreaOcclusionThreshold &&
          !tl_result.light_bboxes.empty()) {
        QLOG(WARNING) << "Hack green color for the common frequency bewteen "
                         "the camera and the traffic light."
                      << " Map:" << GetMap()
                      << " Traffic light id:" << patch_info.tl_id();
        tl_state->set_color(TrafficLightColor::TL_GREEN);
      }
    }
    tl_state->set_score(
        tl_fused_result.fused_color_score);  // Temporal fused color score.
    tl_state->set_shape(convert_to_mapping_shape(tl_result.shape));
    tl_state->set_flashing(tl_fused_result.is_flashing);
    tl_state->set_occluded(tl_result.occlusion_ratio >=
                           kTLVisualAreaOcclusionThreshold);
    tl_state->set_occlusion_ratio(tl_result.occlusion_ratio);
    tl_state->set_raw_color(tl_result.color);
    tl_state->set_countdown(tl_result.countdown);

    tl_state->set_x(patch_info.x);
    tl_state->set_y(patch_info.y);
    tl_state->set_width(patch_info.width);
    tl_state->set_height(patch_info.height);
    tl_state->set_original_width(patch_info.original_width);
    tl_state->set_original_height(patch_info.original_height);

    for (const auto& box : tl_result.light_body_bboxes) {
      auto* bbox = tl_state->add_light_body_bboxes();
      bbox->set_x(box.center_x());
      bbox->set_y(box.center_y());
      bbox->set_width(box.length());
      bbox->set_height(box.width());
    }
    for (const auto& box : tl_result.light_bboxes) {
      auto* bbox = tl_state->add_light_bboxes();
      bbox->set_x(box.center_x());
      bbox->set_y(box.center_y());
      bbox->set_width(box.length());
      bbox->set_height(box.width());
    }
  }
  return tl_state_proto;
}

void TrafficLightClassifier::CheckOcclusionVisually(
    const TlPatchInfos& tl_patch_info_vec,
    const VisualNetDetectionsProto& visual_net_result, TlResults* tl_results) {
  for (int i = 0; i < tl_patch_info_vec.size(); ++i) {
    const auto& patch = tl_patch_info_vec[i];
    auto& tl_result = (*tl_results)[i];
    const AABox2d tl_region_box(
        {static_cast<double>(patch.x), static_cast<double>(patch.y)},
        static_cast<double>(patch.width), static_cast<double>(patch.height));
    for (const auto& detection : visual_net_result.detections()) {
      if (detection.camera_id() != patch.camera_id) {
        continue;
      }
      for (const auto& box : detection.boxes()) {
        // Overlap ratio between overlap and tl_region area.
        double overlap_ratio = AABox2d({static_cast<double>(box.x()),
                                        static_cast<double>(box.y())},
                                       static_cast<double>(box.width()),
                                       static_cast<double>(box.height()))
                                   .ComputeOverlapArea(tl_region_box) /
                               tl_region_box.area();
        tl_result.occlusion_ratio = overlap_ratio;
      }
    }
  }
}

void TrafficLightClassifier::CheckOcclusionVisually(
    const TlPatchInfos& tl_patch_info_vec,
    const std::map<CameraId, const SemanticSegmentationResultProto*>&
        panonet_results_map,
    TlResults* tl_results) {
  SCOPED_QTRACE("TrafficLightClassifier::CheckOcclusionVisually");
  for (const auto& panonet_result : panonet_results_map) {
    const auto semseg_result = panonet_result.second;
    const auto camera_id = panonet_result.first;
    const SemanticSegmentationResult ssr(*semseg_result,
                                         FindOrDie(camera_params_, camera_id));
    const auto& mask_mat = ssr.semantic_mat();
    const double sem_seg_output_scale = ssr.sem_seg_output_scale();
    const auto& uncertainty_mat = ssr.uncertainty_mat();

    QCHECK_EQ(tl_patch_info_vec.size(), tl_results->size());
    for (int i = 0; i < tl_patch_info_vec.size(); ++i) {
      if (panonet_result.first != tl_patch_info_vec[i].camera_id) continue;
      const auto& boxes = tl_results->at(i).light_bboxes;
      // Note(wanzeng): A tl patch currently only corresponds to one traffic
      // light.
      QCHECK_GE(1, boxes.size());
      if (boxes.empty()) continue;
      const auto& box = boxes[0];
      const int left =
          std::max(0, static_cast<int>(box.min_x() / sem_seg_output_scale));
      const int right =
          std::min(mask_mat.cols - 1,
                   static_cast<int>(box.max_x() / sem_seg_output_scale));
      const int top =
          std::max(0, static_cast<int>(box.min_y() / sem_seg_output_scale));
      const int down =
          std::min(mask_mat.rows - 1,
                   static_cast<int>(box.max_y() / sem_seg_output_scale));
      int occlusion_count = 0;
      for (int x = left; x <= right; x++) {
        for (int y = top; y <= down; y++) {
          const int label_index = static_cast<int>(mask_mat.at<uchar>(y, x));
          const float uncertainty_score =
              static_cast<float>(uncertainty_mat.at<uchar>(y, x) / 256.f);
          if (kOcclusionTypeList + kOcclusionTypeNumber !=
                  std::find(kOcclusionTypeList,
                            kOcclusionTypeList + kOcclusionTypeNumber,
                            label_index) &&
              (uncertainty_score > kOcclusionUncertaintyThreshold)) {
            occlusion_count += 1;
          }
        }
      }
      const int tl_area = (right - left + 1) * (down - top + 1);
      (*tl_results)[i].occlusion_ratio =
          occlusion_count / (tl_area + std::numeric_limits<double>::epsilon());
    }
  }
}

}  // namespace qcraft
