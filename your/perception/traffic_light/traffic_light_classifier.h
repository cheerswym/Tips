#ifndef ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_CLASSIFIER_H_
#define ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_CLASSIFIER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/camera/utils/camera_image.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/nets/tl_net_classifier.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/traffic_light/hmm/hmm.h"
#include "onboard/utils/history_buffer.h"
#include "opencv2/core.hpp"

namespace qcraft {

class TrafficLightClassifier {
 public:
  TrafficLightClassifier(const SemanticMapManager* semantic_map_manager,
                         const RunParamsProtoV2& run_params,
                         const NetParam& tl_net_param,
                         const ParamManager& param_manager);

  TrafficLightStatesProto ClassifyTrafficLights(
      const std::map<CameraId, CameraImage>& tl_images,
      const VisualNetDetectionsProto* visual_net_result,
      const std::map<CameraId, const SemanticSegmentationResultProto*>&
          panonet_results_map,
      const CoordinateConverter& coordinate_converter);

  // Note(wanzeng): The occlusion is currently mainly caused by vegetation and
  // large vehicles. The segmentation of the two categories of panonet is more
  // accurate. For the time being, only these two categories are used.
  static constexpr int kOcclusionTypeNumber = 2;
  static constexpr SegmentationType kOcclusionTypeList[kOcclusionTypeNumber]{
      SegmentationType::ST_VEGETATION,  // SegmentationType::ST_OBJECT,
      // SegmentationType::ST_BARRIER,
      SegmentationType::ST_CAR,
      // SegmentationType::ST_XCYCLIST,     SegmentationType::ST_HUMAN,
      // SegmentationType::ST_TRAFFIC_CONE, SegmentationType::ST_TRAFFIC_SIGN
  };

 private:
  using TlCamId = std::pair<int64, CameraId>;

  // Traffic light patch related information.
  struct TlPatchInfo {
    std::vector<int64> tl_ids;
    CameraId camera_id;
    double image_trigger_timestamp;
    double image_center_timestamp;
    bool is_vertical;
    // Tl region, which is a rectangle.
    int x, y;
    int width, height;
    // Keep the original tl region for debug purposes.
    int original_width, original_height;
    // Yaw diff in radian
    double tl_yaw_diff;
    double distance;

    bool is_merged_tl_patch() const { return tl_ids.size() > 1; }
    int64 tl_id() const {
      QCHECK(!tl_ids.empty());
      std::size_t hash_tl_id = static_cast<std::size_t>(tl_ids[0]);
      for (int i = 1; i < tl_ids.size(); i++) {
        boost::hash_combine(hash_tl_id, static_cast<std::size_t>(tl_ids[i]));
      }
      return static_cast<int64>(hash_tl_id);
    }
    TlCamId tl_cam_id() const { return {this->tl_id(), camera_id}; }
  };
  using TlPatchInfos = std::vector<TlPatchInfo>;

  struct TlResult {
    // Initial net outputs.
    TrafficLightColor color;
    float color_score;
    TrafficLightShape shape;
    float shape_score;
    bool is_flashing;
    float flashing_score;
    double occlusion_ratio = 0.0;
    std::vector<AABox2d> light_body_bboxes, light_bboxes;
    TrafficLightCountDown countdown;

    TlResult(TrafficLightColor color, float color_score,
             TrafficLightShape shape, float shape_score, bool is_flashing,
             float flashing_score,
             const std::vector<AABox2d>& light_body_bboxes,
             const std::vector<AABox2d>& light_bboxes,
             TrafficLightCountDown countdown)
        : color(color),
          color_score(color_score),
          shape(shape),
          shape_score(shape_score),
          is_flashing(is_flashing),
          flashing_score(flashing_score),
          occlusion_ratio(0.0),
          light_body_bboxes(light_body_bboxes),
          light_bboxes(light_bboxes),
          countdown(countdown) {}
  };
  using TlResults = std::vector<TlResult>;
  using TlResultBuffer = HistoryBuffer<TlResult>;

  struct TlFusedResult {
    TrafficLightColor fused_color;
    float fused_color_score;
    bool is_flashing;
  };
  using TlFusedResults = std::vector<TlFusedResult>;

  void UpdateRunParams();

  TlPatchInfos FindAndFetchTrafficLightPatches(
      const std::map<CameraId, CameraImage>& tl_images,
      const CoordinateConverter& coordinate_converter);

  TlResults ClassifyTrafficLightPatches(const TlPatchInfos& tl_patch_info_vec);

  TrafficLightStatesProto ConstructTLState(
      const TlPatchInfos& tl_patch_info_vec, const TlResults& tl_results,
      const TlFusedResults& tl_fused_results);

  TlFusedResults FuseTemporally(const TlPatchInfos& tl_patch_info_vec);

  TlFusedResults FuseTemporallyUsingHmm(const TlPatchInfos& tl_patch_info_vec);

  void CheckOcclusionVisually(const TlPatchInfos& tl_patch_info_vec,
                              const VisualNetDetectionsProto& visual_net_result,
                              TlResults* tl_results);

  void CheckOcclusionVisually(
      const TlPatchInfos& tl_patch_info_vec,
      const std::map<CameraId, const SemanticSegmentationResultProto*>&
          panonet_results_map,
      TlResults* tl_results);

  // Check if the light is current flashing, change color and score accordingly.
  bool CheckFlashing(const TlResultBuffer& tl_result_hsitory,
                     TrafficLightColor* color, float* score);

  void SaveTLRegionImage(const TlPatchInfo& patch, const cv::Mat& image);

  const SemanticMapManager* const semantic_map_manager_;

  CameraParamsMap camera_params_;

  std::map<CameraId, AffineTransformation> vehicle_to_camera_per_camera_;

  // int64 is used for tl id
  std::map<TlCamId, TlResultBuffer> tl_result_history_;
  using TLImgBuffer = HistoryBuffer<cv::Mat>;
  std::map<TlCamId, TLImgBuffer> tl_image_seq_;

  std::unique_ptr<TLNetClassifier> tl_net_classifier_;

  std::unique_ptr<HMM> hmm_;

  int tl_image_width_;
  int tl_image_height_;

  const ParamManager& param_manager_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_CLASSIFIER_H_
