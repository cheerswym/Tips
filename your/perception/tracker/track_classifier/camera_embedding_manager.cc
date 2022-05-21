#include "onboard/perception/tracker/track_classifier/camera_embedding_manager.h"  // NOLINT

#include <string>

#include "leveldb/options.h"
#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"

DECLARE_bool(save_tcn_training_data);
DECLARE_string(tcn_training_data_db_prefix);
DECLARE_string(tcn_key_prefix);

namespace qcraft::tracker {

CameraEmbeddingManager::CameraEmbeddingManager() {
  if (ABSL_PREDICT_FALSE(FLAGS_save_tcn_training_data)) {
    const auto result =
        OpenLevelDB(FLAGS_tcn_training_data_db_prefix, "_camera_embedding_db");
    if (!result.ok()) {
      QLOG(FATAL) << "Failed to open the leveldb at "
                  << absl::StrCat(FLAGS_tcn_training_data_db_prefix,
                                  "_laser_embedding_db")
                  << ", err=" << result.status();
    }
    data_db_.reset(*result);
  }
}

void CameraEmbeddingManager::ExtractAndSetCameraFeatureEmbeddings(
    const std::unordered_map<CameraId, CameraImage>& images,
    VisualNetDetectionsProto* visual_net_detections) {
  SCOPED_QTRACE("CameraEmbeddingManager::ExtractCameraFeatureEmbeddings");
  std::vector<std::pair<VisualNetBoxProto*, cv::Mat>> box_patches;
  for (auto& detection : *visual_net_detections->mutable_detections()) {
    const auto& image = FindOrDie(images, detection.camera_id());
    const auto image_mat = image.ToMat();
    for (auto& box : *detection.mutable_boxes()) {
      cv::Rect roi(box.x(), box.y(), box.width(), box.height());
      cv::Mat image_patch = image_mat(roi);

      if (ABSL_PREDICT_FALSE(FLAGS_save_tcn_training_data)) {
        track_classifier_data::CameraMeasurementRawData camera_m_raw_data;

        const std::string db_key =
            absl::StrFormat("%s/%s/%.3f/%d_%d_%d_%d", FLAGS_tcn_key_prefix,
                            CameraId_Name(detection.camera_id()),
                            detection.camera_trigger_timestamp(), box.x(),
                            box.y(), box.width(), box.height());

        std::vector<uchar> encoded_image;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        // Compression level: 0-9, default is 1
        // A higher value means a smaller size and longer compression time
        compression_params.push_back(5);
        cv::imencode(".png", image_patch, encoded_image, compression_params);

        *camera_m_raw_data.mutable_image_patch() =
            std::string(encoded_image.begin(), encoded_image.end());
        camera_m_raw_data.set_camera_id(detection.camera_id());
        camera_m_raw_data.set_camera_trigger_timestamp(
            detection.camera_trigger_timestamp());
        camera_m_raw_data.set_box_center_x(box.x());
        camera_m_raw_data.set_box_center_y(box.y());
        camera_m_raw_data.set_box_width(box.width());
        camera_m_raw_data.set_box_height(box.height());

        const auto s = data_db_->Put(leveldb::WriteOptions(), db_key,
                                     camera_m_raw_data.SerializeAsString());
        if (!s.ok()) {
          QLOG(FATAL) << s.ToString();
        }
      }

      box_patches.emplace_back(&box, image_patch);
    }
  }

  // TODO(wanzeng, yu): Add camera feature embedding net in.
  // Process image patches.
  for (auto& [box, _] : box_patches) {
    std::vector<float> feature(0., kCameraFeatureEmbeddingsSize);
    *box->mutable_feature_embeddings() = {feature.begin(), feature.end()};
  }

  return;
}

}  // namespace qcraft::tracker
