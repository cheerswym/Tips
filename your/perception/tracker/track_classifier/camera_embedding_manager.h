#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_CAMERA_EMBEDDING_MANAGER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_CAMERA_EMBEDDING_MANAGER_H_

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "leveldb/db.h"
#include "onboard/base/macros.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/global/trace.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace qcraft::tracker {

// This class is used for extracting embeddings from camera patches for track
// classifier, which is also going to be used for offboard data extraction.
class CameraEmbeddingManager {
 public:
  CameraEmbeddingManager();

  // Embedding size for camera embeddings.
  static constexpr int kCameraFeatureEmbeddingsSize = 64;
  void ExtractAndSetCameraFeatureEmbeddings(
      const std::unordered_map<CameraId, CameraImage>& images,
      VisualNetDetectionsProto* visual_net_detections);

 private:
  std::unique_ptr<leveldb::DB> data_db_;

  DISALLOW_COPY_AND_ASSIGN(CameraEmbeddingManager);
};

}  // namespace qcraft::tracker

// NOLINTNEXTLINE
#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_CAMERA_EMBEDDING_MANAGER_H_
