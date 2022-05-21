#ifndef ONBOARD_VIS_QSHOW_PUBLISHER_MODULE_H_
#define ONBOARD_VIS_QSHOW_PUBLISHER_MODULE_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "glog/logging.h"
#include "nlohmann/json.hpp"
#include "onboard/async/async_util.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/image_util.h"
// #include "onboard/hci/qview/server/qview_publish_client.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/nets/panonet.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/qview.pb.h"
#include "onboard/vis/qshow_process/fen_tracker.h"

namespace qcraft {

class QShowPublisherModule : public LiteModule {
 public:
  explicit QShowPublisherModule(LiteClientBase* lite_client);
  ~QShowPublisherModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override {}

 private:
  std::string GetImageToCompress(const cv::Mat& img);
  void UpdateRunParams();
  void SendData(const std::string& data);
  void AddMeasurements(const VehiclePose& pose,
                       const std::vector<std::pair<Box2d, int>>& fen_boxes,
                       MeasurementsProto* measurements);
  void Process();

  absl::Notification stop_notification_;
  Future<void> executor_future_;

  ThreadPool fen_thread_pool_;
  // unsigned int qshow_error_count_ = 0;
  std::atomic<uint32_t> latest_cluster_id_ = 0;

  /*
    NOTE: QViewPublishClient is deleted as the grpc python server does not
    exist any more. If still want to publish message to qview frontend, you
    need to start a websocket server for qview frontend. Please refer to
    websocket_server_v1_ & websocket_server_v2_ in qview_publisher_module
  */
  // std::unique_ptr<qview::QViewPublishClient> qshow_publish_client_;
  std::vector<LaserPoint> points_;

  std::unique_ptr<FieryEyeNetClassifier> fen_classifier_;
  std::unique_ptr<FenTracker> fen_tracker_;
  ThreadPool thread_pool_;
  ThreadPool track_thread_pool_;

  absl::Mutex lidar_frame_mutex_;
  absl::Mutex camera_frame_mutex_;
  std::map<LidarId, LidarFrame> lidar_frames_ GUARDED_BY(lidar_frame_mutex_);
  absl::CondVar lidar_frame_cond_var_ GUARDED_BY(lidar_frame_mutex_);
  std::map<CameraId, boost::circular_buffer<CameraImage>> camera_images_;

  CameraParamsMap camera_params_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;
  std::map<CameraId, Future<void>> future_pool_segmentation_;
  std::map<CameraId, ThreadPool> segmentation_loop_executor_thread_pool_;
  std::unique_ptr<panonet::PanoNet> pano_net_;
  std::map<CameraId, nlohmann::json> json_camera_buf;
  std::map<CameraId, cv::Mat> cvimages_;
  std::map<CameraId, std::string> image_lln_;
};

REGISTER_LITE_MODULE(QShowPublisherModule);

}  // namespace qcraft

#endif  // ONBOARD_VIS_QSHOW_PUBLISHER_MODULE_H_
