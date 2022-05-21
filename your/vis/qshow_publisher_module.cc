#include "onboard/vis/qshow_publisher_module.h"

#include <regex>
#include <set>

#include "boost/algorithm/string/case_conv.hpp"
#include "onboard/crypto/base64.h"
#include "onboard/perception/projection_util.h"
#include "onboard/proto/port.pb.h"
#include "onboard/vis/http_request.h"
#include "opencv2/opencv.hpp"

DEFINE_bool(enable_qshow_grpc, false, "Enable qshow grpc instead of websocket");
DEFINE_string(qshow_request_ip, "0.0.0.0", "Qshow request ip.");
DEFINE_int32(qshow_lidar_roi_front, 80, "QShow exhibition lidar front range");
DEFINE_int32(qshow_lidar_roi_rear, 30, "QShow exhibition lidar rear range");
DEFINE_int32(qshow_lidar_roi_side, 40, "QShow exhibition lidar side range");
DEFINE_bool(use_fen_tracker, true, "Enable smoothing FEN detecting results");

namespace qcraft {

namespace {
constexpr int kRadius = 1;
// constexpr int kMaxQShowErrorCount = 100;
constexpr int kImageBufferSize = 3;
// constexpr int64 int16_max = (1 << 15) - 1;
// constexpr int64 int16_min = -(1 << 15);
const std::regex pattern("\n|\r|\t");
const std::regex pattern_blank(" ");

const std::map<int, int> ClassifiedPointHeight = {
    {0, 100}, {1, 3.5}, {2, 2.5}, {3, 2.5}};
const std::vector<cv::Vec3b> SimpleRainbow = {
    cv::Vec3b(255, 255, 0), cv::Vec3b(0, 255, 255), cv::Vec3b(255, 0, 255)};
const std::set<CameraId> CameraUseList = {CAM_L_FRONT_LEFT, CAM_L_FRONT,
                                          CAM_R_FRONT_RIGHT};

inline int64 SetRangeBit(int64 dst, int16 src, int todstmvbit, int srcbitsize) {
  for (int i = 0; i < srcbitsize; i++) {
    int64 cpbit = (1 << i) & src;
    dst |= (cpbit << todstmvbit);
  }
  return dst;
}

}  // namespace

QShowPublisherModule::QShowPublisherModule(LiteClientBase* lite_client)
    : LiteModule(lite_client),
      fen_thread_pool_(1),
      fen_tracker_(std::make_unique<FenTracker>(&track_thread_pool_)),
      thread_pool_(1),
      track_thread_pool_(1) {}

QShowPublisherModule::~QShowPublisherModule() {
  stop_notification_.Notify();
  executor_future_.Wait();
  lidar_frame_cond_var_.Signal();
}

void QShowPublisherModule::OnSubscribeChannels() {
  SubscribeLidarFrame([this](const LidarFrame& lidar_frame) {
    const SpinMetadata& spin_meta = lidar_frame.lidar_frame_metadata();
    // Only process full lidar frames for now.
    if (spin_meta.is_partial()) {
      return;
    }
    absl::MutexLock lock(&lidar_frame_mutex_);
    lidar_frames_[lidar_frame.lidar_id()] = lidar_frame;
    lidar_frame_cond_var_.Signal();
  });
  // Subscribe images.
  SubscribeDecodedImage([this](const CameraImage& camera_image) {
    const auto camera_id = camera_image.camera_id();
    if (!ContainsKey(CameraUseList, camera_id)) return;
    absl::MutexLock lock(&camera_frame_mutex_);
    camera_images_.at(camera_id).push_back(camera_image);
  });
}

void QShowPublisherModule::OnInit() {
  const RunParamsProtoV2 run_params = GetRunParams();
  for (const auto& camera_id : CameraUseList) {
    segmentation_loop_executor_thread_pool_.emplace(std::piecewise_construct,
                                                    std::make_tuple(camera_id),
                                                    std::make_tuple(1));
  }
  // Initialize camera image buffers.
  for (int i = CameraId_MIN; i <= CameraId_MAX; ++i) {
    const CameraId cam_id = static_cast<CameraId>(i);
    if (!ContainsKey(CameraUseList, cam_id)) continue;
    camera_images_.emplace(std::piecewise_construct, std::make_tuple(cam_id),
                           std::make_tuple(kImageBufferSize));
  }

  /*
    NOTE: QViewPublishClient is deleted as the grpc python server does not
    exist any more. If still want to publish message to qview frontend, you
    need to start a websocket server for qview frontend. Please refer to
    websocket_server_v1_ & websocket_server_v2_ in qview_publisher_module

  qshow_publish_client_ =
      std::make_unique<qview::QViewPublishClient>(absl::StrCat(
          FLAGS_qshow_request_ip, ":", qcraft::QVIEW_QSHOW_PUBLISHER_PORT));
  */

  // Initialize fiery eye net.
  NetParam fiery_eye_net_param;
  constexpr char kFieryEyeNetParamKey[] = "fiery_eye_net_param";
  CHECK_OK(param_manager().GetProtoParam(kFieryEyeNetParamKey,
                                         &fiery_eye_net_param));
  fen_classifier_ =
      std::make_unique<FieryEyeNetClassifier>(run_params, fiery_eye_net_param);
  // Initialize ll net.
  NetParam pano_net_param;
  const auto run_context = ModuleRunContext();
  // Handle in Dbqv4 or Pbqv1 mode
  if (!IsDBQConext(run_context) || IsDBQv4(run_context)) {
    constexpr char kPanoNetParamKey[] = "pbq_panonet_param";
    CHECK_OK(param_manager().GetProtoParam(kPanoNetParamKey, &pano_net_param));
  } else {
    constexpr char kPanoNetParamKey[] = "panonet_param";
    CHECK_OK(param_manager().GetProtoParam(kPanoNetParamKey, &pano_net_param));
  }
  pano_net_ = std::make_unique<panonet::PanoNet>(run_params, pano_net_param);

  executor_future_ = ScheduleFuture(
      &thread_pool_, std::bind(&QShowPublisherModule::Process, this));
}

void QShowPublisherModule::UpdateRunParams() {
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  camera_params_ = ComputeAllCameraParams(run_params.vehicle_params());
  const auto& lidar_params = run_params.vehicle_params().lidar_params();
  for (const auto& lidar_param : lidar_params) {
    lidar_params_[lidar_param.installation().lidar_id()] = lidar_param;
  }
}

std::string QShowPublisherModule::GetImageToCompress(const cv::Mat& img) {
  cv::Mat img_small;
  // compress image 1920*1024 to size 1/3.  640*341
  cv::resize(img, img_small, cv::Size(640, 341), 0, 0, cv::INTER_LINEAR);
  std::vector<uchar> vec_img;
  std::vector<int> vec_compression_params;
  vec_compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  vec_compression_params.push_back(90);
  std::string img_type = ".jpeg";
  cv::imencode(img_type, img_small, vec_img, vec_compression_params);
  std::string img_data(vec_img.begin(), vec_img.end());
  return img_data;
}

void QShowPublisherModule::SendData(const std::string& data) {
  {
    if (!FLAGS_enable_qshow_grpc) {
      try {
        http::Request request("http://" + FLAGS_qshow_request_ip +
                              ":8080/send_data/");
        const http::Response response =
            request.send("POST", "car_data=" + data,
                         {"Content-Type: application/x-www-form-urlencoded"});
      } catch (const std::exception& e) {
        QLOG(ERROR) << e.what();
      }
    }
    /*
    NOTE: QViewPublishClient is deleted as the grpc python server does not
    exist any more. If still want to publish message to qview frontend, you
    need to start a websocket server for qview frontend. Please refer to
    websocket_server_v1_ & websocket_server_v2_ in qview_publisher_module

     else {
      grpc::Status status = qshow_publish_client_->UpdateViewMessage(data);
      // when the server is down, after the continuous failure
      // reset the qview_publish_client to reconnect the server
      if (!status.ok()) {
        qshow_error_count_++;
        if (qshow_error_count_ >= kMaxQShowErrorCount) {
          QLOG(INFO) << "Qview publish client sent failed "
                     << qshow_error_count_
                     << " times, reset the publish client";
          qshow_publish_client_->Cancel();
          qshow_publish_client_ = std::make_unique<qview::QViewPublishClient>(
              absl::StrCat(FLAGS_qshow_request_ip, ":",
                           qcraft::QVIEW_QSHOW_PUBLISHER_PORT));
          qshow_error_count_ = 0;
        }
      } else {
        qshow_error_count_ = 0;
      }
    }
    */
  }
}

void QShowPublisherModule::AddMeasurements(
    const VehiclePose& pose,
    const std::vector<std::pair<Box2d, int>>& fen_boxes,
    MeasurementsProto* measurements) {
  QCHECK_NOTNULL(measurements);

  for (int i = 0; i < fen_boxes.size(); ++i) {
    const auto& fen_box = fen_boxes[i];
    auto& measurement = *measurements->add_measurements();
    auto& laser_measurement = *measurement.mutable_laser_measurement();
    measurement.set_timestamp(ToUnixDoubleSeconds(Clock::Now()));
    auto& bb = *laser_measurement.mutable_detection_bounding_box();
    bb.set_x(fen_box.first.center().x());
    bb.set_y(fen_box.first.center().y());
    bb.set_heading(fen_box.first.heading());
    bb.set_length(fen_box.first.length());
    bb.set_width(fen_box.first.width());

    switch (fen_box.second) {
      case 1:
        measurement.set_type(MT_VEHICLE);
        break;
      case 2:
        measurement.set_type(MT_PEDESTRIAN);
        break;
      case 3:
        measurement.set_type(MT_CYCLIST);
        break;
      default:
        QLOG(FATAL) << "Should not reach here";
    }
  }
  return;
}

void QShowPublisherModule::Process() {
  while (!stop_notification_.HasBeenNotified()) {
    // get lidar frames
    std::vector<LidarFrame> lidar_frames;
    VehiclePose pose;
    {
      absl::MutexLock lock(&lidar_frame_mutex_);
      // Start to process spins when all lidar frames of the same frame have
      // arrived.
      if (lidar_frames_.empty()) continue;
      const double first_frame_ts =
          lidar_frames_.begin()->second.StartTimestamp();
      for (const auto& [_, lidar_frame] : lidar_frames_) {
        if (std::abs(first_frame_ts - lidar_frame.StartTimestamp()) > 0.05)
          continue;
        lidar_frames.push_back(lidar_frame);
      }
      // Waiting for lidar frames to come if not ready.
      bool lidar_frames_ready = lidar_frames.size() == lidar_frames_.size();
      if (!lidar_frames_ready) {
        // Stop waiting for lidar frame, if notified as stopped.
        if (!stop_notification_.HasBeenNotified()) {
          lidar_frame_cond_var_.Wait(&lidar_frame_mutex_);
        }
        continue;
      }
      pose = lidar_frames_.begin()->second.MidPose();
    }
    // get camera frames
    std::map<CameraId, boost::circular_buffer<CameraImage>> camera_images_buf;
    {
      absl::MutexLock lock(&camera_frame_mutex_);
      camera_images_buf = camera_images_;
    }
    UpdateRunParams();
    std::vector<std::pair<CameraId, CameraImage>> camera_images;
    for (const auto& [camera_id, images] : camera_images_buf) {
      if (images.empty()) continue;
      CameraImage image = images.back();
      // Skip if the camera is unknown or Flir.
      if (camera_id == CAM_UNKNOWN || camera_id == CAM_FRONT_DIM) continue;
      const auto& camera_param = FindOrDie(camera_params_, camera_id);

      double lidar_timestamp =
          std::accumulate(lidar_frames.begin(), lidar_frames.end(), 0.0,
                          [](double sum, const LidarFrame& lidar_frame) {
                            return sum + lidar_frame.MidTimestamp();
                          }) /
          lidar_frames.size();
      if (camera_param.has_ref_lidar()) {
        for (const auto& lidar_frame : lidar_frames) {
          if (lidar_frame.lidar_id() == camera_param.ref_lidar()) {
            lidar_timestamp = lidar_frame.MidTimestamp();
          }
        }
      }
      const double image_center_timestamp_lidar_time = image.center_timestamp();
      // Check if the camera/lidar is from the same frame, skip if not.
      constexpr double kMaxTimeDiff = 0.075;  // s
      if (std::abs(lidar_timestamp - image_center_timestamp_lidar_time) >
          kMaxTimeDiff) {
        continue;
      }
      camera_images.emplace_back(std::make_pair(camera_id, image));
    }
    const auto pose_transform_inv = pose.ToTransform().Inverse();
    // Generate the JSON object.
    nlohmann::json json_data;
    // fen detect result
    const auto fen_future =
        ScheduleFuture(&fen_thread_pool_, [this, &lidar_frames, &pose]() {
          if (fen_classifier_ != nullptr) {
            return fen_classifier_->Classify(lidar_frames, pose);
          }
          return FieryEyeNetClassifier::DetectionResult();
        });

    for (const auto& [camera_id, image] : camera_images) {
      cvtColor(image.ToMat(), cvimages_[camera_id], cv::COLOR_BGR2RGB);
      nlohmann::json json_camera;
      json_camera["camera_id"] = camera_id;
      json_camera_buf[camera_id] = json_camera;
    }
    FieryEyeNetClassifier::DetectionResult fen_result = fen_future.Get();

    for (const auto& [camera_id, image] : camera_images) {
      // start semantic segmentation
      CameraId camera_id_seg = camera_id;
      CameraImage img = image;
      if (ContainsKey(CameraUseList, camera_id_seg) && pano_net_ != nullptr) {
        future_pool_segmentation_[camera_id_seg] = ScheduleFuture(
            &(segmentation_loop_executor_thread_pool_.at(camera_id_seg)),
            [this, img, camera_id_seg] {
              cv::Mat image_fp;
              cv::Mat img_mat = img.ToMat().clone();
              img_mat.convertTo(image_fp, CV_32FC3, 1 / 255.0);
              auto instance_segmentation_results =
                  pano_net_->ClassifyImagePixels({image_fp});
              QCHECK_EQ(instance_segmentation_results.size(), 3);
              // render color pixels
              const int height = instance_segmentation_results[0].rows;
              const int width = instance_segmentation_results[0].cols;
              for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                  const auto label_index = static_cast<int>(
                      instance_segmentation_results[0].at<uchar>(i, j));
                  const Vec3f color_value =
                      panonet_config::GetSegmentationTypeColor(label_index);
                  img_mat.at<cv::Vec3b>(i, j)[0] =
                      img_mat.at<cv::Vec3b>(i, j)[0] * 0.8 +
                      0.2 * 255 * color_value[0];
                  img_mat.at<cv::Vec3b>(i, j)[1] =
                      img_mat.at<cv::Vec3b>(i, j)[1] * 0.8 +
                      0.2 * 255 * color_value[1];
                  img_mat.at<cv::Vec3b>(i, j)[2] =
                      img_mat.at<cv::Vec3b>(i, j)[2] * 0.8 +
                      0.2 * 255 * color_value[2];
                }
              }
              const std::string& serialize = GetImageToCompress(img_mat);
              std::string ret = std::regex_replace(
                  ::qcraft::crypto::Base64Encode(serialize), pattern, "");
              std::string ret_blank =
                  std::regex_replace(ret, pattern_blank, "+");
              image_lln_[camera_id_seg] = ret_blank;
            });
      }
    }
    std::vector<std::pair<Box2d, int>> fen_boxes;
    for (int idx = 1; idx <= 3; idx++) {
      std::vector<FieryEyeNetClassifier::DetectionBox> boxes_current;
      QViewObjectProto::Type type;
      switch (idx) {
        case 1:
          boxes_current = fen_result.car_boxes;
          type = QViewObjectProto::VEHICLE;
          break;
        case 2:
          boxes_current = fen_result.ped_boxes;
          type = QViewObjectProto::PEDESTRIAN;
          break;
        case 3:
          boxes_current = fen_result.cyc_boxes;
          type = QViewObjectProto::CYCLIST;
          break;
        default:
          break;
      }
      for (const auto& box : boxes_current) {
        auto box_vehicle_center = pose_transform_inv.TransformPoint(
            Vec3d{box.box.center().x(), box.box.center().y(), 0});
        json_data["objects"].push_back(
            {type, box.box.length(), box.box.width(), box_vehicle_center.x(),
             box_vehicle_center.y(), box.box.heading() - pose.yaw});
        const Box2d box_vehicle(
            Vec2d{box_vehicle_center.x(), box_vehicle_center.y()},
            box.box.heading() - pose.yaw, box.box.length(), box.box.width());
        fen_boxes.emplace_back(std::make_pair(box_vehicle, idx));
      }
    }

    // add tracker
    std::vector<std::pair<Box2d, int>> tracker_result;
    if (FLAGS_use_fen_tracker) {
      MeasurementsProto measurements;
      AddMeasurements(pose, fen_boxes, &measurements);
      measurements.set_min_timestamp(ToUnixDoubleSeconds(Clock::Now()));
      measurements.set_max_timestamp(ToUnixDoubleSeconds(Clock::Now()));
      tracker_result = fen_tracker_->TrackObjects(
          ToUnixDoubleSeconds(Clock::Now()), std::move(measurements));
    } else {
      tracker_result = fen_boxes;
    }

    // get lidar points
    std::vector<int> accumulated_num_points_per_spin = {0};
    for (const auto& lidar_frame : lidar_frames) {
      accumulated_num_points_per_spin.push_back(
          lidar_frame.max_num_points() +
          accumulated_num_points_per_spin.back());
    }
    const int num_points = accumulated_num_points_per_spin.back();
    points_.clear();
    points_.resize(num_points);

    unsigned int lidar_frame_index = 0;
    for (const auto& lidar_frame : lidar_frames) {
      const int start_point_index =
          accumulated_num_points_per_spin[lidar_frame_index++];
      lidar_frame.Traverse([&](const LaserPoint& point) {
        // just for debug, so we use valid pose
        const auto point_vehicle =
            pose_transform_inv.TransformPoint(Vec3d{point.x, point.y, point.z});
        const int point_index =
            start_point_index + lidar_frame.GetPointIndex(point);
        points_[point_index].x = point_vehicle.x();
        points_[point_index].y = point_vehicle.y();
        points_[point_index].z = point_vehicle.z();
        // redefine intensity to point painting label
        points_[point_index].intensity = 0;
        bool point_roi_interior = false;
        // put point in box content below
        if (point_vehicle.x() < FLAGS_qshow_lidar_roi_front &&
            point_vehicle.x() > -FLAGS_qshow_lidar_roi_rear &&
            point_vehicle.y() < FLAGS_qshow_lidar_roi_side &&
            point_vehicle.y() > -FLAGS_qshow_lidar_roi_side &&
            point_vehicle.z() < 3) {
          point_roi_interior = true;
          for (const auto& box : tracker_result) {
            const int height = ClassifiedPointHeight.at(box.second);
            int label = box.second;
            if (box.first.IsPointIn(
                    Vec2d(point_vehicle.x(), point_vehicle.y()))) {
              if (point_vehicle.z() < height) {
                points_[point_index].intensity = static_cast<int>(label);
              }
            }
          }
        }
        // project points to image
        if (point_vehicle.x() > 0) {
          for (const auto& [camera_id, image] : camera_images) {
            const auto& camera_param = FindOrDie(camera_params_, camera_id);
            const auto smooth_to_camera_transform =
                (image.pose().ToTransform() *
                 camera_param.camera_to_vehicle_extrinsics().ToTransform())
                    .Inverse();
            if (std::abs(points_[point_index].x) +
                    std::abs(points_[point_index].y) +
                    std::abs(points_[point_index].z) <
                0.1)
              continue;
            if (points_[point_index].intensity == 0) continue;
            const auto image_pos = projection_util::SmoothPointToImagePos(
                {points_[point_index].x, points_[point_index].y,
                 points_[point_index].z},
                smooth_to_camera_transform, camera_param);
            if (image_pos) {
              for (int j = -kRadius; j <= kRadius; ++j) {
                for (int k = -kRadius; k <= kRadius; ++k) {
                  const int x = image_pos->x() + j;
                  const int y = image_pos->y() + k;
                  if (x > 0 && x < cvimages_[camera_id].cols && y > 0 &&
                      y < cvimages_[camera_id].rows) {
                    cvimages_[camera_id].at<cv::Vec3b>(y, x) =
                        SimpleRainbow[static_cast<int>(
                            points_[point_index].intensity)];
                  }
                }
              }
            }
          }
        }
        // filter lidar points
        bool point_idx_even =
            (point.scan_or_point_index % 2 == 0) && (point.beam_index % 2 == 0);
        bool point_idx_singular =
            (point.scan_or_point_index % 2 == 1) && (point.beam_index % 2 == 1);
        bool lidar_beam_cut = false;
        switch (lidar_frame.lidar_id()) {
          case LDR_REAR_BLIND:
            lidar_beam_cut = (point.beam_index % 3 != 0);
            break;
          case LDR_FRONT_LEFT_BLIND:
            lidar_beam_cut = (point.beam_index % 3 != 0);
            break;
          case LDR_FRONT_RIGHT_BLIND:
            lidar_beam_cut = (point.beam_index % 3 != 0);
            break;
          default:
            lidar_beam_cut = (point.beam_index < 10);
            break;
        }
        if ((point_idx_even || point_idx_singular) && !lidar_beam_cut &&
            point_roi_interior) {
          // encode points
          // TODO(zhenye): filter points to compress sending data
          // int64 point_encode = 0;
          int16 point_x = std::floor(point_vehicle.x() * 100 + 0.5);
          int16 point_y = std::floor(point_vehicle.y() * 100 + 0.5);
          int16 point_z = std::floor(point_vehicle.z() * 100 + 0.5);
          int16 label = points_[point_index].intensity;
          // point_encode = SetRangeBit(point_encode, point_x, 48, 16);
          // point_encode = SetRangeBit(point_encode, point_y, 32, 16);
          // point_encode = SetRangeBit(point_encode, point_z, 16, 16);
          // point_encode = SetRangeBit(point_encode, label, 0, 16);
          json_data["lidar"].push_back(point_x);
          json_data["lidar"].push_back(point_y);
          json_data["lidar"].push_back(point_z);
          json_data["lidar"].push_back(label);
        }
      });
    }
    for (const auto& [camera_id, _] : camera_images) {
      const std::string& serialize_pts =
          GetImageToCompress(cvimages_[camera_id]);
      std::string ret_pts = std::regex_replace(
          ::qcraft::crypto::Base64Encode(serialize_pts), pattern, "");
      std::string ret_blank_pts =
          std::regex_replace(ret_pts, pattern_blank, "+");
      future_pool_segmentation_[camera_id].Wait();
      json_camera_buf[camera_id]["image_segment"] = image_lln_[camera_id];
      json_camera_buf[camera_id]["image_points"] = ret_blank_pts;
      json_data["camera"].push_back(json_camera_buf[camera_id]);
    }
    // pub json data
    std::string data_to_send = json_data.dump();
    SendData(data_to_send);
  }
}

}  // namespace qcraft
