#include "onboard/perception/tracker/track_classifier/laser_embedding_manager.h"

#include <algorithm>
#include <limits>
#include <map>
#include <random>
#include <string>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "leveldb/options.h"
#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/nets/trt/tcn_net_utils.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/tracker/track_classifier/tcn_constant.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

DECLARE_bool(save_tcn_training_data);
DECLARE_string(tcn_training_data_db_prefix);
DECLARE_string(tcn_key_prefix);
DECLARE_bool(enable_tcn_model);
DECLARE_bool(tcn_requires_image);

DEFINE_bool(save_image_patch, false,
            "Whether to write image patch to file or not.");

namespace qcraft::tracker {
LaserEmbeddingManager::LaserEmbeddingManager(
    const RunParamsProtoV2& run_params, const ParamManager& param_manager) {
  if (UNLIKELY(FLAGS_save_tcn_training_data)) {
    laser_data_storage_ = LaserDataStorage::Create(
        FLAGS_tcn_training_data_db_prefix, FLAGS_tcn_key_prefix);
  }
  if (FLAGS_enable_tcn_model) {
    QLOG(INFO) << "Init TcnImageNet ...";
    NetParam tcn_image_net_param;
    CHECK_OK(param_manager.GetProtoParam("tcn_image_net_param",
                                         &tcn_image_net_param));
    tcn_image_net_classifier_ = std::make_unique<TcnImageNetClassifier>(
        run_params, tcn_image_net_param);
    QLOG(INFO) << "Init TcnImageNet Done";
    QLOG(INFO) << "Init TcnPointNet ...";
    NetParam tcn_point_net_param;
    CHECK_OK(param_manager.GetProtoParam("tcn_point_net_param",
                                         &tcn_point_net_param));
    tcn_point_net_classifier_ = std::make_unique<TcnPointNetClassifier>(
        run_params, tcn_point_net_param);
    QLOG(INFO) << "Init TcnPointNet Done";
  }
}

std::vector<std::pair<float, int>> LaserEmbeddingManager::GetClusterRank(
    const SegmentedClusters& segmented_clusters, const VehiclePose& pose,
    const std::vector<Polygon2d>& contours) {
  std::vector<std::pair<float, int>> rank;
  const int cluster_num = segmented_clusters.size();
  rank.reserve(cluster_num);
  const auto av_pos = pose.coord2d();
  for (int i = 0; i < cluster_num; ++i) {
    const float distance = contours[i].DistanceSquareTo(av_pos);
    rank.emplace_back(distance, i);
  }
  std::sort(rank.begin(), rank.end());
  return rank;
}

void LaserEmbeddingManager::GetImagePatchForCluster(
    const std::vector<const LaserPoint*>& cluster_points,
    const CameraImages& camera_images, Polygon2d contour,
    const double cluster_ts, ImagePatch* image_patch) {
  constexpr double kMaxTimeDiff = 0.155;  // s
  constexpr float kValidImagePosRatioThres = 0.4f;
  if (cluster_points.size() < kMinPoint) return;
  float max_image_ratio = 0;
  CameraId selected_id;
  std::vector<CameraId> dim_cameras = GetDimCameraIds();

  for (const auto& [camera_id, images_with_transform] : camera_images) {
    if (images_with_transform.empty()) continue;
    if (std::find(dim_cameras.begin(), dim_cameras.end(), camera_id) !=
        dim_cameras.end()) {
      continue;
    }

    // Get image with closest timestamp
    double sensor_time_diff = std::numeric_limits<double>::max();
    auto matched = images_with_transform.rbegin();
    for (auto it = images_with_transform.rbegin();
         it != images_with_transform.rend(); ++it) {
      const auto& image = std::get<CameraImage>(*it);
      const double diff = std::abs(image.center_timestamp() - cluster_ts);
      if (diff < sensor_time_diff) {
        matched = it;
        sensor_time_diff = diff;
      }
      if (image.center_timestamp() - cluster_ts < 0) {
        break;
      }
    }
    const auto& image_with_transform = *matched;
    const auto& image = std::get<CameraImage>(image_with_transform);
    if (sensor_time_diff > kMaxTimeDiff) {
      QLOG(WARNING) << absl::StrFormat(
          "[LaserEmbeddingManager] Can't find corresponding camera image for "
          "cluster timestamp: %.3f, The one found is with center "
          "timestamp lidar time: %.3f and trigger timestamp: %.3f ",
          cluster_ts, image.center_timestamp(), image.trigger_timestamp());
      continue;
    }
    const auto& camera_param = image.params();
    int start_x = image.width();
    int end_x = -1;
    int start_y = image.height();
    int end_y = -1;
    int valid_image_pos_number = 0;

    // Get min max z
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (auto& point : cluster_points) {
      min_z = std::min(min_z, point->z);
      max_z = std::max(max_z, point->z);
    }
    const float mid_z = (min_z + max_z) * 0.5f;
    bool is_valid_camera = false;
    // Ignore the points behind
    const auto& smooth_to_camera_transform =
        std::get<AffineTransformation>(image_with_transform);
    const auto transformed_pt = smooth_to_camera_transform.TransformPoint(
        Vec3d(contour.points()[0], mid_z));
    if (transformed_pt.x() < 0) {
      continue;
    }
    // If no point in contour is in camera image, skip it.
    for (auto& point : contour.points()) {
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
    for (auto& point : cluster_points) {
      const auto image_pos = projection_util::SmoothPointToImagePos(
          point->coord(), smooth_to_camera_transform, camera_param);
      if (image_pos) {
        start_x = std::min(start_x, image_pos->x());
        start_y = std::min(start_y, image_pos->y());
        end_x = std::max(end_x, image_pos->x());
        end_y = std::max(end_y, image_pos->y());
        ++valid_image_pos_number;
      }
    }

    const float valid_image_pos_ratio =
        static_cast<float>(valid_image_pos_number) /
        static_cast<float>(cluster_points.size());
    if (valid_image_pos_ratio <
        std::max(kValidImagePosRatioThres, max_image_ratio)) {
      continue;
    }
    // Check if start_x/y, end_x/y are valid, skip if not.
    if (start_x < 0 || start_x >= image.width() || end_x < 0 ||
        end_x >= image.width() || start_y < 0 || start_y >= image.height() ||
        end_y < 0 || end_y >= image.height() || end_x <= start_x ||
        end_y <= start_y) {
      continue;
    }
    // Padding 10%
    const int kWidthPadding = (end_x - start_x) * 0.1;
    const int kHeightPadding = (end_y - start_y) * 0.1;
    start_x = std::max(1, start_x - kWidthPadding / 2);
    end_x = std::min(image.width() - 1, end_x + kWidthPadding / 2);
    start_y = std::max(1, start_y - kHeightPadding / 2);
    end_y = std::min(image.height() - 1, end_y + kHeightPadding / 2);
    if (end_x - start_x < kImageMinSize || end_y - start_y < kImageMinSize) {
      continue;
    }
    max_image_ratio = valid_image_pos_ratio;
    selected_id = camera_id;
    // Expand patch
    const int expand_size = std::max(end_x - start_x, end_y - start_y);
    const int expand_sx = std::max(0, start_x - expand_size);
    const int expand_ex = std::min(image.width() - 1, end_x + expand_size);
    const int expand_sy = std::max(0, start_y - expand_size);
    const int expand_ey = std::min(image.height() - 1, end_y + expand_size);

    const cv::Rect rect(cv::Point(expand_sx, expand_sy),
                        cv::Point(expand_ex, expand_ey));
    const cv::Mat camera_image = image.ToMat();
    image_patch->image = camera_image(rect);
    image_patch->roi.x = start_x - expand_sx;
    image_patch->roi.y = start_y - expand_sy;
    image_patch->roi.width = end_x - start_x;
    image_patch->roi.height = end_y - start_y;
    image_patch->time_diff = sensor_time_diff;
    image_patch->project_points.clear();  // Used by ACE association
    for (int i = 0; i < cluster_points.size(); ++i) {
      const auto image_pos = projection_util::SmoothPointToImagePos(
          cluster_points[i]->coord(), smooth_to_camera_transform, camera_param);
      if (image_pos) {
        image_patch->project_points.emplace(
            i, Vec2i(image_pos->x() - start_x, image_pos->y() - start_y));
      }
    }
  }
  if (FLAGS_save_image_patch && !image_patch->image.empty()) {
    const std::string save_patch_image_offboard_dir =
        absl::StrFormat("%s_images", FLAGS_tcn_training_data_db_prefix);
    if (!boost::filesystem::is_directory(save_patch_image_offboard_dir)) {
      boost::filesystem::create_directories(save_patch_image_offboard_dir);
    }
    const std::string image_name =
        absl::StrFormat("%s_%.3f.jpg", CameraId_Name(selected_id), cluster_ts);
    const std::string image_name_og =
        absl::StrFormat("%s/%s", save_patch_image_offboard_dir, image_name);
    cv::Mat image_to_save = image_patch->image.clone();

    cv::rectangle(image_to_save, image_patch->roi, cv::Scalar(0, 0, 255));
    // Draw project point
    for (const auto& x : image_patch->project_points) {
      const int px = x.second.x() + image_patch->roi.x;
      const int py = x.second.y() + image_patch->roi.y;
      cv::circle(image_to_save, cv::Point(px, py), 2, cv::Scalar(128, 50, 0));
    }
    cv::imwrite(image_name_og, image_to_save);
  }
}

std::vector<LaserEmbeddingManager::LaserFeatureEmbedding>
LaserEmbeddingManager::ExtractLaserFeatureEmbeddingsByTcn(
    const SegmentedClusters& segmented_clusters, const VehiclePose& pose,
    const std::vector<Polygon2d>& contours,
    const std::vector<std::pair<std::vector<const LaserPoint*>, ImagePatch>>&
        cluster_data_vec) {
  SCOPED_QTRACE("LaserEmbeddingManager::ExtractLaserFeatureEmbeddings_TCN");
  std::vector<LaserFeatureEmbedding> feature_embeddings;
  std::vector<int> image_object_index;
  std::vector<int> point_object_index;
  std::vector<cv::Mat> image_batch;
  std::vector<std::vector<const LaserPoint*>> point_batch;

  image_batch.reserve(kMaxBatchSize);
  point_batch.reserve(kMaxBatchSize);
  const int num_clusters = segmented_clusters.size();
  image_object_index.reserve(num_clusters);
  point_object_index.reserve(num_clusters);

  const std::vector<std::pair<float, int>> rank =
      GetClusterRank(segmented_clusters, pose, contours);

  feature_embeddings.resize(num_clusters);
  // The number of selected images and points should less than kMaxBatchSize
  for (int c = 0; c < num_clusters && (image_batch.size() < kMaxBatchSize ||
                                       point_batch.size() < kMaxBatchSize);
       ++c) {
    const int i = rank[c].second;
    const auto& image_patch = cluster_data_vec[i].second;
    const auto& image = image_patch.image(image_patch.roi);
    auto& laser_points = cluster_data_vec[i].first;
    if (!image.empty() && image_batch.size() < kMaxBatchSize) {
      image_object_index.push_back(i);
      image_batch.push_back(image);
    }
    if (FLAGS_tcn_requires_image) {
      if (!image.empty() && point_batch.size() < kMaxBatchSize) {
        point_object_index.push_back(i);
        point_batch.push_back(std::move(laser_points));
      }
    } else if (laser_points.size() > kMinPoint &&
               point_batch.size() < kMaxBatchSize) {
      // Filter cluster with few points
      point_object_index.push_back(i);
      point_batch.push_back(std::move(laser_points));
    }
  }
  if (FLAGS_tcn_requires_image) {
    QCHECK(image_batch.size() == point_batch.size());
  }
  RunImageModel(image_batch, image_object_index, &feature_embeddings);
  RunPointModel(point_batch, point_object_index, &feature_embeddings);
  return feature_embeddings;
}

std::vector<LaserEmbeddingManager::LaserFeatureEmbedding>
LaserEmbeddingManager::ExtractLaserFeatureEmbeddings(
    const SegmentedClusters& segmented_clusters,
    const CameraImages& camera_images, const VehiclePose& pose,
    const double lidar_host_time_diff) {
  const int num_clusters = segmented_clusters.size();
  SCOPED_QTRACE_ARG1("LaserEmbeddingManager::ExtractLaserFeatureEmbeddings",
                     "num_clusters", num_clusters);
  constexpr int kClusterMaxNumPoints = 500;
  cluster_data_vec_.clear();
  cluster_data_vec_.reserve(num_clusters);
  std::vector<Polygon2d> contours;
  contours.reserve(num_clusters);
  std::vector<const LaserPoint*> cluster_points;
  for (const auto& cluster : segmented_clusters) {
    cluster_points.clear();
    cluster_points.reserve(cluster.NumPoints());
    for (const auto* obstacle : cluster.obstacles()) {
      for (const auto& point : obstacle->points) {
        // Remove under ground z
        if (point.z < obstacle->ground_z) continue;
        cluster_points.push_back(&point);
      }
    }
    if (cluster_points.size() > kClusterMaxNumPoints) {
      const double step =
          cluster_points.size() / static_cast<double>(kClusterMaxNumPoints);
      double index_f = 0.0;
      for (int i = 0; i < kClusterMaxNumPoints; ++i) {
        cluster_points[i] = cluster_points[FloorToInt(index_f)];
        index_f += step;
      }
      cluster_points.resize(kClusterMaxNumPoints);
    }
    const auto real_ground_z = cluster.ComputeGroundZ();
    // Get camera image
    ImagePatch image_patch;
    // Compute Contour
    auto contour = cluster_util::ComputeContour(cluster);
    contours.emplace_back(std::move(contour));
    const double cluster_ts = cluster.timestamp();
    GetImagePatchForCluster(cluster_points, camera_images, contours.back(),
                            cluster_ts, &image_patch);
    if (UNLIKELY(FLAGS_save_tcn_training_data)) {
      laser_data_storage_->SaveData(cluster, cluster_points, image_patch, pose,
                                    real_ground_z);
    }
    cluster_data_vec_.emplace_back(std::move(cluster_points),
                                   std::move(image_patch));
  }

  std::vector<LaserFeatureEmbedding> feature_embeddings;
  if (FLAGS_enable_tcn_model) {
    feature_embeddings = ExtractLaserFeatureEmbeddingsByTcn(
        segmented_clusters, pose, contours, cluster_data_vec_);
  } else {
    feature_embeddings.reserve(num_clusters);
    constexpr int kFeatureLength = 512;
    for (const auto& [laser_points, _] : cluster_data_vec_) {
      LaserFeatureEmbedding feature;
      feature.point_feature.reserve(kFeatureLength);
      // feature.fill(0.);
      for (int i = 0; i < laser_points.size(); ++i) {
        if (i >= kFeatureLength || laser_points[i] == nullptr) {
          break;
        }
        feature.point_feature[i] = laser_points[i]->coord().x();
      }
      feature_embeddings.push_back(feature);
    }
  }

  return feature_embeddings;
}

void LaserEmbeddingManager::RunImageModel(
    const std::vector<cv::Mat>& batch_patch,
    const std::vector<int> object_index,
    std::vector<LaserFeatureEmbedding>* feature_embeddings) {
  if (batch_patch.size() == 0) {
    return;
  }
  const std::vector<float> feature =
      tcn_image_net_classifier_->ExtractImageFeature(batch_patch);
  int cur_pos = 0;
  const int feature_length = tcn_image_net_classifier_->GetFeatureLength();
  for (const int i : object_index) {
    (*feature_embeddings)[i].image_feature.resize(feature_length);
    memcpy((*feature_embeddings)[i].image_feature.data(),
           feature.data() + cur_pos, feature_length * sizeof(float));
    cur_pos += feature_length;
  }
}

void LaserEmbeddingManager::RunPointModel(
    const std::vector<std::vector<const LaserPoint*>>& input_laser_points,
    const std::vector<int> object_index,
    std::vector<LaserFeatureEmbedding>* feature_embeddings) {
  if (input_laser_points.size() == 0) {
    return;
  }
  std::vector<std::vector<const LaserPoint*>> sampled_points_batch =
      tcnutils::SampleInputPoints(tcn_point_net_classifier_->GetPointNumber(),
                                  input_laser_points);
  const std::vector<float> feature =
      tcn_point_net_classifier_->ExtractPointFeature(sampled_points_batch);
  int cur_pos = 0;
  const int feature_length = tcn_point_net_classifier_->GetFeatureLength();
  for (const int i : object_index) {
    (*feature_embeddings)[i].point_feature.resize(feature_length);
    memcpy((*feature_embeddings)[i].point_feature.data(),
           feature.data() + cur_pos, feature_length * sizeof(float));
    cur_pos += feature_length;
  }
}

}  // namespace qcraft::tracker
