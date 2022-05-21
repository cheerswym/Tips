#include "onboard/perception/retroreflector_detector.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"

DEFINE_bool(image_blob_cvs, false, "Enable range image blob cvs.");
DEFINE_bool(retroreflector_detector_cvs, false,
            "Whether to render retroreflector detector debug cvs.");

namespace qcraft {

namespace {

const absl::flat_hash_set<LidarId> kLidarsForDetectRetroreflector = {
    LDR_CENTER, LDR_FRONT, LDR_FRONT_LEFT, LDR_FRONT_RIGHT};

constexpr int kMaxOverhangingBeamIndex = 10;
constexpr int kBlobMinNumPixels = 10;
constexpr double kBlobMinCircularity = 0.05;
constexpr double kBlobMinInertiaRatio = 0.001;
constexpr double kBlobMinConvexity = 0.50;
constexpr int kRetroreflectorMinIntensity = 245;
constexpr float kMaxBlobPointRangeGap = 3.0f;  // m

constexpr double kMinOverHangingRetroreflectorHeight = 5.0;  // m

struct BlobParams {
  int min_num_pixels = 0;
  int max_num_pixels = std::numeric_limits<int>::max();
  double min_area = 0.0;
  double max_area = std::numeric_limits<double>::max();
  double min_circularity = 0.0;
  double max_circularity = 1.0;
  double min_inertia_ratio = 0.0;
  double max_inertia_ratio = 1.0;
  double min_convexity = 0.0;
  double max_convexity = 1.0;
};

void MaybeRenderRetroreflectors(const Retroreflectors& retroreflectors) {
  if (!FLAGS_retroreflector_detector_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/retroreflector_detector_cvs");

  for (const auto& retroreflector : retroreflectors) {
    vis::Color color = vis::Color::kBlue;
    if (retroreflector.lidar_id == LDR_FRONT_LEFT) {
      color = vis::Color::kGreen;
    } else if (retroreflector.lidar_id == LDR_FRONT_RIGHT) {
      color = vis::Color::kRed;
    }
    canvas.DrawPoints(retroreflector.points, color, 3);
  }
}

Blobs FindImageBlobs(const RangeImage& range_image,
                     const int intensity_threshold, const BlobParams& params) {
  const std::string prefix =
      "/hosthome/" + LidarId_Name(range_image.lidar_id()) + "_";
  if (FLAGS_image_blob_cvs) {
    cv::imwrite(prefix + "intensity_image.png", range_image.intensity_image());
  }
  QCHECK_EQ(range_image.intensity_image().type(), CV_8UC1);
  cv::Mat binarized_image;
  cv::threshold(range_image.intensity_image(), binarized_image,
                intensity_threshold, 255, cv::THRESH_BINARY);
  if (FLAGS_image_blob_cvs) {
    cv::imwrite(prefix + "binarized_image.png", binarized_image);
  }
  cv::Mat morphology_image;
  cv::Mat close_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binarized_image, morphology_image, cv::MORPH_CLOSE,
                   close_kernel);
  if (FLAGS_image_blob_cvs) {
    cv::imwrite(prefix + "closed_image.png", morphology_image);
  }
  cv::Mat open_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::Mat open_roi =
      cv::Mat(morphology_image,
              cv::Rect(0, kMaxOverhangingBeamIndex, morphology_image.cols,
                       morphology_image.rows - kMaxOverhangingBeamIndex));
  cv::morphologyEx(open_roi, open_roi, cv::MORPH_OPEN, open_kernel);
  if (FLAGS_image_blob_cvs) {
    cv::imwrite(prefix + "opened_image.png", morphology_image);
  }
  cv::Mat labels, stats, centroids;
  const int num_labels = cv::connectedComponentsWithStats(
      morphology_image, labels, stats, centroids, 8, CV_32S);
  Blobs blobs;
  blobs.reserve(num_labels * 2);
  cv::Mat debug_image = cv::Mat(morphology_image.rows, morphology_image.cols,
                                CV_8UC1, cv::Scalar(0));
  // Skip label 0 as it represents barkground.
  for (int i = 1; i < num_labels; ++i) {
    const cv::Rect rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                        stats.at<int>(i, cv::CC_STAT_TOP),
                        stats.at<int>(i, cv::CC_STAT_WIDTH),
                        stats.at<int>(i, cv::CC_STAT_HEIGHT));
    std::vector<cv::Point> pixels;
    pixels.reserve(rect.width * rect.height);
    std::vector<float> range_points;
    range_points.reserve(rect.width * rect.height);
    const cv::Mat& labels_rect = labels(rect);
    cv::Mat mask = cv::Mat(rect.height, rect.width, CV_8UC1);
    for (int j = 0; j < labels_rect.rows; ++j) {
      for (int k = 0; k < labels_rect.cols; ++k) {
        const cv::Point point_in_rect(k, j);
        if (labels_rect.at<int>(point_in_rect) == i) {
          const auto point_in_img = point_in_rect + rect.tl();
          if (binarized_image.at<uchar>(point_in_img)) {
            pixels.emplace_back(point_in_img);
            const auto range = range_image.RangeAt(point_in_img);
            QCHECK(range);
            range_points.push_back(*range);
          }
          mask.at<uchar>(j, k) = 255;
        } else {
          mask.at<uchar>(j, k) = 0;
        }
      }
    }
    if (pixels.size() < params.min_num_pixels ||
        pixels.size() > params.max_num_pixels) {
      continue;
    }

    std::vector<std::pair<int, float>> sorted_range_points;
    sorted_range_points.reserve(range_points.size());
    for (int i = 0; i < range_points.size(); ++i) {
      sorted_range_points.emplace_back(i, range_points[i]);
    }
    std::sort(sorted_range_points.begin(), sorted_range_points.end(),
              [](const auto& lhs, const auto& rhs) {
                return std::make_pair(lhs.second, lhs.first) <
                       std::make_pair(rhs.second, rhs.first);
              });
    std::vector<std::vector<int>> indices_cluster(1);
    for (int i = 1; i < sorted_range_points.size(); ++i) {
      indices_cluster.back().push_back(sorted_range_points[i - 1].first);
      if (sorted_range_points[i].second - sorted_range_points[i - 1].second >
          kMaxBlobPointRangeGap) {
        indices_cluster.push_back({});
      }
      if (i == sorted_range_points.size() - 1) {
        indices_cluster.back().push_back(sorted_range_points[i].first);
      }
    }
    for (const auto& indices : indices_cluster) {
      if (indices.size() < params.min_num_pixels) continue;
      Blob blob;
      for (const int indice : indices) {
        blob.pixels.emplace_back(pixels[indice]);
      }
      blob.bounding_box = rect;
      blob.is_overhanging = rect.tl().y <= kMaxOverhangingBeamIndex;
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
                       rect.tl());
      QCHECK_EQ(contours.size(), 1);
      const cv::Moments moms = cv::moments(contours[0]);
      blob.area = moms.m00 != 0.0 ? moms.m00 : blob.pixels.size();
      if (blob.area < params.min_area || blob.area > params.max_area) {
        continue;
      }
      const double perimeter = cv::arcLength(contours[0], true);
      if (perimeter < std::numeric_limits<double>::epsilon()) continue;
      blob.circularity = 4 * CV_PI * blob.area / Sqr(perimeter);
      if (!blob.is_overhanging && (blob.circularity < params.min_circularity ||
                                   blob.circularity > params.max_circularity)) {
        continue;
      }
      if (blob.is_overhanging && blob.circularity < 0.01) {
        continue;
      }
      const double denominator =
          std::sqrt(Sqr(2 * moms.mu11) + Sqr(moms.mu20 - moms.mu02));
      const double eps = 1e-2;
      if (denominator > eps) {
        const double cosmin = (moms.mu20 - moms.mu02) / denominator;
        const double sinmin = 2 * moms.mu11 / denominator;
        const double cosmax = -cosmin;
        const double sinmax = -sinmin;
        const double imin = 0.5 * (moms.mu20 + moms.mu02) -
                            0.5 * (moms.mu20 - moms.mu02) * cosmin -
                            moms.mu11 * sinmin;
        const double imax = 0.5 * (moms.mu20 + moms.mu02) -
                            0.5 * (moms.mu20 - moms.mu02) * cosmax -
                            moms.mu11 * sinmax;
        if (imax < std::numeric_limits<double>::epsilon()) continue;
        blob.inertia_ratio = imin / imax;
      } else {
        blob.inertia_ratio = 1.0;
      }
      if (!blob.is_overhanging &&
          (blob.inertia_ratio < params.min_inertia_ratio ||
           blob.inertia_ratio > params.max_inertia_ratio)) {
        continue;
      }
      std::vector<cv::Point> hull;
      cv::convexHull(contours[0], hull);
      const double hull_area = cv::contourArea(hull);
      blob.convexity = hull_area >= std::numeric_limits<double>::epsilon()
                           ? blob.area / hull_area
                           : 1.0;
      if (!blob.is_overhanging && (blob.convexity < params.min_convexity ||
                                   blob.convexity > params.max_convexity)) {
        continue;
      }
      if (blob.is_overhanging && blob.convexity < 0.05) {
        continue;
      }

      for (const auto& pixel : blob.pixels) {
        debug_image.at<uchar>(pixel) = 255;
      }

      blobs.push_back(std::move(blob));
    }
  }

  if (FLAGS_image_blob_cvs) {
    cv::imwrite(prefix + "blobs_image.png", debug_image);
  }

  return blobs;
}

std::vector<Vec3d> GetPointsFromBlob(const RangeImage& range_image,
                                     const Blob& blob) {
  std::vector<Vec3d> points;
  points.reserve(blob.pixels.size());
  for (const auto& pixel : blob.pixels) {
    const auto& point = range_image.PointAt(pixel);
    if (point) {
      points.push_back(*point);
    }
  }
  return points;
}

Retroreflector GenerateRetroreflector(const RangeImage& range_image,
                                      const Blob& blob) {
  Retroreflector retroreflector;
  retroreflector.points = GetPointsFromBlob(range_image, blob);
  retroreflector.lidar_id = range_image.lidar_id();
  retroreflector.is_overhanging = blob.is_overhanging;
  return retroreflector;
}

bool IsRetroreflectorOverHanging(
    const std::vector<Vec3d>& points, const LocalImagery& local_imagery,
    const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
    const AffineTransformation& pose_correction_transform_inv) {
  QCHECK(!points.empty());
  const Vec3d highest_point = *std::max_element(
      points.begin(), points.end(),
      [](const Vec3d& a, const Vec3d& b) { return a.z() < b.z(); });
  const Vec2d pos_global = coordinate_converter.SmoothToGlobal(
      {highest_point.x(), highest_point.y()});
  double ground_z = 0.0;
  if (const auto indexer = local_imagery.GetIndexer(
          pos_global.x(), pos_global.y(), coordinate_converter.GetLevel())) {
    const Vec3d pos_3d = coordinate_converter.GlobalToSmooth(
        {pos_global.x(), pos_global.y(), local_imagery.ElevationAt(*indexer)});
    ground_z = pose_correction_transform_inv.TransformPoint(pos_3d).z();
  } else {
    ground_z = pose.z;
  }

  return highest_point.z() - ground_z > kMinOverHangingRetroreflectorHeight;
}

}  // namespace

Retroreflectors RetroreflectorDetector::Detect(
    const absl::flat_hash_map<LidarId, RangeImage>& range_images,
    const LocalImagery& local_imagery,
    const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
    const VehiclePose& pose_correction_result) {
  SCOPED_QTRACE("RetroreflectorDetector::Detect");
  std::vector<const RangeImage*> range_image_ptrs;
  range_image_ptrs.reserve(range_images.size());
  for (const auto& [_, range_image] : range_images) {
    if (!ContainsKey(kLidarsForDetectRetroreflector, range_image.lidar_id())) {
      continue;
    }
    range_image_ptrs.emplace_back(&range_image);
  }
  std::vector<Retroreflectors> retroreflectors_per_range_image(
      range_image_ptrs.size());
  ParallelFor(0, range_image_ptrs.size(), thread_pool_, [&](int i) {
    const auto* range_image_ptr = range_image_ptrs[i];
    SCOPED_QTRACE_ARG1("generate retroreflectors", "lidar_id",
                       LidarId_Name(range_image_ptr->lidar_id()));

    const auto blobs =
        FindImageBlobs(*range_image_ptr, kRetroreflectorMinIntensity,
                       BlobParams{.min_num_pixels = kBlobMinNumPixels,
                                  .min_circularity = kBlobMinCircularity,
                                  .min_inertia_ratio = kBlobMinInertiaRatio,
                                  .min_convexity = kBlobMinConvexity});
    Retroreflectors retroreflectors;
    retroreflectors.reserve(blobs.size());
    for (const auto& blob : blobs) {
      auto retroreflector = GenerateRetroreflector(*range_image_ptr, blob);
      if (retroreflector.points.empty()) continue;
      retroreflectors.emplace_back(std::move(retroreflector));
    }
    retroreflectors_per_range_image[i] = std::move(retroreflectors);
  });

  const int num_retroreflectors =
      std::accumulate(retroreflectors_per_range_image.begin(),
                      retroreflectors_per_range_image.end(), 0,
                      [](int sum, const Retroreflectors& retroreflectors) {
                        return sum + retroreflectors.size();
                      });
  Retroreflectors all_retroreflectors;
  all_retroreflectors.reserve(num_retroreflectors);
  for (auto& retroreflectors : retroreflectors_per_range_image) {
    all_retroreflectors.insert(all_retroreflectors.end(),
                               std::make_move_iterator(retroreflectors.begin()),
                               std::make_move_iterator(retroreflectors.end()));
  }

  const auto pose_correction_transform_inv =
      pose_correction_result.ToTransform().Inverse();

  for (auto& retroreflector : all_retroreflectors) {
    retroreflector.is_overhanging =
        retroreflector.is_overhanging ||
        IsRetroreflectorOverHanging(retroreflector.points, local_imagery,
                                    coordinate_converter, pose,
                                    pose_correction_transform_inv);
  }

  MaybeRenderRetroreflectors(all_retroreflectors);

  return all_retroreflectors;
}

}  // namespace qcraft
