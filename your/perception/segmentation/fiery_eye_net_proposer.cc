#include "onboard/perception/segmentation/fiery_eye_net_proposer.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(fiery_eye_net_proposer_cvs, false,
            "Render fiery eye net proposer results");
DEFINE_bool(vehicle_cluster_trimmer_cvs, false,
            "Render trimmed obstacles of vehicle clusters");
DEFINE_bool(divided_cluster_cvs, false, "Render divideclusterbyfen clusters");
DEFINE_bool(augmented_bounding_box_cvs, false,
            "Render augmented_bounding_box_cvs");
DEFINE_bool(bounding_box_coordinate_cvs, false,
            "Render bounding box coordinate");
DEFINE_bool(in_out_box_cvs, false, "");

namespace qcraft::segmentation {
namespace {
constexpr double kMinGapWidth = 0.3;  // m
// The minimum number of points in an obstacle that belongs to a vehicle
// cluster. Obstacles with less points in front of AV will be trimmed.
constexpr int kVehicleObstacleMinPoints = 3;
constexpr double kVehicleObstacleMinHeight = 1.2;      // m
constexpr double kLargeCarClusterBuffer = 2.0;         // m
constexpr double kCarClusterBuffer = 1.0;              // m
constexpr double kCycClusterBuffer = 1.25;             // m
constexpr double kPedClusterBuffer = 0.20;             // m
constexpr double kMinCenterHeadCosDiffToMerge = 0.98;  // acos, ~14 degree.
constexpr float kMinOverlapRatioForMerge = 0.6;
constexpr double kMaxMergeWidthDiff = 0.3;
constexpr double kMaxOutClusterLengthRatio = 0.1;
constexpr double kMaxMergeBoundingboxWithClusterLength = 30.0;  // m
constexpr double kMaxVehicleLength = 25;                        // m
constexpr double kMaxVehicleWidth = 5.5;                        // m
constexpr double kMaxObstacleOfMistFromSprintkerHeight = 0.39;  // m
constexpr double kMaxLargeVehicleBoundingBoxLength = 8.0;       // m

struct DetectionBoxWithType {
  DetectionBoxWithType(const Box2d& box, const Box2d& augment_box,
                       const Vec2d& velocity, MeasurementType type, float score,
                       bool is_merged = false)
      : box(box),
        augment_box(augment_box),
        velocity(velocity),
        type(type),
        score(score),
        is_merged(is_merged) {}

  Box2d box;
  Box2d augment_box;
  Vec2d velocity;
  MeasurementType type;
  float score;
  bool is_merged;
};

enum DivideResultType : int { INBOX = 0, OUTBOX = 1, DIVIDED = 2 };
using DivideResult = std::pair<DivideResultType, ProposedClusters>;
struct RowColHash {
  size_t operator()(std::pair<uint16_t, uint16_t> rc) const {
    return std::hash<uint32_t>()((rc.first << 16) + rc.second);
  }
};

void MaybeRenderMergedBoxes(
    const std::vector<DetectionBoxWithType>& sorted_detection_boxes,
    const std::vector<bool>& should_skip_box, const double ground_z) {
  if (FLAGS_fiery_eye_net_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    for (int i = 0; i < sorted_detection_boxes.size(); ++i) {
      const auto& box_with_type = sorted_detection_boxes[i];
      const vis::Color color =
          should_skip_box[i] ? vis::Color::kRed : vis::Color::kBlue;
      const Vec3d box_center(box_with_type.augment_box.center(), ground_z);
      canvas.DrawBox(box_center, box_with_type.augment_box.heading(),
                     {box_with_type.augment_box.length(),
                      box_with_type.augment_box.width()},
                     color);
      canvas.DrawText(absl::StrFormat("%.2f", box_with_type.score), box_center,
                      0.0, 0.5, vis::Color::kWhite);
    }
  }
}

void MaybeRenderProposedClusters(const ProposedClusters& clusters,
                                 const double ground_z) {
  if (FLAGS_fiery_eye_net_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    for (const auto& cluster : clusters) {
      const auto contour = cluster_util::ComputeContour(cluster);
      canvas.DrawPolygon(contour, ground_z, vis::Color::kDarkMagenta);
      if (cluster.bounding_box()) {
        const auto& box = *cluster.bounding_box();
        canvas.DrawBox({box.center(), ground_z}, box.heading(),
                       {box.length(), box.width()}, vis::Color::kViolet);
      }
    }
  }
}

void MayBeRenderDividedClusters(const Cluster& cluster_in,
                                const Cluster& cluster_out,
                                const double ground_z) {
  if (FLAGS_in_out_box_cvs) {
    const auto contour_in = cluster_util::ComputeContour(cluster_in);
    const auto contour_out = cluster_util::ComputeContour(cluster_out);
    constexpr double kRenderHeight = 6.5;
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    canvas.DrawPolygon(contour_in, ground_z + kRenderHeight,
                       vis::Color::kMiddleBlueGreen);
    canvas.DrawPolygon(contour_out, ground_z + kRenderHeight,
                       vis::Color::kMiddleBlueGreen);
  }
}

bool IsOverlappedWithAvBox(const Box2d& detection, const Box2d& av_box) {
  constexpr double kMinAllowedOverlappedRatio = 0.3;
  const Polygon2d detection_poly(detection);
  const Polygon2d av_box_poly(av_box);
  Polygon2d overlapped;
  if (!detection_poly.ComputeOverlap(av_box_poly, &overlapped)) {
    return false;
  }

  return (overlapped.area() > kMinAllowedOverlappedRatio * av_box_poly.area());
}

// Returns true if point1 / point2 are on the different side of line.
bool PointsOnDiffSideOfLine(const Vec2d& line_point1, const Vec2d& line_point2,
                            const Vec2d& point1, const Vec2d& point2) {
  return ((line_point1.y() - line_point2.y()) * (point1.x() - line_point1.x()) +
          (line_point2.x() - line_point1.x()) *
              (point1.y() - line_point1.y())) *
             ((line_point1.y() - line_point2.y()) *
                  (point2.x() - line_point1.x()) +
              (line_point2.x() - line_point1.x()) *
                  (point2.y() - line_point1.y())) <
         0;
}

// Check the clusters divided by fen should be merged based on the width and
// length of the second cluster
bool CheckMergeDividedClusters(const Box2d& detection_box,
                               const ProposedClusters& divided_clusters,
                               const double heading) {
  QCHECK_EQ(divided_clusters.size(), 2);

  // If the distance between divided_clusters[1] and the detection_box is larger
  // than 0.2m, divide the cluster(return false)
  if (divided_clusters[0].NumPoints() < 3 ||
      divided_clusters[1].NumPoints() < 3) {
    constexpr double kMaxDistOfObstacleToBox = 0.2;  // m
    for (const auto* obs_ptr : divided_clusters[1].obstacles()) {
      if (detection_box.DistanceTo(obs_ptr->coord()) >
          kMaxDistOfObstacleToBox) {
        return false;
      }
    }
    return true;
  }
  const Polygon2d polygon_in =
      cluster_util::ComputeContour(divided_clusters[0]);
  const Polygon2d polygon_out =
      cluster_util::ComputeContour(divided_clusters[1]);
  const auto box_in = polygon_in.BoundingBoxWithHeading(heading);
  const auto box_out = polygon_out.BoundingBoxWithHeading(heading);

  // Judge the obstacle in cluster whether in the front or back of detection box
  const auto whether_cluster_in_the_front_or_back_of_box =
      [](const Box2d& detection_box, const Cluster& cluster_out) -> bool {
    const Vec2d central = detection_box.center();
    const Vec2d heading = Vec2d::FastUnitFromAngle(detection_box.heading());
    const double half_box_width = detection_box.width() * 0.5;
    int obs_in_the_front_or_back_of_detectionbox_count = 0;
    constexpr double kWidthBuffer = 0.1;  // m
    for (const auto* ptr : cluster_out.obstacles()) {
      const Vec2d obs_central = ptr->coord();
      const Vec2d box_central_to_obs_central = obs_central - central;
      const double cosing = box_central_to_obs_central.dot(heading) /
                            (box_central_to_obs_central.norm());
      const double dist_sqr = Sqr(central.x() - obs_central.x()) +
                              Sqr(central.y() - obs_central.y());
      const double width_sqr = dist_sqr * (1 - cosing * cosing);
      if (width_sqr < Sqr(half_box_width + kWidthBuffer)) {
        obs_in_the_front_or_back_of_detectionbox_count++;
      }
    }

    return obs_in_the_front_or_back_of_detectionbox_count >
           cluster_out.NumObstacles() * 0.5;
  };

  if (std::fabs(box_in.width() - box_out.width()) < kMaxMergeWidthDiff &&
      box_out.length() < kMaxOutClusterLengthRatio * box_in.length()) {
    return whether_cluster_in_the_front_or_back_of_box(detection_box,
                                                       divided_clusters[1]);
  }

  // BANDAID(zhangtao): add a hack in PBQ which the vehicle is coming in the
  // front lidar fov view and the FEN detection box is not accuracy.
  const auto whether_cluster_near_left_or_right_of_box =
      [&](const Box2d& detection_box, const Cluster& cluster_out) -> bool {
    if (polygon_out.DistanceTo(detection_box) > 0.3 ||
        polygon_out.area() > polygon_in.area() ||
        divided_clusters[1].NumObstacles() >
            divided_clusters[0].NumObstacles()) {
      return false;
    }
    const Vec2d central = detection_box.center();

    const AffineTransformation transform =
        AffineTransformation::FromTranslation(central.x(), central.y(), 0.0)
            .ApplyYawPitchRoll(detection_box.heading(), 0.0, 0.0)
            .Inverse();

    int y_positive_count = 0;
    int y_negetive_count = 0;

    for (const auto* ptr : cluster_out.obstacles()) {
      const Vec2d obs_central = ptr->coord();
      const Vec2d box_central_to_obs_central = obs_central - central;
      const Vec3d transform_point =
          transform.TransformPoint({box_central_to_obs_central, 0});
      if (transform_point.y() > 0) {
        y_positive_count++;
      } else {
        y_negetive_count++;
      }
    }

    std::vector<Vec2d> points;
    points.reserve(divided_clusters[0].NumObstacles() +
                   divided_clusters[1].NumObstacles() + 4);

    if (y_positive_count * y_negetive_count == 0) {
      for (const auto* obstacle : cluster_out.obstacles()) {
        points.emplace_back(obstacle->coord());
      }

      const std::vector<Vec2d> box_corners =
          detection_box.GetCornersCounterClockwise();
      points.insert(points.end(), box_corners.begin(), box_corners.end());
      Polygon2d polygon;
      QCHECK(Polygon2d::ComputeConvexHull(points, &polygon));
      const Box2d box = polygon.BoundingBoxWithHeading(detection_box.heading());
      constexpr double kMaxMergeClusterWidth = 3.2;  // m
      if (box.width() > kMaxMergeClusterWidth) {
        return false;
      } else {
        return true;
      }
    }

    return false;
  };

  return whether_cluster_near_left_or_right_of_box(detection_box,
                                                   divided_clusters[1]);
}

std::vector<ObstaclePtrs> SplitClusterInFourDirections(
    const ProposedCluster& cluster, const Box2d& bounding_box) {
  const auto corners = bounding_box.GetCornersCounterClockwise();
  const auto center = bounding_box.center();
  const auto contour = cluster_util::ComputeContour(cluster);
  if (!contour.HasOverlap(bounding_box)) {
    // Note(zhangtao): If there is no overlap between contour and bounding box,
    // and the cluster's bounding box'width is less than 1.5 * bounding_box's
    // width, split is not required
    const auto cluster_box =
        contour.BoundingBoxWithHeading(bounding_box.heading());
    constexpr double kBoundingBoxWidthRatio = 1.5;
    if (bounding_box.width() * kBoundingBoxWidthRatio > cluster_box.width()) {
      return {};
    }
  }
  ObstaclePtrs obs_ptrs_a, obs_ptrs_b, obs_ptrs_c, obs_ptrs_d,
      obs_ptrs_remained;
  for (const auto* obstacle : cluster.obstacles()) {
    if (PointsOnDiffSideOfLine(corners[0], corners[1], center,
                               obstacle->coord())) {
      obs_ptrs_a.push_back(obstacle);
    } else if (PointsOnDiffSideOfLine(corners[3], corners[2], center,
                                      obstacle->coord())) {
      obs_ptrs_b.push_back(obstacle);
    } else if (PointsOnDiffSideOfLine(corners[0], corners[3], center,
                                      obstacle->coord())) {
      obs_ptrs_c.push_back(obstacle);
    } else if (PointsOnDiffSideOfLine(corners[2], corners[1], center,
                                      obstacle->coord())) {
      obs_ptrs_d.push_back(obstacle);
    } else {
      obs_ptrs_remained.push_back(obstacle);
    }
  }

  if (!obs_ptrs_remained.empty()) {
    QEVENT(
        "dongchen", "remain_obstacles_while_split_cluster_in_four_directions",
        [&](QEvent* qevent) {
          qevent->AddField("num_remained_obstacles", obs_ptrs_remained.size())
              .AddField("total_num_obstacles", cluster.NumObstacles());
        });
  }

  return {obs_ptrs_a, obs_ptrs_b, obs_ptrs_c, obs_ptrs_d, obs_ptrs_remained};
}

// Refine augmented box in longitude using obstacles
Box2d GetRefinedBox(const ProposedCluster& cluster, const Box2d& bounding_box) {
  ObstaclePtrs obs_ptrs;
  for (const auto* obstacle : cluster.obstacles()) {
    if (bounding_box.IsPointIn(obstacle->coord())) {
      obs_ptrs.push_back(obstacle);
    }
  }
  if (obs_ptrs.empty()) {
    return bounding_box;
  }
  Cluster obs_cluster(obs_ptrs);
  const Polygon2d inside_polygon = cluster_util::ComputeContour(obs_cluster);
  return inside_polygon.BoundingBoxWithHeading(bounding_box.heading());
}

Mat3d ComputeSmoothToBoundingBoxCoordinateTransformationMatrix(
    const Box2d& bounding_box) {
  const double heading = bounding_box.heading();
  const Vec2d heading_x = Vec2d::FastUnitFromAngle(heading);
  const Vec2d heading_y = Vec2d::FastUnitFromAngle(heading + M_PI_2);
  Mat2d rotation;
  rotation << heading_x.x(), heading_x.y(), heading_y.x(), heading_y.y();

  QCHECK_GT(rotation.determinant(), 0);
  const Vec2d translation = -1 * rotation * bounding_box.center();

  Mat3d transform;
  transform.block<2, 2>(0, 0) = rotation;
  transform.block<2, 1>(0, 2) = translation;
  transform.block<1, 3>(2, 0) = Vec3d({0, 0, 1});

  return transform;
}

// Note(zhangtao): Judge whether the observer is on the left and right sides
// of the cluster
bool IsAtSideOfDetection(const Box2d& bounding_box, const ObstaclePtr& obs) {
  const double half_width = bounding_box.half_width();
  const double half_length = bounding_box.half_length();
  const Mat3d transform =
      ComputeSmoothToBoundingBoxCoordinateTransformationMatrix(bounding_box);
  if (FLAGS_bounding_box_coordinate_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    constexpr double kTimesOfDir = 2.0;
    const Vec2d heading_x = {transform(0, 0), transform(0, 1)};
    const Vec2d heading_y = {transform(1, 0), transform(1, 1)};
    canvas.DrawLine(
        {bounding_box.center(), obs->ground_z},
        {bounding_box.center() + heading_x * kTimesOfDir * 2, obs->ground_z},
        vis::Color::kYellow);

    canvas.DrawLine(
        {bounding_box.center(), obs->ground_z},
        {bounding_box.center() + heading_y * kTimesOfDir, obs->ground_z},
        vis::Color::kYellow);
  }

  const Vec3d obs_in_bounding_box_coordinate =
      transform * Vec3d({obs->coord(), 1.0});

  if (std::abs(obs_in_bounding_box_coordinate.y()) > half_width &&
      std::abs(obs_in_bounding_box_coordinate.x()) < half_length) {
    return true;
  }

  return false;
}

Box2d ComputeMergedBoundingBox(const std::vector<Box2d>& detections,
                               const double heading) {
  QCHECK(!detections.empty());
  std::vector<Vec2d> corners;
  for (const auto& detection : detections) {
    const std::vector<Vec2d> current_corners =
        detection.GetCornersCounterClockwise();
    corners.insert(corners.end(), current_corners.begin(),
                   current_corners.end());
  }
  Polygon2d polygon;

  QCHECK(Polygon2d::ComputeConvexHull(corners, &polygon));
  return polygon.BoundingBoxWithHeading(heading);
}

DivideResult GenerateDivideClusters(const ProposedCluster& cluster,
                                    const ObstaclePtrs& obs_in,
                                    const ObstaclePtrs& obs_out,
                                    const double ground_z) {
  QCHECK(!obs_in.empty());
  QCHECK(!obs_out.empty());
  Cluster cluster_in(obs_in);
  Cluster cluster_out(obs_out);

  MayBeRenderDividedClusters(cluster_in, cluster_out, ground_z);
  ProposedClusters divided_clusters;
  divided_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(cluster_in)));
  divided_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(cluster_out)));

  return {DIVIDED, divided_clusters};
}

DivideResult DivideClusterWithConnectedComponentsInBuffer(
    const ProposedCluster& cluster, const Box2d& detection,
    const MeasurementType detection_type, const double ground_z,
    const double heading, const ObstaclePtrs& obs_ptrs_in_buffer,
    ObstaclePtrs* obs_ptrs_in, ObstaclePtrs* obs_ptrs_out) {
  if (obs_ptrs_in->empty()) {
    return {OUTBOX, ProposedClusters()};
  }
  if (obs_ptrs_in_buffer.empty() && obs_ptrs_out->empty()) {
    return {INBOX, ProposedClusters()};
  }
  const auto obs_in_bak = *obs_ptrs_in;
  auto obs_out_bak = *obs_ptrs_out;
  if (!obs_ptrs_in_buffer.empty()) {
    obstacle_util::LocalObstacleGrid local_obstacle_grid(obs_ptrs_in_buffer);
    std::vector<bool> visit_obs_ptrs_buffer(obs_ptrs_in_buffer.size(), false);

    std::deque<ObstaclePtr> obs_queue;
    for (const auto& obs_ptr : *obs_ptrs_in) {
      obs_queue.push_back(obs_ptr);
    }

    while (!obs_queue.empty()) {
      const ObstaclePtr obs = obs_queue.front();
      obs_queue.pop_front();
      const auto indices =
          local_obstacle_grid.FindNearestInRadius(obs->row, obs->col, 1);
      for (const auto& idx : indices) {
        if (obs_ptrs_in_buffer[idx]->type == ObstacleProto::IGNORED) {
          continue;
        }

        constexpr int kMaxNumPointsAtSideOfDetectionObstacle = 2;
        if (IsAtSideOfDetection(detection, obs_ptrs_in_buffer[idx])) {
          if (obs_ptrs_in_buffer[idx]->num_points_above_ground <
                  kMaxNumPointsAtSideOfDetectionObstacle &&
              obs->num_points_above_ground <
                  kMaxNumPointsAtSideOfDetectionObstacle) {
            continue;
          }

          // TODO(zhangtao): It will move to splash proposer in the future
          if (detection_type == MT_VEHICLE &&
              obs_ptrs_in_buffer[idx]->height() <
                  kMaxObstacleOfMistFromSprintkerHeight) {
            continue;
          }
        }

        if (!visit_obs_ptrs_buffer[idx]) {
          obs_queue.push_back(obs_ptrs_in_buffer[idx]);
          visit_obs_ptrs_buffer[idx] = true;
        }
      }
    }

    for (int i = 0; i < obs_ptrs_in_buffer.size(); i++) {
      if (visit_obs_ptrs_buffer[i]) {
        obs_ptrs_in->push_back(obs_ptrs_in_buffer[i]);
      } else {
        obs_ptrs_out->push_back(obs_ptrs_in_buffer[i]);
      }
    }
  }

  // Note(zhangtao): Compute the obs_ptrs_in's minarea box. finally,
  // determine whether the point in out_box is in in_box
  std::vector<Vec2d> points;
  points.reserve(obs_ptrs_in->size() * 4);
  for (const auto& obs_ptr : *obs_ptrs_in) {
    points.push_back(obs_ptr->coord());
    const std::vector<Vec2d> obstacle_vertices =
        obstacle_util::ComputeContour(*obs_ptr);
    points.insert(points.end(), obstacle_vertices.begin(),
                  obstacle_vertices.end());
  }

  Polygon2d polygon;
  QCHECK(Polygon2d::ComputeConvexHull(points, &polygon));

  const Box2d in_box = polygon.BoundingBoxWithHeading(detection.heading());
  // Note(zhangtao): If the length of the bounding box exceed 25m, just divide
  // as the detection box.
  if (in_box.length() > kMaxVehicleLength ||
      in_box.width() > kMaxVehicleWidth) {
    obs_out_bak.insert(obs_out_bak.end(), obs_ptrs_in_buffer.begin(),
                       obs_ptrs_in_buffer.end());
    return GenerateDivideClusters(cluster, obs_in_bak, obs_out_bak, ground_z);
  }

  ObstaclePtrs obs_ptr_out_last;

  for (const auto& obs_ptr : *obs_ptrs_out) {
    if (in_box.IsPointIn(obs_ptr->coord())) {
      obs_ptrs_in->push_back(obs_ptr);
    } else {
      obs_ptr_out_last.push_back(obs_ptr);
    }
  }

  if (obs_ptrs_in->empty()) {
    return {OUTBOX, ProposedClusters()};
  }

  if (obs_ptr_out_last.empty()) {
    return {INBOX, ProposedClusters()};
  }

  Cluster cluster_in(*obs_ptrs_in);
  Cluster cluster_out(obs_ptr_out_last);

  if (FLAGS_divided_cluster_cvs) {
    const auto contour_in = cluster_util::ComputeContour(cluster_in);
    const auto contour_out = cluster_util::ComputeContour(cluster_out);
    constexpr double kBoundingBoxHeight = 6.5;  // m
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    canvas.DrawPolygon(contour_in, ground_z + kBoundingBoxHeight,
                       vis::Color::kMiddleBlueGreen);

    canvas.DrawPolygon(contour_out, ground_z + kBoundingBoxHeight,
                       vis::Color::kMiddleBlueGreen);
  }

  const auto divided_result =
      GenerateDivideClusters(cluster, *obs_ptrs_in, obs_ptr_out_last, ground_z);
  if (CheckMergeDividedClusters(detection, divided_result.second, heading)) {
    return {INBOX, ProposedClusters()};
  }
  return divided_result;
}

DivideResult DivideClusterByFen(
    const ProposedCluster& cluster,
    const std::vector<DetectionBoxWithType>& detections,
    const double ground_z) {
  std::vector<Box2d> refined_boxes;
  std::vector<double> headings;
  for (const auto& detection : detections) {
    refined_boxes.push_back(GetRefinedBox(cluster, detection.augment_box));
    headings.push_back(detection.box.heading());
  }
  const double mean_heading = ComputeClusterBoundingBoxMeanHeading(headings);
  ObstaclePtrs obs_ptrs_in, obs_ptrs_out;
  ObstaclePtrs obs_ptrs_in_buffer;

  double cluster_buffer = 0;
  const auto merge_box = ComputeMergedBoundingBox(refined_boxes, mean_heading);
  // we will only call this method when all detections have same type
  if (detections[0].type == MT_VEHICLE) {
    if (merge_box.length() > kMaxLargeVehicleBoundingBoxLength) {
      cluster_buffer = kLargeCarClusterBuffer;
    } else {
      cluster_buffer = kCarClusterBuffer;
    }
  } else if (detections[0].type == MT_CYCLIST) {
    cluster_buffer = kCycClusterBuffer;
  } else if (detections[0].type == MT_PEDESTRIAN) {
    cluster_buffer = kPedClusterBuffer;
  }

  for (const auto* obstacle : cluster.obstacles()) {
    bool point_in_box = false;
    bool point_in_buffer = false;
    if (merge_box.IsPointIn(obstacle->coord())) {
      point_in_box = true;
    }
    if (point_in_box) {
      obs_ptrs_in.push_back(obstacle);
    } else {
      if (merge_box.DistanceTo(obstacle->coord()) < cluster_buffer) {
        point_in_buffer = true;
      }
      if (point_in_buffer && obstacle->type != ObstacleProto::IGNORED) {
        obs_ptrs_in_buffer.push_back(obstacle);
      } else {
        obs_ptrs_out.push_back(obstacle);
      }
    }
  }

  if (FLAGS_divided_cluster_cvs) {
    if (!obs_ptrs_in_buffer.empty()) {
      Cluster cluster_buffer(obs_ptrs_in_buffer);
      const auto contour_buffer = cluster_util::ComputeContour(cluster_buffer);
      constexpr double kBoundingBoxHeight = 6.5;  // m
      vis::Canvas& canvas = vantage_client_man::GetCanvas(
          "perception/fiery_eye_net_proposer_cvs");
      canvas.DrawPolygon(contour_buffer, ground_z + kBoundingBoxHeight,
                         vis::Color::kDarkRed);
    }
  }

  return DivideClusterWithConnectedComponentsInBuffer(
      cluster, merge_box, detections[0].type, ground_z, mean_heading,
      obs_ptrs_in_buffer, &obs_ptrs_in, &obs_ptrs_out);
}

DivideResult DivideClusterByFen(const ProposedCluster& cluster,
                                const DetectionBoxWithType& detection,
                                const double ground_z) {
  const auto& bounding_box = detection.augment_box;
  const auto refined_box = GetRefinedBox(cluster, bounding_box);

  if (FLAGS_augmented_bounding_box_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/fiery_eye_net_proposer_cvs");
    canvas.DrawBox({refined_box.center(), ground_z}, refined_box.heading(),
                   {refined_box.length(), refined_box.width()},
                   vis::Color::kYellow);
  }

  const double target_heading = detection.box.heading();
  double cluster_buffer = 0;
  if (detection.type == MT_VEHICLE) {
    if (detection.box.length() > kMaxLargeVehicleBoundingBoxLength) {
      cluster_buffer = kLargeCarClusterBuffer;
    } else {
      cluster_buffer = kCarClusterBuffer;
    }
  } else if (detection.type == MT_CYCLIST) {
    cluster_buffer = kCycClusterBuffer;
  } else if (detection.type == MT_PEDESTRIAN) {
    cluster_buffer = kPedClusterBuffer;
  }

  ObstaclePtrs obs_ptrs_in, obs_ptrs_in_buffer;
  ObstaclePtrs obs_ptrs_out;
  for (const auto* obstacle : cluster.obstacles()) {
    if (refined_box.IsPointIn(obstacle->coord())) {
      obs_ptrs_in.push_back(obstacle);
    } else if (refined_box.DistanceTo(obstacle->coord()) < cluster_buffer &&
               obstacle->type != ObstacleProto::IGNORED) {
      obs_ptrs_in_buffer.push_back(obstacle);
    } else {
      obs_ptrs_out.push_back(obstacle);
    }
  }

  if (FLAGS_divided_cluster_cvs) {
    if (!obs_ptrs_in_buffer.empty()) {
      Cluster cluster_buffer(obs_ptrs_in_buffer);
      const auto contour_buffer = cluster_util::ComputeContour(cluster_buffer);
      constexpr double kBoundingBoxHeight = 6.5;  // m
      vis::Canvas& canvas = vantage_client_man::GetCanvas(
          "perception/fiery_eye_net_proposer_cvs");
      canvas.DrawPolygon(contour_buffer, ground_z + kBoundingBoxHeight,
                         vis::Color::kDarkRed);
    }
  }

  return DivideClusterWithConnectedComponentsInBuffer(
      cluster, detection.box, detection.type, ground_z, target_heading,
      obs_ptrs_in_buffer, &obs_ptrs_in, &obs_ptrs_out);
}

float ComputeMergeBoxScore(const DetectionBoxWithType& detection_box,
                           const VehiclePose& pose) {
  const auto& box_center = detection_box.box.center();
  return Hypot(box_center.x() - pose.x, box_center.y() - pose.y) *
         detection_box.score;
}

bool CompareObstacle(const std::pair<double, ObstaclePtr>& lhs,
                     const std::pair<double, ObstaclePtr>& rhs) {
  return std::make_tuple(lhs.first, lhs.second->row, lhs.second->col) <
         std::make_tuple(rhs.first, rhs.second->row, rhs.second->col);
}

// Try to divide the cluster into two by separating the part overlapping with
// the bounding box. Assume there is a gap along the heading of the bounding
// box, and we will cut the cluster there.
// Return the divided clusters (the first one overlaps with the detection), or
// empty vector if the divide fails.
ProposedClusters DivideClusterAlongGap(const ProposedCluster& cluster,
                                       const Box2d& bounding_box) {
  // Rotate the centers of all obstacles by -heading in order to find the gap.
  const double heading = bounding_box.heading();
  const double cos_rotation = fast_math::Cos(-heading);
  const double sin_rotation = fast_math::Sin(-heading);
  const double bb_pos =
      bounding_box.center().Rotate(cos_rotation, sin_rotation).y();
  std::vector<std::pair<double, ObstaclePtr>> obstacle_pos_and_ptrs;
  for (const auto* obstacle : cluster.obstacles()) {
    obstacle_pos_and_ptrs.emplace_back(
        obstacle->coord().Rotate(cos_rotation, sin_rotation).y(), obstacle);
  }

  // Sort the positions to find the gap.
  std::sort(obstacle_pos_and_ptrs.begin(), obstacle_pos_and_ptrs.end(),
            CompareObstacle);
  std::vector<std::pair<double, int>> valid_gaps;
  for (int i = 0; i < obstacle_pos_and_ptrs.size() - 1; ++i) {
    const double pos_0 = obstacle_pos_and_ptrs[i].first;
    const double pos_1 = obstacle_pos_and_ptrs[i + 1].first;
    const double gap = pos_1 - pos_0;
    if (gap >= kMinGapWidth) {
      const double dist_to_bb_center = std::abs((pos_0 + pos_1) * 0.5 - bb_pos);
      if (dist_to_bb_center > bounding_box.width() * 0.5) {
        valid_gaps.emplace_back(dist_to_bb_center, i);
      }
    }
  }
  if (valid_gaps.empty()) {
    return {};
  }

  const int gap_index =
      std::min_element(valid_gaps.begin(), valid_gaps.end())->second;
  ObstaclePtrs cluster_a, cluster_b;
  for (int i = 0; i < obstacle_pos_and_ptrs.size(); ++i) {
    if (i <= gap_index) {
      cluster_a.push_back(obstacle_pos_and_ptrs[i].second);
    } else {
      cluster_b.push_back(obstacle_pos_and_ptrs[i].second);
    }
  }
  // Make sure the first cluster overlaps with the detection.
  if (bb_pos > obstacle_pos_and_ptrs[gap_index].first) {
    std::swap(cluster_a, cluster_b);
  }
  ProposedClusters divided_clusters;
  divided_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(cluster_a)));
  divided_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(
          std::move(cluster_b)));
  return divided_clusters;
}

// Trim vehicle cluster by removing noisy obstacles around vehicles in front
// of AV.
void TrimVehicleClusters(const VehiclePose& pose, ProposedClusters* clusters,
                         const float av_height) {
  QCHECK_NOTNULL(clusters);
  // For front nearby clusters, compute the contour to tightly bound all laser
  // points as a more precise contour comparing to the contour from obstacles.
  constexpr double kFrontNearbyZoneLength = 50.0;
  constexpr double kFrontNearbyZoneWidth = 10.0;
  Box2d front_nearby_zone({pose.x, pose.y}, pose.yaw, kFrontNearbyZoneLength,
                          kFrontNearbyZoneWidth);
  front_nearby_zone.Shift(Vec2d::FastUnitFromAngle(pose.yaw) *
                          kFrontNearbyZoneLength * 0.5);

  // For visualization.
  std::vector<ObstaclePtr> total_trimmed_obstacles;
  std::vector<ProposedCluster> trimmed_clusters;

  for (auto& cluster : *clusters) {
    if (cluster.type() != MT_VEHICLE) continue;

    // Check if the vehicle is in front of us.
    bool is_front_cluster = false;
    for (const auto* obstacle : cluster.obstacles()) {
      if (front_nearby_zone.IsPointIn(obstacle->coord())) {
        is_front_cluster = true;
        break;
      }
    }
    if (!is_front_cluster) continue;

    const auto& bounding_box = cluster.bounding_box();
    if (!bounding_box) continue;

    // Decide which side (left or right) to trim.
    const bool trim_left =
        (Vec2d(1.0, 0.0).FastRotate(pose.yaw))
            .CrossProd(Vec2d(bounding_box->center().x() - pose.x,
                             bounding_box->center().y() - pose.y)) < 0;

    // Collect all obstacles and sort them by the projection of each obstacle
    // center to the lateral direction of this vehicle.
    std::vector<std::pair<double, ObstaclePtr>> obstacles;
    const Vec2d cluster_center = bounding_box->center();
    for (const auto* obstacle : cluster.obstacles()) {
      const double projection =
          Vec2d(obstacle->x - pose.x, obstacle->y - pose.y)
              .FastRotate(M_PI_2 - pose.yaw)
              .x();
      obstacles.emplace_back(trim_left ? projection : -projection, obstacle);
    }
    std::sort(obstacles.begin(), obstacles.end(), CompareObstacle);
    std::set<ObstaclePtr> obstacles_to_trim;
    for (const auto& [_, obstacle] : obstacles) {
      if ((obstacle->num_points_above_ground < kVehicleObstacleMinPoints &&
           obstacle->height() < kVehicleObstacleMinHeight) ||
          obstacle->clearance > av_height) {
        obstacles_to_trim.insert(obstacle);
      } else {
        break;
      }
    }
    std::vector<ObstaclePtr> trimmed_obstacles;
    // We don't want to trim all obstacles of the cluster.
    if (!obstacles_to_trim.empty() &&
        obstacles_to_trim.size() != cluster.obstacles().size()) {
      for (const auto* obstacle_to_trim : obstacles_to_trim) {
        cluster.mutable_obstacles()->erase(
            std::find(cluster.obstacles().begin(), cluster.obstacles().end(),
                      obstacle_to_trim));
        trimmed_obstacles.push_back(obstacle_to_trim);
      }
      // integrate trimmed obstacles
      total_trimmed_obstacles.reserve(total_trimmed_obstacles.size() +
                                      trimmed_obstacles.size());
      std::copy(trimmed_obstacles.begin(), trimmed_obstacles.end(),
                std::back_inserter(total_trimmed_obstacles));
      // NOTE(dong): we generate trimmed clusters rather than directly trim
      // noise obstacles to guarantee the design rule: Total obstacles are not
      // changed after any proposer.
      trimmed_clusters.emplace_back(
          ProposedCluster::InheritFrom(cluster).ConstructBase(
              std::move(trimmed_obstacles)));
      trimmed_clusters.back().set_is_proposed(true);
      trimmed_clusters.back().set_property(PP_NOISE);
    }
  }

  clusters->insert(clusters->end(), trimmed_clusters.begin(),
                   trimmed_clusters.end());

  if (FLAGS_vehicle_cluster_trimmer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/cluster_trimmer");
    for (const auto& obstacle : total_trimmed_obstacles) {
      const std::vector<Vec3d> obstacle_contour{
          {obstacle->x - Obstacle::kDiameter * 0.5,
           obstacle->y - Obstacle::kDiameter * 0.5, pose.z},
          {obstacle->x + Obstacle::kDiameter * 0.5,
           obstacle->y - Obstacle::kDiameter * 0.5, pose.z},
          {obstacle->x + Obstacle::kDiameter * 0.5,
           obstacle->y + Obstacle::kDiameter * 0.5, pose.z},
          {obstacle->x - Obstacle::kDiameter * 0.5,
           obstacle->y + Obstacle::kDiameter * 0.5, pose.z},
      };
      canvas.DrawPolygon(obstacle_contour, vis::Color::kLightRed, 1);
    }
  }
}

bool CheckIfSkipCurrentBox(
    const std::vector<DetectionBoxWithType>& sorted_detection_boxes,
    const std::vector<bool>& should_skip_box,
    const ProposedCluster& current_cluster, const int current_index) {
  const auto& det_box = sorted_detection_boxes[current_index];
  for (const auto* obstacle : current_cluster.obstacles()) {
    if (det_box.box.IsPointIn(obstacle->coord())) {
      bool obstacle_found_in_other_box = false;
      for (int i = 0; i < sorted_detection_boxes.size(); i++) {
        if (i == current_index || should_skip_box[i]) {
          continue;
        }
        if (sorted_detection_boxes[i].box.IsPointIn(obstacle->coord())) {
          obstacle_found_in_other_box = true;
          break;
        }
      }
      if (!obstacle_found_in_other_box) {
        return false;
      }
    }
  }
  return true;
}

bool CheckMergeBoxes(const ProposedCluster& cluster,
                     const std::vector<DetectionBoxWithType>& detection_boxes) {
  int overlap_count = 0;
  const Polygon2d contour = cluster_util::ComputeContour(cluster);

  // Note(zhangtao): If the cluster's minarea boundingbox's length is more
  // than 30m, not check.
  std::vector<double> headings;
  headings.reserve(detection_boxes.size());
  for (int i = 0; i < detection_boxes.size(); i++) {
    headings.push_back(detection_boxes[i].box.heading());
  }
  const double mean_heading = ComputeClusterBoundingBoxMeanHeading(headings);
  if (contour.BoundingBoxWithHeading(mean_heading).length() >
      kMaxMergeBoundingboxWithClusterLength) {
    return false;
  }

  // The detection boxes will not be merged if the overlap between the
  // detection and the cluster smaller than kMinOverlapRatioForMerge *
  // detection box's area.
  for (int i = 0; i < detection_boxes.size(); i++) {
    Polygon2d overlap_polygon;
    std::vector<Vec2d> obs_pts;
    for (const auto* obstacle : cluster.obstacles()) {
      if (detection_boxes[i].box.IsPointIn(obstacle->coord())) {
        const auto contour = obstacle_util::ComputeContour(*obstacle);
        obs_pts.insert(obs_pts.end(), contour.begin(), contour.end());
      }
    }
    Polygon2d other_polygon;
    if (!Polygon2d::ComputeConvexHull(obs_pts, &other_polygon)) {
      return false;
    }
    contour.ComputeOverlap(other_polygon, &overlap_polygon);
    auto overlap_ratio = overlap_polygon.area() / other_polygon.area();
    if (overlap_ratio < kMinOverlapRatioForMerge) {
      return false;
    }
    // Compute Overlap
    for (int j = i + 1; j < detection_boxes.size(); j++) {
      const auto& base_box = detection_boxes[i].box;
      const auto& ref_box = detection_boxes[j].box;
      if (base_box.HasOverlap(ref_box)) {
        overlap_count++;
      }
      const Vec2d center_vec(base_box.center_x() - ref_box.center_x(),
                             base_box.center_y() - ref_box.center_y());
      for (int m = 0; m < detection_boxes.size(); m++) {
        const auto& det_box = detection_boxes[m];
        if (det_box.type != MT_VEHICLE) {
          return false;
        }
        const Vec2d heading_vec =
            Vec2d::FastUnitFromAngle(det_box.box.heading());
        const double center_vec_norm = center_vec.norm();
        const double cos_value_abs =
            std::fabs(heading_vec.dot(center_vec) / center_vec_norm);
        if (cos_value_abs < kMinCenterHeadCosDiffToMerge) {
          return false;
        }
      }
    }
  }
  return overlap_count >= detection_boxes.size() - 1;
}

using ClusterToDetections = std::vector<
    std::pair<const ProposedCluster*, std::vector<DetectionBoxWithType>>>;

ClusterToDetections CreateClusterToDetectionsMap(
    const ProposedClusters& clusters, const Context& context,
    const Box2d& av_box, ThreadPool* thread_pool) {
  SCOPED_QTRACE("CreateClusterToDetectionsMap");

  // Make sure every cluster has an entry in cluster_to_detections.
  ClusterToDetections cluster_to_detections(clusters.size());

  ParallelFor(0, clusters.size(), thread_pool, [&](int i) {
    const auto& cluster = clusters[i];
    cluster_to_detections[i].first = &cluster;
    auto& detection_boxes = cluster_to_detections[i].second;
    detection_boxes.reserve(4);
    const Polygon2d contour = cluster_util::ComputeContour(cluster);
    const auto& fen_result = context.fiery_eye_net_result;
    for (const auto& det_box : fen_result.car_boxes) {
      if (IsOverlappedWithAvBox(det_box.box, av_box)) {
        continue;
      }
      // BANDAID(yu): Enlarge the box length for large vehicle to avoid
      // over-segmetnation due to small large vehicle detection box.
      const auto& detection = det_box.box;
      const auto score = det_box.score;
      const auto& velocity = det_box.velocity;
      auto augmented_detection = detection;
      constexpr double kMinLenghToConsiderAsALargeVehicle = 10;  // m
      if (detection.length() >= kMinLenghToConsiderAsALargeVehicle) {
        const double kLongitudinalExtendValue = 1;  // m
        augmented_detection.LongitudinalExtend(kLongitudinalExtendValue);
      } else {
        const double kLongitudinalExtendValue = 0.5;  // m
        augmented_detection.LongitudinalExtend(kLongitudinalExtendValue);
      }
      for (const auto* obstacle : cluster.obstacles()) {
        if (augmented_detection.IsPointIn(obstacle->coord())) {
          detection_boxes.emplace_back(detection, augmented_detection, velocity,
                                       MT_VEHICLE, score);
          break;
        }
      }
    }
    for (const auto& [boxes, type] :
         {std::make_pair(&fen_result.ped_boxes, MT_PEDESTRIAN),
          std::make_pair(&fen_result.cyc_boxes, MT_CYCLIST)}) {
      for (const auto& det_box : *boxes) {
        if (IsOverlappedWithAvBox(det_box.box, av_box)) {
          continue;
        }
        const auto& detection = det_box.box;
        const auto score = det_box.score;
        const auto& velocity = det_box.velocity;
        for (const auto* obstacle : cluster.obstacles()) {
          if (detection.IsPointIn(obstacle->coord())) {
            detection_boxes.emplace_back(detection,
                                         /*augmented_detection=*/detection,
                                         velocity, type, score);
            break;
          }
        }
      }
    }
    // For ped/cyc detections deleted by pcn, we will also do segmentation for
    // them. We will set type of these boxes to unknown and score to -1.
    for (const auto* deleted_boxes :
         {&context.ped_filtering_result.deleted_ped_boxes,
          &context.ped_filtering_result.deleted_cyc_boxes}) {
      for (const auto& [detection_with_score, _] : *deleted_boxes) {
        const auto& detection = detection_with_score.box;
        const auto& velocity = detection_with_score.velocity;
        for (const auto* obstacle : cluster.obstacles()) {
          if (detection.IsPointIn(obstacle->coord())) {
            detection_boxes.emplace_back(detection,
                                         /*augmented_detection=*/detection,
                                         velocity, MT_UNKNOWN, -1.0);
            break;
          }
        }
      }
    }
  });

  return cluster_to_detections;
}

bool IsObstacleConnectWithCluster(
    const Obstacle& obstacle,
    const absl::flat_hash_set<std::pair<uint16_t, uint16_t>, RowColHash>&
        obstacle_rc_set) {
  const int row = obstacle.row;
  const int col = obstacle.col;
  for (int i = row - 1; i <= row + 1; i++) {
    for (int j = col - 1; j <= col + 1; j++) {
      if (ContainsKey(obstacle_rc_set, std::make_pair(row, col))) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

ProposedClusters FieryEyeNetProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  SCOPED_QTRACE("FieryEyeNetProposer::Propose");
  const auto& pose = env_info.pose();
  const auto& av_box = env_info.av_box();
  const auto& context = env_info.context();
  // Proposed clusters with detections.
  ProposedClusters proposed_clusters;

  // For each cluster, collect all detections that overlap with it.
  const ClusterToDetections cluster_to_detections =
      CreateClusterToDetectionsMap(clusters, context, av_box, thread_pool_);

  // If there is no detection overlapping with the cluster, just
  // propose it. Otherwise, for each cluster, if it overlaps with multiple
  // detections, divide this cluster. After that, for each
  // detection, collect all clusters that overlap with it, and merge all
  // those clusters into one.
  const auto cmp = [](const DetectionBoxWithType& a,
                      const DetectionBoxWithType& b) {
    if (a.box.center() != b.box.center()) {
      return a.box.center() < b.box.center();
    }
    if (a.box.length() != b.box.length()) {
      return a.box.length() < b.box.length();
    }
    if (a.box.width() != b.box.width()) {
      return a.box.width() < b.box.width();
    }
    if (a.box.heading() != b.box.heading()) {
      return a.box.heading() < b.box.heading();
    }
    if (a.score != b.score) {
      return a.score < b.score;
    }
    if (a.velocity != b.velocity) {
      return a.velocity < b.velocity;
    }
    if (a.is_merged != b.is_merged) {
      return a.is_merged < b.is_merged;
    }
    return false;
  };
  std::map<DetectionBoxWithType, ProposedClusters, decltype(cmp)>
      detection_to_clusters(cmp);

  const auto split_out_box_cluster = [&](const ProposedCluster& cluster,
                                         const Box2d& detection_box) {
    const auto obs_ptrs_vec =
        SplitClusterInFourDirections(cluster, detection_box);
    if (obs_ptrs_vec.empty()) {
      proposed_clusters.emplace_back(cluster);
      proposed_clusters.back().set_is_proposed(true);
      proposed_clusters.back().set_property(PP_OUTBOX);
    } else {
      for (const auto& obs_ptrs : obs_ptrs_vec) {
        if (!obs_ptrs.empty()) {
          proposed_clusters.emplace_back(
              ProposedCluster::InheritFrom(cluster).ConstructBase(obs_ptrs));
          proposed_clusters.back().set_is_proposed(true);
          proposed_clusters.back().set_property(PP_OUTBOX);
        }
      }
    }
  };

  // According to the divide result, divide the cluster in different way.
  const auto split_cluster_according_divide_result =
      [&](const ProposedCluster& cluster, const DivideResult& divide_result,
          const DetectionBoxWithType& detection_box,
          ProposedClusters* detection_to_clusters) {
        switch (divide_result.first) {
          // Note(zhangtao): The obstacle in cluster all in detection_box.
          case INBOX: {
            detection_to_clusters->push_back(cluster);
            detection_to_clusters->back().set_score(detection_box.score);
            detection_to_clusters->back().set_fen_velocity(
                detection_box.velocity);
            detection_to_clusters->back().set_is_proposed(true);
            detection_to_clusters->back().set_property(PP_INBOX);
            return;
          }
          // Note(zhangtao): The obstacle in cluster not in detection_box.
          case OUTBOX: {
            split_out_box_cluster(cluster, detection_box.box);
            return;
          }
          // Note(zhangtao): The divided clusters can be merge in one cluster.
          case DIVIDED: {
            const ProposedClusters divided_clusters = divide_result.second;
            QCHECK_EQ(divided_clusters.size(), 2);
            detection_to_clusters->push_back(divided_clusters[0]);
            detection_to_clusters->back().set_bounding_box(detection_box.box);
            detection_to_clusters->back().set_type(detection_box.type);
            detection_to_clusters->back().set_score(detection_box.score);
            detection_to_clusters->back().set_fen_velocity(
                detection_box.velocity);
            detection_to_clusters->back().set_type_source(MTS_FIERY_EYE_NET);
            detection_to_clusters->back().set_is_proposed(true);
            detection_to_clusters->back().set_property(PP_INBOX);

            const Polygon2d polygon_a =
                cluster_util::ComputeContour(divided_clusters[0]);
            const Polygon2d polygon_b =
                cluster_util::ComputeContour(divided_clusters[1]);
            if (polygon_b.HasOverlap(polygon_a)) {
              split_out_box_cluster(divided_clusters[1], detection_box.box);
            } else {
              proposed_clusters.emplace_back(divided_clusters[1]);
              proposed_clusters.back().set_is_proposed(true);
              proposed_clusters.back().set_property(PP_OUTBOX);
            }
            return;
          }
        }
      };

  const auto divide_cluster_by_fen =
      [&](const ProposedCluster* cluster,
          const DetectionBoxWithType& detection_box) {
        const auto divide_result =
            DivideClusterByFen(*cluster, detection_box, pose.z);

        ProposedClusters& clusters = detection_to_clusters[detection_box];
        split_cluster_according_divide_result(*cluster, divide_result,
                                              detection_box, &clusters);
        if (clusters.empty()) {
          detection_to_clusters.erase(detection_box);
        }
      };

  std::map<DetectionBoxWithType, std::vector<DetectionBoxWithType>,
           decltype(cmp)>
      merged_detections(cmp);
  std::map<DetectionBoxWithType, int, decltype(cmp)> detection_clusters_count(
      cmp);
  for (const auto& [_, detection_boxes] : cluster_to_detections) {
    for (const auto& det_box : detection_boxes) {
      if (!detection_clusters_count.count(det_box)) {
        detection_clusters_count[det_box] = 1;
      } else {
        detection_clusters_count[det_box] += 1;
      }
    }
  }
  for (const auto& [cluster, detection_boxes] : cluster_to_detections) {
    // If no detections overlap with the current cluster, propose it as it is.
    if (detection_boxes.size() == 0) {
      proposed_clusters.push_back(*cluster);
      proposed_clusters.back().set_is_proposed(false);
      continue;
    }
    // If it has detections overlapped, we try to divide it.
    if (detection_boxes.size() == 1) {
      divide_cluster_by_fen(cluster, detection_boxes[0]);
      continue;
    } else {
      // deal with large vehicle detections, if they are similar in heading, we
      // will merge them into same cluster
      if (CheckMergeBoxes(*cluster, detection_boxes)) {
        const auto divided_result =
            DivideClusterByFen(*cluster, detection_boxes, pose.z);
        ProposedCluster current_cluster = *cluster;
        if (divided_result.first == DIVIDED) {
          QCHECK_EQ(divided_result.second.size(), 2);
          current_cluster = divided_result.second[0];
        }
        float max_score = 0.0;
        Vec2d select_velocity = Vec2d(0.0, 0.0);
        std::vector<bool> should_skip_box(detection_boxes.size(), false);
        std::vector<double> merged_headings;
        auto sorted_detection_boxes = detection_boxes;
        // Sort boxes to skip lower score box first
        std::sort(
            sorted_detection_boxes.begin(), sorted_detection_boxes.end(),
            [&](const DetectionBoxWithType& a, const DetectionBoxWithType& b) {
              const float a_score = ComputeMergeBoxScore(a, pose);
              const float b_score = ComputeMergeBoxScore(b, pose);
              return std::make_pair(a_score, a.box.center()) <
                     std::make_pair(b_score, b.box.center());
            });
        std::vector<Vec2d> corners;
        for (int m = 0; m < sorted_detection_boxes.size(); m++) {
          const auto& det_box = sorted_detection_boxes[m];
          const bool skip_current_det_box = CheckIfSkipCurrentBox(
              sorted_detection_boxes, should_skip_box, current_cluster, m);
          if (!skip_current_det_box || detection_clusters_count[det_box] > 1) {
            const auto& current_corners =
                det_box.box.GetCornersCounterClockwise();
            corners.insert(corners.end(), current_corners.begin(),
                           current_corners.end());
            merged_headings.push_back(det_box.box.heading());
            max_score = std::max(max_score, det_box.score);
            select_velocity = det_box.velocity;
          } else {
            // If a box is skipped, we do not care about obstacles fall in
            // this box anymore
            should_skip_box[m] = true;
          }
        }
        MaybeRenderMergedBoxes(sorted_detection_boxes, should_skip_box, pose.z);
        Polygon2d merged_polygon;
        if (Polygon2d::ComputeConvexHull(corners, &merged_polygon)) {
          const auto merged_bbox = merged_polygon.BoundingBoxWithHeading(
              ComputeClusterBoundingBoxMeanHeading(merged_headings));
          const auto merged_detection =
              DetectionBoxWithType(merged_bbox, merged_bbox, select_velocity,
                                   MT_VEHICLE, max_score, true);
          // mark all detection boxes that will be merged
          for (int i = 0; i < sorted_detection_boxes.size(); ++i) {
            if (should_skip_box[i]) continue;
            std::vector<DetectionBoxWithType>& det_boxs =
                merged_detections[sorted_detection_boxes[i]];
            det_boxs.push_back(merged_detection);
          }
          ProposedClusters& clusters = detection_to_clusters[merged_detection];
          split_cluster_according_divide_result(*cluster, divided_result,
                                                merged_detection, &clusters);
          QCHECK(!clusters.empty());
          continue;
        }
      }
      // If there is more than one detection for this cluster, try to divide
      // this cluster by assigning each obstacle to a new cluster with the
      // nearest detection box.
      std::vector<std::vector<ObstaclePtr>> new_clusters(
          detection_boxes.size());
      for (const auto* obstacle : *cluster) {
        int detection_index = -1;
        Vec2d min_dist(std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max());
        for (int i = 0; i < detection_boxes.size(); ++i) {
          // Compare the distance between the obstacle coordinate and the box.
          // If the obstacle is inside of more than one box, then compare the
          // distance to box center.
          const Vec2d dist(detection_boxes[i].box.DistanceTo(obstacle->coord()),
                           (detection_boxes[i].box.center() - obstacle->coord())
                               .squaredNorm());
          if (dist < min_dist) {
            min_dist = dist;
            detection_index = i;
          }
        }
        QCHECK_GE(detection_index, 0);
        new_clusters[detection_index].push_back(obstacle);
      }
      for (int i = 0; i < new_clusters.size(); ++i) {
        const auto& obstacles = new_clusters[i];
        if (obstacles.empty()) continue;
        ProposedCluster new_cluster =
            ProposedCluster::InheritFrom(*cluster).ConstructBase(obstacles);
        divide_cluster_by_fen(&new_cluster, detection_boxes[i]);
      }
    }
  }

  // for all detection box that has been merged in previous logic, put all of
  // its clusters into the final detection box
  for (const auto& [merged_detection_box, final_detection_boxs] :
       merged_detections) {
    const auto& merging_clusters = detection_to_clusters[merged_detection_box];
    if (detection_to_clusters.count(final_detection_boxs[0])) {
      for (const auto& cluster : merging_clusters) {
        // only mapping detection box to its first matched final box
        detection_to_clusters[final_detection_boxs[0]].push_back(cluster);
      }
    }
  }

  // For each detection, collect all clusters and merge them (if there
  // is more than one cluster for this detection) into single cluster.
  std::vector<ObstaclePtr> obstacles;
  for (const auto& [detection_box_with_type, clusters] :
       detection_to_clusters) {
    // skip the detection box that has been merged
    if (merged_detections.count(detection_box_with_type)) {
      continue;
    }
    obstacles.clear();
    for (const auto& cluster : clusters) {
      obstacles.insert(obstacles.end(), cluster.obstacles().begin(),
                       cluster.obstacles().end());
    }
    QCHECK(!obstacles.empty());
    proposed_clusters.emplace_back(ProposedCluster::InheritFrom(clusters[0])
                                       .ConstructBase(std::move(obstacles)));
    proposed_clusters.back().set_bounding_box(detection_box_with_type.box);
    proposed_clusters.back().set_type(detection_box_with_type.type);
    proposed_clusters.back().set_score(detection_box_with_type.score);
    proposed_clusters.back().set_fen_velocity(detection_box_with_type.velocity);
    proposed_clusters.back().set_type_source(MTS_FIERY_EYE_NET);
    proposed_clusters.back().set_is_proposed(true);
    proposed_clusters.back().set_property(PP_INBOX);
  }

  TrimVehicleClusters(pose, &proposed_clusters, env_info.av_height());

  MaybeRenderProposedClusters(proposed_clusters, pose.z);

  QCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
