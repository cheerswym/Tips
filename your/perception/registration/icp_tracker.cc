#include "onboard/perception/registration/icp_tracker.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/perception_util.h"
#include "onboard/perception/registration/icp.h"
#include "onboard/perception/registration/icp_with_covariance.h"
#include "onboard/perception/registration/icp_with_normal_util.h"
#include "onboard/perception/registration/tic_toc.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/vis/common/color.h"

DEFINE_bool(icp_cvs, false, "Render points");
DEFINE_bool(use_point2plane_icp_tracker, true,
            "Whether to use point2plane icp");

namespace qcraft {
namespace {
constexpr double kTargetMse = Sqr(0.05);           // m^2
constexpr double kMaxMse = Sqr(0.2);               // m^2
constexpr double kFilterGroundPointHeight = 0.15;  // m
// Only perform ICP tracking if the number of points in the cluster exceeds this
// number.
constexpr int kMinNumPointsForIcp = 20;
constexpr float kMinHeightForIcp = 0.5f;
constexpr double kIcpMaxSpeed = 50.0;  // m/s
// ICP is good if the number of matched points / number of points in the cluster
// is no less than the ratio below.
constexpr double kIcpMinMatchedPointsRatio = 0.75;
// The maximum distance between matched track and measurement.
const double kMaxMatchDistForNonPed = 5.0;  // m
const double kMaxMatchDistForPed = 1.5;     // m
// The maximum height of a car.
constexpr double kNoRoofHeight = 1.1;

std::vector<ClusterWithObstacles> CloneClusters(
    const SegmentedClusters& clusters, ThreadPool* thread_pool) {
  std::vector<ClusterWithObstacles> cloned_clusters(clusters.size());
  ParallelFor(0, clusters.size(), thread_pool, [&](int i) {
    cloned_clusters[i] = ClusterWithObstacles(clusters[i]);
  });
  return cloned_clusters;
}

void MaybeRenderPoint2PlaneIcpResultCvs(
    const ClusterWithObstacles& prev_cluster,
    const ClusterWithObstacles& curr_cluster,
    const AffineTransformation& transform, double ground_z, Vec2d velocity,
    const IcpCovMatchResult result, const Vec3d& bias,
    const bool dir_prev2src) {
  if (!FLAGS_icp_cvs) return;

  vis::Canvas& canvas = vantage_client_man::GetCanvas("perception/icp_heading");

  // Draw original points.
  const auto prev_cluster_points = cluster_util::CollectPoints(prev_cluster);
  const auto curr_cluster_points = cluster_util::CollectPoints(curr_cluster);
  canvas.DrawPoints(prev_cluster_points, vis::Color::kGreen, 1, true, false);
  canvas.DrawPoints(curr_cluster_points, vis::Color::kYellow, 1, true, false);

  // Draw registration used points and transformation.
  std::vector<Vec3d> reg_prev_points, reg_curr_points;
  const auto& [prev_cluster_cloud, prev_bias] = prev_cluster.GetBiasCloud();
  reg_prev_points.reserve(prev_cluster_cloud->points.size());
  const auto& [curr_cluster_cloud, curr_bias] = curr_cluster.GetBiasCloud();
  reg_curr_points.reserve(curr_cluster_cloud->points.size());
  for (const auto& pt : prev_cluster_cloud->points) {
    Vec3d point(pt.x, pt.y, pt.z);
    Eigen::Matrix4d mat = transform.mat();
    point = mat.block<3, 3>(0, 0) * point + mat.block<3, 1>(0, 3);
    reg_prev_points.push_back({point.x() + prev_bias.x(),
                               point.y() + prev_bias.y(),
                               point.z() + prev_bias.z() + 20.0});
  }
  for (const auto& pt : curr_cluster_cloud->points)
    reg_curr_points.push_back({pt.x + prev_bias.x(), pt.y + prev_bias.y(),
                               pt.z + prev_bias.z() + 20.0});
  canvas.DrawPoints(reg_prev_points, vis::Color::kGreen, 1, true, false);
  canvas.DrawPoints(reg_curr_points, vis::Color::kYellow, 1, true, false);

  constexpr double kDrawNormalLength = 0.3;
  std::vector<Vec3d> begins, ends;
  if (dir_prev2src) {
    // Use curr_cloud as ref and prev_cloud as src.
    begins = reg_curr_points;
    QCHECK(curr_cluster_cloud->points.size() == reg_curr_points.size());
    ends.reserve(begins.size());
    for (size_t i = 0; i < begins.size(); ++i) {
      const Vec3d cur_normal(curr_cluster_cloud->points[i].normal_x,
                             curr_cluster_cloud->points[i].normal_y,
                             curr_cluster_cloud->points[i].normal_z);
      ends[i] = begins[i] + kDrawNormalLength * cur_normal;
    }
  } else {
    // Use prev_cloud as ref and curr_cloud as src.
    begins = reg_prev_points;
    QCHECK(prev_cluster_cloud->points.size() == reg_prev_points.size());
    ends.reserve(begins.size());
    for (size_t i = 0; i < begins.size(); ++i) {
      ends[i] = begins[i] + kDrawNormalLength *
                                Vec3d(prev_cluster_cloud->points[i].normal_x,
                                      prev_cluster_cloud->points[i].normal_y,
                                      prev_cluster_cloud->points[i].normal_z);
    }
  }
  for (size_t i = 0; i < begins.size(); ++i) {
    canvas.DrawLine(begins[i], ends[i], vis::Color::kRed, /*size=*/1);
  }

  const Vec3d prev_cluster_mass_center =
      prev_cluster.ComputeCentroidFromPoints();
  double time_diff = curr_cluster.timestamp() - prev_cluster.timestamp();
  time_diff = time_diff == 0.0 ? std::numeric_limits<double>::min() : time_diff;
  const Vec3d mass_velocity =
      (result.transform.TransformPoint(prev_cluster_mass_center - bias) + bias -
       prev_cluster_mass_center) /
      time_diff;

  const auto centroid = curr_cluster.ComputeCentroidFromObstacles();
  const Vec3d centroid_point(centroid.x(), centroid.y(), ground_z);
  const Vec3d heading_point_by_mass_velocity = centroid_point + mass_velocity;
  canvas.DrawLine(centroid_point, heading_point_by_mass_velocity,
                  vis::Color::kLightRed,
                  /*size=*/2);
  canvas.DrawText(absl::StrFormat("velocity %f", mass_velocity.norm()),
                  centroid_point + Vec3d(0, 0, 1.8), 0.7, 0.4,
                  vis::Color::kWhite);

  Vec2d prev_center, curr_center;
  if (curr_cluster.bounding_box() && prev_cluster.bounding_box()) {
    curr_center = curr_cluster.bounding_box()->center();
    prev_center = prev_cluster.bounding_box()->center();
  } else {
    curr_center = curr_cluster.ComputeCentroidFromObstacles();
    prev_center = prev_cluster.ComputeCentroidFromObstacles();
  }
  canvas.DrawLine({prev_center, ground_z}, {curr_center, ground_z},
                  vis::Color::kCyan,
                  /*size=*/2);
}

void MaybeRenderPoint2PointIcpResultCvs(const Cluster& prev_cluster,
                                        const Cluster& curr_cluster,
                                        const AffineTransformation& transform,
                                        double ground_z, Vec2d velocity,
                                        const PointMatchResult result) {
  if (!FLAGS_icp_cvs) return;

  vis::Canvas& canvas = vantage_client_man::GetCanvas("perception/icp_heading");

  const auto prev_cluster_points = cluster_util::CollectPoints(prev_cluster);
  const auto curr_cluster_points = cluster_util::CollectPoints(curr_cluster);
  canvas.DrawPoints(prev_cluster_points, vis::Color::kGreen, 3, true, false);
  canvas.DrawPoints(curr_cluster_points, vis::Color::kYellow, 3, true, false);

  // TODO(jingwei) calculate normals.
  std::vector<PointNeighborAndNormal> normals;
  std::vector<Vec3d> ref_points;
  ref_points.reserve(normals.size());
  for (const auto& normal : normals) {
    ref_points.push_back(normal.point);
  }
  canvas.DrawPoints(ref_points, vis::Color::kGreen, 3, true, false);

  const Vec3d bias = result.bias;
  const Vec3d prev_cluster_mass_center =
      prev_cluster.ComputeCentroidFromPoints();
  double time_diff = curr_cluster.timestamp() - prev_cluster.timestamp();
  time_diff = time_diff == 0.0 ? std::numeric_limits<double>::min() : time_diff;
  const Vec3d mass_velocity =
      (result.transform.TransformPoint(prev_cluster_mass_center - bias) + bias -
       prev_cluster_mass_center) /
      time_diff;

  const auto centroid = curr_cluster.ComputeCentroidFromObstacles();
  const Vec3d centroid_point(centroid.x(), centroid.y(), ground_z);
  const Vec3d heading_point_by_mass_velocity = centroid_point + mass_velocity;
  canvas.DrawLine(centroid_point, heading_point_by_mass_velocity,
                  vis::Color::kLightRed,
                  /*size=*/2);
  canvas.DrawText(absl::StrFormat("velocity %f", mass_velocity.norm()),
                  centroid_point + Vec3d(0, 0, 1.8), 0.7, 0.4,
                  vis::Color::kWhite);

  Vec2d prev_center, curr_center;
  if (curr_cluster.bounding_box() && prev_cluster.bounding_box()) {
    curr_center = curr_cluster.bounding_box()->center();
    prev_center = prev_cluster.bounding_box()->center();
  } else {
    curr_center = curr_cluster.ComputeCentroidFromObstacles();
    prev_center = prev_cluster.ComputeCentroidFromObstacles();
  }
  canvas.DrawLine({prev_center, ground_z}, {curr_center, ground_z},
                  vis::Color::kCyan,
                  /*size=*/2);
}

Vec3d UpdateRunParams(const RunParamsProtoV2& run_params) {
  // In jinlv bus we use front two lidar middle pos as anchor pos,
  // while in MKZ, we use the center lidar pos.
  const auto select_lidar_param =
      perception_util::SelectLidarParams(run_params);
  return Vec3d(select_lidar_param.installation().extrinsics().x(), 0.0,
               select_lidar_param.installation().extrinsics().z());
}

bool PointInBlindArea(const VehiclePose& ego_pose, const Vec3d& point) {
  bool is_in_blind_area = false;
  constexpr double kBlindZoneMinX = -4.0;
  constexpr double kBlindZoneMaxX = 8.5;
  constexpr double kBlindZoneMinY = -5.0;
  constexpr double kBlindZoneMaxY = 5.0;
  const Eigen::Matrix4d ego_mat = ego_pose.ToTransform().mat();
  const auto point_under_ego =
      ego_mat.block<3, 3>(0, 0).transpose() * point -
      ego_mat.block<3, 3>(0, 0).transpose() * ego_mat.block<3, 1>(0, 3);
  if (point_under_ego.x() >= kBlindZoneMinX &&
      point_under_ego.x() <= kBlindZoneMaxX &&
      point_under_ego.y() >= kBlindZoneMinY &&
      point_under_ego.y() <= kBlindZoneMaxY) {
    is_in_blind_area = true;
  }
  return is_in_blind_area;
}

bool ClusterInBlindArea(const VehiclePose& ego_pose, const Cluster& cluster) {
  const Vec2d centroid = cluster.bounding_box().has_value()
                             ? cluster.bounding_box()->center()
                             : cluster.ComputeCentroidFromObstacles();
  return PointInBlindArea(
      ego_pose, Eigen::Vector3d(centroid.x(), centroid.y(), ego_pose.z));
}

bool IsInBlindArea(const VehiclePose& pose, const Vec2d& curr_cluster_centroid,
                   const Vec2d& prev_cluster_centroid) {
  bool is_in_blind_area = false;
  constexpr double kBlindZoneMinX = -4.0;
  constexpr double kBlindZoneMaxX = 8.5;
  constexpr double kBlindZoneMinY = -5.0;
  constexpr double kBlindZoneMaxY = 5.0;
  const auto pose_inv = pose.ToTransform().Inverse();
  const auto curr_center = pose_inv.TransformPoint(
      {curr_cluster_centroid.x(), curr_cluster_centroid.y(), pose.z});
  const auto prev_center = pose_inv.TransformPoint(
      {prev_cluster_centroid.x(), prev_cluster_centroid.y(), pose.z});
  if (curr_center.x() >= kBlindZoneMinX && curr_center.x() <= kBlindZoneMaxX &&
      curr_center.y() >= kBlindZoneMinY && curr_center.y() <= kBlindZoneMaxY) {
    is_in_blind_area = true;
  }
  if (prev_center.x() >= kBlindZoneMinX && prev_center.x() <= kBlindZoneMaxX &&
      prev_center.y() >= kBlindZoneMinY && prev_center.y() <= kBlindZoneMaxY) {
    is_in_blind_area = true;
  }
  return is_in_blind_area;
}

}  // namespace

std::vector<int> IcpTracker::NaiveMatch(
    const std::vector<ClusterWithObstacles>& row_clusters,
    const std::vector<ClusterWithObstacles>& col_clusters) const {
  Eigen::MatrixXd cost_matrix(row_clusters.size(), col_clusters.size());
  for (int c = 0; c < col_clusters.size(); ++c) {
    const double max_match_dist2 = col_clusters[c].type() == MT_PEDESTRIAN
                                       ? Sqr(kMaxMatchDistForPed)
                                       : Sqr(kMaxMatchDistForNonPed);
    for (int r = 0; r < row_clusters.size(); ++r) {
      double dist2 = 0;
      if (row_clusters[r].bounding_box() && col_clusters[c].bounding_box()) {
        dist2 = (row_clusters[r].bounding_box()->center() -
                 col_clusters[c].bounding_box()->center())
                    .squaredNorm();
      } else {
        dist2 = (row_clusters[r].ComputeCentroidFromObstacles() -
                 col_clusters[c].ComputeCentroidFromObstacles())
                    .squaredNorm();
      }
      cost_matrix(r, c) =
          dist2 < max_match_dist2 ? 1.0 / (dist2 + DBL_EPSILON) : -1.0;
    }
  }
  return tracker::tracker_util::ComputeMatches(cost_matrix);
}

Vec2d EstimateOffset(const Cluster& current, const Cluster& previous) {
  // If current and previous clusters of the same track both have bounding
  // boxes, use their centers to estimate the initial transformation.
  const auto curr_centroid = current.ComputeCentroidFromPoints();
  const auto prev_centroid = previous.ComputeCentroidFromPoints();
  return (curr_centroid - prev_centroid).block<2, 1>(0, 0);
}

bool IcpTracker::Point2PointICP(
    const std::vector<ClusterWithObstacles>& curr_clusters,
    const std::vector<ClusterWithObstacles>& prev_clusters,
    const std::vector<int>& matches, const VehiclePose& pose,
    std::vector<IcpTracker::TrackerResult>* results) const {
  if (nullptr == results) return false;
  const Icp point_matcher;
  if (!prev_clusters.empty()) {
    // Only use absl::Now() for latency calculation and logging, use
    // Clock::Now() for other purposes.
    const auto before_icp_track = absl::Now();
    const auto icp_match_func = [&](int i) {
      if (matches[i] < 0) return;
      const auto& curr_cluster = curr_clusters[i];
      const auto& prev_cluster = prev_clusters[matches[i]];
      // Exclude static objects for speeding up the icp tracker.
      if (curr_cluster.IsStaticType() || prev_cluster.IsStaticType()) return;
      const int num_points_curr = curr_cluster.NumPoints();
      // Skip too small clusters.
      if (num_points_curr < kMinNumPointsForIcp) return;
      // Skip too low clusters;
      if (curr_cluster.ComputeHeight() < kMinHeightForIcp) return;

      SCOPED_QTRACE_ARG1("IcpTracker::Track_track_cluster", "num_points",
                         num_points_curr);
      // ICP init transformation
      const auto offset = EstimateOffset(curr_cluster, prev_cluster);
      const AffineTransformation hypothesis =
          AffineTransformation::FromTranslation(offset.x(), offset.y(), 0.0);
      // ICP registration
      const auto match_result =
          point_matcher.MatchCluster(curr_cluster, prev_cluster,
                                     {.max_mse = kTargetMse,
                                      .max_num_iters = 10,
                                      .hypothesis = hypothesis,
                                      .max_num_points = 1000,
                                      .max_matching_dist = 1.0});
      // Check ICP result.
      // TODO(jingwei) Use mse.
      // TODO(jingwei) Which point to calculate velocity?
      if (match_result.num_matched_points >
          num_points_curr * kIcpMinMatchedPointsRatio) {
        const Vec3d point = prev_cluster.obstacles()[0]->points[0].coord();
        double time_diff = curr_cluster.timestamp() - prev_cluster.timestamp();
        time_diff =
            time_diff == 0.0 ? std::numeric_limits<double>::min() : time_diff;
        const Vec3d velocity_3d =
            (match_result.transform.TransformPoint(point) - point) / time_diff;
        const Vec2d velocity_2d(velocity_3d.x(), velocity_3d.y());
        const double velocity_sqr = velocity_2d.squaredNorm();
        // Check the speed and filter out explicit shooters.
        if (velocity_sqr < Sqr(kIcpMaxSpeed)) {
          (*results)[i].transform = match_result.transform;
          (*results)[i].mse = match_result.mse;
          (*results)[i].success = match_result.mse < kMaxMse;
          (*results)[i].velocity = Vec2d(velocity_3d.x(), velocity_3d.y());
          (*results)[i].prev_cluster_id = prev_cluster.id();
          (*results)[i].prev_cluster_timestamp = prev_cluster.timestamp();
          MaybeRenderPoint2PointIcpResultCvs(
              prev_cluster, curr_cluster, match_result.transform,
              curr_cluster.obstacles()[0]->ground_z, (*results)[i].velocity,
              match_result);
        }
      }
    };
    ParallelFor(0, curr_clusters.size(), thread_pool_, {.block_size = 1},
                icp_match_func);
    // Only use absl::Now() for latency calculation and logging, use
    // Clock::Now() for other purposes.
    const auto after_icp_track = absl::Now();
    VLOG(2) << "[ICP] Point2Point time_consume is "
            << after_icp_track - before_icp_track << " ns. cur_cluster_num is "
            << curr_clusters.size() << ", prev_cluster_num is "
            << prev_clusters.size();
  }
  return true;
}

bool IcpTracker::ValidType(const Cluster& curr_cluster,
                           const Cluster& prev_cluster) const {
  if (!ValidType(curr_cluster) && !ValidType(prev_cluster)) return false;
  if (MT_UNKNOWN == curr_cluster.type() && MT_UNKNOWN == prev_cluster.type())
    return false;
  return true;
}

bool IcpTracker::ValidType(const Cluster& cluster) const {
  switch (cluster.type()) {
    case MT_VEHICLE:
    case MT_CYCLIST:
    case MT_UNKNOWN:
      return true;
    case MT_PEDESTRIAN:
    case MT_MOTORCYCLIST:
    case MT_VEGETATION:
    case MT_BARRIER:
    case MT_STATIC_OBJECT:
    case MT_ROAD:
    case MT_CONE:
    case MT_MIST:
    case MT_FLYING_BIRD:
    case MT_FOD:
    case MT_WARNING_TRIANGLE:
      return false;
  }
  return false;
}

bool IcpTracker::ValidZone(const VehiclePose& ego_pose,
                           const Cluster& cluster) const {
  if (ClusterInBlindArea(ego_pose, cluster)) return false;
  const auto centroid = cluster.ComputeCentroidFromPoints();
  constexpr double kDistanceThresholdSqr = Sqr(60.0);
  if ((centroid.block<2, 1>(0, 0) - ego_pose.coord2d()).squaredNorm() >
      kDistanceThresholdSqr)
    return false;
  return true;
}

bool IcpTracker::ValidPointNum(const Cluster& cluster) const {
  constexpr int kMinPointNum = 75;
  return cluster.MainLidarNumPoints() > kMinPointNum;
}

bool IcpTracker::WhetherRegistration(const VehiclePose& ego_pose,
                                     const Cluster& cluster) const {
  return ValidType(cluster) && ValidZone(ego_pose, cluster) &&
         ValidPointNum(cluster);
}

bool IcpTracker::WhetherRegistration(const VehiclePose& ego_pose,
                                     const Cluster& curr_cluster,
                                     const Cluster& prev_cluster) const {
  return ValidType(curr_cluster, prev_cluster) &&
         ValidZone(ego_pose, curr_cluster) &&
         ValidZone(ego_pose, prev_cluster) && ValidPointNum(curr_cluster) &&
         ValidPointNum(prev_cluster);
}

bool IcpTracker::Point2PlaneICP(
    const std::vector<ClusterWithObstacles>& prev_clusters,
    const std::vector<int>& matches, const VehiclePose& pose,
    const RunParamsProtoV2& run_params,
    std::vector<ClusterWithObstacles>* curr_clusters,
    std::vector<IcpTracker::TrackerResult>* results) const {
  TicToc tic_toc_outer;
  tic_toc_outer.tic();
  if (nullptr == results || nullptr == curr_clusters) return false;
  const auto tracking_ref_pose = UpdateRunParams(run_params);
  const Vec3d anchor_pose =
      pose.ToTransform().TransformPoint(tracking_ref_pose);
  if (prev_clusters_.empty()) {
    // Prepare normal, for load balancing, when prev is empty, only calculate
    // normal for half of curr_clusters.
    std::vector<size_t> idxs(curr_clusters->size());
    std::iota(idxs.begin(), idxs.end(), 0);
    std::sort(idxs.begin(), idxs.end(), [&](size_t lhs, size_t rhs) {
      return std::make_pair(
                 (*curr_clusters)[lhs].NumPoints(),
                 (*curr_clusters)[lhs].ComputeCentroidFromObstacles()) <
             std::make_pair(
                 (*curr_clusters)[rhs].NumPoints(),
                 (*curr_clusters)[rhs].ComputeCentroidFromObstacles());
    });
    ParallelFor(0, curr_clusters->size() / 2, thread_pool_, [&](int i) {
      auto& curr_cluster = (*curr_clusters)[idxs[2 * i]];
      if (WhetherRegistration(pose, curr_cluster)) {
        curr_cluster.SaveBiasCloud(
            ComputeNormalsOfCluster(anchor_pose, curr_cluster), true);
      }
    });
    VLOG(2) << tic_toc_outer.toc("ICP_Tracker [Point2PlaneICP INIT] use : ");
    return true;
  }
  const auto func = [&](int i) {
    TicToc tt;
    tt.tic();
    auto& curr_cluster = (*curr_clusters)[i];
    const int num_points_curr = curr_cluster.NumPoints();
    // Skip too small and too low clusters.
    if (num_points_curr < kMinNumPointsForIcp ||
        curr_cluster.ComputeHeight() < kMinHeightForIcp)
      return;
    // If no match, calculate normal and return
    // FIXME(jingwei) why calculate normals for unmatched pointcloud?
    if (matches[i] < 0) {
      if (WhetherRegistration(pose, curr_cluster)) {
        curr_cluster.SaveBiasCloud(
            ComputeNormalsOfCluster(anchor_pose, curr_cluster), true);
      }
      return;
    }

    SCOPED_QTRACE_ARG1("IcpTracker::Track_track_cluster", "num_points",
                       num_points_curr);
    const auto& prev_cluster = prev_clusters_[matches[i]];
    if (!WhetherRegistration(pose, curr_cluster, prev_cluster)) return;

    IcpWithNormalLls<pcl::PointXYZ, pcl::PointNormal, double>
        point_matcher_with_normal;
    const auto offset = EstimateOffset(curr_cluster, prev_cluster);
    const double mean_ground_z =
        0.5 * (cluster_util::ComputeGroundZ(curr_cluster) +
               cluster_util::ComputeGroundZ(prev_cluster));

    // Set Registration Param
    bool reg_dir_from_prev_to_curr = false;
    std::vector<Vec3d> src_points;
    const SegmentedCluster* ref_cluster_ptr = nullptr;
    AffineTransformation hypothesis;
    if (!prev_cluster.HasCloud() && !curr_cluster.HasCloud()) {
      reg_dir_from_prev_to_curr = true;
      curr_cluster.SaveBiasCloud(
          ComputeNormalsOfCluster(anchor_pose, curr_cluster), true);
      if (!curr_cluster.HasCloud()) return;
      // No need to use std::move, compiler will optimize it for you, search
      // NRVO.
      src_points = cluster_util::CollectPointsInZRange(
          prev_cluster, mean_ground_z + kFilterGroundPointHeight,
          mean_ground_z + kNoRoofHeight);
      hypothesis =
          AffineTransformation::FromTranslation(offset.x(), offset.y(), 0.0);
      ref_cluster_ptr = &curr_cluster;
    } else {
      src_points = cluster_util::CollectPointsInZRange(
          curr_cluster, mean_ground_z + kFilterGroundPointHeight,
          mean_ground_z + kNoRoofHeight);
      hypothesis =
          AffineTransformation::FromTranslation(-offset.x(), -offset.y(), 0.0);
      ref_cluster_ptr = &prev_cluster;
    }
    if (src_points.size() <= 100 || std::isnan(src_points[0][0])) return;

    PointMatchResult match_result = point_matcher_with_normal.MatchPoints(
        *ref_cluster_ptr, src_points,
        {.max_mse = kTargetMse,
         .max_num_iters = 10,  // previous:20
         .hypothesis = hypothesis,
         .max_num_points = 1000,
         .max_matching_dist = 1.1});  // parameter
    if (!reg_dir_from_prev_to_curr)
      match_result.transform = match_result.transform.Inverse();

    // TODO(jingwei) use mse instead of matched_ratio?
    if (match_result.matched_ratio > kIcpMinMatchedPointsRatio) {
      double time_diff = curr_cluster.timestamp() - prev_cluster.timestamp();
      time_diff =
          time_diff == 0.0 ? std::numeric_limits<double>::min() : time_diff;
      const Vec3d prev_cluster_mass_center =
          prev_cluster.ComputeCentroidFromPoints();
      const Vec3d point_bias = match_result.bias;
      Vec3d velocity_3d = (match_result.transform.TransformPoint(
                               prev_cluster_mass_center - point_bias) +
                           point_bias - prev_cluster_mass_center) /
                          time_diff;
      const Vec2d velocity_2d(velocity_3d.x(), velocity_3d.y());
      // Check the speed and filter out explicit shooters.
      if (velocity_2d.squaredNorm() < Sqr(kIcpMaxSpeed)) {
        (*results)[i].transform = match_result.transform;
        (*results)[i].mse = match_result.mse;
        (*results)[i].success = true;
        (*results)[i].velocity = Vec2d(velocity_3d.x(), velocity_3d.y());
        (*results)[i].prev_cluster_id = prev_cluster.id();
        (*results)[i].prev_cluster_timestamp = prev_cluster.timestamp();
        (*results)[i].cost_time = tt.toc_micro();
        (*results)[i].src_num = src_points.size();
        const auto& [ref_cloud_normals, bias] = ref_cluster_ptr->GetBiasCloud();
        (*results)[i].ref_num = ref_cloud_normals->size();
        (*results)[i].dir_prev2src = reg_dir_from_prev_to_curr;
      }
    }
  };
  ParallelFor(0, curr_clusters->size(), thread_pool_, {.block_size = 1}, func);

  VLOG(2) << tic_toc_outer.toc("ICP_Tracker [Point2PlaneICP WORK] use : ");
  return true;
}

IcpTracker::RegPairs IcpTracker::CorrectMatchAndPrepareNormal(
    const RunParamsProtoV2& run_params, const VehiclePose& pose,
    const std::vector<int>& matches,
    std::vector<ClusterWithObstacles>* prev_clusters,
    std::vector<ClusterWithObstacles>* curr_clusters) const {
  if (nullptr == prev_clusters || nullptr == curr_clusters) return {};

  RegPairs result;
  result.reserve(matches.size());
  const Vec3d anchor_pose =
      pose.ToTransform().TransformPoint(UpdateRunParams(run_params));
  // If prev is empty, for load balancing, calculate
  // normals for only half of curr_clusters.
  if (prev_clusters->empty()) {
    std::vector<size_t> idxs(curr_clusters->size());
    std::iota(idxs.begin(), idxs.end(), 0);
    std::sort(idxs.begin(), idxs.end(), [&](size_t lhs, size_t rhs) {
      return std::make_pair(
                 (*curr_clusters)[lhs].NumPoints(),
                 (*curr_clusters)[lhs].ComputeCentroidFromObstacles()) <
             std::make_pair(
                 (*curr_clusters)[rhs].NumPoints(),
                 (*curr_clusters)[rhs].ComputeCentroidFromObstacles());
    });
    ParallelFor(0, curr_clusters->size() / 2, thread_pool_, [&](int i) {
      auto& curr_cluster = (*curr_clusters)[idxs[2 * i]];
      if (WhetherRegistration(pose, curr_cluster)) {
        curr_cluster.SaveBiasCloud(
            ComputeNormalsOfCluster(anchor_pose, curr_cluster), true);
      }
    });
    return result;
  }

  // Find registration pairs.
  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i] < 0) continue;
    const auto& curr_cluster = (*curr_clusters)[i];
    const auto& prev_cluster = (*prev_clusters)[matches[i]];
    if (WhetherRegistration(pose, curr_cluster, prev_cluster)) {
      // If both cluster have no normal, calculate normal for curr_cluster.
      if (!prev_cluster.HasNormal() && !curr_cluster.HasNormal()) {
        result.push_back({i, matches[i], 1});
      } else {
        result.push_back({i, matches[i], 0});
      }
    }
  }
  // Prepare cloud for cluster.
  constexpr double kLeafSize = 0.1f;
  ParallelFor(0, result.size(), thread_pool_, [&](int i) {
    const auto [curr_id, prev_id, normal_id] = result[i];
    auto& prev_cluster = (*prev_clusters)[prev_id];
    auto& curr_cluster = (*curr_clusters)[curr_id];
    constexpr int kMaxDensePointNum = 5000;
    if (normal_id == 0) {
      // Prev_cluster has normal, collect points from curr_cluster and subtract
      // bias
      const auto& [_, bias] = prev_cluster.GetBiasCloud();

      auto cloud_xyz =
          CollectMainLidarPointsFromCluster<pcl::PointNormal>(curr_cluster);
      if (cloud_xyz->points.size() > kMaxDensePointNum) {
        VoxelDownsample<pcl::PointNormal>(cloud_xyz, kLeafSize, kLeafSize,
                                          kLeafSize);
      }
      for (auto& pt : cloud_xyz->points) {
        pt.x = pt.x - bias.x();
        pt.y = pt.y - bias.y();
        pt.z = pt.z - bias.z();
      }
      curr_cluster.SaveBiasCloud({cloud_xyz, bias}, false);
    } else {
      // Calculate normal for curr_cluster, then collect points from
      // prev_cluster and subtract bias
      curr_cluster.SaveBiasCloud(
          ComputeNormalsOfCluster(anchor_pose, curr_cluster), true);
      const auto& [_, bias] = curr_cluster.GetBiasCloud();

      auto cloud_xyz =
          CollectMainLidarPointsFromCluster<pcl::PointNormal>(prev_cluster);
      if (cloud_xyz->points.size() > kMaxDensePointNum) {
        VoxelDownsample<pcl::PointNormal>(cloud_xyz, kLeafSize, kLeafSize,
                                          kLeafSize);
      }
      for (auto& pt : cloud_xyz->points) {
        pt.x = pt.x - bias.x();
        pt.y = pt.y - bias.y();
        pt.z = pt.z - bias.z();
      }
      prev_cluster.SaveBiasCloud({cloud_xyz, bias}, false);
    }
  });

  return result;
}

bool IcpTracker::Point2PlaneICP(
    const IcpTracker::RegPairs reg_pairs,
    const std::vector<ClusterWithObstacles>& prev_clusters,
    const std::vector<ClusterWithObstacles>& curr_clusters,
    std::vector<IcpTracker::TrackerResult>* results) const {
  if (nullptr == results) return false;
  if (prev_clusters_.empty() || curr_clusters.empty()) return true;

  ParallelFor(0, reg_pairs.size(), thread_pool_, [&](int i) {
    const auto [curr_id, prev_id, normal_id] = reg_pairs[i];
    const auto& prev_cluster = prev_clusters[prev_id];
    const auto& curr_cluster = curr_clusters[curr_id];

    const auto& [curr_cloud, bias] = curr_cluster.GetBiasCloud();
    const auto& [prev_cloud, _] = prev_cluster.GetBiasCloud();
    if (nullptr == curr_cloud || nullptr == prev_cloud) {
      return;
    }
    const auto offset = EstimateOffset(curr_cluster, prev_cluster);

    IcpWithCovariance icp_with_covariance;
    IcpCovMatchResult match_result;
    PointMatcherOptions point_match_option{
        .max_mse = kTargetMse,
        .max_num_iters = 5,  // previous:20
        .hypothesis = {},
        .max_num_points = 1000,
        .max_matching_dist = 0.2,
        .min_diff_angle = 1.0 / 180.0 * M_PI,
        .min_diff_translation_sqr = Sqr(0.05)};  // parameter
    if (normal_id == 1) {
      // Use curr_cloud as ref and prev_cloud as src.
      point_match_option.hypothesis =
          AffineTransformation::FromTranslation(offset.x(), offset.y(), 0.0);
      match_result = icp_with_covariance.MatchPoints(curr_cloud, prev_cloud,
                                                     point_match_option);
      (*results)[curr_id].dir_prev2src = true;
    } else {
      // Use prev_cloud as ref and curr_cloud as src.
      point_match_option.hypothesis =
          AffineTransformation::FromTranslation(-offset.x(), -offset.y(), 0.0);
      match_result = icp_with_covariance.MatchPoints(prev_cloud, curr_cloud,
                                                     point_match_option);
      (*results)[curr_id].dir_prev2src = false;
      if (match_result.success) {
        match_result.transform = match_result.transform.Inverse();
      }
    }

    if (match_result.success) {
      double time_diff = curr_cluster.timestamp() - prev_cluster.timestamp();
      time_diff =
          time_diff == 0.0 ? std::numeric_limits<double>::epsilon() : time_diff;
      const Vec3d prev_cluster_mass_center =
          prev_cluster.ComputeCentroidFromPoints();
      const Vec3d velocity_3d = (match_result.transform.TransformPoint(
                                     prev_cluster_mass_center - bias) +
                                 bias - prev_cluster_mass_center) /
                                time_diff;
      const Vec2d velocity_2d(velocity_3d.x(), velocity_3d.y());

      (*results)[curr_id].transform = match_result.transform;
      (*results)[curr_id].transform_cov = match_result.cov;
      (*results)[curr_id].mse = match_result.mse;
      (*results)[curr_id].success = true;
      (*results)[curr_id].velocity = velocity_2d;
      (*results)[curr_id].velocity_cov = match_result.cov.block<2, 2>(3, 3);
      (*results)[curr_id].prev_cluster_id = prev_cluster.id();
      (*results)[curr_id].prev_cluster_timestamp = prev_cluster.timestamp();

      MaybeRenderPoint2PlaneIcpResultCvs(
          prev_cluster, curr_cluster, match_result.transform,
          curr_cluster.obstacles()[0]->ground_z, (*results)[curr_id].velocity,
          match_result, bias, (*results)[curr_id].dir_prev2src);
    }
  });

  return true;
}

std::vector<IcpTracker::TrackerResult> IcpTracker::FastTrack(
    const VehiclePose& pose, const RunParamsProtoV2& run_params,
    const SegmentedClusters& curr_clusters) {
  SCOPED_QTRACE_ARG1("IcpTracker::Track", "num_clusters", curr_clusters.size());

  TicToc tic_toc, tic_toc_whole;
  tic_toc.tic();
  tic_toc_whole.tic();

  auto curr_clusters_bak = CloneClusters(curr_clusters, thread_pool_);

  VLOG(2) << tic_toc.toc("ICP_Tracker [CloneClusters] use : ");

  const auto matches = NaiveMatch(curr_clusters_bak, prev_clusters_);

  VLOG(2) << tic_toc.toc("ICP_Tracker [NaiveMatch] use : ");

  int pairs_num = 0;

  std::vector<TrackerResult> results(curr_clusters.size());
  if (!FLAGS_use_point2plane_icp_tracker) {
    QCHECK(Point2PointICP(curr_clusters_bak, prev_clusters_, matches, pose,
                          &results));
  } else {
    auto reg_pairs = CorrectMatchAndPrepareNormal(
        run_params, pose, matches, &prev_clusters_, &curr_clusters_bak);

    VLOG(2) << tic_toc.toc(
        "ICP_Tracker [Registration-CorrectMatchAndPrepareNormal] use : ");

    // Icp for at most kMaxIcpPair.
    constexpr int kMaxIcpPair = 25;
    if (reg_pairs.size() > kMaxIcpPair) {
      Box2d roi(pose.coord2d(), pose.yaw, 120.0, 11.25);
      std::sort(
          reg_pairs.begin(), reg_pairs.end(),
          [&](const RegPair& lhs, const RegPair& rhs) {
            const Vec2d lhs_centroid = curr_clusters_bak[std::get<0>(lhs)]
                                           .ComputeCentroidFromObstacles();
            const Vec2d rhs_centroid = curr_clusters_bak[std::get<0>(rhs)]
                                           .ComputeCentroidFromObstacles();
            return std::make_pair(roi.IsPointIn(lhs_centroid), lhs_centroid) >
                   std::make_pair(roi.IsPointIn(rhs_centroid), rhs_centroid);
          });
      reg_pairs.resize(kMaxIcpPair);
    }

    pairs_num = reg_pairs.size();

    VLOG(2) << tic_toc.toc("ICP_Tracker [Registration-Sort] use : ");

    QCHECK(
        Point2PlaneICP(reg_pairs, prev_clusters_, curr_clusters_bak, &results));

    VLOG(2) << tic_toc.toc("ICP_Tracker [Registration-Point2PlaneICP] use : ");
  }

  prev_clusters_.swap(curr_clusters_bak);

  VLOG(2) << absl::StrFormat(
      "ICP_Tracker [Whole-Pipeline] curr=%d, prev=%d, reg_pairs=%d, use %d us.",
      curr_clusters.size(), prev_clusters_.size(), pairs_num,
      tic_toc_whole.toc_micro());

  return results;
}

std::vector<IcpTracker::TrackerResult> IcpTracker::Track(
    const VehiclePose& pose, const RunParamsProtoV2& run_params,
    const SegmentedClusters& curr_clusters) {
  TicToc tic_toc, tic_toc_whole;
  tic_toc.tic();
  tic_toc_whole.tic();

  SCOPED_QTRACE_ARG1("IcpTracker::Track", "num_clusters", curr_clusters.size());

  auto curr_clusters_bak = CloneClusters(curr_clusters, thread_pool_);
  VLOG(2) << tic_toc.toc("ICP_Tracker [CloneClusters] use : ");

  // If FLAGS_use_point2plane_icp_tracker is false, the point2point icp is
  // used, otherwise, the point2plane icp is used.
  const auto matches = NaiveMatch(curr_clusters_bak, prev_clusters_);
  VLOG(2) << tic_toc.toc("ICP_Tracker [NaiveMatch] use : ");

  std::vector<TrackerResult> results(curr_clusters.size());
  if (!FLAGS_use_point2plane_icp_tracker) {
    QCHECK(Point2PointICP(curr_clusters_bak, prev_clusters_, matches, pose,
                          &results));
  } else {
    QCHECK(Point2PlaneICP(prev_clusters_, matches, pose, run_params,
                          &curr_clusters_bak, &results));
  }
  VLOG(2) << tic_toc.toc("ICP_Tracker [Registration-Point2PlaneICP] use : ");

  prev_clusters_.swap(curr_clusters_bak);

  VLOG(2) << absl::StrFormat(
      "ICP_Tracker [Whole-Pipeline] curr=%d, prev=%d, use %d us.",
      curr_clusters.size(), prev_clusters_.size(), tic_toc_whole.toc_micro());
  int valid_reg = 0;
  for (size_t i = 0; i < results.size(); ++i) {
    if (results[i].success) {
      valid_reg++;
      const std::string dir =
          results[i].dir_prev2src ? "prev->src" : "src->prev";
      VLOG(2) << "RegDetail: cost_time=" << results[i].cost_time
              << "us, src_num=" << results[i].src_num
              << ", ref_num=" << results[i].ref_num << ", dir is: " << dir;
    }
  }
  int valid_match = 0;
  for (const auto& m : matches) {
    if (m != -1) valid_match++;
  }
  VLOG(2) << "RegSummary: valid_reg=" << valid_reg
          << ", invalid_reg=" << results.size() - valid_reg << ", match is "
          << valid_match;
  return results;
}
}  // namespace qcraft
