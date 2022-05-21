#include "onboard/perception/registration/icp_with_covariance.h"

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

DEFINE_bool(save_data, false, "save all the data.");

namespace qcraft {

bool VecToPCL(const std::vector<Vec3d>& vec_cloud,
              pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud) {
  if (nullptr == pcl_cloud) return false;
  pcl_cloud->resize(vec_cloud.size());
  for (int i = 0; i < vec_cloud.size(); ++i) {
    pcl_cloud->points[i].x = vec_cloud[i].x();
    pcl_cloud->points[i].y = vec_cloud[i].y();
    pcl_cloud->points[i].z = vec_cloud[i].z();
  }
  return true;
}

bool EstimateNormal(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normal) {
  if (nullptr == cloud_normal) return false;

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> nest;
  nest.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
      new pcl::search::KdTree<pcl::PointNormal>));
  nest.setRadiusSearch(0.01);
  nest.setInputCloud(cloud_normal);
  nest.compute(*cloud_normal);

  return true;
}

TEST(IcpWithCovariance, TestMatchPoints) {
  // Load point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ASSERT_TRUE(pcl::io::loadPCDFile<pcl::PointXYZ>(
                  "onboard/perception/registration/data/bunny_ds0001.pcd",
                  *pcl_cloud) != -1);
  QLOG(INFO) << "Stanford bunny is loadded successfully. " << pcl_cloud->size()
             << " points are loaded.";
  std::vector<Vec3d> vec_cloud;
  vec_cloud.reserve(pcl_cloud->points.size());
  for (const auto& pt : pcl_cloud->points) {
    vec_cloud.push_back({pt.x, pt.y, pt.z});
  }

  // Set transformation ground truth.
  const Eigen::Vector3d YPR(M_PI / 2, 0.0, -0.0);
  Eigen::AngleAxisd rot_vector;
  rot_vector = Eigen::AngleAxisd(YPR[0], Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(YPR[1], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(YPR[2], Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d gt_rotation = rot_vector.toRotationMatrix();
  const Eigen::Vector3d gt_translation(0.0, 0.0, 0.0);
  Eigen::Matrix4d gt_transform = Eigen::Matrix4d::Identity();
  gt_transform.block<3, 3>(0, 0) = gt_rotation;
  gt_transform.block<3, 1>(0, 3) = gt_translation;
  // Hypothesis with noise.
  Eigen::AngleAxisd hyp_rot_vector;
  hyp_rot_vector =
      Eigen::AngleAxisd(YPR[0] - 30.0 / 180.0 * M_PI,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(YPR[1] - 3.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(YPR[2] + 5.0 / 180.0 * M_PI, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d hypothesis = Eigen::Matrix4d::Identity();
  hypothesis.block<3, 3>(0, 0) = hyp_rot_vector.toRotationMatrix();
  hypothesis.block<3, 1>(0, 3) = gt_translation + Vec3d(0.02, 0.01, -0.01);

  // Prepare point cloud.
  std::vector<Vec3d> transformd_vec_cloud;
  transformd_vec_cloud.reserve(vec_cloud.size());
  for (const auto& pt : vec_cloud) {
    transformd_vec_cloud.push_back(gt_rotation * pt + gt_translation);
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr source(
      new pcl::PointCloud<pcl::PointNormal>);
  VecToPCL(vec_cloud, source);
  pcl::PointCloud<pcl::PointNormal>::Ptr target(
      new pcl::PointCloud<pcl::PointNormal>);
  VecToPCL(transformd_vec_cloud, target);

  // Registration
  EstimateNormal(target);

  IcpWithCovariance icp_with_covariance;
  PointMatcherOptions options;
  options.max_matching_dist = 0.01;
  options.max_mse = 1e-9;
  options.max_num_iters = 10;
  options.hypothesis.Set(hypothesis);
  options.max_num_points = 1000;
  const auto res = icp_with_covariance.MatchPoints(target, source, options);

  // Check result.
  EXPECT_TRUE(res.success);
  const Eigen::Matrix4d diff = gt_transform.inverse() * res.transform.mat();
  const Vec3d diff_trans = diff.block<3, 1>(0, 3);
  Eigen::AngleAxisd diff_rot(diff.block<3, 3>(0, 0));
  // Angle precision 0.1degree == 1.7e-3rad
  EXPECT_LT(diff_rot.angle(), 1.7e-3);
  // Translation precision 1cm
  EXPECT_LT(diff_trans.norm(), 0.01);

  // Print log
  QLOG(INFO) << absl::StrFormat(
      "\n===================== REPORT ======================"
      "\nSource cloud has %d points"
      "\nTarget cloud has %d points"
      "\nRegistration Option: --max_matching_dist is %f"
      "\n                     --max_mse is %f"
      "\n                     --max_num_iteration is %d"
      "\n                     --max_num_points is %d"
      "\nRegistration Result: --iteration is %d"
      "\n                     --mse is %f"
      "\n                     --last matched points is %d"
      "\n                     --diff ratation is %lf rad"
      "\n                     --diff translation is %lf m."
      "\n===================================================",
      source->points.size(), target->points.size(), options.max_matching_dist,
      options.max_mse, options.max_num_iters, options.max_num_points,
      res.num_iteration, res.mse, res.num_matched_points, diff_rot.angle(),
      diff_trans.norm());

  // Save for visualization
  if (FLAGS_save_data) {
    pcl::PointCloud<pcl::PointNormal>::Ptr transform_cloud(
        new pcl::PointCloud<pcl::PointNormal>);
    transform_cloud->points.resize(source->points.size());
    transform_cloud->height = 1;
    transform_cloud->width = transform_cloud->points.size();
    for (size_t i = 0; i < source->points.size(); ++i) {
      const auto& pt = source->points[i];
      Vec3d tmp(pt.x, pt.y, pt.z);
      tmp = res.transform.mat().block<3, 3>(0, 0) * tmp +
            res.transform.mat().block<3, 1>(0, 3);
      transform_cloud->points[i].x = tmp.x();
      transform_cloud->points[i].y = tmp.y();
      transform_cloud->points[i].z = tmp.z();
    }
    pcl::io::savePCDFile(
        "onboard/perception/registration/data/target_bunny.pcd", *target);
    pcl::io::savePCDFile(
        "onboard/perception/registration/data/result_bunny.pcd",
        *transform_cloud);
    pcl::io::savePCDFile(
        "onboard/perception/registration/data/source_bunny.pcd", *source);
    QLOG(INFO) << "\n>> Save result out for visualization: <<\n"
                  "onboard/perception/registration/data/target_bunny.pcd\n"
                  "onboard/perception/registration/data/source_bunny.pcd\n"
                  "onboard/perception/registration/data/result_bunny.pcd\n"
                  "TIPS: It is recommended to use Open3d for visualization\n";
  }
}
}  // namespace qcraft

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
