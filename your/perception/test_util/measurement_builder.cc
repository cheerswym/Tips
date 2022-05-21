#include "onboard/perception/test_util/measurement_builder.h"

#include <algorithm>
#include <random>
#include <utility>

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"

namespace qcraft {

namespace {
void SetVec3dProto(Vec3d vec, Vec3dProto* vec_proto) {
  QCHECK_NOTNULL(vec_proto);
  vec_proto->set_x(vec.x());
  vec_proto->set_y(vec.y());
  vec_proto->set_z(vec.z());
}
}  // namespace

ClusterMeasurementBuilder::ClusterMeasurementBuilder() {
  cluster_measurement_.set_height(3.);
  cluster_measurement_.clear_obstacle_info();
  *cluster_measurement_.add_obstacle_info() =
      ObstacleInfoBuilder().set_center({0, 0}).Build();
  *cluster_measurement_.add_obstacle_info() =
      ObstacleInfoBuilder().set_center({0, 1}).Build();
  *cluster_measurement_.add_obstacle_info() =
      ObstacleInfoBuilder().set_center({1, 0}).Build();
  *cluster_measurement_.add_obstacle_info() =
      ObstacleInfoBuilder().set_center({1, 1}).Build();
}

ClusterMeasurementBuilder& ClusterMeasurementBuilder::set_height(
    double height) {
  cluster_measurement_.set_height(height);
  return *this;
}

ClusterMeasurementBuilder& ClusterMeasurementBuilder::set_obstacle_info(
    const std::vector<LaserMeasurementProto::ObstacleInfo>& obstacle_info) {
  cluster_measurement_.clear_obstacle_info();
  for (const auto& info : obstacle_info) {
    *cluster_measurement_.add_obstacle_info() = info;
  }
  return *this;
}

ObstacleInfoBuilder::ObstacleInfoBuilder() {
  obstacle_info_.set_min_z(0);
  obstacle_info_.set_max_z(1);
  Vec2d(0.0, 0.0).ToProto(obstacle_info_.mutable_center());
}

ObstacleInfoBuilder& ObstacleInfoBuilder::set_center(
    const Vec2d& obstacle_center) {
  obstacle_center.ToProto(obstacle_info_.mutable_center());
  return *this;
}
ObstacleInfoBuilder& ObstacleInfoBuilder::set_min_z(double min_z) {
  obstacle_info_.set_min_z(min_z);
  return *this;
}

ObstacleInfoBuilder& ObstacleInfoBuilder::set_max_z(double max_z) {
  obstacle_info_.set_max_z(max_z);
  return *this;
}

LaserMeasurementProto::ObstacleInfo ObstacleInfoBuilder::Build() {
  return obstacle_info_;
}

LaserMeasurementProto::ClusterMeasurement ClusterMeasurementBuilder::Build() {
  return cluster_measurement_;
}

IcpMeasurementBuilder::IcpMeasurementBuilder() {
  icp_measurement_.mutable_vel()->set_x(0.);
  icp_measurement_.mutable_vel()->set_y(0.);
  Mat2d cov;
  cov << 1., 0., 0., 1.;
  Mat2dToProto(cov, icp_measurement_.mutable_velocity_covariance());
}

IcpMeasurementBuilder& IcpMeasurementBuilder::set_vel(const Vec2d& vel) {
  vel.ToProto(icp_measurement_.mutable_vel());
  return *this;
}

LaserMeasurementProto::IcpMeasurement IcpMeasurementBuilder::Build() {
  return icp_measurement_;
}

LaserMeasurementBuilder::LaserMeasurementBuilder() {
  // Minimum valid laser measurement.
  Vec2d(0.0, 0.0).ToProto(laser_measurement_.add_contour());
  Vec2d(0.0, 1.0).ToProto(laser_measurement_.add_contour());
  Vec2d(1.0, 1.0).ToProto(laser_measurement_.add_contour());
  Vec2d(1.0, 0.0).ToProto(laser_measurement_.add_contour());

  *laser_measurement_.mutable_cluster_measurement() =
      ClusterMeasurementBuilder().Build();
  *laser_measurement_.mutable_icp_measurement() =
      IcpMeasurementBuilder().Build();
}

LaserMeasurementBuilder& LaserMeasurementBuilder::set_contour(
    const Polygon2d& contour) {
  laser_measurement_.clear_contour();
  for (const auto& pt : contour.points()) {
    pt.ToProto(laser_measurement_.add_contour());
  }
  return *this;
}

LaserMeasurementBuilder& LaserMeasurementBuilder::set_cluster_measurement(
    LaserMeasurementProto::ClusterMeasurement&& cluster_m) {
  *laser_measurement_.mutable_cluster_measurement() = std::move(cluster_m);
  return *this;
}

LaserMeasurementBuilder& LaserMeasurementBuilder::set_icp_measurement(
    LaserMeasurementProto::IcpMeasurement&& icp_m) {
  *laser_measurement_.mutable_icp_measurement() = std::move(icp_m);
  return *this;
}

LaserMeasurementBuilder&
LaserMeasurementBuilder::AdjustClusterMeasurementGivenContour() {
  laser_measurement_.clear_cluster_measurement();
  for (int i = 0; i < laser_measurement_.contour_size(); ++i) {
    *laser_measurement_.mutable_cluster_measurement()
         ->add_obstacle_info()
         ->mutable_center() = laser_measurement_.contour(i);
  }
  return *this;
}

LaserMeasurementProto LaserMeasurementBuilder::Build() {
  return laser_measurement_;
}

RadarMeasurementBuilder::RadarMeasurementBuilder() {
  radar_measurement_.set_id(0);
  radar_measurement_.set_type(ROT_VEHICLE);
  radar_measurement_.mutable_pos()->set_x(1.0);
  radar_measurement_.mutable_pos()->set_y(2.0);
  radar_measurement_.set_yaw(2.0);
  radar_measurement_.mutable_vel()->set_x(3.0);
  radar_measurement_.mutable_vel()->set_y(4.0);

  Mat2d pos_cov;
  pos_cov << 1.0, 0.0, 0.0, 1.0;
  Mat2dToProto(pos_cov, radar_measurement_.mutable_pos_cov());

  Mat2d vel_cov;
  vel_cov << 1.0, 0.0, 0.0, 1.0;
  Mat2dToProto(vel_cov, radar_measurement_.mutable_vel_cov());
}

RadarMeasurementBuilder& RadarMeasurementBuilder::set_id(int id) {
  radar_measurement_.set_id(id);
  return *this;
}

RadarMeasurementBuilder& RadarMeasurementBuilder::set_pos(Vec2d pos) {
  pos.ToProto(radar_measurement_.mutable_pos());
  return *this;
}

RadarMeasurementProto RadarMeasurementBuilder::Build() {
  return radar_measurement_;
}

Camera3dMeasurementBuilder::Camera3dMeasurementBuilder() {
  camera_3d_measurement_.mutable_pos()->set_x(0.0);
  camera_3d_measurement_.mutable_pos()->set_y(0.0);
  camera_3d_measurement_.set_heading(0.0);
  camera_3d_measurement_.set_width(1.0);
  camera_3d_measurement_.set_length(1.0);
  camera_3d_measurement_.set_height(1.0);
}

Camera3dMeasurementBuilder& Camera3dMeasurementBuilder::set_pos(
    const Vec2d& pos) {
  SetVec3dProto({pos, 0.0}, camera_3d_measurement_.mutable_pos());
  return *this;
}

Camera3dMeasurementProto Camera3dMeasurementBuilder::Build() {
  return camera_3d_measurement_;
}

MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       LaserMeasurementProto&& laser_m) {
  MeasurementProto measurement_proto;
  measurement_proto.set_timestamp(timestamp);
  measurement_proto.set_type(type);
  measurement_proto.set_type_source(type_source);
  *measurement_proto.mutable_laser_measurement() = std::move(laser_m);
  return measurement_proto;
}

MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       RadarMeasurementProto&& radar_m) {
  MeasurementProto measurement_proto;
  measurement_proto.set_timestamp(timestamp);
  measurement_proto.set_type(type);
  measurement_proto.set_type_source(type_source);
  *measurement_proto.mutable_radar_measurement() = std::move(radar_m);
  return measurement_proto;
}
MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       Camera3dMeasurementProto&& camera_m) {
  MeasurementProto measurement_proto;
  measurement_proto.set_timestamp(timestamp);
  measurement_proto.set_type(type);
  measurement_proto.set_type_source(type_source);
  *measurement_proto.mutable_camera3d_measurement() = std::move(camera_m);
  return measurement_proto;
}
}  // namespace qcraft
