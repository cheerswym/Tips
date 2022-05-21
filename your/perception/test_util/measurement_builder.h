#ifndef ONBOARD_PERCEPTION_TEST_UTIL_MEASUREMENT_BUILDER_H_
#define ONBOARD_PERCEPTION_TEST_UTIL_MEASUREMENT_BUILDER_H_

#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/radar.pb.h"

namespace qcraft {

class ClusterMeasurementBuilder {
 public:
  ClusterMeasurementBuilder();

  ClusterMeasurementBuilder& set_height(double height);
  ClusterMeasurementBuilder& set_obstacle_info(
      const std::vector<LaserMeasurementProto::ObstacleInfo>& obstacle_info);
  LaserMeasurementProto::ClusterMeasurement Build();

 private:
  LaserMeasurementProto::ClusterMeasurement cluster_measurement_;
};

class ObstacleInfoBuilder {
 public:
  ObstacleInfoBuilder();
  ObstacleInfoBuilder& set_center(const Vec2d& obstacle_center);
  ObstacleInfoBuilder& set_min_z(double min_z);
  ObstacleInfoBuilder& set_max_z(double max_z);
  LaserMeasurementProto::ObstacleInfo Build();

 private:
  LaserMeasurementProto::ObstacleInfo obstacle_info_;
};

class IcpMeasurementBuilder {
 public:
  IcpMeasurementBuilder();

  IcpMeasurementBuilder& set_vel(const Vec2d& vel);

  LaserMeasurementProto::IcpMeasurement Build();

 private:
  LaserMeasurementProto::IcpMeasurement icp_measurement_;
};

// Creates a laser measurement.
class LaserMeasurementBuilder {
 public:
  LaserMeasurementBuilder();

  LaserMeasurementBuilder& set_contour(const Polygon2d& contour);

  LaserMeasurementBuilder& set_cluster_measurement(
      LaserMeasurementProto::ClusterMeasurement&& cluster_m);

  LaserMeasurementBuilder& set_icp_measurement(
      LaserMeasurementProto::IcpMeasurement&& icp_m);

  // Cluster measurement correlates with contour but sometimes user need
  // obstacle centers but don't care how obstacle centers are distributed in the
  // contour. This function provides the functionality to populate only vertices
  // of the contour polygon to make sure obstacle centers make sense to contour.
  // Note that the obstacles are not strictly fitting in the same grid map.
  LaserMeasurementBuilder& AdjustClusterMeasurementGivenContour();

  LaserMeasurementProto Build();

 private:
  LaserMeasurementProto laser_measurement_;
};

class RadarMeasurementBuilder {
 public:
  RadarMeasurementBuilder();

  RadarMeasurementBuilder& set_id(int id);

  RadarMeasurementBuilder& set_pos(Vec2d pos);

  RadarMeasurementProto Build();

 private:
  RadarMeasurementProto radar_measurement_;
};

class Camera3dMeasurementBuilder {
 public:
  Camera3dMeasurementBuilder();

  Camera3dMeasurementBuilder& set_pos(const Vec2d& pos);

  Camera3dMeasurementProto Build();

 private:
  Camera3dMeasurementProto camera_3d_measurement_;
};

MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       LaserMeasurementProto&& laser_m);

MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       RadarMeasurementProto&& radar_m);

MeasurementProto BuildMeasurementProto(double timestamp, MeasurementType type,
                                       MeasurementTypeSource type_source,
                                       Camera3dMeasurementProto&& camera_m);
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TEST_UTIL_MEASUREMENT_BUILDER_H_
