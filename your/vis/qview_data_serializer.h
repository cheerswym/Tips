#ifndef ONBOARD_VIS_QVIEW_DATA_SERIALIZER_H_
#define ONBOARD_VIS_QVIEW_DATA_SERIALIZER_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/base/integral_types.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/qview.pb.h"
#include "onboard/vis/qview_route_info.h"
#include "rapidjson/writer.h"

namespace qcraft {
class QviewDataSerializer {
 public:
  // Start serialing
  explicit QviewDataSerializer(const QViewElements &elements);

  // serialize objects and polygons
  // They have different json format between V1 and V2
  virtual void SerializeObjects(int64 update_time_obj,
                                int64 update_time_v2x) = 0;
  virtual void SerializeMapPolygons() = 0;
  virtual void SerializeTrajectoryPolygons(
      const std::vector<QViewPolygonProto> &trajectory_polygons) = 0;

  void SerializeTrafficLights();
  void SerializeSpeedInfo();
  void SerializeControlInfo();
  void SerializePose(const VehiclePose &vehicle_pose);
  void SerializeStationInfo(
      const std::shared_ptr<const HmiContentProto> &hmi_content);
  void SerializeGlosaLights();
  void SerializeConstructions();
  void SerializeBusStations();
  void SerializePickUpPoint();
  void SerializePublishDuration(int64 publish_duration);

  // serialize ped points and cyc points
  // They have different json format between V1 and V2
  virtual void SerializePedPoints() = 0;
  virtual void SerializeCycPoints() = 0;

  void SerializeV2xNotifications();
  void SerializeRouteInfo(const QviewRouteInfo &route_info);

  // serialize top level basic type value
  void SerializeString(const char *key, const char *value);
  void SerializeInt64(const char *key, int64 value);
  void SerializeInt(const char *key, int value);

  // End serializing and dump string result
  std::string Dump();

 protected:
  rapidjson::StringBuffer sb_;
  rapidjson::Writer<rapidjson::StringBuffer> writer_;

  const QViewElements &qview_elements_;
};

class QviewDataSerializerV1 : public QviewDataSerializer {
 public:
  explicit QviewDataSerializerV1(const QViewElements &elements)
      : QviewDataSerializer(elements) {}
  virtual ~QviewDataSerializerV1() {}
  virtual void SerializeObjects(int64 update_time_obj, int64 update_time_v2x);
  virtual void SerializeMapPolygons();
  virtual void SerializeTrajectoryPolygons(
      const std::vector<QViewPolygonProto> &trajectory_polygons);
  virtual void SerializePedPoints();
  virtual void SerializeCycPoints();

 private:
  template <typename RepeatedPolygonType>
  void SerializePolygons(const RepeatedPolygonType &polygons);
};

class QviewDataSerializerV2 : public QviewDataSerializer {
 public:
  explicit QviewDataSerializerV2(const QViewElements &elements)
      : QviewDataSerializer(elements) {}
  virtual ~QviewDataSerializerV2() {}
  virtual void SerializeObjects(int64 update_time_obj, int64 update_time_v2x);
  virtual void SerializeMapPolygons();
  virtual void SerializeTrajectoryPolygons(
      const std::vector<QViewPolygonProto> &trajectory_polygons);
  virtual void SerializePedPoints();
  virtual void SerializeCycPoints();
};

}  // namespace qcraft

#endif
