#include "onboard/vis/qview_data_serializer.h"

#include <map>
#include <string>
#include <utility>

namespace qcraft {

QviewDataSerializer::QviewDataSerializer(const QViewElements &elements)
    : writer_(rapidjson::Writer<rapidjson::StringBuffer>(sb_)),
      qview_elements_(elements) {
  writer_.SetMaxDecimalPlaces(3);
  // Start json serialize.
  writer_.StartObject();
}

void QviewDataSerializer::SerializeTrafficLights() {
  writer_.Key("traffic_lights");
  if (qview_elements_.traffic_lights().size() != 0) {
    writer_.StartArray();
    for (const auto &traffic_light : qview_elements_.traffic_lights()) {
      writer_.StartArray();
      writer_.Int(traffic_light.color());
      writer_.Int(traffic_light.shape());
      writer_.Bool(traffic_light.flashing());
      writer_.EndArray();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializeSpeedInfo() {
  writer_.Key("speed_info");
  writer_.StartArray();
  writer_.StartArray();
  writer_.Double(qview_elements_.speed_info().speed());
  writer_.Double(qview_elements_.speed_info().accel());
  writer_.Int(qview_elements_.speed_info().speed_limit());
  writer_.EndArray();
  writer_.EndArray();
}

void QviewDataSerializer::SerializeControlInfo() {
  writer_.Key("control_info");
  writer_.StartArray();
  writer_.StartArray();
  writer_.Double(qview_elements_.control_info().steering());
  writer_.Double(qview_elements_.control_info().braking());
  writer_.Double(qview_elements_.control_info().throttle());
  writer_.Bool(qview_elements_.control_info().left_blinker());
  writer_.Bool(qview_elements_.control_info().right_blinker());
  writer_.Int64(qview_elements_.control_info().blinker_reason());
  writer_.EndArray();
  writer_.EndArray();
}

void QviewDataSerializer::SerializePose(const VehiclePose &vehicle_pose) {
  writer_.Key("pose");
  writer_.StartArray();
  writer_.StartArray();
  writer_.Double(vehicle_pose.x);
  writer_.Double(vehicle_pose.y);
  writer_.Double(vehicle_pose.yaw);
  writer_.EndArray();
  writer_.EndArray();
}

void QviewDataSerializer::SerializeStationInfo(
    const std::shared_ptr<const HmiContentProto> &hmi_content) {
  if (hmi_content) {
    int next_station = hmi_content->route_content().next_station();
    writer_.Key("next_station");
    writer_.Int(next_station);
    writer_.Key("next_station_distance");
    writer_.Double(hmi_content->route_content().distance_to_next_station());
    constexpr double kApproachingStationDistanceThreshold = 30.0;  // m.
    if (hmi_content->route_content().distance_to_next_station() <
        kApproachingStationDistanceThreshold) {
      writer_.Key("approaching_station");
      writer_.Int(next_station);
    }
    writer_.Key("station_names");
    writer_.StartArray();
    for (const std::string &station_name :
         hmi_content->route_content().stations_list()) {
      writer_.String(station_name.c_str());
    }
    writer_.EndArray();
    writer_.Key("message");
    writer_.Int(static_cast<int>(hmi_content->message()));
  }
}

void QviewDataSerializer::SerializeGlosaLights() {
  writer_.Key("glosa_lights");

  if (qview_elements_.glosa_lights_size() != 0) {
    writer_.StartArray();
    for (const auto &glosa_light : qview_elements_.glosa_lights()) {
      writer_.StartArray();
      writer_.Int(glosa_light.color());
      writer_.Int(glosa_light.shape());
      writer_.Bool(glosa_light.flashing());
      writer_.Int(glosa_light.timing());
      writer_.EndArray();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializeConstructions() {
  writer_.Key("constructions");
  if (qview_elements_.constructions_size() != 0) {
    writer_.StartArray();
    for (const auto &construction : qview_elements_.constructions()) {
      writer_.StartArray();
      writer_.Double(construction.x());
      writer_.Double(construction.y());
      writer_.EndArray();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializeBusStations() {
  // bus stations
  writer_.Key("bus_stations");
  if (qview_elements_.bus_stations_size() != 0) {
    writer_.StartArray();
    for (const auto &bus_station : qview_elements_.bus_stations()) {
      writer_.StartArray();
      writer_.Int(bus_station.x_cm());
      writer_.Int(bus_station.y_cm());
      writer_.Int(bus_station.z_cm());
      writer_.Double(bus_station.heading());
      writer_.EndArray();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializePickUpPoint() {
  // pick up points
  writer_.Key("pick_up_points");
  if (qview_elements_.pick_up_points_size() != 0) {
    writer_.StartArray();
    for (const auto &pick_up_point : qview_elements_.pick_up_points()) {
      writer_.StartArray();
      writer_.String(pick_up_point.id().c_str());
      writer_.Double(pick_up_point.heading());
      writer_.Int(pick_up_point.x_cm());
      writer_.Int(pick_up_point.y_cm());
      writer_.EndArray();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializePublishDuration(int64 publish_duration) {
  writer_.Key("publish_duration");
  writer_.StartArray();
  writer_.StartArray();
  writer_.Int64(publish_duration);
  writer_.EndArray();
  writer_.EndArray();
}

void QviewDataSerializer::SerializeV2xNotifications() {
  writer_.Key("v2x_notifications");
  if (qview_elements_.v2x_notifications_size() != 0) {
    writer_.StartArray();
    for (const auto &notification : qview_elements_.v2x_notifications()) {
      writer_.StartObject();
      writer_.Key("type");
      writer_.Int(notification.type());
      writer_.Key("sub_type");
      writer_.Int(notification.sub_type());
      writer_.Key("alarm_level");
      writer_.Int(notification.alarm_level());
      writer_.Key("direction");
      writer_.Int(notification.direction());
      writer_.Key("distance");
      writer_.Int(notification.distance_m());
      if (notification.data_size() != 0) {
        writer_.Key("data");
        writer_.StartArray();
        for (int i = 0; i < notification.data_size(); i++) {
          writer_.Int(notification.data(i));
        }
        writer_.EndArray();
      }
      writer_.EndObject();
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializer::SerializeRouteInfo(const QviewRouteInfo &route_info) {
  writer_.Key("route_info");
  writer_.StartObject();
  writer_.Key("uid");
  writer_.String(route_info.uid_.c_str());
  writer_.Key("city");
  writer_.String(route_info.city_.c_str());
  writer_.Key("name");
  writer_.String(route_info.name_.c_str());
  writer_.Key("is_loop");
  writer_.Bool(route_info.is_loop_);
  writer_.Key("service_start_time");
  writer_.String(route_info.service_start_time_.c_str());
  writer_.Key("service_stop_time");
  writer_.String(route_info.service_stop_time_.c_str());
  writer_.Key("service_break_time");
  writer_.String(route_info.service_break_time_.c_str());
  writer_.Key("update_timestamp");
  writer_.Int64(route_info.update_timestamp_);
  writer_.Key("stations");
  writer_.StartArray();
  for (const auto &station : route_info.stations_) {
    writer_.StartObject();
    writer_.Key("id");
    writer_.Int(station.id_);
    writer_.Key("city");
    writer_.String(station.city_.c_str());
    writer_.Key("name");
    writer_.String(station.name_.c_str());
    writer_.Key("longitude");
    writer_.Double(station.longtitude_);
    writer_.Key("latitude");
    writer_.Double(station.latitude_);
    writer_.Key("altitude");
    writer_.Double(station.altitude_);
    writer_.Key("heading");
    writer_.Double(station.heading_);
    writer_.EndObject();
  }
  writer_.EndArray();
  writer_.EndObject();
}

void QviewDataSerializer::SerializeString(const char *key, const char *value) {
  writer_.Key(key);
  writer_.String(value);
}

void QviewDataSerializer::SerializeInt64(const char *key, int64 value) {
  writer_.Key(key);
  writer_.Int64(value);
}

void QviewDataSerializer::SerializeInt(const char *key, int value) {
  writer_.Key(key);
  writer_.Int(value);
}

std::string QviewDataSerializer::Dump() {
  // End json serialize.
  writer_.EndObject();
  return std::move(sb_.GetString());
}

void QviewDataSerializerV1::SerializeObjects(int64 update_time_obj,
                                             int64 update_time_v2x) {
  writer_.Key("objects");
  writer_.StartArray();
  for (const auto &object : qview_elements_.objects()) {
    writer_.StartArray();
    writer_.String(QViewObjectProto::Type_Name(object.type()).c_str());
    writer_.Double(object.bounding_box().length());
    writer_.Double(object.bounding_box().width());
    writer_.Double(object.bounding_box().x());
    writer_.Double(object.bounding_box().y());
    writer_.Double(object.bounding_box().heading());
    writer_.String(
        QViewObjectProto::DataSource_Name(object.data_src()).c_str());
    writer_.Int(std::stoi(object.id()));
    writer_.EndArray();
  }
  writer_.EndArray();
}

void QviewDataSerializerV1::SerializeMapPolygons() {
  writer_.Key("map_polygon");
  if (qview_elements_.polygons().size() != 0) {
    SerializePolygons(qview_elements_.polygons());
  } else {
    writer_.Null();
  }
}

template <typename RepeatedPolygonType>
void QviewDataSerializerV1::SerializePolygons(
    const RepeatedPolygonType &polygons) {
  writer_.StartArray();
  for (const auto &polygon : polygons) {
    std::vector<std::pair<int, int>> points;
    points.reserve(polygon.points_size());
    // Store the offset for all points except the first one to reduce the JSON
    // string size.
    const auto &first_point = polygon.points(0);
    for (int i = 0; i < polygon.points_size(); ++i) {
      const auto &point = polygon.points(i);
      if (i == 0) {
        points.emplace_back(point.x_cm(), point.y_cm());
      } else {
        points.emplace_back(point.x_cm() - first_point.x_cm(),
                            point.y_cm() - first_point.y_cm());
      }
    }
    writer_.StartArray();
    writer_.String(QViewPolygonProto::Color_Name(polygon.color()).c_str());
    writer_.Double(polygon.height_cm());

    writer_.StartArray();
    for (const auto &point : points) {
      writer_.StartArray();
      writer_.Int(point.first);
      writer_.Int(point.second);
      writer_.EndArray();
    }
    writer_.EndArray();
    writer_.String(polygon.polygon_id().c_str());
    writer_.EndArray();
  }
  writer_.EndArray();
}

void QviewDataSerializerV1::SerializeTrajectoryPolygons(
    const std::vector<QViewPolygonProto> &trajectory_polygons) {
  writer_.Key("polygon");
  if (trajectory_polygons.size() != 0) {
    SerializePolygons(trajectory_polygons);
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV1::SerializePedPoints() {
  writer_.Key("ped_points");
  if (qview_elements_.ped_points_size() != 0) {
    writer_.StartArray();
    for (const auto &point : qview_elements_.ped_points()) {
      writer_.Int(RoundToInt(point.x() * 100.0));
      writer_.Int(RoundToInt(point.y() * 100.0));
      writer_.Int(RoundToInt(point.z() * 100.0));
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV1::SerializeCycPoints() {
  writer_.Key("cyc_points");
  if (qview_elements_.cyc_points_size() != 0) {
    writer_.StartArray();
    for (const auto &point : qview_elements_.cyc_points()) {
      writer_.Int(RoundToInt(point.x() * 100.0));
      writer_.Int(RoundToInt(point.y() * 100.0));
      writer_.Int(RoundToInt(point.z() * 100.0));
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV2::SerializeObjects(int64 update_time_obj,
                                             int64 update_time_v2x) {
  // reconstruct the objects to related map list -> optimize performance in
  // frontend page
  std::map<std::string, std::map<std::string, QViewObjectProto>> objects_lists;
  for (const auto &object : qview_elements_.objects()) {
    std::string object_hash_key;
    if (object.type() == QViewObjectProto::VEHICLE ||
        object.type() == QViewObjectProto::CYCLIST ||
        object.type() == QViewObjectProto::PEDESTRIAN) {
      object_hash_key = QViewObjectProto::Type_Name(object.type()) +
                        QViewObjectProto::DataSource_Name(object.data_src());
    } else {
      object_hash_key = QViewObjectProto::Type_Name(object.type());
    }
    objects_lists[object_hash_key][object_hash_key + object.id()] = object;
  }

  writer_.Key("objects");

  writer_.StartObject();
  writer_.Key("last_update_time_obj");
  writer_.Int64(update_time_obj / 1000);
  writer_.Key("last_update_time_v2x");
  writer_.Int64(update_time_v2x / 1000);

  writer_.Key("data");
  writer_.StartObject();
  for (const auto &object_list : objects_lists) {
    writer_.Key(object_list.first.c_str());
    writer_.StartObject();
    for (const auto &object : object_list.second) {
      writer_.Key(object.first.c_str());
      writer_.StartArray();
      writer_.Double(object.second.bounding_box().length());
      writer_.Double(object.second.bounding_box().width());
      writer_.Double(object.second.bounding_box().x());
      writer_.Double(object.second.bounding_box().y());
      writer_.Double(object.second.bounding_box().heading());
      writer_.EndArray();
    }
    writer_.EndObject();
  }
  writer_.EndObject();

  writer_.EndObject();
}

void QviewDataSerializerV2::SerializeMapPolygons() {
  writer_.Key("map_polygon");
  if (qview_elements_.polygons().size() != 0) {
    writer_.StartObject();
    for (const auto &polygon : qview_elements_.polygons()) {
      writer_.Key(polygon.polygon_id().c_str());
      writer_.StartArray();
      writer_.String(QViewPolygonProto::Color_Name(polygon.color()).c_str());
      writer_.Int(polygon.height_cm());
      writer_.StartArray();
      for (const auto &point : polygon.points()) {
        writer_.StartArray();
        writer_.Double(-0.01 * point.y_cm());
        writer_.Double(-0.01 * point.x_cm());
        writer_.EndArray();
      }
      writer_.EndArray();
      writer_.String(polygon.polygon_id().c_str());
      writer_.EndArray();
    }
    writer_.EndObject();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV2::SerializeTrajectoryPolygons(
    const std::vector<QViewPolygonProto> &trajectory_polygons) {
  writer_.Key("polygon");
  if (trajectory_polygons.size() != 0) {
    // migration the computer process from frontend to onboard
    // optimize frontend page showing performance
    std::vector<double> points_polygon;
    for (const auto &polygon : trajectory_polygons) {
      std::vector<std::pair<double, double>> points;
      points.reserve(polygon.points_size());
      for (const auto &point : polygon.points()) {
        points.emplace_back(std::pair<double, double>(
            -0.01 * point.y_cm(), -0.01 * point.x_cm() + 0.02));
      }

      std::vector<int> index_list = {0, 1, 2, 2, 3, 0};
      for (auto i : index_list) {
        points_polygon.emplace_back(points[i].first);
        points_polygon.emplace_back(points[i].second);
        points_polygon.emplace_back(0);
      }
    }

    writer_.StartArray();
    writer_.StartArray();
    writer_.StartArray();
    for (const auto &point : points_polygon) {
      writer_.Double(point);
    }
    writer_.EndArray();
    writer_.EndArray();
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV2::SerializePedPoints() {
  writer_.Key("ped_points");
  if (qview_elements_.ped_points_size() != 0) {
    writer_.StartArray();
    for (const auto &point : qview_elements_.ped_points()) {
      writer_.Double(point.x());
      writer_.Double(point.y());
      writer_.Double(point.z());
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

void QviewDataSerializerV2::SerializeCycPoints() {
  writer_.Key("cyc_points");
  if (qview_elements_.cyc_points_size() != 0) {
    writer_.StartArray();
    for (const auto &point : qview_elements_.cyc_points()) {
      writer_.Double(point.x());
      writer_.Double(point.y());
      writer_.Double(point.z());
    }
    writer_.EndArray();
  } else {
    writer_.Null();
  }
}

}  // namespace qcraft
