#include "onboard/vis/qview_route_info.h"

#include "onboard/vis/json_lookahead_parser.h"
#include "rapidjson/reader.h"

namespace qcraft {

QviewRouteInfo::QviewRouteInfo() { Clear(); }

/*
  parsing data format example
  "id":25,
  "uid":"QR00001",
  "city":"苏州",
  "geoMap":"suzhou",
  "name":"轻舟苏1线",
  "direction":"高铁苏州北站东-高铁苏州北站东",
  "isPrivate":true,
  "originalPrice":0,
  "discountPrice":0,
  "status":1,
  "isLoop":true,
  "operationMode":0,
  "stations":[
  {
      "id":62,
      "uid":"chengyangluxi",
      "city":"苏州",
      "name":"澄阳路西",
      "direction":"chengyangluxi",
      "type":0,
      "data":{
          "longitude":"120.6433520000",
          "latitude":"31.4169350000",
          "altitude":"11.630000"
      },
      "gpsCoordinate":"31.4169350000,120.6433520000,11.630000",
      "tencentCoordinate":"31.4148260000,120.6475690000",
      "updateUserId":"00000000-0000-0000-0000-000000000000",
      "createTime":"2021-11-11T19:39:08.000+0800",
      "updateTime":"2022-03-11T10:48:14.000+0800",
      "version":2
  }]
*/
bool QviewRouteInfo::Parse(const std::string& route_info) {
  LookaheadParser parser(const_cast<char*>(route_info.c_str()));
  Clear();

  if (parser.PeekType() != rapidjson::kObjectType) {
    return false;
  }

  parser.EnterObject();
  while (const char* key = parser.NextObjectKey()) {
    if (0 == strcmp(key, "uid")) {
      uid_ = parser.GetString();
    } else if (0 == strcmp(key, "city")) {
      city_ = parser.GetString();
    } else if (0 == strcmp(key, "name")) {
      name_ = parser.GetString();
    } else if (0 == strcmp(key, "isLoop")) {
      is_loop_ = parser.GetBool();
    } else if (0 == strcmp(key, "serviceStartTime")) {
      service_start_time_ = parser.GetString();
    } else if (0 == strcmp(key, "serviceStopTime")) {
      service_stop_time_ = parser.GetString();
    } else if (0 == strcmp(key, "serviceBreakTime")) {
      service_break_time_ = parser.GetString();
    } else if (0 == strcmp(key, "stations")) {
      parser.EnterArray();
      while (parser.NextArrayValue()) {
        parser.EnterObject();
        // parser station
        Station station;
        while (const char* key_station = parser.NextObjectKey()) {
          if (0 == strcmp(key_station, "id")) {
            station.id_ = parser.GetInt();
          } else if (0 == strcmp(key_station, "city")) {
            station.city_ = parser.GetString();
          } else if (0 == strcmp(key_station, "name")) {
            station.name_ = parser.GetString();
          } else if (0 == strcmp(key_station, "data")) {
            parser.EnterObject();
            // parser data
            while (const char* key_data = parser.NextObjectKey()) {
              if (0 == strcmp(key_data, "longitude")) {
                station.longtitude_ = std::atof(parser.GetString());
              } else if (0 == strcmp(key_data, "latitude")) {
                station.latitude_ = std::atof(parser.GetString());
              } else if (0 == strcmp(key_data, "altitude")) {
                station.heading_ = std::atof(parser.GetString());
              } else {
                parser.SkipValue();
              }
            }
          } else {
            parser.SkipValue();
          }
        }
        stations_.push_back(station);
      }
    } else {
      parser.SkipValue();
    }
  }

  has_route_info_ = parser.IsValid();

  return has_route_info_;
}

void QviewRouteInfo::Clear() {
  has_route_info_ = false;
  uid_.clear();
  city_.clear();
  name_.clear();
  is_loop_ = false;
  service_start_time_.clear();
  service_stop_time_.clear();
  service_break_time_.clear();
  update_timestamp_ = -1;
  stations_.clear();
}

}  // namespace qcraft
