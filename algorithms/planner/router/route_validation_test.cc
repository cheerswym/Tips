#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/router/route.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteValidationTest, ValidateRoutes) {
  std::map<std::string, std::vector<std::string>> map_routes;

  for (const auto &kv : map_routes) {
    const std::string &map_name = kv.first;
    SetMap(map_name);
    LOG(INFO) << "Map: " << map_name;
    for (const std::string &route_name : kv.second) {
      const std::string route_file_name =
          "onboard/planner/testdata/" + route_name;
      LOG(INFO) << "Map: " << map_name << " route: " << route_file_name;
      RouteProto route_proto;
      const bool success =
          file_util::TextFileToProto(route_file_name, &route_proto);
      EXPECT_TRUE(success) << "Failed to load route file " << route_file_name;
    }
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
