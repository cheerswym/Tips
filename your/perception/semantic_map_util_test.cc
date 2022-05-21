#include "onboard/perception/semantic_map_util.h"

#include <set>

#include "gtest/gtest.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
// BANDAID(dong): Temporary disable this test because CI only support dojo map.
// However, there is no perception zone on dojo map by now.
TEST(SemanticMapUtilTest, DISABLED_FetchAllPerceptionZones) {
  SetMap("suzhou");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const auto coordinate_converter = CoordinateConverter::FromLocale();
  const auto& barrier_zones =
      perception_semantic_map_util::CollectNearPerceptionZones(
          semantic_map_manager, mapping::PerceptionZoneProto::BARRIER,
          VehiclePose(0, 0, 0, 0, 0, 0), coordinate_converter);
  QCHECK_EQ(0, barrier_zones.size());
  const auto& ignorance_zones =
      perception_semantic_map_util::CollectNearPerceptionZones(
          semantic_map_manager, mapping::PerceptionZoneProto::IGNORANCE,
          VehiclePose(5796.2, 13512.2, 0, 0, 0, 0), coordinate_converter);
  QCHECK_EQ(3, ignorance_zones.size());
}

}  // namespace qcraft
