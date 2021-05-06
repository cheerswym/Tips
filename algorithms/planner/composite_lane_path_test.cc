#include "onboard/planner/composite_lane_path.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
DECLARE_bool(allow_txt_semantic_map);

namespace qcraft {
namespace planner {
namespace {

const auto set_map = []() {
  // TODO(luzou): the map files on CI machine is very old, they don't have
  // binary files. Remove this hack once the files were updated
  FLAGS_allow_txt_semantic_map = true;
  SetMap("global_legacy");
  return 0;
}();

TEST(CompositeLanePathTest, Length) {
  constexpr double kOverlapLength = 0.1;  // m.

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const mapping::LanePath lane_path0(&semantic_map_manager,
                                     {294, 295, 296, 297}, 0.3, 0.7);
  const mapping::LanePath lane_path1(&semantic_map_manager,
                                     {301, 302, 303, 304}, 0.3, 0.7);
  const CompositeLanePath composite_lane_path(
      {lane_path0, lane_path1},
      CompositeLanePath::TransitionInfo{kOverlapLength, 0.5});
  EXPECT_EQ(composite_lane_path.length(),
            lane_path0.length() + lane_path1.length() - kOverlapLength);

  double length = 0.0;
  int count = 0;
  for (const auto &lane : lane_path0) {
    length += (lane.end_fraction - lane.start_fraction) *
              semantic_map_manager.GetLaneLengthOrDie(lane.lane_id);
    ++count;
  }
  EXPECT_EQ(length, lane_path0.length());
  EXPECT_EQ(count, 4);

  length = 0.0;
  count = 0;
  for (const auto &lane : lane_path1) {
    length += (lane.end_fraction - lane.start_fraction) *
              semantic_map_manager.GetLaneLengthOrDie(lane.lane_id);
    ++count;
  }
  EXPECT_EQ(length, lane_path1.length());
  EXPECT_EQ(count, 4);

  length = 0.0;
  count = 0;
  for (const auto &lane : composite_lane_path) {
    length += (lane.end_fraction - lane.start_fraction) *
              semantic_map_manager.GetLaneLengthOrDie(lane.lane_id);
    ++count;
  }
  EXPECT_EQ(length, composite_lane_path.length());
  EXPECT_EQ(count, 8);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
