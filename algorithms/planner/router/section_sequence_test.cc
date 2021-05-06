#include <regex>
#include <unordered_map>

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_searcher.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {

using testing::ElementsAreArray;

TEST(SectionSequenceTest, ChoseBestSectionSequence1) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  // section sequence test 1
  std::vector<mapping::ElementId> expected_section_ids = {
      2929, 2928, 3034, 3033, 3041, 3045, 3047, 2885, 3061, 2910, 3053};

  mapping::LanePoint origin(95, 0.5);
  mapping::LanePoint destination(142, 0.5);
  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  EXPECT_THAT(section_ids, ElementsAreArray(expected_section_ids));
}

TEST(SectionSequenceTest, ChoseBestSectionSequence2) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  // section sequence test 2
  std::vector<mapping::ElementId> expected_section_ids = {
      3034, 2715, 2950, 3032, 2702, 3035, 3046, 3040, 3042, 2935, 3029};

  mapping::LanePoint origin(2, 0.5);
  mapping::LanePoint destination1(952, 0.5);
  mapping::LanePoint destination2(68, 0.5);
  std::vector<mapping::LanePoint> destinations;

  destinations.emplace_back(destination1);
  destinations.emplace_back(destination2);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  EXPECT_THAT(section_ids, ElementsAreArray(expected_section_ids));
}

TEST(SectionSequenceTest, ChoseBestSectionSequence3) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  // section sequence test 3
  std::vector<mapping::ElementId> expected_section_ids = {
      2798, 2809, 2929, 2928, 3034, 3033, 3041, 3045, 3047, 2903, 3057, 3056};

  mapping::LanePoint origin(643, 0.5);
  mapping::LanePoint destination(1592, 0.5);
  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  EXPECT_THAT(section_ids, ElementsAreArray(expected_section_ids));
}

TEST(SectionSequenceTest, ChoseBestSectionSequence4) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  // section sequence test 4
  std::vector<mapping::ElementId> expected_section_ids = {
      2798, 2809, 2929, 2928, 3034, 3033, 3041, 3045, 3047, 2903, 3057,
      3056, 2623, 2605, 2599, 2604, 2602, 2603, 2601, 2606, 2608, 3058};

  mapping::LanePoint origin(643, 0.5);
  mapping::LanePoint destination(1771, 0.5);
  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  EXPECT_THAT(section_ids, ElementsAreArray(expected_section_ids));
}

TEST(SectionSequenceTest, ChoseBestSectionSequence5) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  // section sequence test 5
  std::vector<mapping::ElementId> expected_section_ids = {
      3034, 2719, 2946, 3042, 2933, 3027, 3026, 3023, 2945, 3034};

  mapping::LanePoint origin(3, 0.5);
  mapping::LanePoint destination(2448, 0.2);
  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  EXPECT_THAT(section_ids, ElementsAreArray(expected_section_ids));
}

TEST(RouteSectionToProtoTest, ToProto) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RouteSearcher route_searcher(&semantic_map_manager);

  std::vector<mapping::ElementId> expected_section_ids = {3034, 3033, 3041,
                                                          3045, 3047};

  mapping::LanePoint origin(2448, 0.5);
  mapping::LanePoint destination(54, 0.2);
  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);
  CompositeLanePath rlp;
  bool success =
      route_searcher.SearchForRoutePathToLanePoints(origin, destinations, &rlp);
  if (!success) {
    EXPECT_TRUE(false);
  }
  RouteSectionSequence section_seq(rlp, &semantic_map_manager);
  RouteSectionSequenceProto actual_seq_proto;
  section_seq.ToProto(&actual_seq_proto);

  RouteSectionSequenceProto section_seq_proto;
  section_seq_proto.set_start_fraction(0.5);
  section_seq_proto.set_end_fraction(0.2);
  for (const auto &section_id : expected_section_ids) {
    section_seq_proto.add_section_id(section_id);
  }

  EXPECT_THAT(section_seq_proto, ProtoEq(actual_seq_proto));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
