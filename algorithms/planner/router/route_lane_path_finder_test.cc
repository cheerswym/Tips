#include "onboard/planner/router/route_lane_path_finder.h"

#include "gtest/gtest.h"
#include "onboard/global/timer.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_searcher.h"

DECLARE_bool(route_allow_solid_line_lane_change);

namespace qcraft::planner {
namespace {

TEST(RouteLanePathFinder, DummyTest1) {
  // TEST 1: Origin is in bus_only and go through bus_only.
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path0(&semantic_map_manager, {3}, 0.5, 1.0);
  const mapping::LanePath lane_path1(&semantic_map_manager, {2448, 1, 34, 2471},
                                     1.0, 1.0);
  const mapping::LanePath lane_path2(&semantic_map_manager,
                                     {2472, 54, 168, 129}, 1.0, 0.5);

  const CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3034};

  const CompositeLanePath::TransitionInfo trans2 = {
      .overlap_length = 0.0, .lc_left = false, .lc_section_id = 3047};

  const CompositeLanePath expected_clp({lane_path0, lane_path1, lane_path2},
                                       {trans1, trans2});

  RouteSearcher route_searcher(&semantic_map_manager);

  mapping::LanePoint origin(3, 0.5);
  mapping::LanePoint destination(129, 0.5);

  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  // Local search.
  RouteSectionSequence route_section_sequence(section_ids, semantic_map_manager,
                                              origin, destination);

  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist = {2472, 3,   21,
                                                               66,   270, 610};
  RouteLanePathFinderInput input;
  input.section_sequence = &route_section_sequence;
  input.lane_id_blacklist = lane_id_blacklist;
  input.start_point = origin;
  input.destination_point = destination;

  auto composite_lane_path =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);

  bool expected = false;
  if (composite_lane_path.ok()) {
    // SendRouteLanePathToCanvas(semantic_map_manager,
    // composite_lane_path.value(),
    //                           "route/local_search_route");
    expected = expected_clp.IsEqual(composite_lane_path.value());
  }

  EXPECT_TRUE(expected);
}

TEST(RouteLanePathFinder, DummyTest2) {
  // TEST 2: Origin and destination are in bus only  .
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path0(&semantic_map_manager, {3}, 0.5, 1.0);
  const mapping::LanePath lane_path1(&semantic_map_manager, {2448, 1, 34, 2471},
                                     1.0, 0.5);
  const mapping::LanePath lane_path2(&semantic_map_manager, {2472}, 0.5, 0.5);

  const CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3034};

  const CompositeLanePath::TransitionInfo trans2 = {
      .overlap_length = 0.0, .lc_left = false, .lc_section_id = 3045};

  const CompositeLanePath expected_clp({lane_path0, lane_path1, lane_path2},
                                       {trans1, trans2});

  RouteSearcher route_searcher(&semantic_map_manager);

  mapping::LanePoint origin(3, 0.5);
  mapping::LanePoint destination(2472, 0.5);

  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  // Local search.
  RouteSectionSequence route_section_sequence(section_ids, semantic_map_manager,
                                              origin, destination);

  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist = {2472, 3,   21,
                                                               66,   270, 610};
  RouteLanePathFinderInput input;
  input.section_sequence = &route_section_sequence;
  input.lane_id_blacklist = lane_id_blacklist;
  input.start_point = origin;
  input.destination_point = destination;

  auto composite_lane_path =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);

  bool expected = false;
  if (composite_lane_path.ok()) {
    // SendRouteLanePathToCanvas(semantic_map_manager,
    // composite_lane_path.value(),
    //                           "route/local_search_route");
    expected = expected_clp.IsEqual(composite_lane_path.value());
  }

  EXPECT_TRUE(expected);
}

TEST(RouteLanePathFinder, DummyTest3) {
  // TEST 3: Origin is behind initial point.
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path0(&semantic_map_manager, {3}, 0.5, 1.0);
  const mapping::LanePath lane_path1(&semantic_map_manager, {2448, 1, 34, 2471},
                                     1.0, 0.5);
  const mapping::LanePath lane_path2(&semantic_map_manager, {2472}, 0.5, 0.5);

  const CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3034};

  const CompositeLanePath::TransitionInfo trans2 = {
      .overlap_length = 0.0, .lc_left = false, .lc_section_id = 3045};

  const CompositeLanePath expected_clp({lane_path0, lane_path1, lane_path2},
                                       {trans1, trans2});

  RouteSearcher route_searcher(&semantic_map_manager);

  mapping::LanePoint initial_point(613, 0.5);
  mapping::LanePoint origin(3, 0.5);
  mapping::LanePoint destination(2472, 0.5);

  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(initial_point, destinations,
                                        section_costs, &section_ids);

  // Local search.
  RouteSectionSequence route_section_sequence(section_ids, semantic_map_manager,
                                              origin, destination);

  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist = {2472, 3,   21,
                                                               66,   270, 610};
  RouteLanePathFinderInput input;
  input.section_sequence = &route_section_sequence;
  input.lane_id_blacklist = lane_id_blacklist;
  input.start_point = origin;
  input.destination_point = destination;

  auto composite_lane_path =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);

  bool expected = false;
  if (composite_lane_path.ok()) {
    // SendRouteLanePathToCanvas(semantic_map_manager,
    // composite_lane_path.value(),
    //                           "route/local_search_route");
    expected = expected_clp.IsEqual(composite_lane_path.value());
  }

  EXPECT_TRUE(expected);
}

TEST(RouteLanePathFinder, DummyTest4) {
  // TEST 4: Need lc early.
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path0(&semantic_map_manager, {236}, 0.0, 1.0);
  const mapping::LanePath lane_path1(&semantic_map_manager, {264, 234}, 1.0,
                                     0.9);

  const CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = true, .lc_section_id = 3067};

  const CompositeLanePath expected_clp({lane_path0, lane_path1}, {trans1});

  RouteSearcher route_searcher(&semantic_map_manager);

  mapping::LanePoint origin(236, 0.0);
  mapping::LanePoint destination(234, 0.9);

  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  // Local search.
  RouteSectionSequence route_section_sequence(section_ids, semantic_map_manager,
                                              origin, destination);

  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist = {2472, 3,   21,
                                                               66,   270, 610};
  RouteLanePathFinderInput input;
  input.section_sequence = &route_section_sequence;
  input.lane_id_blacklist = lane_id_blacklist;
  input.start_point = origin;
  input.destination_point = destination;

  auto composite_lane_path =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);

  bool expected = false;
  if (composite_lane_path.ok()) {
    // SendRouteLanePathToCanvas(semantic_map_manager,
    // composite_lane_path.value(),
    //                           "route/local_search_route");
    expected = expected_clp.IsEqual(composite_lane_path.value());
  }

  EXPECT_TRUE(expected);
}

TEST(RouteLanePathFinder, DummyTest5) {
  // TEST 5: FLAGS_route_allow_solid_line_lane_change test.
  // Load map.
  FLAGS_route_allow_solid_line_lane_change = false;
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path0(&semantic_map_manager, {2470}, 0.2, 1.0);
  const mapping::LanePath lane_path1(&semantic_map_manager, {2471}, 1.0, 1.0);
  const mapping::LanePath lane_path2(&semantic_map_manager,
                                     {2472, 54, 168, 129}, 1.0, 0.9);

  const CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0, .lc_left = false, .lc_section_id = 3045};

  const CompositeLanePath::TransitionInfo trans2 = {
      .overlap_length = 0.0, .lc_left = false, .lc_section_id = 3045};

  const CompositeLanePath expected_clp({lane_path0, lane_path1, lane_path2},
                                       {trans1, trans2});

  RouteSearcher route_searcher(&semantic_map_manager);

  mapping::LanePoint origin(2470, 0.2);
  mapping::LanePoint destination(129, 0.9);

  std::vector<mapping::LanePoint> destinations;
  destinations.emplace_back(destination);

  std::vector<mapping::ElementId> section_ids;
  std::vector<double> section_costs(
      semantic_map_manager.semantic_map().sections_size());
  route_searcher.CreateDefaultSectionCosts(&section_costs);

  route_searcher.SearchForRouteSections(origin, destinations, section_costs,
                                        &section_ids);

  // Local search.
  RouteSectionSequence route_section_sequence(section_ids, semantic_map_manager,
                                              origin, destination);

  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist = {2472, 3,   21,
                                                               66,   270, 610};
  RouteLanePathFinderInput input;
  input.section_sequence = &route_section_sequence;
  input.lane_id_blacklist = lane_id_blacklist;
  input.start_point = origin;
  input.destination_point = destination;

  auto composite_lane_path =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);
  bool expected = false;
  if (composite_lane_path.ok()) {
    // SendRouteLanePathToCanvas(semantic_map_manager,
    // composite_lane_path.value(),
    //                           "route/local_search_route");
    expected = expected_clp.IsEqual(composite_lane_path.value());
  }

  EXPECT_TRUE(expected);
}

TEST(RouteLanePathFinder, InvalidRouteTest) {
  // TEST 6: test if the request must cross solid boundary.
  // Load map.
  FLAGS_route_allow_solid_line_lane_change = false;
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const mapping::LanePath lane_path(&semantic_map_manager, {2448, 1}, 0.5, 1.0);
  RouteSectionSequence route_section_sequence(CompositeLanePath(lane_path),
                                              &semantic_map_manager);

  RouteLanePathFinderInput input{
      .section_sequence = &route_section_sequence,
      .start_point = mapping::LanePoint(938, 0.2),
      .destination_point = mapping::LanePoint(1, 0.8),
      .empty_if_must_cross_solid_boundary = true};

  auto composite_lane_path_or =
      FindRouteLanePathOnSectionSequence(input, semantic_map_manager);
  ASSERT_FALSE(composite_lane_path_or.ok());
  EXPECT_EQ(composite_lane_path_or.status().message(),
            "Must cross solid boundary");
}

TEST(RouteLanePathFinder, MinLaneChangeBlacklistTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  auto start_point = mapping::LanePoint(3, 0.98);
  auto destination_point = mapping::LanePoint(10, 0.1);
  std::vector<mapping::ElementId> black_origins =
      internal::MinLaneChangeBlacklist(semantic_map_manager, start_point, true);
  EXPECT_TRUE(std::find(black_origins.begin(), black_origins.end(),
                        start_point.lane_id()) == black_origins.end());

  std::vector<mapping::ElementId> black_destinations =
      internal::MinLaneChangeBlacklist(semantic_map_manager, destination_point,
                                       false);

  EXPECT_TRUE(std::find(black_destinations.begin(), black_destinations.end(),
                        destination_point.lane_id()) ==
              black_destinations.end());
}

}  // namespace
}  // namespace qcraft::planner
