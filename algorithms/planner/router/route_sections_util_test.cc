#include "onboard/planner/router/route_sections_util.h"

#include "gtest/gtest.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

TEST(RouteSectionsUtilTest, AlignSectionsTest) {
  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.0, 1.0, {1, 2, 3, 4, 5}, destination);
    const RouteSections local_sections(0.2, 1.0, {1}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);
    EXPECT_EQ(aligned_sections->size(), 5);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);
    EXPECT_EQ(aligned_sections->section_ids().front(), 1);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.0, 1.0, {1, 2, 3, 4, 5}, destination);
    const RouteSections local_sections(0.2, 1.0, {6}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_TRUE(!aligned_sections.ok());
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.0, 1.0, {1, 2, 3, 4, 5}, destination);
    const RouteSections local_sections(0.2, 1.0, {0, 1}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 6);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.2, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);

    for (int i = 0; i < 6; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i], i);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.0, 1.0, {1, 2, 3}, destination);
    const RouteSections local_sections(0.0, 0.9, {1, 2}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    EXPECT_NEAR(aligned_sections->start_fraction(), 0.0, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 1.0, 1e-10);

    for (int i = 0; i < 3; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i], i + 1);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.0, 0.9, {1}, destination);
    const RouteSections local_sections(0.1, 1.0, {1}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id, 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.2, 0.9, {1}, destination);
    const RouteSections local_sections(0.1, 1.0, {1}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id, 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.2, 0.9, {1, 2}, destination);
    const RouteSections local_sections(0.1, 1.0, {0, 1, 2}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    for (int i = 0; i < 3; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i], i);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.2, 0.9, {1}, destination);
    const RouteSections local_sections(0.1, 0.8, {1}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 1);
    EXPECT_EQ(aligned_sections->front().id, 1);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }

  {
    const mapping::LanePoint destination(10, 1.0);

    const RouteSections global_sections(0.2, 0.9, {1, 2}, destination);
    const RouteSections local_sections(0.1, 0.7, {0, 1, 2}, destination);

    const auto aligned_sections =
        AlignRouteSections(global_sections, local_sections);
    EXPECT_OK(aligned_sections);

    EXPECT_EQ(aligned_sections->size(), 3);
    for (int i = 0; i < 3; ++i)
      EXPECT_EQ(aligned_sections->section_ids()[i], i);

    EXPECT_NEAR(aligned_sections->start_fraction(), 0.1, 1e-10);
    EXPECT_NEAR(aligned_sections->end_fraction(), 0.9, 1e-10);
  }
}

TEST(RouteSectionsUtilTest, FindClosestLanePathOnRouteSectionsToSmoothPoint) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&smm, PlannerSemanticMapModification());

  const mapping::LanePoint destination(2555, 0.9);
  const RouteSections route_sections(0.2, 0.9, {12442, 12443}, destination);

  double proj_s;
  const auto closest_lane_path_or =
      FindClosestLanePathOnRouteSectionsToSmoothPoint(
          psmm, route_sections, Vec2d(345.0, 12.5), &proj_s);
  ASSERT_OK(closest_lane_path_or);

  EXPECT_EQ(closest_lane_path_or->front().lane_id(), 230);
  EXPECT_NEAR(closest_lane_path_or->start_fraction(), 0.2, 1e-5);

  EXPECT_EQ(closest_lane_path_or->back().lane_id(), 2554);
  EXPECT_NEAR(closest_lane_path_or->end_fraction(), 0.9, 1e-5);

  EXPECT_NEAR(proj_s, 19.0, 0.1);
}

TEST(ClampRouteSectionsBeforeArcLengthTest, ClampRouteSections) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&semantic_map_manager,
                                       PlannerSemanticMapModification());

  const mapping::LanePoint dest(10, 1.0);

  {
    RouteSections raw_sections(0.0, 0.9, {12415}, dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 10.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->end_fraction(), 0.286, 0.01);
  }

  {
    RouteSections raw_sections(0.1, 0.9, {12318, 12412, 12414}, dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 30.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 2);
  }

  {
    RouteSections raw_sections(0.0, 1.0, {12004}, dest);

    const auto clamped_sections =
        ClampRouteSectionsBeforeArcLength(psmm, raw_sections, 70.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->end_fraction(), 1.0, 0.01);
  }
}

TEST(ClampRouteSectionsAfterArcLength, ClampRouteSections) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&semantic_map_manager,
                                       PlannerSemanticMapModification());

  const mapping::LanePoint dest(10, 1.0);

  {
    RouteSections raw_sections(0.0, 0.9, {12357}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 10.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.1, 0.01);
  }

  {
    RouteSections raw_sections(0.0, 0.9, {12357, 12356}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 105.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.11, 0.01);
  }

  {
    RouteSections raw_sections(0.0, 0.99, {12357, 12356, 12355}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 135.0);
    EXPECT_OK(clamped_sections);

    EXPECT_EQ(clamped_sections->size(), 1);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.11, 0.01);
  }

  {
    RouteSections raw_sections(0.0, 0.99, {12357, 12356, 12355}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 50.0);
    EXPECT_OK(clamped_sections);
    EXPECT_EQ(clamped_sections->size(), 3);
    EXPECT_NEAR(clamped_sections->start_fraction(), 0.5, 0.01);
  }

  {
    RouteSections raw_sections(0.0, 0.99, {12357, 12356, 12355}, dest);

    const auto clamped_sections =
        ClampRouteSectionsAfterArcLength(psmm, raw_sections, 160.0);

    EXPECT_FALSE(clamped_sections.ok());
  }
}

TEST(BackwardExtendRouteSections, BackExtend) {
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&semantic_map_manager,
                                       PlannerSemanticMapModification());

  mapping::LanePoint destination(10, 1.0);

  {
    RouteSections raw_sections(0.0, 1.0, {12335}, destination);

    const auto extend_sections =
        BackwardExtendRouteSections(psmm, raw_sections, 50.0);

    EXPECT_EQ(extend_sections.size(), 3);
  }

  {
    RouteSections raw_sections(0.1, 1.0, {12335}, destination);

    const auto extend_sections =
        BackwardExtendRouteSections(psmm, raw_sections, 50.0);
    LOG(INFO) << raw_sections.DebugString();
    EXPECT_EQ(extend_sections.size(), 3);
  }
}

TEST(BackwardExtendRouteSectionsFromPos, BackExtend) {
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&semantic_map_manager,
                                       PlannerSemanticMapModification());

  mapping::LanePoint destination(10, 1.0);

  {
    RouteSections raw_sections(0.0, 1.0, {12401}, destination);

    const auto extend_sections_or = BackwardExtendRouteSectionsFromPos(
        psmm, raw_sections, Vec2d(1.0, -3.5), 5.0);

    ASSERT_OK(extend_sections_or);

    EXPECT_EQ(extend_sections_or->size(), 2);
    EXPECT_TRUE(extend_sections_or->section_ids().front() == 12310 ||
                extend_sections_or->section_ids().front() == 12201);
  }

  {
    RouteSections raw_sections(0.01, 1.0, {12415}, destination);

    const auto extend_sections_or = BackwardExtendRouteSectionsFromPos(
        psmm, raw_sections, Vec2d(118.615, 68.89), 10.0);
    ASSERT_OK(extend_sections_or);

    EXPECT_EQ(extend_sections_or->size(), 2);
    EXPECT_EQ(extend_sections_or->section_ids().front(), 12298);
  }
}

TEST(RouteSectionsUtilTest, FindClosestTargetLanePathOnReset) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const PlannerSemanticMapManager psmm(&smm, PlannerSemanticMapModification());

  const mapping::LanePoint destination(162, 0.9);
  const RouteSections route_sections(
      0.0, 0.9,
      {12401, 12400, 12408, 12412, 12414, 12285, 12433, 12445, 12219, 12088,
       12078, 12087, 12218, 12442, 12443, 12441},
      destination);
  const double behind_length = 10.0;
  const double local_horizon =
      route_sections.planning_horizon(psmm) + kLocalMapExtension;

  {
    const auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        psmm, route_sections, Vec2d(behind_length, 0.9));
    ASSERT_OK(target_lane_path_or);

    EXPECT_EQ(target_lane_path_or->front().lane_id(), 2448);
    EXPECT_NEAR(target_lane_path_or->start_fraction(), 0.0, 1e-5);

    EXPECT_NEAR(target_lane_path_or->length(), local_horizon + behind_length,
                0.2);

    const auto last_lane_pt = target_lane_path_or->ArclengthToLanePoint(
        local_horizon + behind_length);
    EXPECT_EQ(target_lane_path_or->back().lane_id(), last_lane_pt.lane_id());
    EXPECT_NEAR(target_lane_path_or->end_fraction(), last_lane_pt.fraction(),
                1e-2);
  }

  {
    const auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        psmm, route_sections, Vec2d(behind_length, 2.9));
    ASSERT_OK(target_lane_path_or);

    EXPECT_EQ(target_lane_path_or->front().lane_id(), 2);
    EXPECT_NEAR(target_lane_path_or->start_fraction(), 0.0, 1e-5);

    EXPECT_EQ(target_lane_path_or->back().lane_id(), 51);
    EXPECT_NEAR(target_lane_path_or->end_fraction(), 1.0, 1e-5);

    EXPECT_NEAR(target_lane_path_or->length(), 255.5, 0.1);
  }
}
}  // namespace qcraft::planner
