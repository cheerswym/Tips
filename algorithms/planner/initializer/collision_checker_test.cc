#include "onboard/planner/initializer/collision_checker.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/initializer/brute_force_collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {
std::ostream& operator<<(std::ostream& os,
                         const CollisionInfo::ObjectCollision& o) {
  return os << "{id: " << o.traj->traj_id() << ", time: " << o.time << "}";
}
std::ostream& operator<<(std::ostream& os,
                         const CollisionInfo& collision_info) {
  for (const auto& o : collision_info.collision_objects) {
    os << o << ", ";
  }
  return os;
}

// operator== Must in the same namespace with the class definition to make
// EXPECT_EQ work.
bool operator==(const CollisionInfo::ObjectCollision& a,
                const CollisionInfo::ObjectCollision& b) {
  return a.traj->traj_id() == b.traj->traj_id() && a.time == b.time;
}
bool operator!=(const CollisionInfo::ObjectCollision& a,
                const CollisionInfo::ObjectCollision& b) {
  return !(a == b);
}

bool operator==(const CollisionInfo& lhs, const CollisionInfo& rhs) {
  if (lhs.collision_objects.size() != rhs.collision_objects.size()) {
    return false;
  }
  const auto sorter = [](const CollisionInfo::ObjectCollision& a,
                         const CollisionInfo::ObjectCollision& b) {
    if (a.traj->traj_id() < b.traj->traj_id()) return true;
    if (a.traj->traj_id() > b.traj->traj_id()) return false;
    return a.time < b.time;
  };
  auto lhs_collisions = lhs.collision_objects;
  auto rhs_collisions = rhs.collision_objects;
  absl::c_sort(lhs_collisions, sorter);
  absl::c_sort(rhs_collisions, sorter);
  for (int i = 0; i < lhs_collisions.size(); ++i) {
    if (lhs_collisions[i] != rhs_collisions[i]) {
      return false;
    }
  }
  return true;
}
bool operator!=(const CollisionInfo& lhs, const CollisionInfo& rhs) {
  return !(lhs == rhs);
}

namespace {
constexpr double kBuffer = 0.25;  // m.

using testing::UnorderedElementsAre;

void CheckResult(const BoxGroupCollisionChecker& checker,
                 const BruteForceCollisionChecker& gt_checker, double init_t,
                 const MotionForm& motion, qcraft::SourceLocation loc) {
  ASSERT_EQ(checker.sample_step(), gt_checker.sample_step());
  CollisionInfo info;
  const auto states = motion.FastSample(checker.sample_step());
  checker.CheckCollisionWithTrajectories(init_t, states, &info);
  CollisionInfo bf_info;
  gt_checker.CheckCollisionWithTrajectories(init_t, states, &bf_info);
  EXPECT_EQ(info, bf_info) << loc.ToString();

  info.Clear();
  checker.CheckCollision(init_t, states, &info);
  bf_info.Clear();
  gt_checker.CheckCollision(init_t, states, &bf_info);
  EXPECT_EQ(info, bf_info) << loc.ToString();
}
TEST(CollisionChecker, StraightLineStatic) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj =
      perception_builder.set_id("abc")
          .set_type(OT_VEHICLE)
          .set_timestamp(1.0)
          .set_velocity(0.0)
          .set_yaw(0.0)
          .set_length_width(4.0, 2.0)
          .set_contour(Polygon2d(Box2d(Vec2d(8.0, 8.0), 0.0, 2.0, 2.0)))
          .set_box_center(Vec2d::Zero())
          .Build();
  PlannerObjectBuilder builder;

  builder.set_type(OT_VEHICLE).set_object(perception_obj).set_stationary(true);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_stationary_traj(Vec2d(8.0, 8.0), 0.0);
  const PlannerObject object = builder.Build();

  SpacetimeTrajectoryManager mgr(absl::MakeSpan(&object, 1));

  const auto vehicle_geom = DefaultVehicleGeometry();

  const BoxGroupCollisionChecker checker(
      &mgr, &vehicle_geom,
      /*sample_step=*/1, /*stationary_object_buffer=*/kBuffer,
      /*moving_object_buffer=*/2.0 * kBuffer);
  const BruteForceCollisionChecker brute_force(
      &mgr, &vehicle_geom,
      /*sample_step=*/1, /*stationary_object_buffer=*/kBuffer,
      /*moving_object_buffer=*/2.0 * kBuffer);

  {
    const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
    const ConstAccelMotion motion(/*init_v=*/1.0, /*init_a=*/1.0, &line);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion, QCRAFT_LOC);
  }
}

TEST(CollisionChecker, StraightLine) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();
  PlannerObjectBuilder builder;

  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  const PlannerObject object = builder.Build();

  SpacetimeTrajectoryManager mgr(absl::MakeSpan(&object, 1));

  const auto vehicle_geom = DefaultVehicleGeometry();

  const BoxGroupCollisionChecker checker(
      &mgr, &vehicle_geom,
      /*sample_step=*/1, /*stationary_object_buffer=*/kBuffer,
      /*moving_object_buffer=*/2.0 * kBuffer);
  const BruteForceCollisionChecker brute_force(
      &mgr, &vehicle_geom,
      /*sample_step=*/1, /*stationary_object_buffer=*/kBuffer,
      /*moving_object_buffer=*/2.0 * kBuffer);

  {
    const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
    const ConstAccelMotion motion(/*init_v=*/1.0, /*init_a=*/1.0, &line);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion, QCRAFT_LOC);
  }

  {
    // No collision when no time overlap.
    const double last_time =
        mgr.trajectories()[1]->states().back().traj_point->t();
    const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
    const ConstAccelMotion motion(/*init_v=*/1.0, /*init_a=*/1.0, &line);
    CheckResult(checker, brute_force, /*init_t=*/last_time + 1.0, motion,
                QCRAFT_LOC);
  }

  {
    // Collision with one prediction trajectory.
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/10.0, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion, QCRAFT_LOC);
  }

  {
    // No collision as SDC disappears before object arrives.
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/0.5, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion, QCRAFT_LOC);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
