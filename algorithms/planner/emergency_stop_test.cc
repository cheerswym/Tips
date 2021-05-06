#include "onboard/planner/emergency_stop.h"

#include <optional>

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {
bool TestEmergencyStop(const ObjectsProto &objects_proto,
                       const std::string &canvas_name, double vehicle_vel,
                       double steering_percentage) {
  auto to_object_polygon = [](const auto &object_proto) {
    std::vector<Vec2d> vertices;
    vertices.reserve(object_proto.contour_size());
    for (int i = 0; i < object_proto.contour_size(); ++i) {
      vertices.push_back(Vec2dFromProto(object_proto.contour(i)));
    }
    return Polygon2d(std::move(vertices));
  };

  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleDriveParamsProto vehicle_drive_params = DefaultVehicleDriveParams();
  qcraft::Chassis chassis;
  chassis.set_steering_percentage(steering_percentage);
  PoseProto vehicle_pose;
  vehicle_pose.mutable_pos_smooth()->set_x(0.0);
  vehicle_pose.mutable_pos_smooth()->set_y(0.0);
  vehicle_pose.mutable_vel_smooth()->set_x(vehicle_vel);
  vehicle_pose.mutable_vel_smooth()->set_y(0.0);
  vehicle_pose.set_yaw(0.0);
  Vec2d vec_to_vehicle_center((vehicle_geom.front_edge_to_center() +
                               vehicle_geom.back_edge_to_center()) *
                                      0.5 -
                                  vehicle_geom.back_edge_to_center(),
                              0.0);
  Vec2d vehicle_pos(vehicle_pose.pos_smooth().x(),
                    vehicle_pose.pos_smooth().y());
  Vec2d vehicle_center(vehicle_pos +
                       vec_to_vehicle_center.FastRotate(vehicle_pose.yaw()));
  const auto vehicle_polygon =
      Polygon2d(Box2d(vehicle_center, vehicle_pose.yaw(), vehicle_geom.length(),
                      vehicle_geom.width()));

  // Plot ego vehicle and object.
  auto *canvas = &(vantage_client_man::GetCanvas(canvas_name));
  canvas->SetGroundZero(1);
  std::vector<Vec3d> vehicle_points;
  for (const auto &vertice : vehicle_polygon.GetAllVertices()) {
    vehicle_points.emplace_back(vertice.x(), vertice.y(), 0.0);
  }
  canvas->DrawPolygon(vehicle_points, vis::Color(0.1, 0.2, 0.55), 2);
  for (const auto &object : objects_proto.objects()) {
    const auto object_polygon = to_object_polygon(object);
    std::vector<Vec3d> object_points;
    for (const auto &vertice : object_polygon.GetAllVertices()) {
      object_points.emplace_back(vertice.x(), vertice.y(), 0.0);
    }
    canvas->DrawPolygon(object_points, vis::Color(0.6, 0.2, 0.55), 2);
  }

  const auto emerency_stop_check_start_time = absl::Now();

  Polygon2d risk_area;
  auto is_emergency_or = aeb::CheckEmergencyStopByCircularMotion(
      vehicle_geom, planner_params.emergency_stop_params(),
      vehicle_drive_params, vehicle_pose, objects_proto, chassis, &risk_area);

  const absl::Duration emergency_stop_check_time =
      absl::Now() - emerency_stop_check_start_time;

  qcraft::vantage_client_man::FlushAll();

  QLOG(INFO) << "check emergency time is: " << emergency_stop_check_time;

  return is_emergency_or.ok();
}

TEST(EmergencStopTest, RightSideTestOutside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object..
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(4.314);
  contour_point_1->set_y(-1.521);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(4.624);
  contour_point_2->set_y(-1.585);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(4.552);
  contour_point_3->set_y(-1.820);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(4.089);
  contour_point_4->set_y(-1.647);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "right_side_test_outside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_FALSE(emergency_stop);
}

TEST(EmergencStopTest, RightSideTestInside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object..
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(5.082);
  contour_point_1->set_y(-1.761);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(5.033);
  contour_point_2->set_y(-2.125);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(5.295);
  contour_point_3->set_y(-2.456);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(5.600);
  contour_point_4->set_y(-2.114);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "right_side_test_inside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_TRUE(emergency_stop);
}

TEST(EmergencStopTest, LeftSideTestOutside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object..
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(4.296);
  contour_point_1->set_y(1.078);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(4.763);
  contour_point_2->set_y(0.790);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(4.860);
  contour_point_3->set_y(1.303);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(4.341);
  contour_point_4->set_y(1.488);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "left_side_test_outside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_FALSE(emergency_stop);
}

TEST(EmergencStopTest, LeftSideTestInside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object.
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(5.743);
  contour_point_1->set_y(0.042);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(5.685);
  contour_point_2->set_y(0.738);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(5.354);
  contour_point_3->set_y(0.962);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(5.24);
  contour_point_4->set_y(0.658);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "left_side_test_inside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_TRUE(emergency_stop);
}

TEST(EmergencStopTest, UpperSideTestOutside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object.
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(6.495);
  contour_point_1->set_y(-1.110);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(6.857);
  contour_point_2->set_y(-1.096);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(6.660);
  contour_point_3->set_y(-1.723);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(6.313);
  contour_point_4->set_y(-1.786);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "upper_side_test_outside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_FALSE(emergency_stop);
}

TEST(EmergencStopTest, UpperSideTestInside) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Need to feed: vehicle_vel,steering_percentage.
  constexpr double vehicle_vel = 5.0;
  constexpr double steering_percentage = -70.0;
  // Need to feed: object.
  ObjectProto object = PerceptionObjectBuilder().set_id("aeb_object").Build();
  object.clear_contour();
  auto *contour_point_1 = object.add_contour();
  contour_point_1->set_x(6.088);
  contour_point_1->set_y(-1.947);
  auto *contour_point_2 = object.add_contour();
  contour_point_2->set_x(5.810);
  contour_point_2->set_y(-1.815);
  auto *contour_point_3 = object.add_contour();
  contour_point_3->set_x(5.686);
  contour_point_3->set_y(-2.323);
  auto *contour_point_4 = object.add_contour();
  contour_point_4->set_x(5.951);
  contour_point_4->set_y(-2.625);
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = object;

  std::string canvas_name = "upper_side_test_inside";

  bool emergency_stop = TestEmergencyStop(objects_proto, canvas_name,
                                          vehicle_vel, steering_percentage);

  EXPECT_TRUE(emergency_stop);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
