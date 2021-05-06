#ifndef ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {

class PlannerObjectBuilder {
 public:
  PlannerObjectBuilder& set_id(std::string id);

  // TODO(lidong) Delete all the pose functions, as pose is a derived field in
  // PlannerObject.
  PlannerObjectBuilder& set_type(ObjectType type);

  PlannerObjectBuilder& set_object(const ObjectProto& object);

  PlannerObjectBuilder& set_pose(const SecondOrderTrajectoryPoint& pose);

  PlannerObjectBuilder& set_pos(Vec2d pos);

  PlannerObjectBuilder& set_v(double v);
  PlannerObjectBuilder& set_theta(double theta);
  PlannerObjectBuilder& set_t(double t);
  PlannerObjectBuilder& set_a(double a);

  PlannerObjectBuilder& set_stationary(bool stationary);

  PlannerObjectBuilder& set_contour(Polygon2d contour);

  ObjectPredictionBuilder* get_object_prediction_builder();

  PlannerObject Build();

 private:
  // TODO(lidong): Delete this function in the future.
  void FromObjectProto(const ObjectProto& object_proto);

  ObjectPredictionBuilder object_prediction_builder_;
  std::optional<ObjectProto> object_proto_;
  PlannerObject object_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_
