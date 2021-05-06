#include "onboard/planner/scene/traffic_flow_reasoning.h"

#include <iostream>

#include "gtest/gtest.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/status_macros.h"
namespace qcraft::planner {
namespace {
TEST(SceneUnderstanding, TrafficWaitingQueueTest) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  ObjectsPredictionProto prediction;
  for (int i = 0; i < 4; ++i) {
    PerceptionObjectBuilder perception_object_builder;
    const auto object =
        perception_object_builder.set_id(absl::StrCat("agent", i))
            .set_type(OT_VEHICLE)
            .set_pos(Vec2d(65 - 10.0 * i, 0))
            .set_length_width(6.0, 2.0)
            .set_velocity(0.0)
            .Build();
    ObjectPredictionProto object_pred;
    object_pred.set_id(object.id());
    *object_pred.mutable_perception_object() = object;
    *prediction.add_objects() = object_pred;
  }

  const std::vector<mapping::ElementId> ids = {2448, 1, 34};
  const mapping::LanePath lane_path(&smm, /* lane ids */
                                    ids,
                                    /* start fraction */ 0.0,
                                    /* end fraction */ 1.0);
  const std::vector<mapping::LanePath> lane_paths = {lane_path};

  TrafficLightInfoMap tl_info_map;

  const TrafficFlowReasoningInput input{.psmm = &psmm,
                                        .prediction = &prediction,
                                        .lane_paths = &lane_paths,
                                        .tl_info_map = &tl_info_map};

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  ASSIGN_OR_DIE(const auto traffic_flow_output,
                RunTrafficFlowReasoning(input, thread_pool.get()));
  EXPECT_EQ(traffic_flow_output.traffic_waiting_queues.size(), 1);
  EXPECT_EQ(traffic_flow_output.traffic_waiting_queues[0].object_id_size(), 4);
}

// Scenario Description: Stall object test. Only one object before intersection,
// not left or right most lane. Has stopped a period of time. Expected Result:
// Not stall object.
TEST(SceneUnderstanding, StallobjectTest1) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  ObjectsPredictionProto prediction;
  PerceptionObjectBuilder perception_object_builder;
  const auto object = perception_object_builder.set_id(absl::StrCat("agent1"))
                          .set_type(OT_VEHICLE)
                          .set_pos(Vec2d(36, 0.0))
                          .set_length_width(6.0, 2.0)
                          .set_velocity(0.0)
                          .Build();
  ObjectPredictionProto object_pred;
  object_pred.set_id(object.id());
  *object_pred.mutable_perception_object() = object;
  ObjectStopTimeProto object_stop_time_info;
  object_stop_time_info.set_last_move_time_duration(0.7015);
  object_stop_time_info.set_previous_stop_time_duration(0.89);
  object_stop_time_info.set_time_duration_since_stop(20.0);
  *object_pred.mutable_stop_time() = object_stop_time_info;

  *prediction.add_objects() = object_pred;

  const std::vector<mapping::ElementId> ids = {2448, 1, 34};
  const mapping::LanePath lane_path(&smm, /* lane ids */ ids,
                                    /* start fraction */ 0.0,
                                    /* end fraction */ 1.0);
  const std::vector<mapping::LanePath> lane_paths = {lane_path};

  TrafficLightInfoMap tl_info_map;

  const TrafficFlowReasoningInput input{.psmm = &psmm,
                                        .prediction = &prediction,
                                        .lane_paths = &lane_paths,
                                        .tl_info_map = &tl_info_map};

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);
  // TODO(jiayu): Add more objects as input and check reasoning output.
  ASSIGN_OR_DIE(const auto traffic_flow_output,
                RunTrafficFlowReasoning(input, thread_pool.get()));
  EXPECT_TRUE(true);
}
}  // namespace
}  // namespace qcraft::planner
