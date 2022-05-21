#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_

#include <vector>

#include "onboard/planner/scene/proto/scene_understanding.pb.h"

namespace qcraft::planner {
struct TrafficFlowReasoningOutput {
  std::vector<TrafficWaitingQueueProto> traffic_waiting_queues;
  std::vector<ObjectAnnotationProto> object_annotations;
};

}  // namespace qcraft::planner
#endif
