
#ifndef ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/plan/fallback_planner.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct PathBoundedEstPlannerOutput {
  std::vector<SchedulerOutput> scheduler_output_list;
  std::vector<PlannerStatus> est_status_list;
  std::vector<EstPlannerOutput> est_planner_output_list;
  std::vector<EstPlannerDebug> est_planner_debug_list;
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_list;

  PlannerStatus fallback_status;
  FallbackPlannerDebugProto fallback_debug;
  vis::vantage::ChartDataBundleProto fallback_chart_data;

  PlannerStatus expert_status;
  ExpertPlannerDebugProto expert_debug;
  vis::vantage::ChartDataBundleProto expert_chart_data;

  SelectorDebugProto selector_debug;

  ObjectsPredictionProto speed_considered_objects_prediction;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PATH_BOUNDED_EST_PLANNER_OUTPUT_H_
