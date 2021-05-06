#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_

#include <string>
#include <vector>

#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/vis/common/colormap.h"

namespace qcraft::planner {

void SendSingleTrajectoryToCanvas(
    const MotionSearchOutput::TrajWithLeadingObject& traj_info, int idx,
    int planner_id);

void SendMultiTrajectoriesToCanvas(
    const std::vector<MotionSearchOutput::TrajWithLeadingObject>& multi_trajs,
    const std::string& planner_id);

void SendGeometryGraphToCanvas(const GeometryGraph* graph,
                               const std::string& channel);

void SendRefSpeedTableToCanvas(const RefSpeedTable& ref_speed_table,
                               const DrivePassage& drive_passage);

void SendPathPointsToCanvas(const std::vector<PathPoint>& points,
                            const std::string& channel, int planner_id,
                            vis::Color color, int point_size);

void ParseMotionSearchOutputToSearchResultProto(
    const MotionSearchOutput& search_output, MotionSearchResultProto* proto);

void ParseMotionSearchOutputToInitializerResultTrajectoryProto(
    const MotionSearchOutput& search_output,
    InitializerResultTrajectoryProto* proto);

void ExportMoionSpeedProfileToChart(const MotionSearchOutput& search_output,
                                    vis::vantage::ChartDataProto* chart);

void ParseDpMotionSearchOutputToDpSearchResultProto(
    const MotionSearchOutput& search_output, DpMotionSearchResultProto* proto);

void ParseDpMotionSearchTrajectoryWithLeadingObjToProto(
    const MotionSearchOutput& search_output, DpMotionSearchResultProto* proto);

/**
 * @brief: Dumping expert trajectory raw feature cost and all searched DP
 * trajectories's raw feature costs.
 * **/
void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
