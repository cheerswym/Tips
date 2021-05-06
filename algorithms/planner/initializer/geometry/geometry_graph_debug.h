#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {

enum class ResampleReason {
  RESAMPLED = 1,
  NR_ZERO_REACHABLE = 2,
  NR_ALL_REACHABLE = 3,
  NR_INVALID_RANGE = 4,
  NR_LATERAL_RESOLUTION = 5,
  NOT_INITIALIZED = 6
};

struct EdgeDebugInfo {
  int start_layer_idx;
  int start_node_on_layer_idx;
  int end_layer_idx;
  int end_node_on_layer_idx;
  int start_station_idx;
  int end_station_idx;

  std::string Debug() const {
    std::string debug_str = "\n\tLayer Idx\tNode on Layer Idx\tStation Idx\n";
    debug_str +=
        absl::StrCat("Start\t", start_layer_idx, "\t", start_node_on_layer_idx,
                     "\t", start_station_idx, "\n");
    debug_str +=
        absl::StrCat("End\t", end_layer_idx, "\t", end_node_on_layer_idx, "\t",
                     end_station_idx, "\n");
    debug_str += "\n---------------------------------------------";
    return debug_str;
  }
};

void ParseResampleResultToProto(
    const std::vector<ResampleReason> &resample_result,
    GeometryGraphDebugProto *graph_debug_proto);

void ParseEdgeDebugInfoToGeometryGraphDebugProto(
    const EdgeDebugInfo &debug_info,
    GeometryGraphDebugProto *graph_debug_proto);

std::string ConstraintAccumSDebug(double constraint_stop_sampling_s,
                                  double max_deceleration_accumulated_s,
                                  double leading_obj_caused_target_s,
                                  double stopline_accumulated_s);

absl::Status CheckGeometryGraphConnectivity(const GeometryGraph &graph);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_
