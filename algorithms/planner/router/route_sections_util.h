#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_

#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_section_sequence.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner {

struct PointOnRouteSections {
  double accum_s;
  int section_idx;
  double fraction;
  mapping::ElementId lane_id;
};

// Assume that global sections and local sections have overlaps and global
// sections are longer than local sections.
absl::StatusOr<RouteSections> AlignRouteSections(
    const RouteSections &global_sections, const RouteSections &local_sections);

// TODO(weijun): delete RouteSectionSequence
RouteSections ConvertRouteSectionSequenceToRouteSections(
    const RouteSectionSequence &route_section_sequence);

RouteSections RouteSectionsFromCompositeLanePath(const SemanticMapManager &smm,
                                                 const CompositeLanePath &clp);

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByLateralOffset(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, Vec2d query_point,
    double lat_dist_thres = kMaxHalfLaneWidth);

absl::StatusOr<mapping::LanePath>
FindClosestLanePathOnRouteSectionsToSmoothPoint(
    const PlannerSemanticMapManager &psmm, const RouteSections &sections,
    Vec2d query_point, double *proj_s = nullptr);

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByDrivePassage(
    const PlannerSemanticMapManager &psmm, const RouteSections &sections,
    Vec2d query_point);

absl::StatusOr<RouteSections> ClampRouteSectionsBeforeArcLength(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &raw_route_sections, double len);

absl::StatusOr<RouteSections> ClampRouteSectionsAfterArcLength(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &raw_route_sections, double len);

// first: route sections from pos.
// second: route sections including behind parts.
absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager &psmm,
                            const RouteSections &route_sections, Vec2d pos,
                            double projection_range, double keep_behind_length);

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSections(const PlannerSemanticMapManager &psmm,
                                  const RouteSections &route_sections);

RouteSections BackwardExtendRouteSections(const PlannerSemanticMapManager &psmm,
                                          const RouteSections &raw_sections,
                                          double extend_len);

absl::StatusOr<RouteSections> BackwardExtendRouteSectionsFromPos(
    const PlannerSemanticMapManager &psmm, const RouteSections &raw_sections,
    Vec2d pos, double extend_len);

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager &psmm, const RouteSections &route_sections,
    const mapping::LanePath &raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> FindClosestTargetLanePathOnReset(
    const PlannerSemanticMapManager &psmm, const RouteSections &prev_sections,
    Vec2d ego_pos);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_
