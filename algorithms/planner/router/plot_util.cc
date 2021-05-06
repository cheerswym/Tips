#include "onboard/planner/router/plot_util.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {

namespace {

std::vector<Polygon2d> SampleLaneBoundaryPolyLine(
    const mapping::LaneInfo &left_lane_info,
    const mapping::LaneInfo &right_lane_info, double start_fraction,
    double end_fraction) {
  constexpr double kStepS = 10.0;  // m.

  const double left_len =
      left_lane_info.length() * (end_fraction - start_fraction);
  const double right_len =
      right_lane_info.length() * (end_fraction - start_fraction);

  const int n =
      Max(1, RoundToInt(left_len / kStepS), RoundToInt(right_len / kStepS));

  const double left_step_fraction = (left_len / n) / left_lane_info.length();
  const double right_step_fraction = (right_len / n) / right_lane_info.length();

  std::vector<Polygon2d> polygons;
  polygons.reserve(n);

  const auto lerp_point = [](const mapping::LaneInfo &lane_info,
                             double fraction, int sign) {
    Vec2d tan;
    lane_info.GetTangent(fraction, &tan);
    const Vec2d nor = tan.Perp();
    return lane_info.LerpPointFromFraction(fraction) +
           nor * (sign * kDefaultHalfLaneWidth);
  };

  double left_fraction = start_fraction;
  double right_fraction = start_fraction;
  for (int i = 0; i < n; ++i) {
    const auto p0 = lerp_point(left_lane_info, Min(left_fraction, 1.0), 1);
    const auto p1 = lerp_point(left_lane_info,
                               Min(left_fraction + left_step_fraction, 1.0), 1);
    const auto p2 = lerp_point(
        right_lane_info, Min(right_fraction + right_step_fraction, 1.0), -1);
    const auto p3 = lerp_point(right_lane_info, Min(right_fraction, 1.0), -1);

    polygons.emplace_back(Polygon2d({p0, p1, p2, p3}));

    left_fraction += left_step_fraction;
    right_fraction += right_step_fraction;
  }

  return polygons;
}

}  // namespace

void SendDrivePassageToCanvas(const DrivePassage &drive_passage,
                              const std::string &channel) {
  VLOG(3) << "draw drive passage...";
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto &station : drive_passage.stations()) {
    canvas.DrawPoint(Vec3d(station.xy(), 0), vis::Color::kLightBlue, 10);
    canvas.DrawText(absl::StrCat(station.accumulated_s()),
                    Vec3d(station.xy(), 0), station.tangent().FastAngle(), 0.1,
                    vis::Color::kLightRed);
    double left_max_dis = 0;
    double right_max_dis = 0;
    for (const auto &boundary : station.boundaries()) {
      if (boundary.lat_offset > left_max_dis)
        left_max_dis = boundary.lat_offset;
      if (boundary.lat_offset < right_max_dis)
        right_max_dis = boundary.lat_offset;
      switch (boundary.type) {
        case StationBoundaryType::BROKEN_WHITE:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset), 0),
                           vis::Color(0.7, 0.7, 0.7, 0.1), 8);
          break;
        case StationBoundaryType::SOLID_WHITE:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset), 0),
                           vis::Color(1.0, 1.0, 1.0, 1), 10);
          break;
        case StationBoundaryType::BROKEN_YELLOW:
        case StationBoundaryType::SOLID_YELLOW:
        case StationBoundaryType::SOLID_DOUBLE_YELLOW:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset), 0),
                           vis::Color(1.0, 1.0, 0.0, 1), 10);
          break;
        case StationBoundaryType::CURB:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset), 0),
                           vis::Color(1.0, 0.1, 0.0, 1), 10);
          break;
        case StationBoundaryType::VIRTUAL_CURB:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset), 0),
                           vis::Color(0.55, 0.0, 0.0, 1), 10);
          break;
        case StationBoundaryType::UNKNOWN_TYPE:
          break;
      }
    }
    canvas.DrawLine(Vec3d(station.lat_point(right_max_dis), 0),
                    Vec3d(station.lat_point(left_max_dis), 0),
                    vis::Color(0.8, 0.8, 0.8, 0.2), 1);
  }

  vantage_client_man::FlushAll();
}

void SendRouteLanePathToCanvas(const SemanticMapManager &semantic_map_manager,
                               const CompositeLanePath &route_lane_path,
                               const std::string &topic) {
  QCHECK(!route_lane_path.IsEmpty());
  vis::Canvas &canvas = vantage_client_man::GetCanvas(topic);
  canvas.SetGroundZero(1);
  const auto render_lane_points = [&canvas](const std::vector<Vec2d> &points,
                                            vis::Color color) {
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](Vec2d p) { return Vec3d(p, 1.0); });
    canvas.DrawLineStrip(points_3d, color, 3);
    canvas.DrawCircle(points_3d.front(), 0.1, color, color);
  };
  const auto render_lane_portion = [&semantic_map_manager, &render_lane_points](
                                       const mapping::ElementId lane_id,
                                       double start_fraction,
                                       double end_fraction, vis::Color color) {
    const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(lane_id);
    const auto start_seg_frac = lane_info.SegmentFraction(start_fraction);
    const auto end_seg_frac = lane_info.SegmentFraction(end_fraction);
    std::vector<Vec2d> points;
    points.push_back(Lerp(lane_info.points_smooth[start_seg_frac.first],
                          lane_info.points_smooth[start_seg_frac.first + 1],
                          start_seg_frac.second));
    for (int i = start_seg_frac.first; i < end_seg_frac.first; ++i) {
      points.push_back(lane_info.points_smooth[i + 1]);
    }
    points.push_back(Lerp(lane_info.points_smooth[end_seg_frac.first],
                          lane_info.points_smooth[end_seg_frac.first + 1],
                          end_seg_frac.second));
    render_lane_points(points, color);
  };
  const auto render_lane_path_portion =
      [&render_lane_portion](const mapping::LanePath &lane_path, int lane_index,
                             double start_fraction, double end_fraction,
                             vis::Color color) {
        const double clamped_start_fraction = std::max(
            lane_index == 0 ? lane_path.start_fraction() : 0.0, start_fraction);
        const double clamped_end_fraction = std::min(
            lane_index + 1 == lane_path.size() ? lane_path.end_fraction() : 1.0,
            end_fraction);
        render_lane_portion(lane_path.lane_id(lane_index),
                            clamped_start_fraction, clamped_end_fraction,
                            color);
      };

  const vis::Color kColorBeforeEntry(0.4, 0.6, 0.8);
  const vis::Color kColorActive(0.6, 1.0, 0.6);
  const vis::Color kColorAfterExit(0.8, 0.6, 0.4);

  for (int i = 0; i < route_lane_path.num_lane_paths(); ++i) {
    const mapping::LanePath &lane_path_i = route_lane_path.lane_path(i);
    const auto &entry = route_lane_path.LanePathEntryIndexPoint(i);
    const auto &exit = route_lane_path.LanePathExitIndexPoint(i);
    VLOG(3) << "lane path index " << i << ": " << lane_path_i.DebugString();
    VLOG(3) << "  entry: " << entry.first << " " << entry.second;
    VLOG(3) << "  entry: " << exit.first << " " << exit.second;

    // Draw the portion before entry point.
    for (int j = 0; j < entry.first; ++j) {
      render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorBeforeEntry);
    }
    render_lane_path_portion(lane_path_i, entry.first, 0.0, entry.second,
                             kColorBeforeEntry);
    if (entry.first == exit.first) {
      render_lane_path_portion(lane_path_i, entry.first, entry.second,
                               exit.second, kColorActive);
    } else {
      render_lane_path_portion(lane_path_i, entry.first, entry.second, 1.0,
                               kColorActive);
      for (int j = entry.first + 1; j < exit.first; ++j) {
        render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorActive);
      }
      render_lane_path_portion(lane_path_i, exit.first, 0.0, exit.second,
                               kColorActive);
    }
    render_lane_path_portion(lane_path_i, exit.first, exit.second, 1.0,
                             kColorAfterExit);
    for (int j = exit.first + 1; j < lane_path_i.size(); ++j) {
      render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorAfterExit);
    }
  }

  vantage_client_man::FlushAll();
}

std::vector<Polygon2d> SampleRouteSectionsPolygons(
    const SemanticMapManager *semantic_map_manager,
    const RouteSections &route_sections) {
  std::vector<Polygon2d> polygons;
  for (int i = 0; i < route_sections.size(); ++i) {
    const double start_fraction =
        i == 0 ? route_sections.start_fraction() : 0.0;
    const double end_fraction =
        i + 1 == route_sections.size() ? route_sections.end_fraction() : 1.0;

    const auto &section_info = semantic_map_manager->FindSectionInfoOrDie(
        route_sections.section_ids()[i]);
    const auto &left_lane_info =
        semantic_map_manager->FindLaneInfoOrDie(section_info.lane_ids.front());
    const auto &right_lane_info =
        semantic_map_manager->FindLaneInfoOrDie(section_info.lane_ids.back());

    for (auto &polygon : SampleLaneBoundaryPolyLine(
             left_lane_info, right_lane_info, start_fraction, end_fraction)) {
      polygons.emplace_back(std::move(polygon));
    }
  }
  return polygons;
}

void SendRouteSectionsAreaToCanvas(
    const SemanticMapManager *semantic_map_manager,
    const RouteSections &route_sections, const std::string &channel) {
  vis::Canvas &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);

  const vis::Color dark_blue(0.2, 0.81, 0.92, 0.2);
  for (const auto &polygon :
       SampleRouteSectionsPolygons(semantic_map_manager, route_sections)) {
    const auto &points = polygon.points();
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](Vec2d p) { return Vec3d(p, 0.1); });
    canvas.DrawPolygon(points_3d, vis::Color::kTransparent, dark_blue);
  }
  vantage_client_man::FlushAll();
}

}  // namespace planner
}  // namespace qcraft
